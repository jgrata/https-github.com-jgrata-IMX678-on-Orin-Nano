/*
 * raw_capture.cpp - Continuous streaming buffer for fast RAW16 capture
 *
 * Server mode: pipeline runs continuously, latest frame in memory.
 * Client gets latest buffered frame immediately (no wait for new capture).
 * Frame rate limited by Argus RAW16 throughput (~sensor fps when pipelined).
 *
 * Single-shot mode unchanged (~7s per capture).
 *
 * Build:  cd /home/metro/raw_capture && cmake . && make -j4
 * Test:   ./raw_capture --mode 1 --exposure 33000000 --out /tmp/raw10.bin
 * Server: ./raw_capture --server --port 9001 --mode 1 --exposure 33000000
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <algorithm>
#include <set>
#include <string>
#include <vector>
#include <fstream>
#include <sys/socket.h>
#include <sys/select.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/mman.h>       /* shm_open, mmap — in-process zero-copy frame tap */
#include <fcntl.h>
#include <atomic>
#include <arm_neon.h>       /* NEON RAW10/RAW12 packers */
#include <thread>           /* serve thread — decouple TCP pack+send from capture */
#include <mutex>
#include <memory>

#include <EGL/egl.h>
#include <EGL/eglext.h>
#include <cuda.h>
#include <cudaEGL.h>
#include <Argus/Argus.h>
#include <Argus/Ext/DolWdrSensorMode.h>
#include <EGLStream/EGLStream.h>

using namespace Argus;
using namespace EGLStream;

// ── Globals ───────────────────────────────────────────────────────────────────

static EGLDisplay g_display = EGL_NO_DISPLAY;
static CUcontext  g_cuCtx   = nullptr;
static volatile bool g_running = true;
static void sig_handler(int) { g_running = false; }

typedef EGLBoolean (*PFN_eglQueryStreamKHR)(
    EGLDisplay, EGLStreamKHR, EGLenum, EGLint*);
static PFN_eglQueryStreamKHR fp_eglQueryStreamKHR = nullptr;

#define CU_CHECK(x) do { \
    CUresult _r=(x); if(_r!=CUDA_SUCCESS){ \
    const char*_s="?"; cuGetErrorString(_r,&_s); \
    fprintf(stderr,"[CUDA] %s: %s\n",#x,_s); return false;} \
} while(0)

static bool egl_init()
{
    g_display = eglGetDisplay(EGL_DEFAULT_DISPLAY);
    if (g_display==EGL_NO_DISPLAY) return false;
    EGLint maj=0,min=0;
    if (!eglInitialize(g_display,&maj,&min)) return false;
    printf("[EGL] %d.%d\n",maj,min); fflush(stdout);
    fp_eglQueryStreamKHR = (PFN_eglQueryStreamKHR)
        eglGetProcAddress("eglQueryStreamKHR");
    return true;
}

static bool cuda_init()
{
    CU_CHECK(cuInit(0));
    CUdevice dev; CU_CHECK(cuDeviceGet(&dev,0));
    CU_CHECK(cuDevicePrimaryCtxRetain(&g_cuCtx,dev));
    CU_CHECK(cuCtxSetCurrent(g_cuCtx));
    char name[128]=""; cuDeviceGetName(name,sizeof(name),dev);
    printf("[CUDA] %s\n",name); fflush(stdout);
    return true;
}

// ── Config ────────────────────────────────────────────────────────────────────

struct Config {
    int         mode       = 1;
    int         n_frames   = 1;
    int         fps        = 30;
    uint64_t    exp_ns     = 0;
    float       gain       = 0.0f;
    bool        server     = false;
    int         port       = 9001;
    std::string outfile    = "/tmp/raw_capture.bin";
    bool        list_modes = false;
};

#pragma pack(push,1)
struct FileHdr { uint32_t magic,w,h,bpp,nf,pad; };  /* 24 bytes */
struct NetHdr  {                                       /* 36 bytes */
    uint32_t magic,w,h,bpp,nf;
    uint64_t exp_ns;
    uint32_t gain_x1000,pad;
};
#pragma pack(pop)

// ── In-memory frame buffer (no disk I/O) ─────────────────────────────────────

struct FrameBuffer {
    std::vector<uint16_t> pixels;
    uint32_t w=0, h=0, bpp=0;
    bool     valid=false;
    int      seq=0;           /* increments each new frame */
    uint64_t exp_ns=0;
    float    gain=0.0f;
    double   capture_time=0.0; /* monotonic seconds */

    void update(std::vector<uint16_t>&& px,
                uint32_t fw, uint32_t fh, uint32_t fbpp,
                uint64_t fexp, float fgain)
    {
        pixels       = std::move(px);
        w=fw; h=fh; bpp=fbpp;
        exp_ns=fexp; gain=fgain;
        valid=true;
        seq++;
        struct timespec ts;
        clock_gettime(CLOCK_MONOTONIC,&ts);
        capture_time = ts.tv_sec + ts.tv_nsec*1e-9;
    }

    double age_ms() const {
        if (!valid) return 1e9;
        struct timespec ts;
        clock_gettime(CLOCK_MONOTONIC,&ts);
        double now = ts.tv_sec + ts.tv_nsec*1e-9;
        return (now - capture_time) * 1000.0;
    }
} g_buf;

// ── Published frame for server mode (capture thread swaps; serve thread reads) ─
// A shared_ptr swap hands the latest frame to the TCP serve thread with zero copy:
// the capture loop builds a new FrameBuffer and atomically republishes the pointer,
// so serve can pack+send from an immutable frame (kept alive by its own shared_ptr)
// WITHOUT blocking capture. Only the pointer swap is locked (microseconds).
static std::shared_ptr<FrameBuffer> g_pub;
static std::mutex                   g_pub_mtx;
static std::shared_ptr<FrameBuffer> pub_get()
{
    std::lock_guard<std::mutex> lk(g_pub_mtx);
    return g_pub;
}

// ── Shared-memory frame publisher (in-process zero-copy tap) ──────────────────
// Publishes each decoded uint16 frame into a POSIX shm ring so LOCAL consumers
// (the Jetson-side web UI / sensor-timing tools) read frames directly -- no
// RAW10 pack, no localhost TCP, no NumPy unpack. Remote clients keep using TCP
// (the 1 GbE port can't carry full-rate 4K RAW anyway). Layout is fixed and
// little-endian so Python parses it with struct + numpy (see shm_reader.py).
// SPSC: one writer (this capture loop), many readers; 4 slots + a published
// seq give a reader ~4 frame-periods to copy a slot before it's reused.
#pragma pack(push,1)
struct ShmHeader {                 /* at offset 0 */
    uint32_t magic;                /* 0x5741524D 'MRAW' (LE) */
    uint32_t version;
    uint32_t nslots;
    uint32_t slot_stride;          /* bytes per slot (meta + pixel data, 64-aligned) */
    uint32_t max_w, max_h;
    uint32_t data_off;             /* pixel-data offset within a slot */
    uint32_t _pad;
    uint64_t latest_seq;           /* published last (acts as the release signal) */
    uint32_t latest_slot;
    uint32_t _pad2;
};
struct ShmSlotMeta {               /* at the start of each slot */
    uint64_t seq;
    uint32_t w, h, bpp, _pad;
    uint64_t exp_ns;
    uint64_t sof_ns;               /* sensor start-of-frame timestamp (kernel) */
    double   gain;
    double   capture_time;         /* host CLOCK_MONOTONIC seconds */
};
#pragma pack(pop)

struct ShmPublisher {
    int        fd    = -1;
    uint8_t*   base  = nullptr;
    size_t     total = 0;
    ShmHeader* hdr   = nullptr;
    uint32_t   nslots=0, slot_stride=0, data_off=64;
    uint64_t   seq   = 0;
    static constexpr uint32_t HDR_SZ = 64;

    bool init(uint32_t w, uint32_t h, uint32_t nslots_=4, const char* name="/metro_raw") {
        nslots = nslots_;
        size_t slot_data = (size_t)w*h*2;
        slot_stride = (uint32_t)(((data_off + slot_data) + 63) & ~size_t(63));
        total = HDR_SZ + (size_t)slot_stride * nslots;
        fd = shm_open(name, O_CREAT|O_RDWR, 0666);
        if (fd < 0) { perror("[shm] shm_open"); return false; }
        if (ftruncate(fd, total) != 0) { perror("[shm] ftruncate"); return false; }
        base = (uint8_t*)mmap(nullptr, total, PROT_READ|PROT_WRITE, MAP_SHARED, fd, 0);
        if (base == MAP_FAILED) { perror("[shm] mmap"); base=nullptr; return false; }
        hdr = (ShmHeader*)base;
        hdr->magic=0x5741524D; hdr->version=1; hdr->nslots=nslots;
        hdr->slot_stride=slot_stride; hdr->max_w=w; hdr->max_h=h;
        hdr->data_off=data_off; hdr->_pad=0; hdr->latest_seq=0; hdr->latest_slot=0; hdr->_pad2=0;
        printf("[shm] /dev/shm%s ready: %ux%u  %u slots x %u B  (%.1f MB)\n",
               name, w, h, nslots, slot_stride, total/1e6);
        return true;
    }
    void publish(const uint16_t* px, uint32_t w, uint32_t h, uint32_t bpp,
                 uint64_t exp_ns, uint64_t sof_ns, double gain, double ctime) {
        if (!base) return;
        uint64_t s = ++seq;
        uint32_t slot = (uint32_t)(s % nslots);
        uint8_t* sp = base + HDR_SZ + (size_t)slot*slot_stride;
        ShmSlotMeta* m = (ShmSlotMeta*)sp;
        m->w=w; m->h=h; m->bpp=bpp; m->_pad=0; m->exp_ns=exp_ns; m->sof_ns=sof_ns;
        m->gain=gain; m->capture_time=ctime;
        memcpy(sp + data_off, px, (size_t)w*h*2);
        std::atomic_thread_fence(std::memory_order_release);
        m->seq = s;                    /* slot's own seq, for reader tear-check */
        hdr->latest_slot = slot;
        hdr->latest_seq  = s;          /* publish */
    }
} g_shm;

// ── CUDA EGL frame → uint16 ───────────────────────────────────────────────────

/* Reused staging buffer for the EGL-frame -> host copy. The main loop is
 * single-threaded, so file-scope statics are safe. Reusing avoids a per-frame
 * cuMemAllocHost; PINNED host memory lets the copy engine DMA directly (a
 * pageable destination forces the driver to stage through an internal pinned
 * buffer first -- an extra full-frame copy we were paying every frame). */
static uint16_t* g_hpin    = nullptr;
static size_t    g_hpin_n  = 0;
static double    g_dec_ms  = 0.0;       /* last decode time, for the throughput log */
static double    g_serve_ms = 0.0;      /* last clients.poll (pack + TCP) time */

static bool cuda_frame_to_u16(CUgraphicsResource res,
                               uint32_t w, uint32_t h, uint32_t bpp,
                               std::vector<uint16_t>& out)
{
    struct timespec _t0; clock_gettime(CLOCK_MONOTONIC,&_t0);
    CUeglFrame f; memset(&f,0,sizeof(f));
    CU_CHECK(cuGraphicsResourceGetMappedEglFrame(&f,res,0,0));

    uint32_t fw=f.width?f.width:w, fh=f.height?f.height:h, pitch=f.pitch;
    bool  isPitch = (f.frameType==CU_EGL_FRAME_TYPE_PITCH);
    CUarray arr = nullptr;
    if (isPitch) {
        if (!pitch) pitch=fw*2;
    } else {
        arr=f.frame.pArray[0];
        CUDA_ARRAY_DESCRIPTOR d{}; cuArrayGetDescriptor(&d,arr);
        fw=(uint32_t)d.Width; fh=(uint32_t)d.Height;
    }
    size_t n=(size_t)fw*fh;

    if (g_hpin==nullptr || g_hpin_n<n) {           /* (re)alloc the pinned buffer once */
        if (g_hpin) cuMemFreeHost(g_hpin);
        if (cuMemAllocHost((void**)&g_hpin, n*sizeof(uint16_t))!=CUDA_SUCCESS){ g_hpin=nullptr; return false; }
        g_hpin_n=n;
    }

    /* ONE copy straight to pinned host. A block-linear CUDA array is de-tiled by
     * the driver on copy-out, so the old array->device->host round trip (with a
     * per-frame cuMemAllocPitch) was unnecessary. */
    CUDA_MEMCPY2D cp{};
    cp.dstMemoryType=CU_MEMORYTYPE_HOST; cp.dstHost=g_hpin; cp.dstPitch=fw*2;
    cp.WidthInBytes=fw*2; cp.Height=fh;
    if (isPitch) { cp.srcMemoryType=CU_MEMORYTYPE_DEVICE; cp.srcDevice=(CUdeviceptr)f.frame.pPitch[0]; cp.srcPitch=pitch; }
    else         { cp.srcMemoryType=CU_MEMORYTYPE_ARRAY;  cp.srcArray=arr; }
    if (cuMemcpy2D(&cp)!=CUDA_SUCCESS) return false;

    /* Argus RAW16 is MSB-aligned: an N-bit sensor value is stored as
     * sensor<<(16-N), so sensor = stored>>(16-N). 10-bit -> >>6, 12-bit -> >>4.
     * Vectorized under -O3/NEON (~2ms). */
    uint32_t sh = (bpp < 16) ? (16 - bpp) : 0;
    if (sh) for (size_t i=0;i<n;i++) g_hpin[i]>>=sh;

    if (fw==w && fh==h) {
        out.assign(g_hpin, g_hpin+n);
    } else {
        out.assign((size_t)w*h, 0);
        for (uint32_t y=0;y<std::min(fh,h);y++)
            memcpy(&out[(size_t)y*w], &g_hpin[(size_t)y*fw], std::min(fw,w)*sizeof(uint16_t));
    }
    struct timespec _t1; clock_gettime(CLOCK_MONOTONIC,&_t1);
    g_dec_ms = (_t1.tv_sec-_t0.tv_sec)*1e3 + (_t1.tv_nsec-_t0.tv_nsec)*1e-6;
    return true;
}

// ── Session ───────────────────────────────────────────────────────────────────

struct Session {
    UniqueObj<CameraProvider>  prov;
    ICameraProvider*           iProv      = nullptr;
    UniqueObj<CaptureSession>  sess;
    ICaptureSession*           iSess      = nullptr;
    UniqueObj<OutputStream>    os;
    IEGLOutputStream*          iEGLOS     = nullptr;
    EGLStreamKHR               eglStream  = EGL_NO_STREAM_KHR;
    CUeglStreamConnection      cuConn;
    bool                       cuConnected = false;
    UniqueObj<Request>         req;
    IRequest*                  iReq       = nullptr;
    ISourceSettings*           iSrc       = nullptr;
    uint32_t                   W=0,H=0,BPP=0;
    /* DOL/WDR interleaved-output layout (physical frame carries N exposures +
     * line-info markers + vertical-blank rows; used to de-interleave later). */
    bool                       is_dol = false;
    uint32_t                   baseW=0, baseH=0;
    uint32_t                   dol_expcount=1, dol_limarker=0, dol_vbp=0;

    /* Currently-applied exp/gain on the live request (for live updates). */
    uint64_t                   cur_exp_ns = 0;
    float                      cur_gain   = 0.0f;

    /* CAPTURE_COMPLETE event queue → ACTUAL sensor exp/gain the AE/AGC chose
     * (read from CaptureMetadata), so exposure_ns=0 / gain=0 report back what
     * the sensor actually settled on rather than the requested 0. */
    IEventProvider*            iEvents       = nullptr;
    UniqueObj<EventQueue>      eventQueue;
    IEventQueue*               iEventQueue   = nullptr;
    uint64_t                   actual_exp_ns = 0;
    float                      actual_gain   = 0.0f;
    uint64_t                   actual_sof_ns = 0;   /* sensor start-of-frame timestamp */
    uint64_t                   frames_recv   = 0;   /* CAPTURE_COMPLETE count = frames received over the link */

    /* Drain capture-complete events; keep the most recent actual exp/gain.
     * Non-blocking; called from the server loop (single Argus thread). */
    void poll_metadata()
    {
        if (!iEvents || !iEventQueue) return;
        iEvents->waitForEvents(eventQueue.get(), 0);   /* timeout 0: drain only */
        const Event* ev;
        while ((ev = iEventQueue->getNextEvent()) != nullptr) {
            const IEvent* iEv = interface_cast<const IEvent>(ev);
            if (!iEv || iEv->getEventType() != EVENT_TYPE_CAPTURE_COMPLETE) continue;
            const IEventCaptureComplete* iCC =
                interface_cast<const IEventCaptureComplete>(ev);
            if (!iCC) continue;
            frames_recv++;   /* one sensor frame received over the CSI/GMSL link */
            const CaptureMetadata* m = iCC->getMetadata();
            const ICaptureMetadata* iM = interface_cast<const ICaptureMetadata>(m);
            if (iM) {
                actual_exp_ns = iM->getSensorExposureTime();
                actual_gain   = iM->getSensorAnalogGain();
                actual_sof_ns = iM->getSensorTimestamp();   /* kernel SOF ts, for timing tests */
            }
        }
    }

    /* Live-update exposure/gain on the running pipeline without a session
     * rebuild. On R36/JetPack6, mutating ISourceSettings and re-submitting
     * the request applies the new values to subsequent frames — no teardown
     * of the EGL stream or CUDA consumer needed. Returns true if a change
     * was applied. Called from the single Argus thread (server loop) only. */
    bool set_exposure_gain_live(uint64_t exp_ns, float gain)
    {
        if (!iSrc) return false;
        if (exp_ns == cur_exp_ns && gain == cur_gain) return false;
        apply_exposure(exp_ns, gain);
        cur_exp_ns = exp_ns;
        cur_gain   = gain;
        /* Re-submit so the mutated request takes effect. Under repeat() we
         * re-arm the repeating request; otherwise one-shot submit (warmup). */
        if (repeating) iSess->repeat(req.get());
        else           iSess->capture(req.get());
        return true;
    }

    /* Continuous streaming. repeat() makes Argus re-issue the capture request at
     * the sensor's frame-duration rate, INDEPENDENT of how fast the CUDA consumer
     * acquires. Without it, one-shot capture()-per-acquire pins the sensor (and
     * thus the CSI/GMSL link) to the consumer's ~30fps decode rate — the frame
     * duration only sets the MAX rate, not a free-run. With repeat()+MAILBOX the
     * sensor free-runs at the configured fps (link fully driven) and the consumer
     * just reads the latest frame. Canonical Argus video path (what econ's
     * eCAM_argus_camera uses to sustain 60/72fps). */
    bool repeating = false;
    void start_repeat() { if (iSess) { iSess->repeat(req.get()); repeating = true; } }
    void stop_repeat()  { if (iSess && repeating) { iSess->stopRepeat(); iSess->waitForIdle(); repeating = false; } }

    bool init(const Config& cfg)
    {
        prov.reset(CameraProvider::create());
        iProv=interface_cast<ICameraProvider>(prov);
        if (!iProv){fprintf(stderr,"[Session] no provider\n");return false;}
        printf("[Session] Argus %s\n",iProv->getVersion().c_str());
        fflush(stdout);

        std::vector<CameraDevice*> devs;
        iProv->getCameraDevices(&devs);
        if (devs.empty()){fprintf(stderr,"[Session] no devices\n");return false;}

        ICameraProperties *iP=interface_cast<ICameraProperties>(devs[0]);
        std::vector<SensorMode*> modes;
        iP->getAllSensorModes(&modes);
        if (cfg.mode>=(int)modes.size()){
            fprintf(stderr,"[Session] mode OOB\n");return false;}

        SensorMode  *sm   =modes[cfg.mode];
        ISensorMode *iMode=interface_cast<ISensorMode>(sm);
        baseW = iMode->getResolution().width();
        baseH = iMode->getResolution().height();
        W  = baseW;
        H  = baseH;
        BPP=iMode->getInputBitDepth();

        /* DOL/WDR modes output an INTERLEAVED multi-exposure frame at a larger
         * "physical" resolution (line-info markers + VBP rows + N exposures),
         * so we size the EGL stream to getPhysicalResolution() below.
         *
         * STATUS (2026-07-16): native DOL RAW capture does NOT work on this
         * e-CAM86 / JetPack 6.1 BSP -- it is a tegra VI / e-con driver issue,
         * NOT our code, NOT the MCU firmware, NOT the device tree:
         *   - the sensor DOES stream DOL (30fps confirmed on the direct V4L2
         *     path), so the MCU/sensor are fine;
         *   - the device-tree mode-3 entry is correct (bayer_wdr_dol,
         *     3856x4450, 2 exposures, 16x) and matches IDolWdrSensorMode;
         *   - but this Argus RAW16 consumer, even sized to the physical
         *     3856x4450, receives ZERO frames (producer never delivers); and
         *   - direct V4L2 delivers frames flagged V4L2_BUF_FLAG_ERROR because
         *     the node clamps to 3840x2160 (a 2160-row slice of the 4450-row
         *     readout) -> frame-size mismatch.
         * i.e. the tegra VI DOL-capture path never delivers the full frame.
         * Filed with e-con; use software-bracket HDR (modes 0/1/2) meanwhile.
         * (An earlier commit wrongly called this a "vendor MCU-firmware bug";
         *  the MCU streams DOL fine -- the failure is in VI/driver capture.) */
        Ext::IDolWdrSensorMode* dol =
            interface_cast<Ext::IDolWdrSensorMode>(sm);
        if (dol) {
            Size2D<uint32_t> pr = dol->getPhysicalResolution();
            W = pr.width(); H = pr.height();
            is_dol       = true;
            dol_expcount = dol->getExposureCount();
            dol_limarker = dol->getLineInfoMarkerWidth();
            std::vector<uint32_t> vbp;
            dol->getVerticalBlankPeriodRowCount(&vbp);
            dol_vbp = vbp.empty() ? 0 : vbp[0];
            printf("[Session] DOL base=%ux%u physical=%ux%u exp=%u LImarker=%u VBP=%u\n",
                   baseW, baseH, W, H, dol_expcount, dol_limarker, dol_vbp);
        }
        printf("[Session] mode=%d  %ux%u  bpp=%u\n",cfg.mode,W,H,BPP);
        fflush(stdout);

        exp_range  = iMode->getExposureTimeRange();
        gain_range = iMode->getAnalogGainRange();

        sess.reset(iProv->createCaptureSession(devs[0]));
        iSess=interface_cast<ICaptureSession>(sess);
        if (!iSess){fprintf(stderr,"[Session] no session\n");return false;}
        printf("[Session] capture session created\n"); fflush(stdout);

        /* CAPTURE_COMPLETE event queue for actual exp/gain readback (metadata).
         * Non-fatal if unavailable — actuals then fall back to requested. */
        iEvents = interface_cast<IEventProvider>(sess);
        if (iEvents) {
            std::vector<EventType> types;
            types.push_back(EVENT_TYPE_CAPTURE_COMPLETE);
            eventQueue.reset(iEvents->createEventQueue(types));
            iEventQueue = interface_cast<IEventQueue>(eventQueue);
        }
        printf("[Session] metadata queue %s\n",
               iEventQueue ? "ready" : "UNAVAILABLE (actuals=requested)");
        fflush(stdout);

        /* EGL stream — MAILBOX mode: producer always has latest frame */
        UniqueObj<OutputStreamSettings> oss(
            iSess->createOutputStreamSettings(STREAM_TYPE_EGL));
        IEGLOutputStreamSettings *iSet=
            interface_cast<IEGLOutputStreamSettings>(oss);
        iSet->setEGLDisplay(g_display);
        iSet->setPixelFormat(PIXEL_FMT_RAW16);
        iSet->setResolution(Size2D<uint32_t>(W,H));
        iSet->setMode(EGL_STREAM_MODE_MAILBOX);  /* latest frame always available */
        printf("[Session] EGL stream: RAW16 %ux%u MAILBOX\n",W,H);
        fflush(stdout);

        os.reset(iSess->createOutputStream(oss.get()));
        iEGLOS=interface_cast<IEGLOutputStream>(os);
        if (!iEGLOS){fprintf(stderr,"[Session] no EGLOutputStream\n");return false;}
        eglStream=iEGLOS->getEGLStream();

        /* CUDA consumer */
        CUresult cr=cuEGLStreamConsumerConnect(&cuConn,eglStream);
        if (cr!=CUDA_SUCCESS){
            const char*s="?"; cuGetErrorString(cr,&s);
            fprintf(stderr,"[CUDA] ConsumerConnect: %s\n",s);return false;}
        cuConnected=true;
        printf("[CUDA] consumer connected\n"); fflush(stdout);

        /* Request */
        req.reset(iSess->createRequest(CAPTURE_INTENT_VIDEO_RECORD));
        iReq=interface_cast<IRequest>(req);
        iSrc=interface_cast<ISourceSettings>(iReq->getSourceSettings());
        if (iSrc) {
            iSrc->setSensorMode(sm);
            base_frame_dur_ns = 1000000000ULL/(uint64_t)cfg.fps;
            apply_exposure(cfg.exp_ns, cfg.gain);   // sets frame duration too
        }
        /* Track what the running request currently reflects, so the server
         * loop can detect a pending change and re-submit only when needed. */
        cur_exp_ns = cfg.exp_ns;
        cur_gain   = cfg.gain;
        iReq->enableOutputStream(os.get());
        return true;
    }

    /* Cached mode limits so live updates can clamp without re-querying. */
    Range<uint64_t> exp_range{0,0};
    Range<float>    gain_range{0.0f,0.0f};
    /* Frame period from the requested fps. Exposure can't exceed the frame
     * duration, so apply_exposure() extends it for long exposures (fps drops). */
    uint64_t        base_frame_dur_ns = 33333333;   /* 30 fps default */

    /* Apply exposure/gain to iSrc following the 2x2 auto/manual rule:
     *
     *   exp_ns==0, gain==0  -> full auto  (AE on, AGC on)   leave both unset
     *   exp_ns==0, gain>0   -> auto exposure, gain pinned    set gain only
     *   exp_ns>0,  gain==0  -> fixed exposure, auto gain     set exposure only
     *   exp_ns>0,  gain>0   -> both fixed                    set both
     *
     * Not calling setExposureTimeRange / setGainRange leaves that axis under
     * Argus AC (auto) control. Passing a degenerate [v,v] range pins it.
     * This is called both at session init and on every live re-submit, so the
     * running pipeline always reflects the current persisted exp/gain.
     */
    void apply_exposure(uint64_t exp_ns, float gain)
    {
        if (!iSrc) return;

        IAutoControlSettings* iAC = interface_cast<IAutoControlSettings>(
            iReq->getAutoControlSettings());

        /* Frame duration MUST be set BEFORE exposure: Argus clamps the exposure
         * to the CURRENT frame duration at set-time, so a long exposure set
         * while the period is still ~1/fps gets clamped. Extend the period
         * first (fps drops), then set the exposure. */
        uint64_t fd = base_frame_dur_ns;
        uint64_t e  = 0;
        if (exp_ns > 0) {
            e = std::max((uint64_t)exp_range.min(),
                std::min((uint64_t)exp_range.max(), exp_ns));
            if (e > fd) fd = e;
        }
        iSrc->setFrameDurationRange(Range<uint64_t>(fd, fd));
        if (exp_ns > 0) {
            iSrc->setExposureTimeRange(Range<uint64_t>(e,e));
        } else {
            /* auto exposure within the fps-derived frame period */
            iSrc->setExposureTimeRange(exp_range);
        }

        if (gain > 0.0f) {
            float g = std::max(gain_range.min(),
                      std::min(gain_range.max(), gain));
            iSrc->setGainRange(Range<float>(g,g));
            /* pinning gain while exposure is auto: keep AE from also
             * driving ISP digital gain, so the pin actually holds */
            if (iAC) iAC->setIspDigitalGainRange(Range<float>(1.0f,1.0f));
        } else {
            iSrc->setGainRange(gain_range);
            if (iAC) iAC->setIspDigitalGainRange(gain_range);
        }
    }

    /* Fill the pipeline with N capture requests */
    void fill_pipeline(int n)
    {
        for (int i=0;i<n&&g_running;i++)
            iSess->capture(req.get());
    }

    /* Wait for state=0x3218, submitting captures every 50ms */
    bool wait_for_frame(int timeout_ms=30000)
    {
        int n_iter = timeout_ms / 10;
        int submitted = 0;
        for (int i=0;i<n_iter&&g_running;i++) {
            if (!repeating && i%5==0) {  /* every 50ms (skip if repeat() is streaming) */
                iSess->capture(req.get());
                submitted++;
            }
            EGLint state=0;
            if (fp_eglQueryStreamKHR) {
                fp_eglQueryStreamKHR(g_display,eglStream,
                                     EGL_STREAM_STATE_KHR,&state);
                if (state==0x3218||state==0x3219) {
                    printf("[Session] frame ready at t=%.1fs submitted=%d\n",
                           i*0.01f,submitted);
                    fflush(stdout);
                    return true;
                }
                if (state==0x321A) {
                    fprintf(stderr,"[Session] stream disconnected\n");
                    return false;
                }
                if (i%100==0)
                    printf("[Session]   t=%.0fs state=0x%x\n",
                           i*0.01f,(unsigned)state);
            } else {
                if (i>=200) return true;
            }
            usleep(10000);
        }
        fprintf(stderr,"[Session] wait_for_frame timed out\n");
        return false;
    }

    void shutdown()
    {
        if (cuConnected) {
            cuEGLStreamConsumerDisconnect(&cuConn);
            cuConnected=false;
        }
        req.reset(); iReq=nullptr; iSrc=nullptr;
        os.reset();  iEGLOS=nullptr;
        sess.reset(); iSess=nullptr;
        prov.reset(); iProv=nullptr;
    }
};

// ── TCP helpers ───────────────────────────────────────────────────────────────

static bool send_all(int fd,const void*p,size_t n)
{
    const uint8_t*b=(uint8_t*)p;
    while (n>0){
        ssize_t s=::send(fd,b,n,MSG_NOSIGNAL);
        if (s<=0) return false;
        b+=s; n-=s;
    }
    return true;
}

/* Read exactly n bytes from fd. Returns false on close/error. */
static bool recv_all(int fd, void* p, size_t n)
{
    uint8_t* b=(uint8_t*)p;
    while (n>0){
        ssize_t r=::recv(fd,b,n,0);
        if (r<=0) return false;
        b+=(size_t)r; n-=(size_t)r;
    }
    return true;
}

/* ── Localhost request protocol (persistent connection) ───────────────────────
 *
 * The Python image_server opens ONE socket to this server and reuses it for
 * every frame, eliminating per-frame connect/accept/teardown (the dominant
 * cost in the old design). Each request is a fixed 20-byte command:
 *
 *   cmd        uint32   REQ_FRAME=1  SET_EXPGAIN=2  PING=3
 *   want_exp   uint64   (SET_EXPGAIN) new exposure_ns, 0=auto
 *   want_gain  float    (SET_EXPGAIN) new gain, 0=auto
 *   pad        uint32
 *
 * REQ_FRAME  -> server replies NetHdr(36B) + pixel payload (w*h*2 bytes)
 * SET_EXPGAIN-> server replies 4-byte ack (0=applied, 1=nochange); the actual
 *               live re-apply is performed on the Argus thread (see loop),
 *               not here, so this only stashes the request.
 * PING       -> server replies NetHdr with magic only (nf=0), no payload
 */
#pragma pack(push,1)
struct ReqHdr { uint32_t cmd; uint64_t want_exp; float want_gain; uint32_t pad; };
#pragma pack(pop)
enum { REQ_FRAME=1, REQ_SET_EXPGAIN=2, REQ_PING=3, REQ_FRAME_PACKED=4 };

/* RAW10 (4 px -> 5 bytes) and RAW12 (2 px -> 3 bytes) packers. Byte layout matches
 * the Python/MATLAB unpackers exactly (verified bit-exact against the scalar refs):
 *   RAW10: o0..o3 = p0..p3>>2;  o4 = (p0&3)|(p1&3)<<2|(p2&3)<<4|(p3&3)<<6
 *   RAW12: o0 = p0>>4; o1 = p1>>4; o2 = (p0&0xF)|((p1&0xF)<<4)
 * NEON-vectorized (RAW10 via vld4q + vqtbl3q shuffle; RAW12 via vld2q + vst3q); the
 * scalar versions below stay as the reference and the (rare) tail handler. */
static void pack_raw10_scalar(const uint16_t* px, size_t n, uint8_t* o)
{
    size_t groups = (n + 3) / 4, i = 0;
    for (size_t g = 0; g < groups; ++g, o += 5, i += 4) {
        uint16_t p0 = i<n?px[i]:0, p1 = i+1<n?px[i+1]:0, p2 = i+2<n?px[i+2]:0, p3 = i+3<n?px[i+3]:0;
        o[0]=(uint8_t)(p0>>2); o[1]=(uint8_t)(p1>>2); o[2]=(uint8_t)(p2>>2); o[3]=(uint8_t)(p3>>2);
        o[4]=(uint8_t)((p0&3)|((p1&3)<<2)|((p2&3)<<4)|((p3&3)<<6));
    }
}
static void pack_raw12_scalar(const uint16_t* px, size_t n, uint8_t* o)
{
    size_t groups = (n + 1) / 2, i = 0;
    for (size_t g = 0; g < groups; ++g, o += 3, i += 2) {
        uint16_t p0 = i<n?px[i]:0, p1 = i+1<n?px[i+1]:0;
        o[0]=(uint8_t)(p0>>4); o[1]=(uint8_t)(p1>>4); o[2]=(uint8_t)((p0&0xF)|((p1&0xF)<<4));
    }
}

/* 5-way interleave shuffle tables: 8 groups (32 px) -> 40 bytes. Source layout in
 * the vqtbl3q table: [0:8]=H0(p0>>2), [8:16]=H1, [16:24]=H2, [24:32]=H3, [32:40]=L. */
static const uint8_t RAW10_IDX0[16]={0,8,16,24,32,1,9,17,25,33,2,10,18,26,34,3};
static const uint8_t RAW10_IDX1[16]={11,19,27,35,4,12,20,28,36,5,13,21,29,37,6,14};
static const uint8_t RAW10_IDX2[16]={22,30,38,7,15,23,31,39,0,0,0,0,0,0,0,0};

static void pack_raw10(const uint16_t* px, size_t n, std::vector<uint8_t>& out)
{
    size_t groups = (n + 3) / 4;
    out.resize(groups * 5);
    uint8_t* o = out.data();
    size_t vg = (groups / 8) * 8, g = 0, i = 0;
    uint8x16_t i0 = vld1q_u8(RAW10_IDX0), i1 = vld1q_u8(RAW10_IDX1), i2 = vld1q_u8(RAW10_IDX2);
    uint16x8_t three = vdupq_n_u16(3);
    for (; g < vg; g += 8, i += 32, o += 40) {
        uint16x8x4_t p = vld4q_u16(px + i);          /* val[k] = k-th pixel of each group */
        uint8x8_t H0 = vshrn_n_u16(p.val[0], 2), H1 = vshrn_n_u16(p.val[1], 2);
        uint8x8_t H2 = vshrn_n_u16(p.val[2], 2), H3 = vshrn_n_u16(p.val[3], 2);
        uint16x8_t L16 = vorrq_u16(
            vorrq_u16(vandq_u16(p.val[0], three), vshlq_n_u16(vandq_u16(p.val[1], three), 2)),
            vorrq_u16(vshlq_n_u16(vandq_u16(p.val[2], three), 4), vshlq_n_u16(vandq_u16(p.val[3], three), 6)));
        uint8x16x3_t tbl = { vcombine_u8(H0, H1), vcombine_u8(H2, H3), vcombine_u8(vmovn_u16(L16), vdup_n_u8(0)) };
        vst1q_u8(o,      vqtbl3q_u8(tbl, i0));
        vst1q_u8(o + 16, vqtbl3q_u8(tbl, i1));
        vst1_u8 (o + 32, vget_low_u8(vqtbl3q_u8(tbl, i2)));
    }
    if (g < groups) pack_raw10_scalar(px + i, n - i, o);
}

static void pack_raw12(const uint16_t* px, size_t n, std::vector<uint8_t>& out)
{
    size_t groups = (n + 1) / 2;
    out.resize(groups * 3);
    uint8_t* o = out.data();
    size_t vg = (groups / 8) * 8, g = 0, i = 0;
    uint16x8_t nib = vdupq_n_u16(0xF);
    for (; g < vg; g += 8, i += 16, o += 24) {
        uint16x8x2_t a = vld2q_u16(px + i);          /* val[0]=even px, val[1]=odd px */
        uint8x8_t A = vshrn_n_u16(a.val[0], 4), B = vshrn_n_u16(a.val[1], 4);
        uint16x8_t C16 = vorrq_u16(vandq_u16(a.val[0], nib), vshlq_n_u16(vandq_u16(a.val[1], nib), 4));
        uint8x8x3_t o3 = { A, B, vmovn_u16(C16) };
        vst3_u8(o, o3);
    }
    if (g < groups) pack_raw12_scalar(px + i, n - i, o);
}

/* Pending exp/gain change requested by a client (on the serve thread), consumed
 * by the Argus/capture loop. Guarded by a mutex: the serve thread and capture
 * thread are now separate, so a plain volatile flag+payload is NOT safe on ARM's
 * weak memory model (the flag could be seen before the payload). */
struct PendingCtl {
    bool     have = false;
    uint64_t exp_ns = 0;
    float    gain = 0.0f;
} g_pending;
static std::mutex g_pending_mtx;

static void serve_frame(int cli, const FrameBuffer& buf)
{
    if (!buf.valid) {
        NetHdr err{}; err.magic=0xDEADBEEF;
        send_all(cli,&err,sizeof(err));
        return;
    }
    NetHdr nh{};
    nh.magic      = 0x52413130;
    nh.w          = buf.w;
    nh.h          = buf.h;
    nh.bpp        = buf.bpp;
    nh.nf         = 1;
    nh.exp_ns     = buf.exp_ns;
    nh.gain_x1000 = (uint32_t)(buf.gain*1000);
    nh.pad        = 0;
    send_all(cli,&nh,sizeof(nh));
    send_all(cli,buf.pixels.data(),buf.pixels.size()*2);
}

/* Serve the latest frame bit-packed instead of uint16: RAW10 (4px->5B) for a
 * 10-bit sensor mode, RAW12 (2px->3B) for 12-bit. Packing is done here in C++
 * (~few ms) rather than in the Python image_server (~0.10s). The packed length
 * is self-describing via NetHdr.pad (and the format via NetHdr.bpp) so the
 * client reads exactly that many bytes. Runs on the single Argus loop thread. */
static void serve_frame_packed(int cli, const FrameBuffer& buf)
{
    if (!buf.valid) {
        NetHdr err{}; err.magic=0xDEADBEEF;
        send_all(cli,&err,sizeof(err));
        return;
    }
    struct timespec _p0; clock_gettime(CLOCK_MONOTONIC,&_p0);
    static std::vector<uint8_t> packed;   // reused; single serve thread
    if (buf.bpp == 12) pack_raw12(buf.pixels.data(), buf.pixels.size(), packed);
    else               pack_raw10(buf.pixels.data(), buf.pixels.size(), packed);
    NetHdr nh{};
    nh.magic      = 0x52413130;
    nh.w          = buf.w;
    nh.h          = buf.h;
    nh.bpp        = buf.bpp;
    nh.nf         = 1;
    nh.exp_ns     = buf.exp_ns;
    nh.gain_x1000 = (uint32_t)(buf.gain*1000);
    nh.pad        = (uint32_t)packed.size();   // packed payload length in bytes
    send_all(cli,&nh,sizeof(nh));
    send_all(cli,packed.data(),packed.size());
    struct timespec _p1; clock_gettime(CLOCK_MONOTONIC,&_p1);
    g_serve_ms = (_p1.tv_sec-_p0.tv_sec)*1e3 + (_p1.tv_nsec-_p0.tv_nsec)*1e-6;  /* pack+send (serve thread) */
}

/* Handle one request on an already-open client socket.
 * Returns false if the connection should be closed. */
static bool handle_request(int cli)
{
    ReqHdr rq{};
    if (!recv_all(cli,&rq,sizeof(rq))) return false;  /* client closed */

    switch (rq.cmd) {
    case REQ_FRAME: {
        auto s = pub_get(); FrameBuffer empty;
        serve_frame(cli, s ? *s : empty);             /* s keeps the frame alive during send */
        return true;
    }
    case REQ_FRAME_PACKED: {
        auto s = pub_get(); FrameBuffer empty;
        serve_frame_packed(cli, s ? *s : empty);
        return true;
    }
    case REQ_SET_EXPGAIN: {
        {   std::lock_guard<std::mutex> lk(g_pending_mtx);
            g_pending.exp_ns = rq.want_exp;
            g_pending.gain   = rq.want_gain;
            g_pending.have   = true;                  /* applied on the Argus/capture thread */
        }
        uint32_t ack = 0;
        return send_all(cli,&ack,sizeof(ack));
    }
    case REQ_PING: {
        /* Lightweight actual-value probe: NetHdr only (nf=0), exp/gain from the
         * latest frame, so the Python AE monitor can poll without a full transfer. */
        auto s = pub_get();
        NetHdr nh{};
        nh.magic = 0x52413130; nh.nf = 0; nh.pad = 0;
        if (s) { nh.w=s->w; nh.h=s->h; nh.bpp=s->bpp; nh.exp_ns=s->exp_ns; nh.gain_x1000=(uint32_t)(s->gain*1000); }
        return send_all(cli,&nh,sizeof(nh));
    }
    default:
        return false;  /* unknown command: drop connection */
    }
}

/* Client set management. Runs on the SERVE thread (not the capture loop), so
 * pack+TCP never blocks capture. poll() blocks up to timeout_ms on srv + all
 * client fds, then accepts new connections and services readable ones. Each
 * request pulls the current published frame via pub_get() (zero-copy). */
struct ClientSet {
    std::vector<int> fds;

    void poll(int srv, int timeout_ms)
    {
        fd_set rfds; FD_ZERO(&rfds); FD_SET(srv,&rfds); int maxfd=srv;
        for (int fd : fds) { FD_SET(fd,&rfds); if (fd>maxfd) maxfd=fd; }
        struct timeval tv={ timeout_ms/1000, (timeout_ms%1000)*1000 };
        if (select(maxfd+1,&rfds,nullptr,nullptr,&tv) <= 0) return;   /* idle -> return (thread rechecks g_running) */

        if (FD_ISSET(srv,&rfds)) {                                    /* accept one pending connection */
            int cli=accept(srv,nullptr,nullptr);
            if (cli>=0) {
                int one=1; setsockopt(cli,IPPROTO_TCP,TCP_NODELAY,&one,sizeof(one));
                fds.push_back(cli);
            }
        }
        std::vector<int> keep; keep.reserve(fds.size());
        for (int fd : fds) {
            if (FD_ISSET(fd,&rfds)) {
                if (handle_request(fd)) keep.push_back(fd);
                else close(fd);
            } else {
                keep.push_back(fd);
            }
        }
        fds.swap(keep);
    }

    void close_all() { for (int fd : fds) close(fd); fds.clear(); }
};

/* Serve thread: services TCP clients from the published frame, independent of
 * the capture loop. Exits when g_running clears (select timeout bounds latency). */
static void serve_thread_fn(int srv)
{
    ClientSet clients;
    while (g_running) clients.poll(srv, 5);
    clients.close_all();
}

// ── File save (single-shot mode) ──────────────────────────────────────────────

static void save_file(const std::string& p,
                       const FrameBuffer& buf)
{
    std::ofstream f(p,std::ios::binary|std::ios::trunc);
    FileHdr hdr{0x52413130,buf.w,buf.h,buf.bpp,1,0};
    f.write((char*)&hdr,sizeof(hdr));
    f.write((char*)buf.pixels.data(),buf.pixels.size()*2);
    printf("[RAW] saved %s  (%.2fMB)  seq=%d\n",
           p.c_str(),
           (sizeof(hdr)+buf.pixels.size()*2)/1048576.0,
           buf.seq);
}

// ── Sensor mode enumeration (--list-modes) ─────────────────────────────────────
//
// Dumps every Argus sensor mode with resolution, in/out bit depth, mode type
// (BAYER / BAYER_PWL / BAYER_DOL — the PWL/DOL types are the sensor's native
// HDR modes), exposure/gain/frame-duration/HDR-ratio ranges, and DOL exposure
// count. This tells us whether the IMX678 exposes native HDR (single-shot WDR)
// or whether HDR must be done by software exposure bracketing.

static const char* mode_type_name(SensorModeType t)
{
    if (t == SENSOR_MODE_TYPE_BAYER)     return "BAYER";
    if (t == SENSOR_MODE_TYPE_BAYER_PWL) return "BAYER_PWL(HDR)";
    if (t == SENSOR_MODE_TYPE_BAYER_DOL) return "BAYER_DOL(HDR)";
    if (t == SENSOR_MODE_TYPE_YUV)       return "YUV";
    if (t == SENSOR_MODE_TYPE_RGB)       return "RGB";
    if (t == SENSOR_MODE_TYPE_DEPTH)     return "DEPTH";
    return "OTHER";
}

static bool list_modes()
{
    UniqueObj<CameraProvider> prov(CameraProvider::create());
    ICameraProvider* iProv = interface_cast<ICameraProvider>(prov);
    if (!iProv) { fprintf(stderr,"[Modes] no provider\n"); return false; }
    printf("[Modes] Argus %s\n", iProv->getVersion().c_str());

    std::vector<CameraDevice*> devs;
    iProv->getCameraDevices(&devs);
    if (devs.empty()) { fprintf(stderr,"[Modes] no devices\n"); return false; }
    printf("[Modes] %zu camera device(s)\n", devs.size());

    ICameraProperties* iProps = interface_cast<ICameraProperties>(devs[0]);
    if (!iProps) { fprintf(stderr,"[Modes] no properties\n"); return false; }
    std::vector<SensorMode*> modes;
    iProps->getAllSensorModes(&modes);
    printf("[Modes] device 0: %zu sensor mode(s)\n\n", modes.size());

    for (size_t i=0;i<modes.size();i++) {
        ISensorMode* m = interface_cast<ISensorMode>(modes[i]);
        if (!m) continue;
        Size2D<uint32_t> r   = m->getResolution();
        Range<uint64_t>  er  = m->getExposureTimeRange();
        Range<uint64_t>  fdr = m->getFrameDurationRange();
        Range<float>     gr  = m->getAnalogGainRange();
        Range<float>     hr  = m->getHdrRatioRange();
        double maxfps = fdr.min()>0 ? 1e9/(double)fdr.min() : 0.0;
        int expcount = 1;
        Ext::IDolWdrSensorMode* dol =
            interface_cast<Ext::IDolWdrSensorMode>(modes[i]);
        if (dol) expcount = (int)dol->getExposureCount();

        double minfps = fdr.max()>0 ? 1e9/(double)fdr.max() : 0.0;
        printf("  [%zu] %ux%u  type=%-14s  inBits=%u outBits=%u  fps=[%.2f..%.1f]\n",
               i, r.width(), r.height(), mode_type_name(m->getSensorModeType()),
               m->getInputBitDepth(), m->getOutputBitDepth(), minfps, maxfps);
        printf("       exp=[%llu..%llu]ns  frameDur=[%llu..%llu]ns (max exp %.1fms)  gain=[%.2f..%.2fx]  hdrRatio=[%.2f..%.2f]  exposures=%d\n",
               (unsigned long long)er.min(), (unsigned long long)er.max(),
               (unsigned long long)fdr.min(), (unsigned long long)fdr.max(),
               (double)std::min((uint64_t)er.max(), fdr.max())/1e6,
               gr.min(), gr.max(), hr.min(), hr.max(), expcount);
        if (dol) {
            Size2D<uint32_t> pr = dol->getPhysicalResolution();
            std::vector<uint32_t> vbp;
            dol->getVerticalBlankPeriodRowCount(&vbp);
            printf("       DOL: physRes=%ux%u  OBrows=%u  LImarker=%upx  margins L=%u R=%u  VBP=[",
                   pr.width(), pr.height(), dol->getOpticalBlackRowCount(),
                   dol->getLineInfoMarkerWidth(),
                   dol->getLeftMarginWidth(), dol->getRightMarginWidth());
            for (size_t j=0;j<vbp.size();j++) printf("%s%u", j?",":"", vbp[j]);
            printf("]\n");
        }
    }
    printf("\n[Modes] done\n");
    return true;
}

// ── Main ──────────────────────────────────────────────────────────────────────

int main(int argc,char*argv[])
{
    signal(SIGINT,sig_handler); signal(SIGTERM,sig_handler);

    if (!egl_init()) {fprintf(stderr,"[EGL] failed\n");return 1;}
    if (!cuda_init()){fprintf(stderr,"[CUDA] failed\n");return 1;}

    Config cfg;
    for (int i=1;i<argc;i++){
        if (!strcmp(argv[i],"--mode")    &&i+1<argc) cfg.mode  =(int)atoi(argv[++i]);
        if (!strcmp(argv[i],"--frames")  &&i+1<argc) cfg.n_frames=(int)atoi(argv[++i]);
        if (!strcmp(argv[i],"--fps")     &&i+1<argc) cfg.fps   =(int)atoi(argv[++i]);
        if (!strcmp(argv[i],"--exposure")&&i+1<argc) cfg.exp_ns=(uint64_t)atoll(argv[++i]);
        if (!strcmp(argv[i],"--gain")    &&i+1<argc) cfg.gain  =(float)atof(argv[++i]);
        if (!strcmp(argv[i],"--out")     &&i+1<argc) cfg.outfile=argv[++i];
        if (!strcmp(argv[i],"--port")    &&i+1<argc) cfg.port  =(int)atoi(argv[++i]);
        if (!strcmp(argv[i],"--server"))              cfg.server=true;
        if (!strcmp(argv[i],"--list-modes"))          cfg.list_modes=true;
    }

    /* --list-modes: enumerate sensor modes and exit (no session needed). */
    if (cfg.list_modes) { bool ok = list_modes(); return ok ? 0 : 1; }

    printf("[RAW] mode=%d fps=%d exp=%llu gain=%.3f server=%d\n",
           cfg.mode,cfg.fps,(unsigned long long)cfg.exp_ns,
           cfg.gain,cfg.server);
    fflush(stdout);

    /* Initialize session */
    Session session;
    if (!session.init(cfg)) {
        fprintf(stderr,"[RAW] session init failed\n"); return 1;
    }

    /* Start streaming. Server mode uses repeat() so Argus free-runs the sensor at
     * the full frame-duration rate (the CSI/GMSL link is driven at the configured
     * fps regardless of consumer speed); single-shot keeps the one-shot fill. */
    const int PIPELINE_DEPTH = 10;
    if (cfg.server) {
        printf("[RAW] starting continuous capture via repeat() @ %d fps...\n", cfg.fps);
        fflush(stdout);
        session.start_repeat();
    } else {
        printf("[RAW] filling pipeline (%d captures)...\n",PIPELINE_DEPTH);
        fflush(stdout);
        session.fill_pipeline(PIPELINE_DEPTH);
    }

    /* Wait for first frame */
    printf("[RAW] waiting for first frame...\n"); fflush(stdout);
    if (!session.wait_for_frame(30000)) {
        fprintf(stderr,"[RAW] no frame arrived\n");
        session.shutdown(); return 1;
    }

    if (!cfg.server) {
        /* ── Single-shot mode ──────────────────────────────────────────────── */
        for (int fn=0;fn<cfg.n_frames&&g_running;fn++) {
            CUgraphicsResource cuRes=0;
            CUresult cr=cuEGLStreamConsumerAcquireFrame(
                &session.cuConn,&cuRes,nullptr,5000U);
            if (cr!=CUDA_SUCCESS){
                const char*s="?"; cuGetErrorString(cr,&s);
                fprintf(stderr,"[CUDA] AcquireFrame: %s\n",s); break;}
            std::vector<uint16_t> px;
            bool ok=cuda_frame_to_u16(cuRes,session.W,session.H,session.BPP,px);
            cuEGLStreamConsumerReleaseFrame(&session.cuConn,cuRes,nullptr);
            if (ok) {
                g_buf.update(std::move(px),
                             session.W,session.H,session.BPP,
                             cfg.exp_ns,cfg.gain);
                printf("[CUDA] frame %d: min=%u max=%u distinct=%zu\n",
                       fn,
                       *std::min_element(g_buf.pixels.begin(),g_buf.pixels.end()),
                       *std::max_element(g_buf.pixels.begin(),g_buf.pixels.end()),
                       std::set<uint16_t>(g_buf.pixels.begin(),g_buf.pixels.end()).size());
                session.fill_pipeline(3);
            }
        }
        if (g_buf.valid) save_file(cfg.outfile, g_buf);
        else fprintf(stderr,"[RAW] no frame captured\n");

    } else {
        /* ── Server mode — continuous streaming buffer ─────────────────────
         *
         * Main loop (single thread — safe for Argus/EGL):
         *
         *   1. Try to acquire latest frame (100ms timeout)
         *   2. If frame arrived: decode → update buffer → submit next capture
         *   3. Check for TCP clients (non-blocking select)
         *   4. Serve all waiting clients from the in-memory buffer
         *   5. Repeat
         *
         * Clients always get the most recently buffered frame.
         * No client waits for a new capture — they get what's in the buffer.
         * Frame age is reported so client knows how fresh the data is.
         *
         * Pipeline kept full by submitting a new capture after each acquire.
         * This keeps Argus streaming continuously at (near) sensor rate.
         */

        int srv=socket(AF_INET,SOCK_STREAM,0);
        int one=1; setsockopt(srv,SOL_SOCKET,SO_REUSEADDR,&one,sizeof(one));
        /* Non-blocking accept so the Argus loop never stalls on connect. */
        sockaddr_in a{}; a.sin_family=AF_INET;
        a.sin_addr.s_addr=INADDR_ANY; a.sin_port=htons(cfg.port);
        bind(srv,(sockaddr*)&a,sizeof(a)); listen(srv,16);

        printf("[Server] port %d  (continuous RAW16 buffer, persistent conns)\n",cfg.port);
        printf("[Server] clients served from latest in-memory frame\n");
        printf("[Server] READY — fast captures enabled\n");
        fflush(stdout);

        /* Publish frames to shared memory for LOCAL zero-copy consumers (webui /
         * sensor-timing tools) -- no pack, no TCP, no unpack. */
        if (!g_shm.init(session.W, session.H))
            printf("[Server] WARN: shm publish disabled (init failed)\n");
        fflush(stdout);

        /* TCP serve runs on its own thread so pack+send never throttles capture. */
        std::thread serve_thr(serve_thread_fn, srv);

        /* Stats */
        int frames_captured = 0;
        uint64_t last_pub_sof = 0;   /* SOF of last published frame (duplicate-skip) */

        while (g_running) {

            /* ── Apply any pending live exp/gain change (Argus thread) ────── */
            bool have=false; uint64_t we=0; float wg=0.0f;
            {   std::lock_guard<std::mutex> lk(g_pending_mtx);
                if (g_pending.have) { have=true; we=g_pending.exp_ns; wg=g_pending.gain; g_pending.have=false; }
            }
            if (have && session.set_exposure_gain_live(we, wg)) {
                printf("[Server] live exp=%.1fms gain=%s\n",
                       we/1e6, wg>0.0f ? std::to_string(wg).c_str() : "auto");
                fflush(stdout);
            }

            /* ── Try to get latest frame (short timeout) ─────────────────── */
            CUgraphicsResource cuRes=0;
            CUresult cr=cuEGLStreamConsumerAcquireFrame(
                &session.cuConn,&cuRes,nullptr,
                100U);  /* 100ms — short so we can check clients often */

            if (cr==CUDA_SUCCESS && cuRes) {
                /* Drain capture metadata FIRST (updates actual exp/gain + the
                 * sensor SOF of the latest completed frame). Under repeat()+
                 * MAILBOX, acquire returns the latest frame immediately even if
                 * we already published it, so DROP duplicates: if no new sensor
                 * frame (SOF unchanged) has arrived, release and retry without
                 * re-decoding/re-publishing. Keeps measured_fps == true sensor
                 * rate and avoids burning CPU on duplicate frames. */
                session.poll_metadata();
                if (session.actual_sof_ns != 0 && session.actual_sof_ns == last_pub_sof) {
                    cuEGLStreamConsumerReleaseFrame(&session.cuConn, cuRes, nullptr);
                    if (!session.repeating) session.iSess->capture(session.req.get());
                    usleep(1000);   /* real frames arrive every ~14-17ms; brief yield */
                    continue;
                }

                /* New frame — decode into buffer */
                std::vector<uint16_t> px;
                bool ok=cuda_frame_to_u16(cuRes,session.W,session.H,session.BPP,px);
                cuEGLStreamConsumerReleaseFrame(
                    &session.cuConn,cuRes,nullptr);

                if (ok) {
                    uint64_t rep_exp  = session.actual_exp_ns > 0
                                        ? session.actual_exp_ns : session.cur_exp_ns;
                    float    rep_gain = session.actual_gain > 0.0f
                                        ? session.actual_gain : session.cur_gain;
                    /* Build the new frame and republish the pointer (zero-copy
                     * hand-off to the serve thread; only the swap is locked). */
                    auto fb = std::make_shared<FrameBuffer>();
                    fb->update(std::move(px), session.W, session.H, session.BPP, rep_exp, rep_gain);
                    fb->seq = frames_captured + 1;         /* running seq (per-object update() would reset) */
                    { std::lock_guard<std::mutex> lk(g_pub_mtx); g_pub = fb; }
                    g_shm.publish(fb->pixels.data(), fb->w, fb->h, fb->bpp,
                                  rep_exp, session.actual_sof_ns, rep_gain, fb->capture_time);
                    if (g_shm.hdr) g_shm.hdr->_pad2 = (uint32_t)session.frames_recv;  /* link frames received -> received_fps */
                    last_pub_sof = session.actual_sof_ns;   /* mark for duplicate-skip */
                    frames_captured++;

                    if (frames_captured%30==1) {
                        static double t_last=0; static int f_last=0; static uint64_t r_last=0;
                        struct timespec ts; clock_gettime(CLOCK_MONOTONIC,&ts);
                        double now=ts.tv_sec+ts.tv_nsec*1e-9;
                        double fps=(t_last>0)?(frames_captured-f_last)/(now-t_last):0.0;
                        double recv_fps=(t_last>0)?(double)(session.frames_recv-r_last)/(now-t_last):0.0;
                        printf("[Buffer] frame=%d  decode=%.1fms  serve=%.1fms(thread)  pub_fps=%.1f  recv_fps=%.1f(link)\n",
                               frames_captured, g_dec_ms, g_serve_ms, fps, recv_fps);
                        fflush(stdout);
                        t_last=now; f_last=frames_captured; r_last=session.frames_recv;
                    }

                    /* repeat() keeps the sensor streaming — no per-frame submit
                     * (an extra capture() under repeat would double-queue). */
                    if (!session.repeating)
                        session.iSess->capture(session.req.get());
                }

            } else if (cr==CUDA_ERROR_LAUNCH_TIMEOUT) {
                /* No frame in 100ms — under repeat() re-arm; else top up the queue. */
                if (session.repeating) session.iSess->repeat(session.req.get());
                else                   session.fill_pipeline(3);
            }
            /* CUDA_ERROR_UNKNOWN or other: ignore, try again */
            /* Serving is handled by serve_thr — the capture loop no longer blocks on it. */
        }

        printf("[Server] shutting down  frames=%d\n", frames_captured);
        fflush(stdout);
        serve_thr.join();                                  /* g_running cleared -> serve exits within ~5ms */
        close(srv);
    }

    session.stop_repeat();
    session.shutdown();
    return 0;
}
