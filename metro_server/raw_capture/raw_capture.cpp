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

#include <EGL/egl.h>
#include <EGL/eglext.h>
#include <cuda.h>
#include <cudaEGL.h>
#include <Argus/Argus.h>
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

// ── CUDA EGL frame → uint16 ───────────────────────────────────────────────────

static bool cuda_frame_to_u16(CUgraphicsResource res,
                               uint32_t w, uint32_t h, uint32_t bpp,
                               std::vector<uint16_t>& out)
{
    CUeglFrame f; memset(&f,0,sizeof(f));
    CU_CHECK(cuGraphicsResourceGetMappedEglFrame(&f,res,0,0));

    uint32_t fw=f.width?f.width:w, fh=f.height?f.height:h, pitch=f.pitch;
    CUdeviceptr dptr=0; bool alloc=false;

    if (f.frameType==CU_EGL_FRAME_TYPE_PITCH) {
        dptr=(CUdeviceptr)f.frame.pPitch[0];
        if (!pitch) pitch=fw*2;
    } else {
        CUarray arr=f.frame.pArray[0];
        CUDA_ARRAY_DESCRIPTOR d{}; cuArrayGetDescriptor(&d,arr);
        fw=(uint32_t)d.Width; fh=(uint32_t)d.Height;
        size_t dp=0; CU_CHECK(cuMemAllocPitch(&dptr,&dp,fw*2,fh,2));
        pitch=(uint32_t)dp; alloc=true;
        CUDA_MEMCPY2D cp{};
        cp.srcMemoryType=CU_MEMORYTYPE_ARRAY; cp.srcArray=arr;
        cp.dstMemoryType=CU_MEMORYTYPE_DEVICE; cp.dstDevice=dptr;
        cp.dstPitch=pitch; cp.WidthInBytes=fw*2; cp.Height=fh;
        if (cuMemcpy2D(&cp)!=CUDA_SUCCESS){cuMemFree(dptr);return false;}
    }
    if (!dptr) return false;

    out.resize(fw*fh);
    CUDA_MEMCPY2D cp2{};
    cp2.srcMemoryType=CU_MEMORYTYPE_DEVICE; cp2.srcDevice=dptr; cp2.srcPitch=pitch;
    cp2.dstMemoryType=CU_MEMORYTYPE_HOST;   cp2.dstHost=out.data(); cp2.dstPitch=fw*2;
    cp2.WidthInBytes=fw*2; cp2.Height=fh;
    CUresult mr=cuMemcpy2D(&cp2);
    if (alloc) cuMemFree(dptr);
    if (mr!=CUDA_SUCCESS) return false;

    /* Argus RAW16 is MSB-aligned: an N-bit sensor value is stored as
     * sensor<<(16-N), so sensor = stored>>(16-N). 10-bit -> >>6 (0-1023),
     * 12-bit -> >>4 (0-4095). Vectorized under -O3/NEON (~2ms). */
    uint32_t sh = (bpp < 16) ? (16 - bpp) : 0;
    if (sh) for (auto& v:out) v>>=sh;

    if (fw!=w||fh!=h) {
        std::vector<uint16_t> tmp(w*h,0);
        for (uint32_t y=0;y<std::min(fh,h);y++)
            for (uint32_t x=0;x<std::min(fw,w);x++)
                tmp[y*w+x]=out[y*fw+x];
        out=std::move(tmp);
    }
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
            const CaptureMetadata* m = iCC->getMetadata();
            const ICaptureMetadata* iM = interface_cast<const ICaptureMetadata>(m);
            if (iM) {
                actual_exp_ns = iM->getSensorExposureTime();
                actual_gain   = iM->getSensorAnalogGain();
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
        /* Re-submit so the mutated request takes effect. The server loop
         * submits a fresh capture after each acquire anyway; this ensures
         * the change lands even if the pipeline is momentarily drained. */
        iSess->capture(req.get());
        return true;
    }

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
        W  =iMode->getResolution().width();
        H  =iMode->getResolution().height();
        BPP=iMode->getInputBitDepth();
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
            uint64_t dur=1000000000ULL/(uint64_t)cfg.fps;
            iSrc->setFrameDurationRange(Range<uint64_t>(dur,dur));
            apply_exposure(cfg.exp_ns, cfg.gain);
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

        if (exp_ns > 0) {
            uint64_t e = std::max((uint64_t)exp_range.min(),
                         std::min((uint64_t)exp_range.max(), exp_ns));
            iSrc->setExposureTimeRange(Range<uint64_t>(e,e));
        } else {
            /* auto exposure: hand the full sensor range back to AE */
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
            if (i%5==0) {  /* every 50ms */
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

/* Pack uint16 10-bit pixels (0-1023) into RAW10: 4 px -> 5 bytes.
 * out size = ceil(n/4)*5. Byte layout matches the Python pack_raw10 and the
 * MATLAB unpackRaw10 exactly:
 *   o0..o3 = p0..p3 >> 2   (high 8 bits)
 *   o4     = (p0&3) | (p1&3)<<2 | (p2&3)<<4 | (p3&3)<<6   (low 2 bits, LE)
 * Doing this in C++ replaces the ~0.10s NumPy pack in image_server. */
static void pack_raw10(const uint16_t* px, size_t n, std::vector<uint8_t>& out)
{
    size_t groups = (n + 3) / 4;
    out.resize(groups * 5);
    uint8_t* o = out.data();
    size_t i = 0;
    for (size_t g = 0; g < groups; ++g, o += 5, i += 4) {
        uint16_t p0 = i   < n ? px[i]   : 0;
        uint16_t p1 = i+1 < n ? px[i+1] : 0;
        uint16_t p2 = i+2 < n ? px[i+2] : 0;
        uint16_t p3 = i+3 < n ? px[i+3] : 0;
        o[0] = (uint8_t)(p0 >> 2);
        o[1] = (uint8_t)(p1 >> 2);
        o[2] = (uint8_t)(p2 >> 2);
        o[3] = (uint8_t)(p3 >> 2);
        o[4] = (uint8_t)((p0 & 3) | ((p1 & 3) << 2) |
                         ((p2 & 3) << 4) | ((p3 & 3) << 6));
    }
}

/* Pack uint16 12-bit pixels (0-4095) into RAW12: 2 px -> 3 bytes.
 * out size = ceil(n/2)*3. Byte layout matches the MATLAB unpackRaw12:
 *   o0 = p0 >> 4                              (high 8 bits of p0)
 *   o1 = p1 >> 4                              (high 8 bits of p1)
 *   o2 = (p0 & 0xF) | ((p1 & 0xF) << 4)       (low nibbles, LE) */
static void pack_raw12(const uint16_t* px, size_t n, std::vector<uint8_t>& out)
{
    size_t groups = (n + 1) / 2;
    out.resize(groups * 3);
    uint8_t* o = out.data();
    size_t i = 0;
    for (size_t g = 0; g < groups; ++g, o += 3, i += 2) {
        uint16_t p0 = i   < n ? px[i]   : 0;
        uint16_t p1 = i+1 < n ? px[i+1] : 0;
        o[0] = (uint8_t)(p0 >> 4);
        o[1] = (uint8_t)(p1 >> 4);
        o[2] = (uint8_t)((p0 & 0xF) | ((p1 & 0xF) << 4));
    }
}

/* Pending exp/gain change requested by a client, consumed by the Argus loop.
 * Guarded by a simple flag pair; only the loop writes cur_* on the Session. */
struct PendingCtl {
    volatile bool     have = false;
    volatile uint64_t exp_ns = 0;
    volatile float    gain = 0.0f;
} g_pending;

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
    static std::vector<uint8_t> packed;   // reused; single-threaded serve path
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
}

/* Handle one request on an already-open client socket.
 * Returns false if the connection should be closed. */
static bool handle_request(int cli, const FrameBuffer& buf)
{
    ReqHdr rq{};
    if (!recv_all(cli,&rq,sizeof(rq))) return false;  /* client closed */

    switch (rq.cmd) {
    case REQ_FRAME:
        serve_frame(cli, buf);
        return true;
    case REQ_FRAME_PACKED:
        serve_frame_packed(cli, buf);
        return true;
    case REQ_SET_EXPGAIN: {
        g_pending.exp_ns = rq.want_exp;
        g_pending.gain   = rq.want_gain;
        g_pending.have   = true;
        uint32_t ack = 0;
        return send_all(cli,&ack,sizeof(ack));
    }
    case REQ_PING: {
        /* Lightweight actual-value probe: return a NetHdr (no pixel payload)
         * carrying the exp/gain currently tagged on the latest buffer, so the
         * Python AE monitor can poll without a full-frame transfer or any
         * v4l2-ctl subprocess. nf=0 signals "header only". */
        NetHdr nh{};
        nh.magic      = 0x52413130;
        nh.w          = buf.w;
        nh.h          = buf.h;
        nh.bpp        = buf.bpp;
        nh.nf         = 0;
        nh.exp_ns     = buf.exp_ns;
        nh.gain_x1000 = (uint32_t)(buf.gain*1000);
        nh.pad        = 0;
        return send_all(cli,&nh,sizeof(nh));
    }
    default:
        return false;  /* unknown command: drop connection */
    }
}

/* Non-blocking client set management. Accept new connections, service any
 * readable client sockets, drop closed ones. clients[] holds open fds. */
struct ClientSet {
    std::vector<int> fds;

    void poll(int srv, const FrameBuffer& buf)
    {
        /* Accept every pending new connection (non-blocking). */
        while (true) {
            fd_set afds; FD_ZERO(&afds); FD_SET(srv,&afds);
            struct timeval tv={0,0};
            if (select(srv+1,&afds,nullptr,nullptr,&tv)<=0) break;
            int cli=accept(srv,nullptr,nullptr);
            if (cli<0) break;
            int one=1; setsockopt(cli,IPPROTO_TCP,TCP_NODELAY,&one,sizeof(one));
            fds.push_back(cli);
        }

        if (fds.empty()) return;

        /* Which existing clients have a request waiting? */
        fd_set rfds; FD_ZERO(&rfds); int maxfd=-1;
        for (int fd : fds) { FD_SET(fd,&rfds); if (fd>maxfd) maxfd=fd; }
        struct timeval tv={0,0};
        if (select(maxfd+1,&rfds,nullptr,nullptr,&tv)<=0) return;

        std::vector<int> keep;
        keep.reserve(fds.size());
        for (int fd : fds) {
            if (FD_ISSET(fd,&rfds)) {
                if (handle_request(fd, buf)) keep.push_back(fd);
                else close(fd);
            } else {
                keep.push_back(fd);
            }
        }
        fds.swap(keep);
    }

    void close_all() { for (int fd : fds) close(fd); fds.clear(); }
};

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
    }
    printf("[RAW] mode=%d fps=%d exp=%llu gain=%.3f server=%d\n",
           cfg.mode,cfg.fps,(unsigned long long)cfg.exp_ns,
           cfg.gain,cfg.server);
    fflush(stdout);

    /* Initialize session */
    Session session;
    if (!session.init(cfg)) {
        fprintf(stderr,"[RAW] session init failed\n"); return 1;
    }

    /* Pre-fill pipeline with requests so sensor starts streaming */
    const int PIPELINE_DEPTH = 10;
    printf("[RAW] filling pipeline (%d captures)...\n",PIPELINE_DEPTH);
    fflush(stdout);
    session.fill_pipeline(PIPELINE_DEPTH);

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

        ClientSet clients;

        /* Stats */
        int frames_captured = 0;

        while (g_running) {

            /* ── Apply any pending live exp/gain change (Argus thread) ────── */
            if (g_pending.have) {
                uint64_t we = g_pending.exp_ns;
                float    wg = g_pending.gain;
                g_pending.have = false;
                if (session.set_exposure_gain_live(we, wg)) {
                    printf("[Server] live exp=%.1fms gain=%s\n",
                           we/1e6,
                           wg>0.0f ? std::to_string(wg).c_str() : "auto");
                    fflush(stdout);
                }
            }

            /* ── Try to get latest frame (short timeout) ─────────────────── */
            CUgraphicsResource cuRes=0;
            CUresult cr=cuEGLStreamConsumerAcquireFrame(
                &session.cuConn,&cuRes,nullptr,
                100U);  /* 100ms — short so we can check clients often */

            if (cr==CUDA_SUCCESS && cuRes) {
                /* Got a frame — decode into buffer */
                std::vector<uint16_t> px;
                bool ok=cuda_frame_to_u16(cuRes,session.W,session.H,session.BPP,px);
                cuEGLStreamConsumerReleaseFrame(
                    &session.cuConn,cuRes,nullptr);

                if (ok) {
                    /* Tag with the ACTUAL sensor exp/gain from capture metadata
                     * (what AE/AGC settled on), falling back to the requested
                     * values if metadata isn't available yet. This is what the
                     * client reads back as actual_exposure_ns / actual_gain. */
                    session.poll_metadata();
                    uint64_t rep_exp  = session.actual_exp_ns > 0
                                        ? session.actual_exp_ns : session.cur_exp_ns;
                    float    rep_gain = session.actual_gain > 0.0f
                                        ? session.actual_gain : session.cur_gain;
                    g_buf.update(std::move(px),
                                 session.W,session.H,session.BPP,
                                 rep_exp, rep_gain);
                    frames_captured++;

                    if (frames_captured%30==1) {
                        printf("[Buffer] frame=%d  conns=%zu\n",
                               g_buf.seq, clients.fds.size());
                        fflush(stdout);
                    }

                    /* Immediately submit next capture to keep pipeline full */
                    session.iSess->capture(session.req.get());
                }

            } else if (cr==CUDA_ERROR_LAUNCH_TIMEOUT) {
                /* No frame in 100ms — pipeline might be stalling */
                session.fill_pipeline(3);
            }
            /* CUDA_ERROR_UNKNOWN or other: ignore, try again */

            /* ── Service persistent client connections (non-blocking) ────── */
            clients.poll(srv, g_buf);
        }

        printf("[Server] shutting down  frames=%d  conns=%zu\n",
               frames_captured, clients.fds.size());
        fflush(stdout);
        clients.close_all();
        close(srv);
    }

    session.shutdown();
    return 0;
}
