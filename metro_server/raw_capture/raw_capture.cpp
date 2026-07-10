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
                               uint32_t w, uint32_t h,
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

    /* Left-aligned 10-bit: stored = sensor<<6  →  sensor = stored>>6 (0-1023) */
    for (auto& v:out) v>>=6;

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

        Range<uint64_t> er=iMode->getExposureTimeRange();
        Range<float>    gr=iMode->getAnalogGainRange();

        sess.reset(iProv->createCaptureSession(devs[0]));
        iSess=interface_cast<ICaptureSession>(sess);
        if (!iSess){fprintf(stderr,"[Session] no session\n");return false;}
        printf("[Session] capture session created\n"); fflush(stdout);

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
            apply_exposure(cfg, er, gr);
        }
        iReq->enableOutputStream(os.get());
        return true;
    }

    void apply_exposure(const Config& cfg,
                        const Range<uint64_t>& er,
                        const Range<float>& gr)
    {
        if (!iSrc) return;
        if (cfg.exp_ns>0) {
            uint64_t e=std::max((uint64_t)er.min(),
                       std::min((uint64_t)er.max(),cfg.exp_ns));
            iSrc->setExposureTimeRange(Range<uint64_t>(e,e));
        }
        if (cfg.gain>0.0f) {
            float g=std::max(gr.min(),std::min(gr.max(),cfg.gain));
            iSrc->setGainRange(Range<float>(g,g));
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

static void serve_client(int cli, const FrameBuffer& buf,
                          const Config& cfg)
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

/* Non-blocking: serve any waiting clients, return number served */
static int serve_waiting_clients(int srv, const FrameBuffer& buf,
                                  const Config& cfg)
{
    int served=0;
    while (true) {
        fd_set fds; FD_ZERO(&fds); FD_SET(srv,&fds);
        struct timeval tv={0,0};  /* instant poll, no wait */
        if (select(srv+1,&fds,nullptr,nullptr,&tv)<=0) break;
        int cli=accept(srv,nullptr,nullptr);
        if (cli<0) break;
        double age=buf.age_ms();
        /* No per-client print in hot path */
        serve_client(cli,buf,cfg);
        close(cli);
        served++;
    }
    return served;
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
            bool ok=cuda_frame_to_u16(cuRes,session.W,session.H,px);
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
        sockaddr_in a{}; a.sin_family=AF_INET;
        a.sin_addr.s_addr=INADDR_ANY; a.sin_port=htons(cfg.port);
        bind(srv,(sockaddr*)&a,sizeof(a)); listen(srv,16);

        printf("[Server] port %d  (continuous RAW16 buffer)\n",cfg.port);
        printf("[Server] clients served from latest in-memory frame\n");
        printf("[Server] READY — fast captures enabled\n");
        fflush(stdout);

        /* Stats */
        int frames_captured = 0;
        int clients_served  = 0;
        double t_last_frame = 0;

        while (g_running) {

            /* ── Try to get latest frame (short timeout) ─────────────────── */
            CUgraphicsResource cuRes=0;
            CUresult cr=cuEGLStreamConsumerAcquireFrame(
                &session.cuConn,&cuRes,nullptr,
                100U);  /* 100ms — short so we can check clients often */

            if (cr==CUDA_SUCCESS && cuRes) {
                /* Got a frame — decode into buffer */
                std::vector<uint16_t> px;
                bool ok=cuda_frame_to_u16(cuRes,session.W,session.H,px);
                cuEGLStreamConsumerReleaseFrame(
                    &session.cuConn,cuRes,nullptr);

                if (ok) {
                    g_buf.update(std::move(px),
                                 session.W,session.H,session.BPP,
                                 cfg.exp_ns,cfg.gain);
                    frames_captured++;

                    /* Print stats every 10 frames */
                    if (frames_captured%10==1) {
                        printf("[Buffer] frame=%d  clients=%d\n",
                           g_buf.seq, clients_served);
                    fflush(stdout);
                        fflush(stdout);
                    }

                    /* Immediately submit next capture to keep pipeline full */
                    session.iSess->capture(session.req.get());
                }

            } else if (cr==CUDA_ERROR_LAUNCH_TIMEOUT) {
                /* No frame in 100ms — pipeline might be stalling */
                /* Submit more captures to keep it flowing */
                session.fill_pipeline(3);
            }
            /* CUDA_ERROR_UNKNOWN or other: ignore, try again */

            /* ── Serve all waiting TCP clients (non-blocking) ────────────── */
            int n=serve_waiting_clients(srv,g_buf,cfg);
            clients_served+=n;
        }

        printf("[Server] shutting down  frames=%d  clients=%d\n",
               frames_captured,clients_served);
        fflush(stdout);
        close(srv);
    }

    session.shutdown();
    return 0;
}
