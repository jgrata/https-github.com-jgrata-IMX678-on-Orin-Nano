// EcamReceiver.cs — background TCP frame receiver for ECamHDRClient.
//
// MATLAB is single-threaded (no Parallel Toolbox here), so it cannot receive
// frame N+1 while it unpacks/stores frame N. This helper runs a background
// thread that pipelines CMD_CAPTURE requests and receives packed frames into a
// latest-frame slot, so the ~90ms gigabit transfer overlaps MATLAB's ~50ms
// unpack+store. MATLAB pulls the newest packed payload via GetFrame().
//
// It opens its OWN TCP connection to the image_server (separate from the
// control connection ECamHDRClient holds), and speaks the same wire protocol:
//   request  : [cmd uint32 LE][payload_len uint32 LE]      (CMD_CAPTURE=1, len 0)
//   response : [status u8]; if 0x00 -> [H u32][W u32][dtype u8][payload]
//              dtype 0x11 = RAW10 packed (ceil(H*W/4)*5 B), 0x10 = uint16
//
// Build (no SDK needed):
//   csc /target:library /optimize+ /out:EcamReceiver.dll EcamReceiver.cs
//
// Unpacking stays in MATLAB (unpackRaw10) so the tested decode path is reused;
// this helper only removes the receive serialization.

using System;
using System.Net.Sockets;
using System.Threading;

namespace Ecam
{
    public class Receiver
    {
        private const uint CMD_CAPTURE = 1;

        private TcpClient      _tcp;
        private NetworkStream  _ns;
        private Thread         _thread;
        private volatile bool  _running;
        private readonly object _lock = new object();

        private byte[] _latest;      // latest frame payload (packed or uint16)
        private int    _H, _W, _dtype, _seq;
        private string _err;

        public Receiver(string host, int port)
        {
            _tcp = new TcpClient();
            _tcp.NoDelay = true;
            try { _tcp.ReceiveBufferSize = 16 * 1024 * 1024; } catch { }
            _tcp.Connect(host, port);
            _ns = _tcp.GetStream();
        }

        public void Start(int depth)
        {
            if (depth < 1) depth = 1;
            _running = true;
            _thread = new Thread(() => Loop(depth));
            _thread.IsBackground = true;
            _thread.Start();
        }

        private void SendCapture()
        {
            // [cmd uint32 LE = 1][len uint32 LE = 0]
            byte[] hdr = new byte[8];
            hdr[0] = (byte)(CMD_CAPTURE & 0xFF);   // rest already zero
            _ns.Write(hdr, 0, 8);
            _ns.Flush();
        }

        private void ReadExact(byte[] buf, int off, int n)
        {
            int got = 0;
            while (got < n)
            {
                int r = _ns.Read(buf, off + got, n - got);
                if (r <= 0) throw new Exception("connection closed");
                got += r;
            }
        }

        private void Loop(int depth)
        {
            try
            {
                for (int i = 0; i < depth; i++) SendCapture();   // prime pipeline
                byte[] one = new byte[1], h9 = new byte[9];

                while (_running)
                {
                    ReadExact(one, 0, 1);
                    if (one[0] != 0x00)
                    {
                        byte[] lb = new byte[4]; ReadExact(lb, 0, 4);
                        int len = BitConverter.ToInt32(lb, 0);
                        byte[] mb = new byte[len]; if (len > 0) ReadExact(mb, 0, len);
                        throw new Exception("server error: " +
                            System.Text.Encoding.ASCII.GetString(mb));
                    }
                    ReadExact(h9, 0, 9);
                    int  H     = BitConverter.ToInt32(h9, 0);
                    int  W     = BitConverter.ToInt32(h9, 4);
                    byte dtype = h9[8];

                    int nBytes;
                    if (dtype == 0x11)      nBytes = ((H * W + 3) / 4) * 5;  // RAW10 packed
                    else if (dtype == 0x12) nBytes = ((H * W + 1) / 2) * 3;  // RAW12 packed
                    else if (dtype == 0x10) nBytes = H * W * 2;             // uint16
                    else throw new Exception("unknown dtype " + dtype);

                    byte[] payload = new byte[nBytes];
                    ReadExact(payload, 0, nBytes);

                    SendCapture();   // keep the pipeline full

                    lock (_lock)
                    {
                        _latest = payload; _H = H; _W = W; _dtype = dtype; _seq++;
                        Monitor.PulseAll(_lock);
                    }
                }
            }
            catch (Exception e)
            {
                lock (_lock) { _err = e.Message; Monitor.PulseAll(_lock); }
            }
        }

        // Block until a frame newer than lastSeq is available; return its packed
        // payload. Returns null on timeout. H/W/dtype/seq come back as out args
        // (MATLAB receives them as extra return values).
        public byte[] GetFrame(int lastSeq, int timeoutMs,
                               out int H, out int W, out int dtype, out int seq)
        {
            lock (_lock)
            {
                int deadline = Environment.TickCount + timeoutMs;
                while (_seq <= lastSeq && _err == null)
                {
                    int wait = deadline - Environment.TickCount;
                    if (wait <= 0) { H = 0; W = 0; dtype = 0; seq = lastSeq; return null; }
                    Monitor.Wait(_lock, wait);
                }
                if (_err != null) throw new Exception(_err);
                H = _H; W = _W; dtype = _dtype; seq = _seq;
                return _latest;
            }
        }

        public void Stop()
        {
            _running = false;
            try { if (_ns  != null) _ns.Close();  } catch { }
            try { if (_tcp != null) _tcp.Close(); } catch { }
        }
    }
}
