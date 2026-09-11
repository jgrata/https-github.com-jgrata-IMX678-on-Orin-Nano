# IQ9 image-timestamp clock discipline (NTP now, PTP later)

Every captured frame carries **`host_epoch_ns` (CLOCK_REALTIME)** in its meta sidecar
(`camera_worker.py::_publish_meta`). Its accuracy = however the OS clock is disciplined,
so making timestamps NTP- or PTP-accurate is purely a clock-discipline choice — no
per-frame code change. The webui surfaces sync state at `/api/info` → `"clock"` and the
info-block **"clock sync"** row (`chronyc tracking`).

## Interim: NTP from the PC — ACTIVE
- IQ9 runs **chrony** (`chronyd.service`) as an NTP client of the PC `192.168.99.1`.
  `chrony.conf`: `server 192.168.99.1 iburst`, `maxdistance 30` (accepts the PC's
  high-dispersion Windows NTP — timesyncd's default 5 s limit rejected it), `makestep 0.1 3`.
- Measured ~tens-of-µs RELATIVE sync to the PC (`chronyc tracking` RMS ~60 µs).
- `systemd-timesyncd` is **disabled** (it and chrony must not both steer the clock).
- **PC side (elevated, one-time):** enable W32Time's NTP server —
  `reg add HKLM\SYSTEM\CurrentControlSet\Services\W32Time\TimeProviders\NtpServer /v Enabled /t REG_DWORD /d 1 /f`,
  `reg add HKLM\...\W32Time\Config /v AnnounceFlags /t REG_DWORD /d 5 /f`, `Restart-Service w32time`,
  and allow UDP 123 inbound.
- `chronyd`/`chronyc` are aarch64 (built via `bitbake chrony`, 4.5) and live in `/usr/sbin`
  on the board — NOT committed here (build from the Yocto tree; chronyd deps = glibc only,
  chronyc also needs libedit which the board has).

## Future: PTP from a grandmaster (e.g. 5G) — sub-µs, STAGED (disabled)
- `ptp4l.service` (slave on `end0`, PHC `/dev/ptp0`) + `phc2sys.service` (PHC → CLOCK_REALTIME).
- Switch NTP → PTP when a grandmaster is present:
  ```
  systemctl disable --now systemd-timesyncd chronyd
  systemctl enable  --now ptp4l phc2sys
  ```
  `host_epoch_ns` then becomes PTP-disciplined automatically.
- `ptp4l`/`phc2sys` are aarch64 (`bitbake linuxptp`, 4.1) in `/usr/sbin` — not committed
  (build from the Yocto tree; deps = glibc only).

**Note:** W32Time canNOT be a PTP grandmaster (NTP server only). True PTP needs the 5G
source, a hardware grandmaster, or 3rd-party Windows PTP software. Both `end0` (IQ9) and
the PC's Intel I225-LMvP are PTP-hardware-capable.
