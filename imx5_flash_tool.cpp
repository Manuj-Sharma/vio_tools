// imx5_flash_tool.cpp -- SAFE flash-config tool for the IMX-5 over SPI.  Written 2026-11-09 for ONE job: change
// DID_FLASH_CONFIG.startupNavDtMs (and nothing else), with a verified backup and read-back verification.
//
//   imx5_flash_tool show                                   read + verify + print (read-only)
//   imx5_flash_tool backup  --out DIR                      8 reads must be identical + checksum valid; writes DIR/{flashcfg.bin,.hex,.txt,sysparams.txt}
//   imx5_flash_tool set-nav-dt MS --backup DIR [--write]   DRY RUN unless --write. Refuses unless DIR/flashcfg.bin == the config currently on the IMX.
//   imx5_flash_tool restore --from DIR [--write]           DRY RUN unless --write. Puts back the backed-up checksum/key/startupImuDtMs/startupNavDtMs.
//   common: [--dev /dev/spidev0.0] [--speed 3000000]       stop imx5_spi_node first (one SPI owner; the tool refuses if it runs)
//
// How the write works (from the vendored SDK, ISDevice.cpp SetImxFlashConfig/UploadFlashConfigDiff): the host recomputes
// flashChecksum32 (covers bytes 8..end), then sends ONLY the changed bytes with SET_DATA(DID_FLASH_CONFIG, size, offset); the IMX accepts
// the config when its own checksum matches, and reports it in DID_SYS_PARAMS.flashCfgChecksum. Here checksum(4) + key(8) + startupImuDtMs(12)
// + startupNavDtMs(16) are sent as ONE 16-byte packet at offset 4, so the IMX never sees a new value with an old checksum.
// startupNavDtMs is a STARTUP value: it takes effect after the IMX restarts (this tool never resets the IMX).
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <cstdlib>
#include <ctime>
#include <string>
#include <vector>
#include <fcntl.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include "ISComm.h"
#include "data_sets.h"

static int fd = -1; static uint32_t speed_hz = 3000000;
static const int CFG = (int)sizeof(nvm_flash_cfg_t);          // 288 on this SDK/firmware
static const int OFF_CK = 4, OFF_KEY = 8, OFF_IMU = 12, OFF_NAV = 16, WR_LEN = 16;   // write region [4,20)
static is_comm_instance_t comm; static uint8_t cbuf[8192];

static int xfer(const uint8_t *tx, uint8_t *rx, size_t n) {
    spi_ioc_transfer tr; memset(&tr, 0, sizeof(tr));
    tr.tx_buf = (unsigned long)tx; tr.rx_buf = (unsigned long)rx; tr.len = (uint32_t)n; tr.speed_hz = speed_hz; tr.bits_per_word = 8;
    return ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
}
static void fl16(const uint8_t *d, size_t n, uint8_t *a, uint8_t *b) { uint8_t x = 0, y = 0; for (size_t i = 0; i < n; ++i) { x += d[i]; y += x; } *a = x; *b = y; }
static void msleep(int ms) { usleep(ms * 1000); }

static void stop_and_drain() {
    uint8_t b[16] = {0xEF, 0x49, 6, 0, 0, 0}; fl16(b, 6, &b[6], &b[7]); uint8_t t[256]; memset(t, 0xFF, 256); xfer(b, t, 8); msleep(50);
    uint8_t dr[2048]; for (int i = 0; i < 2; ++i) { memset(dr, 0xFF, sizeof(dr)); xfer(dr, dr, sizeof(dr)); }
}
// one-shot GET_DATA(did), returns the packet payload (whole struct) or empty
static bool get_did(uint32_t did, std::vector<uint8_t> &out) {
    const int N = 512; uint8_t tx[N], rx[N];
    for (int attempt = 0; attempt < 4; ++attempt) {
        memset(tx, 0xFF, N);
        if (is_comm_get_data_to_buf(tx, N, &comm, did, 0, 0, 0) <= 0) return false;
        for (int f = 0; f < 10; ++f) {
            if (xfer(tx, rx, N) < 0) return false;
            memset(tx, 0xFF, N);
            int fr = is_comm_free(&comm), nc = fr < N ? fr : N; memcpy(comm.rxBuf.tail, rx, nc); comm.rxBuf.tail += nc;
            protocol_type_t pt;
            while ((pt = is_comm_parse(&comm)) != _PTYPE_NONE) {
                if (pt != _PTYPE_INERTIAL_SENSE_DATA || comm.rxPkt.dataHdr.id != did) continue;
                if (comm.rxPkt.dataHdr.offset != 0) continue;
                out.assign(comm.rxPkt.data.ptr, comm.rxPkt.data.ptr + comm.rxPkt.dataHdr.size);
                return true;
            }
        }
    }
    return false;
}
static bool read_cfg(std::vector<uint8_t> &c, uint32_t *sys_ck) {
    if (!get_did(DID_FLASH_CONFIG, c) || (int)c.size() != CFG) return false;
    if (sys_ck) { std::vector<uint8_t> s; if (!get_did(DID_SYS_PARAMS, s) || s.size() < sizeof(sys_params_t)) return false; sys_params_t sp; memcpy(&sp, s.data(), sizeof(sp)); *sys_ck = sp.flashCfgChecksum; }
    return true;
}
static uint32_t u32(const std::vector<uint8_t> &c, int off) { uint32_t v; memcpy(&v, c.data() + off, 4); return v; }
static uint32_t calc_ck(const std::vector<uint8_t> &c) { return flashChecksum32(c.data(), CFG); }

// N consecutive reads must be identical, checksum must verify with the SDK algorithm, and the IMX's own reported checksum must equal it.
static bool stable_read(std::vector<uint8_t> &cur, uint32_t &sys_ck, int n, bool verbose) {
    std::vector<uint8_t> first;
    for (int i = 0; i < n; ++i) {
        std::vector<uint8_t> c; uint32_t sck = 0;
        if (!read_cfg(c, &sck)) { fprintf(stderr, "  read %d/%d FAILED\n", i + 1, n); return false; }
        if (i == 0) { first = c; sys_ck = sck; }
        else if (c != first || sck != sys_ck) { fprintf(stderr, "  read %d differs from read 1 -- config not stable\n", i + 1); return false; }
        msleep(40);
    }
    const uint32_t stored = u32(first, OFF_CK), calc = calc_ck(first);
    if (verbose) printf("  %d identical reads. stored checksum 0x%08x, recomputed (flashChecksum32 over bytes 8..%d) 0x%08x, IMX sysParams.flashCfgChecksum 0x%08x\n", n, stored, CFG - 1, calc, sys_ck);
    if (stored != calc) { fprintf(stderr, "  ABORT: recomputed checksum does not match the stored one -- my checksum routine is not valid for this config\n"); return false; }
    if (sys_ck != stored) { fprintf(stderr, "  ABORT: IMX-reported checksum differs from the config checksum\n"); return false; }
    cur = first; return true;
}

static std::vector<uint8_t> load(const std::string &p) {
    std::vector<uint8_t> v; FILE *f = fopen(p.c_str(), "rb"); if (!f) return v; uint8_t b[1024]; size_t n;
    while ((n = fread(b, 1, sizeof(b), f)) > 0) v.insert(v.end(), b, b + n); fclose(f); return v;
}
static bool save(const std::string &p, const void *d, size_t n) { FILE *f = fopen(p.c_str(), "wb"); if (!f) return false; bool ok = fwrite(d, 1, n, f) == n; ok &= fclose(f) == 0; return ok; }

static void describe(const std::vector<uint8_t> &c, FILE *o) {
    nvm_flash_cfg_t cfg; memcpy(&cfg, c.data(), sizeof(cfg));
    fprintf(o, "size=%u checksum=0x%08x key=%u\nstartupImuDtMs=%u startupNavDtMs=%u\nser0BaudRate=%u ser1BaudRate=%u\nioConfig=0x%08x ioConfig2=0x%08x\nsensorConfig=0x%08x platformConfig=0x%08x\n",
            cfg.size, cfg.checksum, cfg.key, cfg.startupImuDtMs, cfg.startupNavDtMs, cfg.ser0BaudRate, cfg.ser1BaudRate, cfg.ioConfig, cfg.ioConfig2, cfg.sensorConfig, cfg.platformConfig);
}
static void hexdump(const std::vector<uint8_t> &c, FILE *o) { for (size_t i = 0; i < c.size(); ++i) fprintf(o, "%02x%s", c[i], (i % 16 == 15 || i + 1 == c.size()) ? "\n" : " "); }
static std::string field_of(int off) { if (off < 4) return "size"; if (off < 8) return "checksum"; if (off < 12) return "key"; if (off < 16) return "startupImuDtMs"; if (off < 20) return "startupNavDtMs"; return "OTHER(offset " + std::to_string(off) + ")"; }
static std::vector<std::string> diff_fields(const std::vector<uint8_t> &a, const std::vector<uint8_t> &b) {
    std::vector<std::string> out; for (int i = 0; i < CFG; ++i) if (a[i] != b[i]) { std::string f = field_of(i); if (out.empty() || out.back() != f) out.push_back(f); } return out;
}

// SET_DATA of bytes [4,20) from `want`, then verify by read-back
static bool write_region(const std::vector<uint8_t> &want, const std::vector<uint8_t> &old, bool restore_on_fail) {
    uint8_t tx[512]; memset(tx, 0xFF, sizeof(tx));
    std::vector<uint8_t> region(want.begin() + OFF_CK, want.begin() + OFF_CK + WR_LEN);
    int n = is_comm_set_data_to_buf(tx, sizeof(tx), &comm, DID_FLASH_CONFIG, WR_LEN, OFF_CK, region.data());
    if (n <= 0) { fprintf(stderr, "  could not build the SET_DATA packet\n"); return false; }
    printf("  sending SET_DATA(DID_FLASH_CONFIG, size=%d, offset=%d): ", WR_LEN, OFF_CK); for (int i = 0; i < WR_LEN; ++i) printf("%02x ", region[i]); printf("  (packet %d bytes)\n", n);
    uint8_t rx[512]; if (xfer(tx, rx, sizeof(tx)) < 0) { perror("  xfer"); return false; }
    const uint32_t want_ck = u32(want, OFF_CK);
    for (int t = 0; t < 40; ++t) {   // up to ~10 s
        msleep(250);
        std::vector<uint8_t> c; uint32_t sck = 0;
        if (!read_cfg(c, &sck)) { printf("  (read-back %d: no reply yet)\n", t + 1); continue; }
        if (c == want && sck == want_ck) { printf("  VERIFIED after %.2f s: config on the IMX == intended config, IMX reports checksum 0x%08x\n", 0.25 * (t + 1), sck); return true; }
        if (t % 4 == 3) printf("  (read-back %d: not yet; nav=%u checksum 0x%08x sys 0x%08x)\n", t + 1, u32(c, OFF_NAV), u32(c, OFF_CK), sck);
    }
    std::vector<uint8_t> c; uint32_t sck = 0;
    bool got = read_cfg(c, &sck);
    if (got && c == old) { printf("  NOT APPLIED: the IMX still holds the ORIGINAL config (nothing changed).\n"); return false; }
    fprintf(stderr, "  UNEXPECTED STATE after the write (config is neither the old nor the new one, or unreadable).\n");
    if (restore_on_fail && got) {
        fprintf(stderr, "  attempting ONE restore of the original region...\n");
        std::vector<uint8_t> o2(old.begin() + OFF_CK, old.begin() + OFF_CK + WR_LEN); memset(tx, 0xFF, sizeof(tx));
        if (is_comm_set_data_to_buf(tx, sizeof(tx), &comm, DID_FLASH_CONFIG, WR_LEN, OFF_CK, o2.data()) > 0) xfer(tx, rx, sizeof(tx));
        msleep(1500); std::vector<uint8_t> c2; uint32_t s2 = 0;
        if (read_cfg(c2, &s2) && c2 == old) fprintf(stderr, "  restore verified: original config is back.\n"); else fprintf(stderr, "  RESTORE NOT CONFIRMED -- do not power cycle; use the backup in the backup directory.\n");
    }
    return false;
}

int main(int argc, char **argv) {
    if (argc < 2) { fprintf(stderr, "usage: see the header of imx5_flash_tool.cpp\n"); return 2; }
    std::string cmd = argv[1], dev = "/dev/spidev0.0", out, backup, from; bool write = false; int ms = -1;
    for (int i = 2; i < argc; ++i) { std::string a = argv[i];
        if (a == "--dev" && i + 1 < argc) dev = argv[++i]; else if (a == "--speed" && i + 1 < argc) speed_hz = atol(argv[++i]);
        else if (a == "--out" && i + 1 < argc) out = argv[++i]; else if (a == "--backup" && i + 1 < argc) backup = argv[++i];
        else if (a == "--from" && i + 1 < argc) from = argv[++i]; else if (a == "--write") write = true;
        else if (cmd == "set-nav-dt" && ms < 0 && a[0] != '-') ms = atoi(a.c_str());
        else { fprintf(stderr, "unknown argument %s\n", a.c_str()); return 2; } }
    if (system("pgrep -x imx5_spi_node >/dev/null") == 0) { fprintf(stderr, "ABORT: imx5_spi_node is running (it must not share the SPI bus with this tool)\n"); return 1; }
    fd = open(dev.c_str(), O_RDWR); if (fd < 0) { perror("open"); return 1; }
    uint8_t mode = SPI_MODE_3, bits = 8;
    if (ioctl(fd, SPI_IOC_WR_MODE, &mode) < 0 || ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0 || ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed_hz) < 0) { perror("spi"); return 1; }
    is_comm_init(&comm, cbuf, sizeof(cbuf), nullptr); is_comm_enable_protocol(&comm, _PTYPE_INERTIAL_SENSE_DATA); is_comm_set_protocol_mask(&comm, ENABLE_PROTOCOL_ISB);
    stop_and_drain();
    if (CFG != 288) { fprintf(stderr, "ABORT: sizeof(nvm_flash_cfg_t)=%d, expected 288 (the size the IMX reports)\n", CFG); return 1; }

    std::vector<uint8_t> cur; uint32_t sck = 0;
    printf("== link + config stability check (read-only)\n");
    if (!stable_read(cur, sck, cmd == "backup" ? 8 : 5, true)) { fprintf(stderr, "ABORT: no stable, verified read of the flash config. Nothing was written.\n"); return 1; }

    if (cmd == "show") { describe(cur, stdout); return 0; }

    if (cmd == "backup") {
        if (out.empty()) { fprintf(stderr, "--out DIR required\n"); return 2; }
        if (mkdir(out.c_str(), 0755) != 0) { perror("mkdir (the directory must not exist)"); return 1; }
        std::string p = out + "/";
        bool ok = save(p + "flashcfg.bin", cur.data(), cur.size());
        FILE *h = fopen((p + "flashcfg.hex").c_str(), "w"); if (h) { hexdump(cur, h); fclose(h); }
        FILE *t = fopen((p + "flashcfg.txt").c_str(), "w"); if (t) { fprintf(t, "IMX-5 DID_FLASH_CONFIG backup %ld (unix time)\n", (long)time(nullptr)); describe(cur, t); fclose(t); }
        std::vector<uint8_t> s; if (get_did(DID_SYS_PARAMS, s) && s.size() >= sizeof(sys_params_t)) { sys_params_t sp; memcpy(&sp, s.data(), sizeof(sp));
            FILE *q = fopen((p + "sysparams.txt").c_str(), "w"); if (q) { fprintf(q, "imuSamplePeriodMs=%u navUpdatePeriodMs=%u navOutputPeriodMs=%u sensorTruePeriod=%.9f flashCfgChecksum=0x%08x upTime=%.1f\n", sp.imuSamplePeriodMs, sp.navUpdatePeriodMs, sp.navOutputPeriodMs, sp.sensorTruePeriod, sp.flashCfgChecksum, sp.upTime); fclose(q); } }
        if (!ok) { fprintf(stderr, "backup write failed\n"); return 1; }
        printf("BACKUP OK -> %s  (flashcfg.bin %zu bytes, checksum 0x%08x)\n", out.c_str(), cur.size(), sck); describe(cur, stdout); return 0;
    }

    if (cmd == "set-nav-dt") {
        if (ms < 4 || ms > 14) { fprintf(stderr, "ABORT: MS must be 4..14 (got %d)\n", ms); return 2; }
        if (backup.empty()) { fprintf(stderr, "ABORT: --backup DIR is required\n"); return 2; }
        std::vector<uint8_t> b = load(backup + "/flashcfg.bin");
        if ((int)b.size() != CFG) { fprintf(stderr, "ABORT: %s/flashcfg.bin missing or wrong size\n", backup.c_str()); return 1; }
        if (b != cur) { fprintf(stderr, "ABORT: the backup is NOT identical to the config currently on the IMX (fields: "); for (auto &f : diff_fields(b, cur)) fprintf(stderr, "%s ", f.c_str()); fprintf(stderr, ")\n"); return 1; }
        printf("== backup %s is byte-identical to the config on the IMX\n", backup.c_str());
        if (u32(cur, OFF_IMU) > (uint32_t)ms) { fprintf(stderr, "ABORT: startupImuDtMs (%u) must not be larger than startupNavDtMs (%d)\n", u32(cur, OFF_IMU), ms); return 1; }
        std::vector<uint8_t> nw = cur; uint32_t v = ms; memcpy(nw.data() + OFF_NAV, &v, 4);
        uint32_t nck = calc_ck(nw); memcpy(nw.data() + OFF_CK, &nck, 4);
        auto df = diff_fields(cur, nw);
        printf("== planned change: startupNavDtMs %u -> %d, checksum 0x%08x -> 0x%08x ; changed fields:", u32(cur, OFF_NAV), ms, u32(cur, OFF_CK), nck); for (auto &f : df) printf(" %s", f.c_str()); printf("\n");
        for (auto &f : df) if (f != "checksum" && f != "startupNavDtMs") { fprintf(stderr, "ABORT: unexpected changed field %s\n", f.c_str()); return 1; }
        if (u32(cur, OFF_NAV) == (uint32_t)ms) { printf("already %d: nothing to do\n", ms); return 0; }
        if (!write) { printf("DRY RUN: nothing written. Re-run with --write to apply. Rollback: imx5_flash_tool restore --from %s --write\n", backup.c_str()); return 0; }
        printf("== WRITING\n");
        bool ok = write_region(nw, cur, true);
        printf(ok ? "RESULT: written and verified. The new value applies after the IMX restarts. Rollback: imx5_flash_tool restore --from %s --write\n" : "RESULT: NOT confirmed (see above)\n", backup.c_str());
        return ok ? 0 : 1;
    }

    if (cmd == "restore") {
        std::vector<uint8_t> b = load(from + "/flashcfg.bin");
        if ((int)b.size() != CFG) { fprintf(stderr, "ABORT: %s/flashcfg.bin missing or wrong size\n", from.c_str()); return 1; }
        if (u32(b, OFF_CK) != calc_ck(b)) { fprintf(stderr, "ABORT: the backup file's own checksum does not verify\n"); return 1; }
        if (b == cur) { printf("the IMX already holds the backed-up config: nothing to do\n"); return 0; }
        auto df = diff_fields(b, cur); printf("== differences current vs backup:"); for (auto &f : df) printf(" %s", f.c_str()); printf("\n");
        for (auto &f : df) if (f != "checksum" && f != "key" && f != "startupImuDtMs" && f != "startupNavDtMs") { fprintf(stderr, "ABORT: differences outside the region this tool writes (%s); use EvalTool/cltool with the backup instead\n", f.c_str()); return 1; }
        if (!write) { printf("DRY RUN: nothing written. Re-run with --write.\n"); return 0; }
        bool ok = write_region(b, cur, false);
        printf(ok ? "RESULT: restored and verified\n" : "RESULT: restore NOT confirmed\n"); return ok ? 0 : 1;
    }
    fprintf(stderr, "unknown command %s\n", cmd.c_str()); return 2;
}
