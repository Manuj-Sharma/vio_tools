// imx5_bit_test.cpp -- IMX-5 built-in self-test (DID_BIT) over SPI, with a stationarity pre-check.
//   imx5_bit_test status                       READ-ONLY: read DID_BIT and decode (no command sent)
//   imx5_bit_test run [--cmd stationary|stationary-hi|basic] [--go] [--force] [--timeout 60]
//        DRY RUN unless --go. Pre-check: reads DID_IMU_RAW for 3 s and refuses (unless --force) if the rig is not still
//        (gyro std > 0.02 rad/s, or | |acc| - 9.81 | > 0.6 m/s^2). Then SET_DATA(DID_BIT, command byte), polls DID_BIT until
//        state==DONE, prints hdwBitStatus / calBitStatus / pqr / acc / sigmas decoded. On timeout sends BIT_CMD_OFF.
//   common: [--dev /dev/spidev0.0] [--speed 3000000]   stop imx5_spi_node (and imx5_noros_poll) first: ONE SPI owner.
// DID_BIT is a RAM/runtime command: it does NOT write the IMX flash config. Build: build_imx5_bit_test.sh
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <cstdlib>
#include <cmath>
#include <ctime>
#include <string>
#include <vector>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include "ISComm.h"
#include "data_sets.h"

static int fd = -1; static uint32_t speed_hz = 3000000;
static is_comm_instance_t comm; static uint8_t cbuf[8192];
static double now_s() { timespec t; clock_gettime(CLOCK_MONOTONIC, &t); return t.tv_sec + t.tv_nsec * 1e-9; }
static int xfer(const uint8_t *tx, uint8_t *rx, size_t n) {
    spi_ioc_transfer tr; memset(&tr, 0, sizeof(tr));
    tr.tx_buf = (unsigned long)tx; tr.rx_buf = (unsigned long)rx; tr.len = (uint32_t)n; tr.speed_hz = speed_hz; tr.bits_per_word = 8;
    return ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
}
static void fl16(const uint8_t *d, size_t n, uint8_t *a, uint8_t *b) { uint8_t x = 0, y = 0; for (size_t i = 0; i < n; ++i) { x += d[i]; y += x; } *a = x; *b = y; }
static bool proc_running(const char *name) { std::string c = std::string("pgrep -x ") + name + " >/dev/null 2>&1"; return system(c.c_str()) == 0; }

// feed received bytes to the parser; copy the payload of the first packet with DID `want` (if want != 0) into out
static bool pump(const uint8_t *rx, int n, uint32_t want, void *out, size_t outsz) {
    int fr = is_comm_free(&comm), nc = fr < n ? fr : n; memcpy(comm.rxBuf.tail, rx, nc); comm.rxBuf.tail += nc;
    protocol_type_t pt; bool got = false;
    while ((pt = is_comm_parse(&comm)) != _PTYPE_NONE) {
        if (pt != _PTYPE_INERTIAL_SENSE_DATA || !want || comm.rxPkt.dataHdr.id != want) continue;
        size_t sz = comm.rxPkt.dataHdr.size < outsz ? comm.rxPkt.dataHdr.size : outsz; memset(out, 0, outsz); memcpy(out, comm.rxPkt.data.ptr, sz); got = true;
    }
    return got;
}
static bool request(uint32_t did, void *out, size_t outsz, double timeout_s) {
    const int N = 256; uint8_t tx[N], rx[N]; double t0 = now_s();
    while (now_s() - t0 < timeout_s) {
        memset(tx, 0xFF, N); is_comm_get_data_to_buf(tx, N, &comm, did, 0, 0, 0);
        if (xfer(tx, rx, N) < 0) { perror("xfer"); return false; }
        if (pump(rx, N, did, out, outsz)) return true; usleep(2000);
    }
    return false;
}
struct Fl { uint32_t m; const char *n; };
static void decode(uint32_t v, const Fl *t, int n, const char *what) {
    bool any = false; for (int i = 0; i < n; ++i) if (v & t[i].m) { printf("      %s: %s\n", what, t[i].n); any = true; } if (!any) printf("      %s: (none)\n", what);
}
static const Fl HDW_FAIL[] = {{0x100,"FAULT_NOISE_PQR (gyro noise too high)"},{0x200,"FAULT_NOISE_ACC (accel noise too high)"},{0x400,"FAULT_MAGNETOMETER"},{0x800,"FAULT_BAROMETER"},
    {0x1000,"GNSS_NO_COM"},{0x2000,"GNSS_POOR_CNO"},{0x4000,"GNSS_POOR_ACCURACY"},{0x8000,"GNSS_NOISE"},{0x10000,"IMU_FAULT_REJECTION"},{0x01000000,"INCORRECT_HARDWARE_TYPE"},{0x02000000,"EKF_NOT_INITIALIZED"}};
static const Fl CAL_FAIL[] = {{0x100,"TCAL_EMPTY"},{0x200,"TCAL_TSPAN"},{0x400,"TCAL_INCONSISTENT"},{0x800,"TCAL_CORRUPT"},{0x1000,"TCAL_PQR_BIAS"},{0x2000,"TCAL_PQR_SLOPE"},{0x4000,"TCAL_PQR_LIN"},
    {0x8000,"TCAL_ACC_BIAS"},{0x10000,"TCAL_ACC_SLOPE"},{0x20000,"TCAL_ACC_LIN"},{0x40000,"CAL_SERIAL_NUM"}};
static void show(const bit_t &b) {
    printf("  DID_BIT: lastCommand=%u state=%u (0 off, 1 done, 6 running, 7 finishing) testMode=%u hwId=%u\n", b.lastCommand, b.state, b.testMode, b.detectedHardwareId);
    printf("    hdwBitStatus=0x%08x  passed=%s%s  mode=%u\n", b.hdwBitStatus, (b.hdwBitStatus & 1) ? "ALL" : ((b.hdwBitStatus & 2) ? "NO_GNSS" : "NO"), "", (unsigned)((b.hdwBitStatus >> 4) & 0xF));
    decode(b.hdwBitStatus & 0xFFFFFF00u, HDW_FAIL, sizeof(HDW_FAIL) / sizeof(Fl), "hdw FAIL");
    printf("    calBitStatus=0x%08x  passed=%s  mode=%u\n", b.calBitStatus, (b.calBitStatus & 1) ? "ALL" : "NO", (unsigned)((b.calBitStatus >> 4) & 0xF));
    decode(b.calBitStatus & 0x00FFFF00u, CAL_FAIL, sizeof(CAL_FAIL) / sizeof(Fl), "cal FAIL");
    printf("    gyro error pqr=%.6f rad/s (sigma %.6f)   accel error acc=%.6f m/s^2 (sigma %.6f)\n", b.pqr, b.pqrSigma, b.acc, b.accSigma);
    printf("    tcal bias pqr=%.6g acc=%.6g  slope pqr=%.6g acc=%.6g  linearity pqr=%.6g acc=%.6g\n", b.tcPqrBias, b.tcAccBias, b.tcPqrSlope, b.tcAccSlope, b.tcPqrLinearity, b.tcAccLinearity);
}

int main(int argc, char **argv) {
    if (argc < 2) { fprintf(stderr, "usage: see the header of imx5_bit_test.cpp\n"); return 2; }
    std::string mode = argv[1], dev = "/dev/spidev0.0", cmdname = "stationary"; bool go = false, force = false; double tmo = 60;
    for (int i = 2; i < argc; ++i) { std::string a = argv[i];
        if (a == "--dev" && i + 1 < argc) dev = argv[++i]; else if (a == "--speed" && i + 1 < argc) speed_hz = atol(argv[++i]);
        else if (a == "--cmd" && i + 1 < argc) cmdname = argv[++i]; else if (a == "--timeout" && i + 1 < argc) tmo = atof(argv[++i]);
        else if (a == "--go") go = true; else if (a == "--force") force = true; else { fprintf(stderr, "bad arg %s\n", a.c_str()); return 2; } }
    uint8_t cmd = cmdname == "stationary" ? BIT_CMD_FULL_STATIONARY : cmdname == "stationary-hi" ? BIT_CMD_FULL_STATIONARY_HIGH_ACCURACY : cmdname == "basic" ? BIT_CMD_BASIC_MOVING : 0;
    if (mode != "status" && mode != "run") { fprintf(stderr, "mode must be status|run\n"); return 2; }
    if (mode == "run" && !cmd) { fprintf(stderr, "--cmd must be stationary|stationary-hi|basic\n"); return 2; }
    if (proc_running("imx5_spi_node") || proc_running("imx5_noros_poll") || proc_running("imx5_poll_rate")) { fprintf(stderr, "ABORT: another SPI owner is running (imx5_spi_node / imx5_noros_poll / imx5_poll_rate). Stop it first.\n"); return 3; }
    fd = open(dev.c_str(), O_RDWR); if (fd < 0) { perror("open"); return 1; }
    uint8_t m = SPI_MODE_3, bits = 8;
    if (ioctl(fd, SPI_IOC_WR_MODE, &m) < 0 || ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0 || ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed_hz) < 0) { perror("spi"); return 1; }
    is_comm_init(&comm, cbuf, sizeof(cbuf), nullptr); is_comm_enable_protocol(&comm, _PTYPE_INERTIAL_SENSE_DATA); is_comm_set_protocol_mask(&comm, ENABLE_PROTOCOL_ISB);
    {   uint8_t b[16] = {0xEF, 0x49, 6, 0, 0, 0}; fl16(b, 6, &b[6], &b[7]); uint8_t t[256]; memset(t, 0xFF, 256); xfer(b, t, 8); usleep(50000);   // stop any broadcast, drain
        uint8_t dr[2048]; memset(dr, 0xFF, sizeof(dr)); xfer(dr, dr, sizeof(dr)); }

    bit_t b; memset(&b, 0, sizeof(b));
    if (!request(DID_BIT, &b, sizeof(b), 3.0)) { fprintf(stderr, "could not read DID_BIT (timeout)\n"); return 1; }
    printf("BEFORE:\n"); show(b);
    if (mode == "status") return 0;

    // stationarity pre-check from DID_IMU_RAW for 3 s
    std::vector<double> g[3], am; double t0 = now_s(), lastt = -1;
    while (now_s() - t0 < 3.0) { imu_t s; if (!request(DID_IMU_RAW, &s, sizeof(s), 0.5)) continue; if (s.time == lastt) continue; lastt = s.time;
        for (int k = 0; k < 3; ++k) g[k].push_back(s.I.pqr[k]); am.push_back(std::sqrt(s.I.acc[0]*s.I.acc[0] + s.I.acc[1]*s.I.acc[1] + s.I.acc[2]*s.I.acc[2])); usleep(1000); }
    if (am.size() < 50) { fprintf(stderr, "pre-check: only %zu IMU samples in 3 s, cannot judge stillness\n", am.size()); if (!force) return 1; }
    double gsd = 0, amean = 0; for (double x : am) amean += x; amean /= std::max<size_t>(am.size(), 1);
    for (int k = 0; k < 3; ++k) { double mu = 0; for (double x : g[k]) mu += x; mu /= std::max<size_t>(g[k].size(), 1); double v = 0; for (double x : g[k]) v += (x - mu) * (x - mu); v /= std::max<size_t>(g[k].size(), 1); gsd = std::max(gsd, std::sqrt(v)); }
    bool still = gsd <= 0.02 && std::fabs(amean - 9.81) <= 0.6;
    printf("PRE-CHECK (%zu samples/3 s): worst gyro std %.5f rad/s (limit 0.02), mean |acc| %.3f m/s^2 (9.81 +/- 0.6)  => %s\n", am.size(), gsd, amean, still ? "STILL" : "NOT STILL");
    if (!still && !force) { fprintf(stderr, "ABORT: rig is not still enough for the stationary test (use --force to override, or --cmd basic).\n"); return 4; }
    printf("PLAN: SET_DATA(DID_BIT, size=1, offset=0, command=%u [%s]), then poll until state==DONE (timeout %.0f s). No flash write.\n", cmd, cmdname.c_str(), tmo);
    if (!go) { printf("DRY RUN: nothing sent. Re-run with --go to run the test.\n"); return 0; }

    const int N = 512; uint8_t tx[N], rx[N]; memset(tx, 0xFF, N);
    int n = is_comm_set_data_to_buf(tx, N, &comm, DID_BIT, 1, 0, &cmd);
    if (n <= 0) { fprintf(stderr, "could not build SET_DATA packet\n"); return 1; }
    if (xfer(tx, rx, N) < 0) { perror("xfer"); return 1; } pump(rx, N, 0, nullptr, 0);
    printf("command sent at %.1f s; polling...\n", now_s() - t0);
    double ts = now_s(); bool done = false; bit_t r; memset(&r, 0, sizeof(r));
    while (now_s() - ts < tmo) {
        usleep(500000); if (!request(DID_BIT, &r, sizeof(r), 1.5)) continue;
        printf("  t=%4.1f s state=%u lastCommand=%u hdw=0x%08x cal=0x%08x\n", now_s() - ts, r.state, r.lastCommand, r.hdwBitStatus, r.calBitStatus);
        if (r.state == BIT_STATE_DONE && r.lastCommand == cmd) { done = true; break; }
    }
    if (!done) { uint8_t off = BIT_CMD_OFF; memset(tx, 0xFF, N); if (is_comm_set_data_to_buf(tx, N, &comm, DID_BIT, 1, 0, &off) > 0) xfer(tx, rx, N);
        fprintf(stderr, "TIMEOUT: test did not report DONE in %.0f s; sent BIT_CMD_OFF.\n", tmo); return 5; }
    printf("AFTER:\n"); show(r);
    const uint32_t IGNORE = 0x0000F000u | 0x02000000u;   // GNSS_* (no GPS on the bench) and EKF_NOT_INITIALIZED are not IMU faults
    bool pass = (r.hdwBitStatus & 3) != 0 && (r.calBitStatus & 1) != 0 && !((r.hdwBitStatus & 0xFFFFFF00u) & ~IGNORE) && !(r.calBitStatus & 0x00FFFF00u);
    printf("RESULT: %s\n", pass ? "PASS (hardware + calibration BIT passed; GNSS/EKF flags ignored on a no-GPS bench rig)" : "NOT CLEAN -- see the non-GNSS flags above");
    close(fd); return pass ? 0 : 6;
}
