// imx5_read_pimu.cpp -- READ-ONLY: what rate does the IMX-5 really deliver DID_PIMU (and DID_IMU) at?
// Reads DID_SYS_PARAMS (imuSamplePeriodMs, navUpdatePeriodMs, navOutputPeriodMs, sensorTruePeriod), then one-shot-polls
// DID_PIMU (and DID_IMU) every 1 ms for --secs and reports the spacing of the unique samples (their own .time) and pimu.dt.
// Usage: imx5_read_pimu [--secs 5] [--pimu-csv FILE] [--raw-csv FILE] [--stream N]   (--stream N: ONE broadcast request for DID_PIMU at periodMultiple N, then idle frames; PIMU only) [--speed 3000000] [--dev /dev/spidev0.0]        (stop imx5_spi_node first: one SPI owner)
// Build: build_imx5_read_pimu.sh
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <cstdlib>
#include <cmath>
#include <ctime>
#include <string>
#include <vector>
#include <map>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include "ISComm.h"
#include "data_sets.h"

static int fd = -1; static uint32_t speed_hz = 3000000;
static double now_s() { timespec t; clock_gettime(CLOCK_MONOTONIC, &t); return t.tv_sec + t.tv_nsec * 1e-9; }
static int xfer(const uint8_t *tx, uint8_t *rx, size_t n) {
    spi_ioc_transfer tr; memset(&tr, 0, sizeof(tr));
    tr.tx_buf = (unsigned long)tx; tr.rx_buf = (unsigned long)rx; tr.len = (uint32_t)n; tr.speed_hz = speed_hz; tr.bits_per_word = 8;
    return ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
}
static void fl16(const uint8_t *d, size_t n, uint8_t *a, uint8_t *b) { uint8_t x = 0, y = 0; for (size_t i = 0; i < n; ++i) { x += d[i]; y += x; } *a = x; *b = y; }

struct Acc { std::vector<double> t; std::vector<double> dt; };

int main(int argc, char **argv) {
    std::string dev = "/dev/spidev0.0", pcsv, rcsv; int secs = 5, stream = 0; uint64_t repeats = 0;
    for (int i = 1; i < argc; ++i) { std::string a = argv[i];
        if (a == "--dev" && i + 1 < argc) dev = argv[++i]; else if (a == "--speed" && i + 1 < argc) speed_hz = atol(argv[++i]); else if (a == "--secs" && i + 1 < argc) secs = atoi(argv[++i]); else if (a == "--stream" && i + 1 < argc) stream = atoi(argv[++i]); else if (a == "--pimu-csv" && i + 1 < argc) pcsv = argv[++i]; else if (a == "--raw-csv" && i + 1 < argc) rcsv = argv[++i]; }
    FILE *pf = pcsv.empty() ? nullptr : fopen(pcsv.c_str(), "w"), *rf = rcsv.empty() ? nullptr : fopen(rcsv.c_str(), "w");
    if (pf) fprintf(pf, "time,dt,status,tx,ty,tz,vx,vy,vz\n"); if (rf) fprintf(rf, "time,gx,gy,gz,ax,ay,az\n");
    fd = open(dev.c_str(), O_RDWR); if (fd < 0) { perror("open"); return 1; }
    uint8_t mode = SPI_MODE_3, bits = 8;
    if (ioctl(fd, SPI_IOC_WR_MODE, &mode) < 0 || ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0 || ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed_hz) < 0) { perror("spi"); return 1; }
    static uint8_t cbuf[8192]; is_comm_instance_t comm; is_comm_init(&comm, cbuf, sizeof(cbuf), nullptr);
    is_comm_enable_protocol(&comm, _PTYPE_INERTIAL_SENSE_DATA); is_comm_set_protocol_mask(&comm, ENABLE_PROTOCOL_ISB);
    {   uint8_t b[16] = {0xEF, 0x49, 6, 0, 0, 0}; fl16(b, 6, &b[6], &b[7]); uint8_t t[256]; memset(t, 0xFF, 256); xfer(b, t, 8); usleep(50000);
        uint8_t dr[2048]; memset(dr, 0xFF, sizeof(dr)); xfer(dr, dr, sizeof(dr)); }
    const int N = 256; uint8_t tx[N], rx[N];
    Acc pimu, imu; bool have_sys = false; uint64_t polls = 0;
    const double t0 = now_s();
    while (now_s() - t0 < secs) {
        memset(tx, 0xFF, N);
        if (!stream) {
            int off = is_comm_get_data_to_buf(tx, N, &comm, DID_PIMU, 0, 0, 0);
            off += is_comm_get_data_to_buf(tx + off, N - off, &comm, DID_IMU_RAW, 0, 0, 0);
            if (!have_sys) is_comm_get_data_to_buf(tx + off, N - off, &comm, DID_SYS_PARAMS, 0, 0, 0);
        } else if (polls == 0) {   // one broadcast request, then only idle 0xFF frames
            int off = is_comm_get_data_to_buf(tx, N, &comm, DID_PIMU, 0, 0, stream);
            is_comm_get_data_to_buf(tx + off, N - off, &comm, DID_SYS_PARAMS, 0, 0, 0);
        }
        if (xfer(tx, rx, N) < 0) { perror("xfer"); return 1; }
        ++polls;
        int fr = is_comm_free(&comm), nc = fr < N ? fr : N; memcpy(comm.rxBuf.tail, rx, nc); comm.rxBuf.tail += nc;
        protocol_type_t pt;
        while ((pt = is_comm_parse(&comm)) != _PTYPE_NONE) {
            if (pt != _PTYPE_INERTIAL_SENSE_DATA) continue;
            const uint8_t *p = comm.rxPkt.data.ptr; uint32_t id = comm.rxPkt.dataHdr.id;
            if (id == DID_SYS_PARAMS && !have_sys) {
                sys_params_t sp; memset(&sp, 0, sizeof(sp)); memcpy(&sp, p, comm.rxPkt.dataHdr.size < sizeof(sp) ? comm.rxPkt.dataHdr.size : sizeof(sp));
                printf("DID_SYS_PARAMS: imuSamplePeriodMs=%u  navUpdatePeriodMs=%u  navOutputPeriodMs=%u  sensorTruePeriod=%.9f s\n", sp.imuSamplePeriodMs, sp.navUpdatePeriodMs, sp.navOutputPeriodMs, sp.sensorTruePeriod);
                have_sys = true;
            } else if (id == DID_PIMU) {
                pimu_t m; memset(&m, 0, sizeof(m)); memcpy(&m, p, comm.rxPkt.dataHdr.size < sizeof(m) ? comm.rxPkt.dataHdr.size : sizeof(m));
                if (!pimu.t.empty() && m.time == pimu.t.back()) ++repeats;
                if (pimu.t.empty() || m.time > pimu.t.back()) { pimu.t.push_back(m.time); pimu.dt.push_back(m.dt); if (pf) fprintf(pf, "%.6f,%.6f,%u,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g\n", m.time, m.dt, m.status, m.theta[0], m.theta[1], m.theta[2], m.vel[0], m.vel[1], m.vel[2]); }
            } else if (id == DID_IMU_RAW) {
                imu_t m; memset(&m, 0, sizeof(m)); memcpy(&m, p, comm.rxPkt.dataHdr.size < sizeof(m) ? comm.rxPkt.dataHdr.size : sizeof(m));
                if (imu.t.empty() || m.time > imu.t.back()) { imu.t.push_back(m.time); if (rf) fprintf(rf, "%.6f,%.7g,%.7g,%.7g,%.7g,%.7g,%.7g\n", m.time, m.I.pqr[0], m.I.pqr[1], m.I.pqr[2], m.I.acc[0], m.I.acc[1], m.I.acc[2]); }
            }
        }
        usleep(800);
    }
    double el = now_s() - t0;
    auto report = [&](const char *name, const Acc &a, bool has_dt) {
        printf("%s: %zu unique samples in %.1f s = %.1f Hz (polls %llu)\n", name, a.t.size(), el, a.t.size() / el, (unsigned long long)polls);
        if (a.t.size() < 3) return;
        std::map<int, long> h;
        for (size_t i = 1; i < a.t.size(); ++i) h[(int)std::lround((a.t[i] - a.t[i - 1]) * 1000.0)]++;
        printf("   spacing of unique sample times (ms:count):"); for (auto &kv : h) printf("  %d:%ld", kv.first, kv.second); printf("\n");
        double span = a.t.back() - a.t.front(); printf("   mean spacing %.3f ms over %.2f s\n", 1e3 * span / (a.t.size() - 1), span);
        if (has_dt) { double mn = 1e9, mx = 0; for (double d : a.dt) { mn = std::min(mn, d); mx = std::max(mx, d); } printf("   pimu.dt field: min %.4f ms  max %.4f ms\n", mn * 1e3, mx * 1e3); }
    };
    report("DID_PIMU", pimu, true); if (stream) printf("   repeated PIMU samples (same time seen again): %llu\n", (unsigned long long)repeats);
    if (stream) {   // stop the broadcast and drain, leave the IMX in its normal state
        uint8_t b[16] = {0xEF, 0x49, 6, 0, 0, 0}; fl16(b, 6, &b[6], &b[7]); uint8_t t[256]; memset(t, 0xFF, 256); xfer(b, t, 8); usleep(50000);
        uint8_t dr[2048]; for (int i = 0; i < 4; ++i) { memset(dr, 0xFF, sizeof(dr)); xfer(dr, dr, sizeof(dr)); }
    } report("DID_IMU_RAW", imu, false);
    if (pf) fclose(pf); if (rf) fclose(rf);
    close(fd); return 0;
}
