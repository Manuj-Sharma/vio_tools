// imx5_poll_rate.cpp -- how many of the IMX-5's 1 kHz DID_IMU_RAW samples can we actually capture over SPI?
//
// ROS-free. Polls GET_DATA(DID_IMU_RAW) in a clock_nanosleep loop, DEDUPES by imu_t.time (so polling faster
// than the IMX tick is fine), and reports the spacing of consecutive unique IMX samples: dt ~1 ms = no loss,
// dt ~2 ms = one sample missed, etc. "captured %" = unique samples / ticks elapsed in the IMX's own time.
//
//   imx5_poll_rate [--period-us 500] [--speed 6000000] [--len 128] [--secs 20] [--retry-us 0 --retry-max 1]
//                  [--stream N: IMX broadcast with periodMultiple=N instead of one-shot polls] [--rt N] [--spin] [--dev /dev/spidev0.0] [--dump FILE]
//   --spin   busy-wait the last ~100 us before each poll (needs a spare core, tighter timing than nanosleep)
//   --dump   CSV, one line per unique sample: imx_time_s, host_mono_s_at_read_end, gx,gy,gz (rad/s), ax,ay,az (m/s^2)
// Needs the SPI device to itself: stop imx5_spi_node / imx5_noros_poll first.
// Build: build_imx5_poll_rate.sh
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <cstdlib>
#include <cmath>
#include <ctime>
#include <csignal>
#include <string>
#include <vector>
#include <map>
#include <algorithm>
#include <fcntl.h>
#include <unistd.h>
#include <sched.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>

#include "ISComm.h"
#include "data_sets.h"

static volatile sig_atomic_t g_stop = 0;
static void on_sig(int) { g_stop = 1; }
static double now_s() { timespec ts; clock_gettime(CLOCK_MONOTONIC, &ts); return ts.tv_sec + ts.tv_nsec * 1e-9; }

static int fd = -1;
static uint32_t speed_hz = 3000000;   // datasheet max for fixed-size polling is 3 Mbps
static int spi_xfer(const uint8_t *tx, uint8_t *rx, size_t n) {
    spi_ioc_transfer tr; memset(&tr, 0, sizeof(tr));
    tr.tx_buf = (unsigned long)tx; tr.rx_buf = (unsigned long)rx; tr.len = (uint32_t)n;
    tr.speed_hz = speed_hz; tr.bits_per_word = 8;
    return ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
}
static void fletcher16(const uint8_t *d, size_t n, uint8_t *a_out, uint8_t *b_out) {
    uint8_t a = 0, b = 0;
    for (size_t i = 0; i < n; ++i) { a = (uint8_t)(a + d[i]); b = (uint8_t)(b + a); }
    *a_out = a; *b_out = b;
}

int main(int argc, char **argv) {
    int period_us = 500, len = 128, secs = 20, retry_us = 0, retry_max = 1, rt = 0;
    bool spin = false; int stream = 0;
    std::string dev = "/dev/spidev0.0", dump;
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        auto nx = [&]() { return i + 1 < argc ? argv[++i] : (char *)"0"; };
        if (a == "--period-us") period_us = atoi(nx());
        else if (a == "--speed") speed_hz = (uint32_t)atol(nx());
        else if (a == "--len") len = atoi(nx());
        else if (a == "--secs") secs = atoi(nx());
        else if (a == "--retry-us") retry_us = atoi(nx());
        else if (a == "--retry-max") retry_max = atoi(nx());
        else if (a == "--rt") rt = atoi(nx());
        else if (a == "--spin") spin = true;
        else if (a == "--stream") stream = atoi(nx());
        else if (a == "--dev") dev = nx();
        else if (a == "--dump") dump = nx();
        else { fprintf(stderr, "unknown arg %s\n", a.c_str()); return 2; }
    }
    signal(SIGINT, on_sig); signal(SIGTERM, on_sig);
    fd = open(dev.c_str(), O_RDWR);
    if (fd < 0) { perror("open spidev"); return 1; }
    uint8_t mode = SPI_MODE_3, bits = 8;
    if (ioctl(fd, SPI_IOC_WR_MODE, &mode) < 0 || ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0 ||
        ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed_hz) < 0) { perror("spi ioctl"); return 1; }
    if (rt > 0) { sched_param sp{}; sp.sched_priority = rt; if (sched_setscheduler(0, SCHED_FIFO, &sp) < 0) perror("sched_setscheduler (needs sudo)"); }

    static uint8_t cbuf[4096];
    is_comm_instance_t comm;
    is_comm_init(&comm, cbuf, sizeof(cbuf), nullptr);
    is_comm_enable_protocol(&comm, _PTYPE_INERTIAL_SENSE_DATA);
    is_comm_set_protocol_mask(&comm, ENABLE_PROTOCOL_ISB);
    {   // stop broadcasts + drain, as the node does
        uint8_t b[16] = {0xEF, 0x49, 6, 0, 0, 0}; fletcher16(b, 6, &b[6], &b[7]);
        uint8_t thr[256]; memset(thr, 0xFF, sizeof(thr)); spi_xfer(b, thr, 8); usleep(50000);
        uint8_t dr[2048]; memset(dr, 0xFF, sizeof(dr)); spi_xfer(dr, dr, sizeof(dr));
    }
    printf("# period=%d us speed=%u Hz len=%d B (frame %.3f ms) retry=%d us x%d rt=%d spin=%d stream=%d secs=%d\n", period_us, speed_hz, len,
           len * 8.0 / speed_hz * 1e3, retry_us, retry_max, rt, (int)spin, stream, secs);

    FILE *df = dump.empty() ? nullptr : fopen(dump.c_str(), "w");
    if (df) fprintf(df, "imx_t,host_t,gx,gy,gz,ax,ay,az\n");
    std::vector<uint8_t> tx(len), rx(len);
    std::vector<double> imx_t;            // unique IMX sample times
    imx_t.reserve((size_t)secs * 1100 + 1000);
    long polls = 0, empty = 0, repeats = 0, retries = 0, recov = 0;
    double last_imx = -1, maxgap = 0, last_start = 0, spi_max = 0;
    const double period = period_us * 1e-6, t0 = now_s();
    double next = t0 + period;

    auto parse_rx = [&](double t_rd) {
        int fr = is_comm_free(&comm), nc = fr < len ? fr : len;
        memcpy(comm.rxBuf.tail, rx.data(), nc); comm.rxBuf.tail += nc;
        int fresh = 0; protocol_type_t pt;
        while ((pt = is_comm_parse(&comm)) != _PTYPE_NONE) {
            if (pt != _PTYPE_INERTIAL_SENSE_DATA || comm.rxPkt.dataHdr.id != DID_IMU_RAW) continue;
            const imu_t *im = (const imu_t *)comm.rxPkt.data.ptr;
            if (im->time == last_imx) { ++repeats; continue; }
            if (im->time < last_imx) continue;
            last_imx = im->time; imx_t.push_back(im->time); ++fresh;
            if (df) fprintf(df, "%.6f,%.6f,%.7f,%.7f,%.7f,%.6f,%.6f,%.6f\n", im->time, t_rd, im->I.pqr[0], im->I.pqr[1], im->I.pqr[2], im->I.acc[0], im->I.acc[1], im->I.acc[2]);
        }
        return fresh;
    };

    while (!g_stop && now_s() - t0 < secs) {
        timespec ts; ts.tv_sec = (time_t)next; ts.tv_nsec = (long)((next - (time_t)next) * 1e9);
        if (spin) { double w = next - 100e-6; timespec tw{(time_t)w, (long)((w - (time_t)w) * 1e9)}; clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &tw, nullptr); while (now_s() < next) {} }
        else clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, nullptr);
        double start = now_s();
        if (last_start > 0) maxgap = std::max(maxgap, start - last_start);
        last_start = start;
        next += period;
        if (next < start) next = start + period - fmod(start - next, period);

        memset(tx.data(), 0xFF, len);
        if (!stream || polls == 0) {   // stream mode: ask for the broadcast once, then clock out idle 0xFF frames
            int req = is_comm_get_data_to_buf(tx.data(), len, &comm, DID_IMU_RAW, 0, 0, stream);
            if (req <= 0) { fprintf(stderr, "request build failed\n"); return 1; }
        }
        double s0 = now_s();
        if (spi_xfer(tx.data(), rx.data(), len) < 0) { perror("spi_xfer"); return 1; }
        double s1 = now_s(); spi_max = std::max(spi_max, s1 - s0);
        ++polls;
        int got = parse_rx(s1);
        for (int r = 0; !got && r < retry_max && retry_us > 0; ++r) {
            timespec rs{0, (long)retry_us * 1000L}; nanosleep(&rs, nullptr);
            if (spi_xfer(tx.data(), rx.data(), len) < 0) { perror("spi_xfer retry"); return 1; }
            ++retries; got = parse_rx(now_s()); if (got) ++recov;
        }
        if (!got) ++empty;
    }
    const double el = now_s() - t0;

    // spacing of consecutive unique IMX samples, in whole ticks (round(dt / 1 ms))
    std::map<int, long> hist; long big = 0;
    for (size_t i = 1; i < imx_t.size(); ++i) {
        int k = (int)std::lround((imx_t[i] - imx_t[i - 1]) * 1000.0);
        if (k > 10) ++big; else ++hist[k];
    }
    double span = imx_t.size() > 1 ? imx_t.back() - imx_t.front() : 0;
    long ticks = (long)std::lround(span * 1000.0);
    printf("unique samples : %zu in %.1f s host = %.1f Hz\n", imx_t.size(), el, imx_t.size() / el);
    printf("IMX-time span  : %.3f s -> %ld ticks at 1 kHz, captured %.2f%%  (missed %ld)\n", span, ticks,
           ticks ? 100.0 * (imx_t.size() - 1) / ticks : 0.0, ticks - (long)imx_t.size() + 1);
    printf("polls=%ld empty=%ld (%.1f%%) repeats(same sample again)=%ld retries=%ld recovered=%ld maxPollGap=%.2f ms maxSpi=%.2f ms\n",
           polls, empty, 100.0 * empty / std::max(1L, polls), repeats, retries, recov, maxgap * 1e3, spi_max * 1e3);
    printf("dt histogram (ms between consecutive unique IMX samples):");
    for (auto &kv : hist) printf("  %dms:%ld", kv.first, kv.second);
    if (big) printf("  >10ms:%ld", big);
    printf("\n");
    if (stream) {   // leave the IMX in poll mode for the node: stop broadcasts, drain
        uint8_t b[16] = {0xEF, 0x49, 6, 0, 0, 0}; fletcher16(b, 6, &b[6], &b[7]);
        uint8_t thr[256]; memset(thr, 0xFF, sizeof(thr)); spi_xfer(b, thr, 8); usleep(50000);
        uint8_t dr[2048]; for (int i = 0; i < 4; ++i) { memset(dr, 0xFF, sizeof(dr)); spi_xfer(dr, dr, sizeof(dr)); }
    }
    if (df) fclose(df);
    close(fd);
    return 0;
}
