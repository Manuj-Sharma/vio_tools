// imx5_noros_poll.cpp -- ROS-free twin of imx5_spi_node's poll loop.
//
// Same SDK parser, same GET_DATA(DID_IMU_RAW) one-shot request, same 512 B full-duplex frame, SPI
// mode 3, but a plain clock_nanosleep(TIMER_ABSTIME) loop: no rclcpp, no executor, no DDS publish.
// Question it answers: do the ~30 s sag (333 -> ~283 Hz) and the 130 Hz collapse come from ROS
// overhead, or from the IMX / SPI side?
//
//   imx5_noros_poll [--period-us 3000] [--speed 3000000] [--len 512] [--secs 120]
//                   [--all-protocols] [--dev /dev/spidev0.0] [--rt N] [--csv FILE]
//                   [--retry-us N [--retry-max M]]   re-poll N us after an EMPTY poll (up to M times, default 1)
//
// Every 5 s it prints: unique IMU samples/s (the number comparable to the node's "/imu0 N Hz"),
// polls, polls with no packet ("empty"), polls that started >1.5 ms after schedule ("late"),
// worst gap between poll starts, worst SPI ioctl time, and where in the 512 B frame the first
// 0xEF 0x49 preamble sat: "early" (<32 B = pipelined reply to the previous request),
// "mid" (32..len-100), "late" (>= len-100, truncation zone), "none".
// Needs the SPI device to itself: stop imx5_spi_node first.
//
// Build:  see build_imx5_noros_poll.sh
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <cstdlib>
#include <cmath>
#include <ctime>
#include <csignal>
#include <string>
#include <vector>
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

static double now_s() {
    timespec ts; clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + ts.tv_nsec * 1e-9;
}
static void fletcher16(const uint8_t *d, size_t n, uint8_t *a_out, uint8_t *b_out) {
    uint8_t a = 0, b = 0;
    for (size_t i = 0; i < n; ++i) { a = (uint8_t)(a + d[i]); b = (uint8_t)(b + a); }
    *a_out = a; *b_out = b;
}

static int fd = -1;
static uint32_t speed_hz = 3000000;

static int spi_xfer(const uint8_t *tx, uint8_t *rx, size_t len) {
    spi_ioc_transfer tr; memset(&tr, 0, sizeof(tr));
    tr.tx_buf = (unsigned long)tx; tr.rx_buf = (unsigned long)rx;
    tr.len = (uint32_t)len; tr.speed_hz = speed_hz; tr.bits_per_word = 8; tr.cs_change = 0;
    return ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
}

int main(int argc, char **argv) {
    int period_us = 3000, len = 512, secs = 120, rt = 0, retry_us = 0, retry_max = 1;
    bool all_protocols = false;
    std::string dev = "/dev/spidev0.0", csv;
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        auto nx = [&]() { return i + 1 < argc ? argv[++i] : (char *)"0"; };
        if (a == "--period-us") period_us = atoi(nx());
        else if (a == "--speed") speed_hz = (uint32_t)atol(nx());
        else if (a == "--len") len = atoi(nx());
        else if (a == "--secs") secs = atoi(nx());
        else if (a == "--rt") rt = atoi(nx());
        else if (a == "--retry-us") retry_us = atoi(nx());
        else if (a == "--retry-max") retry_max = atoi(nx());
        else if (a == "--dev") dev = nx();
        else if (a == "--csv") csv = nx();
        else if (a == "--all-protocols") all_protocols = true;
        else { fprintf(stderr, "unknown arg %s\n", a.c_str()); return 2; }
    }
    signal(SIGINT, on_sig); signal(SIGTERM, on_sig);

    fd = open(dev.c_str(), O_RDWR);
    if (fd < 0) { perror("open spidev"); return 1; }
    uint8_t mode = SPI_MODE_3, bits = 8;
    if (ioctl(fd, SPI_IOC_WR_MODE, &mode) < 0 || ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0 ||
        ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed_hz) < 0) { perror("spi ioctl"); return 1; }
    if (rt > 0) {
        sched_param sp{}; sp.sched_priority = rt;
        if (sched_setscheduler(0, SCHED_FIFO, &sp) < 0) perror("sched_setscheduler (needs sudo)");
    }

    static uint8_t cbuf[4096];
    is_comm_instance_t comm;
    is_comm_init(&comm, cbuf, sizeof(cbuf), nullptr);
    is_comm_enable_protocol(&comm, _PTYPE_INERTIAL_SENSE_DATA);
    if (!all_protocols) is_comm_set_protocol_mask(&comm, ENABLE_PROTOCOL_ISB);

    // stop any broadcasts + drain, as the node does
    {
        uint8_t b[16]; b[0] = 0xEF; b[1] = 0x49; b[2] = 6; b[3] = 0; b[4] = 0; b[5] = 0;
        fletcher16(b, 6, &b[6], &b[7]);
        uint8_t thr[256]; memset(thr, 0xFF, sizeof(thr));
        spi_xfer(b, thr, 8); usleep(50000);
        uint8_t dr[2048]; memset(dr, 0xFF, sizeof(dr)); spi_xfer(dr, dr, sizeof(dr));
    }
    printf("# period=%d us  speed=%u Hz  len=%d B (frame=%.3f ms)  protocols=%s  rt=%d  retry=%d us x%d\n", period_us,
           speed_hz, len, len * 8.0 / speed_hz * 1e3, all_protocols ? "default(ISB+NMEA+UBX+RTCM3)" : "ISB only", rt, retry_us, retry_max);
    printf("# %-6s %8s %6s %6s %6s %8s %8s %6s %6s | first-preamble in frame: %5s %5s %5s %5s\n", "t(s)", "Hz", "polls",
           "empty", "late", "gap_ms", "spi_ms", "retry", "recov", "early", "mid", "late", "none");
    FILE *cf = csv.empty() ? nullptr : fopen(csv.c_str(), "w");
    if (cf) fprintf(cf, "t,hz,polls,empty,late,maxgap_ms,spi_max_ms,retries,recovered,early,mid,late,none\n");

    std::vector<uint8_t> tx(len), rx(len);
    const double period = period_us * 1e-6;
    const double t0 = now_s();
    double next = t0 + period, last_start = 0, bin_t = t0;
    long retries = 0, recov = 0, polls = 0, empty = 0, late = 0, pk = 0, e_early = 0, e_mid = 0, e_late = 0, e_none = 0;
    double maxgap = 0, spi_max = 0;
    uint64_t tot_pk = 0, tot_polls = 0, tot_empty = 0;
    double last_imx_t = -1, first_imx_t = -1; long dup = 0;

    while (!g_stop && now_s() - t0 < secs) {
        timespec ts; ts.tv_sec = (time_t)next; ts.tv_nsec = (long)((next - (time_t)next) * 1e9);
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, nullptr);
        double start = now_s();
        if (start - next > 0.0015) ++late;
        if (last_start > 0) maxgap = std::max(maxgap, start - last_start);
        last_start = start;
        next += period;
        if (next < start) next = start + period - fmod(start - next, period);   // skip missed ticks

        memset(tx.data(), 0xFF, len);
        int req = is_comm_get_data_to_buf(tx.data(), len, &comm, DID_IMU_RAW, 0, 0, 0);
        if (req <= 0) { fprintf(stderr, "request build failed\n"); return 1; }
        double s0 = now_s();
        if (spi_xfer(tx.data(), rx.data(), len) < 0) { perror("spi_xfer"); return 1; }
        spi_max = std::max(spi_max, now_s() - s0);

        int off = -1;
        for (int i = 0; i + 1 < len; ++i) if (rx[i] == 0xEF && rx[i + 1] == 0x49) { off = i; break; }
        if (off < 0) ++e_none; else if (off < 32) ++e_early; else if (off < len - 100) ++e_mid; else ++e_late;

        auto parse_rx = [&]() {
            int fr = is_comm_free(&comm), nc = fr < len ? fr : len;
            memcpy(comm.rxBuf.tail, rx.data(), nc); comm.rxBuf.tail += nc;
            int n = 0;
            protocol_type_t pt;
            while ((pt = is_comm_parse(&comm)) != _PTYPE_NONE) {
                if (pt != _PTYPE_INERTIAL_SENSE_DATA) continue;
                if (comm.rxPkt.dataHdr.id != DID_IMU_RAW) continue;
                const imu_t *im = (const imu_t *)comm.rxPkt.data.ptr;
                if (im->time == last_imx_t) ++dup;   // same sample seen twice
                last_imx_t = im->time;
                if (first_imx_t < 0) {
                    first_imx_t = im->time;
                    double up = 0; if (FILE *u = fopen("/proc/uptime", "r")) { if (fscanf(u, "%lf", &up) != 1) up = 0; fclose(u); }
                    printf("# IMX uptime (imu_t.time) at first packet = %.1f s   Jetson uptime = %.1f s   -> IMX is %.1f s older than the kernel (~20-30 s = bootloader time is normal; hours = IMX survived the reboot)\n", im->time, up, im->time - up);
                }
                ++n;
            }
            return n;
        };
        int got = parse_rx();
        // optional retry: an empty poll is re-polled after retry_us (the reply may just be late)
        for (int r = 0; !got && r < retry_max && retry_us > 0; ++r) {
            timespec rs{0, (long)retry_us * 1000L}; nanosleep(&rs, nullptr);
            if (spi_xfer(tx.data(), rx.data(), len) < 0) { perror("spi_xfer retry"); return 1; }
            ++retries;
            got = parse_rx();
            if (got) ++recov;
        }
        ++polls; ++tot_polls; pk += got; tot_pk += got;
        if (!got) { ++empty; ++tot_empty; }

        double t = now_s();
        if (t - bin_t >= 5.0) {
            double el = t - bin_t;
            printf("  %-6.0f %8.1f %6ld %6ld %6ld %8.2f %8.2f %6ld %6ld | %43ld %5ld %5ld %5ld\n", t - t0, pk / el, polls, empty,
                   late, maxgap * 1e3, spi_max * 1e3, retries, recov, e_early, e_mid, e_late, e_none);
            if (cf) fprintf(cf, "%.1f,%.1f,%ld,%ld,%ld,%.2f,%.2f,%ld,%ld,%ld,%ld,%ld,%ld\n", t - t0, pk / el, polls, empty, late,
                            maxgap * 1e3, spi_max * 1e3, retries, recov, e_early, e_mid, e_late, e_none);
            fflush(stdout);
            bin_t = t; retries = recov = polls = empty = late = pk = e_early = e_mid = e_late = e_none = 0; maxgap = spi_max = 0;
        }
    }
    double el = now_s() - t0;
    printf("# done: %.0f s, %llu packets (%.1f Hz mean), %llu polls, %llu empty (%.1f%%), dup-samples=%ld, parse errs=%u, pkts=%u\n",
           el, (unsigned long long)tot_pk, tot_pk / el, (unsigned long long)tot_polls, (unsigned long long)tot_empty,
           100.0 * tot_empty / std::max<uint64_t>(1, tot_polls), dup, comm.rxErrorCount, comm.rxPktCount);
    if (cf) fclose(cf);
    close(fd);
    return 0;
}
