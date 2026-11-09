// imx5_read_flash_cfg.cpp -- READ-ONLY dump of the IMX-5's DID_FLASH_CONFIG over SPI (one-shot GET_DATA, never writes).
// Usage: imx5_read_flash_cfg [--dev /dev/spidev0.0] [--speed 3000000]      (stop imx5_spi_node first: one SPI owner)
// Prints the IMU-relevant fields: startupImuDtMs/NavDtMs, ioConfig, sensorConfig decoded (FS ranges, gyro/acc DLPF
// override fields, rotation) and a raw hex dump of the first bytes.
// Build: build_imx5_read_flash_cfg.sh
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <cstdlib>
#include <string>
#include <vector>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include "ISComm.h"
#include "data_sets.h"

static int fd = -1; static uint32_t speed_hz = 3000000;
static int xfer(const uint8_t *tx, uint8_t *rx, size_t n) {
    spi_ioc_transfer tr; memset(&tr, 0, sizeof(tr));
    tr.tx_buf = (unsigned long)tx; tr.rx_buf = (unsigned long)rx; tr.len = (uint32_t)n; tr.speed_hz = speed_hz; tr.bits_per_word = 8;
    return ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
}
static void fl16(const uint8_t *d, size_t n, uint8_t *a, uint8_t *b) { uint8_t x = 0, y = 0; for (size_t i = 0; i < n; ++i) { x += d[i]; y += x; } *a = x; *b = y; }

int main(int argc, char **argv) {
    std::string dev = "/dev/spidev0.0";
    for (int i = 1; i < argc; ++i) { std::string a = argv[i]; if (a == "--dev" && i + 1 < argc) dev = argv[++i]; else if (a == "--speed" && i + 1 < argc) speed_hz = atol(argv[++i]); }
    fd = open(dev.c_str(), O_RDWR); if (fd < 0) { perror("open"); return 1; }
    uint8_t mode = SPI_MODE_3, bits = 8;
    if (ioctl(fd, SPI_IOC_WR_MODE, &mode) < 0 || ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0 || ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed_hz) < 0) { perror("spi"); return 1; }
    static uint8_t cbuf[8192]; is_comm_instance_t comm; is_comm_init(&comm, cbuf, sizeof(cbuf), nullptr);
    is_comm_enable_protocol(&comm, _PTYPE_INERTIAL_SENSE_DATA); is_comm_set_protocol_mask(&comm, ENABLE_PROTOCOL_ISB);
    {   uint8_t b[16] = {0xEF, 0x49, 6, 0, 0, 0}; fl16(b, 6, &b[6], &b[7]); uint8_t t[256]; memset(t, 0xFF, 256); xfer(b, t, 8); usleep(50000);
        uint8_t dr[2048]; memset(dr, 0xFF, sizeof(dr)); xfer(dr, dr, sizeof(dr)); }
    const int N = 512; uint8_t tx[N], rx[N];
    bool done = false;
    for (int attempt = 0; attempt < 5 && !done; ++attempt) {
        memset(tx, 0xFF, N);
        if (is_comm_get_data_to_buf(tx, N, &comm, DID_FLASH_CONFIG, 0, 0, 0) <= 0) { fprintf(stderr, "request build failed\n"); return 1; }
        for (int f = 0; f < 12 && !done; ++f) {          // reply may be larger than one frame: keep clocking idle frames
            if (xfer(tx, rx, N) < 0) { perror("xfer"); return 1; }
            memset(tx, 0xFF, N);
            int fr = is_comm_free(&comm), nc = fr < N ? fr : N; memcpy(comm.rxBuf.tail, rx, nc); comm.rxBuf.tail += nc;
            protocol_type_t pt;
            while ((pt = is_comm_parse(&comm)) != _PTYPE_NONE) {
                if (pt != _PTYPE_INERTIAL_SENSE_DATA || comm.rxPkt.dataHdr.id != DID_FLASH_CONFIG) continue;
                const uint8_t *p = comm.rxPkt.data.ptr; const uint32_t len = comm.rxPkt.dataHdr.size;
                nvm_flash_cfg_t cfg; memset(&cfg, 0, sizeof(cfg)); memcpy(&cfg, p, len < sizeof(cfg) ? len : sizeof(cfg));
                printf("DID_FLASH_CONFIG received: %u bytes (sizeof struct %zu)\n", len, sizeof(cfg));
                printf("  size=%u checksum=0x%08x\n", cfg.size, cfg.checksum);
                printf("  startupImuDtMs=%u  startupNavDtMs=%u\n", cfg.startupImuDtMs, cfg.startupNavDtMs);
                printf("  ioConfig=0x%08x\n", cfg.ioConfig);
                uint32_t s = cfg.sensorConfig;
                static const int gyfs[] = {250, 500, 1000, 2000, 4000};
                static const char *gdl[] = {"250", "184", "92", "41", "20", "10", "5"};
                static const char *adl[] = {"218", "218b", "99", "45", "21", "10", "5"};
                static const int acfs[] = {2, 4, 8, 16, 32};
                uint32_t gf = s & 7, af = (s >> 4) & 7, gd = (s >> 8) & 0xF, ad = (s >> 12) & 0xF, rot = (s >> 16) & 0x1F;
                printf("  sensorConfig=0x%08x\n", s);
                printf("    gyro FS field=%u (%s)   accel FS field=%u (%s)\n", gf, gf < 5 ? (std::to_string(gyfs[gf]) + " deg/s").c_str() : "max range", af, af < 5 ? (std::to_string(acfs[af]) + " g").c_str() : "max range");
                printf("    gyro DLPF field=%u (%s Hz if it is an override)   accel DLPF field=%u (%s Hz if an override)\n", gd, gd < 7 ? gdl[gd] : "?", ad, ad < 7 ? adl[ad] : "?");
                printf("    sensor rotation field=%u   mag disabled=%d   baro disabled=%d   fault-detect bits=0x%02x\n", rot, !!(s & 0x00400000), !!(s & 0x00800000), s >> 24);
                printf("  first 64 raw bytes:"); for (uint32_t i = 0; i < 64 && i < len; ++i) printf("%s%02x", i % 16 ? " " : "\n   ", p[i]); printf("\n");
                done = true; break;
            }
        }
    }
    if (!done) { fprintf(stderr, "no DID_FLASH_CONFIG packet received\n"); return 2; }
    close(fd); return 0;
}
