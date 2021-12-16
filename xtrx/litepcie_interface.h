#include "liblitepcie.h"

#include <fcntl.h>
#include <unistd.h>

#define LITEPCIE_SPI_CS_HIGH (0 << 0)
#define LITEPCIE_SPI_CS_LOW  (1 << 0)
#define LITEPCIE_SPI_START   (1 << 0)
#define LITEPCIE_SPI_DONE    (1 << 0)
#define LITEPCIE_SPI_LENGTH  (1 << 8)

const char *bitstring(int x, int N_bits){
    static char b[512];
    char *p = b;
    b[0] = '\0';

    for(int i=(N_bits-1); i>=0; i--){
      if (i < N_bits-1 && !((i+1)%8))
        *p++ = ' ';
      *p++ = (x & (1<<i)) ? '1' : '0';
    }
    return b;
}

static inline uint32_t litepcie_interface_transact(void *handle, const uint32_t data_in, const bool readback)
{
    int *fd = (int *)handle;

    //load tx data
    uint16_t addr = (data_in >> 16) & ((1<<15)-1);
    uint16_t val = data_in & ((1<<16)-1);
    if (data_in & (1<<31)) {
        LMS7_logf(LMS7_TRACE, "SPI request: 0x%08x (write 0x%04x, or %s, at 0x%04x)", data_in, val, bitstring(val, 16), addr);
    } else {
        LMS7_logf(LMS7_TRACE, "SPI request: 0x%08x (read from 0x%04x)", data_in, addr);
    }
    litepcie_writel(*fd, CSR_LMS7002M_SPI_MOSI_ADDR, data_in);

    //start transaction
    litepcie_writel(*fd, CSR_LMS7002M_SPI_CONTROL_ADDR, 32*LITEPCIE_SPI_LENGTH | LITEPCIE_SPI_START);

    //wait for completion
    while ((litepcie_readl(*fd, CSR_LMS7002M_SPI_STATUS_ADDR) & LITEPCIE_SPI_DONE) == 0);

    //load rx data
    if (readback) {
        uint32_t data_out = litepcie_readl(*fd, CSR_LMS7002M_SPI_MISO_ADDR) & 0xffff;
        val = data_out & ((1<<16)-1);
        LMS7_logf(LMS7_TRACE, "SPI reply: 0x%08x (read 0x%04x, or %s)", data_out, val, bitstring(val, 16));
        return data_out;
    } else {
        return 0;
    }
}
