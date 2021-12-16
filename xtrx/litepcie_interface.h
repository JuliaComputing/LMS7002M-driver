#include "liblitepcie.h"

#include <fcntl.h>
#include <unistd.h>

#define LITEPCIE_SPI_CS_HIGH (0 << 0)
#define LITEPCIE_SPI_CS_LOW  (1 << 0)
#define LITEPCIE_SPI_START   (1 << 0)
#define LITEPCIE_SPI_DONE    (1 << 0)
#define LITEPCIE_SPI_LENGTH  (1 << 8)

static inline uint32_t litepcie_interface_transact(void *handle, const uint32_t data_in, const bool readback)
{
    int *fd = (int *)handle;

    //load tx data
    uint16_t addr = (data_in >> 16) & ((1<<15)-1);
    uint16_t val = data_in & ((1<<16)-1);
    if (data_in & (1<<31)) {
        LMS7_logf(LMS7_TRACE, "SPI request: 0x%08x (write 0x%04x at 0x%04x)", data_in, val, addr);
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
        LMS7_logf(LMS7_TRACE, "SPI reply: 0x%08x (read 0x%04x)", data_out, val);
        return data_out;
    } else {
        return 0;
    }
}
