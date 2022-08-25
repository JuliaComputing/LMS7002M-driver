
#include "LMS7002M_impl.h"
#include "LMS7002M_mcu_calibration.h"
#include <LMS7002M/LMS7002M_time.h>
#include <LMS7002M/LMS7002M_logger.h>

#include <unistd.h>

#define MCU_PROGRAM_SIZE 16384

#define LMS_MCU_P0_REG      0x0000
#define LMS_MCU_P1_REG      0x0001
#define LMS_MCU_CONTROL_REG 0x0002
#define LMS_MCU_STATUS_REG  0x0003
#define LMS_MCU_DATAWR_REG  0x0004
#define LMS_MCU_DATARD_REG  0x0005
#define LMS_MCU_SWITCH_REG  0x0006

int LMS7002M_mcu_write_program(LMS7002M_t *self, const LMS7002M_mcu_progmode_t mode,
                               const uint8_t* program, size_t program_size)
{
    int i, status;

    if (program_size > MCU_PROGRAM_SIZE)
        return -1;

    // reset MCU, set mode
    LMS7002M_spi_write(self, LMS_MCU_CONTROL_REG, 0);
    if (mode == MCU_EEPROM_SRAM)
        LMS7002M_spi_write(self, LMS_MCU_CONTROL_REG, 1);
    else // mode == LMS_SRAM
        LMS7002M_spi_write(self, LMS_MCU_CONTROL_REG, 2);

    uint16_t bytes_written = 0;
    while (bytes_written < MCU_PROGRAM_SIZE) {
        // wait for buffer to empty
        i = 0;
        while (true) {
            status = LMS7002M_spi_read(self, LMS_MCU_STATUS_REG);
            if (status & 1)
                break;
            if (++i > 100) {
                LMS7_logf(LMS7_ERROR, "Timeout waiting for MCU buffer to empty");
                return -1;
            }
        }

        // write 32 bytes
        // TODO multi-byte transaction
        for (i = 0; i < 32; i++) {
            // pad short programs with zeros
            uint8_t val;
            if (bytes_written < program_size)
                val = program[bytes_written];
            else
                val = 0;

            LMS7002M_spi_write(self, LMS_MCU_DATAWR_REG, val);
            bytes_written += 1;
        }
    }

    // wait for program to be accepted
    i = 0;
    while (true) {
        status = LMS7002M_spi_read(self, LMS_MCU_STATUS_REG);
        if (status & (1<<6))
            break;
        if (++i > 100) {
            LMS7_logf(LMS7_ERROR, "Timeout waiting for MCU buffer to accept program");
            return -1;
        }
    }
    return 0;
}

int LMS7002M_mcu_set_parameter(LMS7002M_t *self, LMS7002M_mcu_param_t param,
                               float value)
{
    const uint8_t control_val = LMS7002M_spi_read(self, LMS_MCU_CONTROL_REG);
    const uint8_t interupt7 = 1<<2;

    // split the parameter in three bytes (kHz LSB, kHz MSB, MHz int)
    uint8_t inputRegs[3];
    value /= 1e6;
    inputRegs[0] = (uint8_t)value; //frequency integer part
    uint16_t fracPart = value * 1000.0 - inputRegs[0]*1000.0;
    inputRegs[1] = (fracPart >> 8) & 0xFF;
    inputRegs[2] = fracPart & 0xFF;

    // shift the values in
    for (uint8_t i = 0; i < 3; ++i) {
        LMS7002M_spi_write(self, LMS_MCU_P0_REG, inputRegs[2-i]);

        // trigger an interrupt
        LMS7002M_spi_write(self, LMS_MCU_CONTROL_REG, control_val | interupt7);
        LMS7002M_spi_write(self, LMS_MCU_CONTROL_REG, control_val & ~interupt7);

        usleep(5);
    }

    // call the update parameter procedures
    if (param==MCU_REF_CLK)
        LMS7002M_mcu_run_procedure(self, 4);
    if (param == MCU_BW)
        LMS7002M_mcu_run_procedure(self, 3);

    int status = LMS7002M_mcu_wait(self, 100);
    if (status != 0) {
        LMS7_logf(LMS7_ERROR, "Could not set MCU parameter (failed with status %d)", status);
        return -1;
    }

    return 0;
}

void LMS7002M_mcu_run_procedure(LMS7002M_t *self, uint8_t procedure)
{
    // switch SPI controls to MCU
    LMS7002M_spi_write(self, LMS_MCU_SWITCH_REG, 1);

    // write procedure ID
    LMS7002M_spi_write(self, 0x0000, procedure);

    // trigger an interrupt
    uint8_t control_val = LMS7002M_spi_read(self, LMS_MCU_CONTROL_REG);
    const uint8_t interupt6 = 1<<3;
    LMS7002M_spi_write(self, LMS_MCU_CONTROL_REG, control_val & ~interupt6);
    LMS7002M_spi_write(self, LMS_MCU_CONTROL_REG, control_val | interupt6);
    LMS7002M_spi_write(self, LMS_MCU_CONTROL_REG, control_val & ~interupt6);

    //MCU seems to be stuck at this point until any SPI operation is performed
    LMS7002M_spi_read(self, LMS_MCU_CONTROL_REG); //random spi action
    usleep(1000);
}

// -1 on timeout, otherwise returns the procedure result
int LMS7002M_mcu_wait(LMS7002M_t *self, unsigned int timeout_ms)
{
    unsigned short value = 0;
    while (timeout_ms > 0) {
        value = LMS7002M_spi_read(self, LMS_MCU_P1_REG) & 0xFF;
        if (value != 0xFF)
            break;

        usleep(1000);
        timeout_ms -= 1;
    }

    // return SPI control to PC
    LMS7002M_spi_write(self, 0x0006, 0);

    if (value == 0xFF)
        return -1;
    else
        return value & 0x7F;
}

int LMS7002M_mcu_write_calibration_program(LMS7002M_t *self)
{
    LMS7002M_mcu_run_procedure(self, MCU_FUNCTION_GET_PROGRAM_ID);
    int mcuID = LMS7002M_mcu_wait(self, 10);

    if (mcuID != MCU_ID_CALIBRATIONS_SINGLE_IMAGE) {
        LMS7_logf(LMS7_DEBUG, "Programming MCU (program ID was %d, expected %d)", mcuID, MCU_ID_CALIBRATIONS_SINGLE_IMAGE);

        LMS7002M_mcu_write_program(self, MCU_SRAM, mcu_program_lms7_dc_iq_calibration_bin, MCU_PROGRAM_SIZE);

        LMS7002M_mcu_run_procedure(self, MCU_FUNCTION_GET_PROGRAM_ID);
        mcuID = LMS7002M_mcu_wait(self, 10);
        if (mcuID != MCU_ID_CALIBRATIONS_SINGLE_IMAGE) {
            LMS7_logf(LMS7_ERROR, "MCU programming failed");
            return -1;
        }
    }

    return 0;
}

const char* status_message(int status)
{
    if (status > MCU_CALIBRATION_STATUS_COUNT)
        return "Unknown error";
    else
        return mcu_calibration_status[status];
}

LMS7002M_API int LMS7002M_mcu_calibration_rx(LMS7002M_t *self, float clk, float bw)
{
    if (LMS7002M_mcu_write_calibration_program(self)) {
        LMS7_logf(LMS7_ERROR, "Failed to write MCU calibration program");
        return -1;
    }

    LMS7002M_mcu_set_parameter(self, MCU_REF_CLK, clk);
    LMS7002M_mcu_set_parameter(self, MCU_BW, bw);
    LMS7002M_mcu_run_procedure(self, 5);

    int status = LMS7002M_mcu_wait(self, 10000);

    int cfb_tia_rfe = LMS7002M_regs(self)->reg_0x0112_cfb_tia_rfe;
    int ccomp_tia_rfe = LMS7002M_regs(self)->reg_0x0112_ccomp_tia_rfe;
    int rcomp_tia_rfe = LMS7002M_regs(self)->reg_0x0114_rcomp_tia_rfe;
    int rcc_ctl_lpfl_rbb = LMS7002M_regs(self)->reg_0x0117_rcc_ctl_lpfl_rbb;
    int c_ctl_lpfl_rbb = LMS7002M_regs(self)->reg_0x0117_c_ctl_lpfl_rbb;
    int rcc_ctl_lpfh_rbb = LMS7002M_regs(self)->reg_0x0116_rcc_ctl_lpfh_rbb;
    LMS7_logf(LMS7_DEBUG, "Rx filter BW: %0.f MHz, CFB_TIA: %i, CCOMP: %i, RCOMP: %i, "
        "RCTL_LPFL: %i, RCTL_LPFH: %i, C_CTL_LPFL: %i",
        bw/1e6, cfb_tia_rfe, ccomp_tia_rfe, rcomp_tia_rfe, rcc_ctl_lpfl_rbb,
        rcc_ctl_lpfh_rbb, c_ctl_lpfl_rbb);

    if (status != 0) {
        LMS7_logf(LMS7_ERROR, "MCU Rx calibration failed: %s", status_message(status));
        return -1;
    }
    return 0;
}

LMS7002M_API int LMS7002M_mcu_calibration_tx(LMS7002M_t *self, float clk, float bw)
{
    if (LMS7002M_mcu_write_calibration_program(self)) {
        LMS7_logf(LMS7_ERROR, "Failed to write MCU calibration program");
        return -1;
    }

    LMS7002M_mcu_set_parameter(self, MCU_REF_CLK, clk);
    LMS7002M_mcu_set_parameter(self, MCU_BW, bw);
    LMS7002M_mcu_run_procedure(self, 6);

    int status = LMS7002M_mcu_wait(self, 10000);

    int rcal_lpflad_tbb = LMS7002M_regs(self)->reg_0x0109_rcal_lpflad_tbb;
    int ccal_lpflad_tbb = LMS7002M_regs(self)->reg_0x010a_ccal_lpflad_tbb;
    int rcal_lpfh_tbb = LMS7002M_regs(self)->reg_0x0109_rcal_lpfh_tbb;
    LMS7_logf(LMS7_DEBUG, "Tx filter BW: %.0f MHz, RCAL_LPFLAD: %i, CCAL_LPFLAD: %i, RCAL_LPFH: %i",
        bw/1e6, rcal_lpflad_tbb, ccal_lpflad_tbb, rcal_lpfh_tbb);

    if (status != MCU_NO_ERROR) {
        LMS7_logf(LMS7_ERROR, "MCU Tx calibration failed: %s", status_message(status));
        return -1;
    }
    return 0;
}
