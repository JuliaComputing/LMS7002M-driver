///
/// \file LMS7002M_txtsp.c
///
/// TX DSP chain config for the LMS7002M C driver.
///
/// \copyright
/// Copyright (c) 2014-2015 Fairwaves, Inc.
/// Copyright (c) 2014-2015 Rice University
/// SPDX-License-Identifier: Apache-2.0
/// http://www.apache.org/licenses/LICENSE-2.0
///

#include <stdlib.h>
#include <math.h> //M_PI
#include "LMS7002M_impl.h"

void LMS7002M_txtsp_enable(LMS7002M_t *self, const LMS7002M_chan_t channel, const bool enable)
{
    LMS7002M_set_mac_ch(self, channel);

    self->regs->reg_0x0200_en = enable?1:0;
    self->regs->reg_0x0200_bstart = 0;
    self->regs->reg_0x0200_insel = REG_0X0200_INSEL_LML;
    LMS7002M_regs_spi_write(self, 0x0200);

    self->regs->reg_0x0203_hbi_ovr = REG_0X0203_HBI_OVR_BYPASS;

    self->regs->reg_0x0208_cmix_byp = 1;
    self->regs->reg_0x0208_isinc_byp = 1;
    self->regs->reg_0x0208_gfir3_byp = 1;
    self->regs->reg_0x0208_gfir2_byp = 1;
    self->regs->reg_0x0208_gfir1_byp = 1;
    self->regs->reg_0x0208_dc_byp = 1;
    self->regs->reg_0x0208_gc_byp = 1;
    self->regs->reg_0x0208_ph_byp = 1;

    LMS7002M_regs_spi_write(self, 0x0200);
    LMS7002M_regs_spi_write(self, 0x0203);
    LMS7002M_regs_spi_write(self, 0x0208);
}

void LMS7002M_txtsp_set_interp(LMS7002M_t *self, const LMS7002M_chan_t channel, const size_t interp)
{
    LMS7002M_set_mac_ch(self, channel);

    if (interp == 1) self->regs->reg_0x0203_hbi_ovr = REG_0X0203_HBI_OVR_BYPASS;
    if (interp == 2) self->regs->reg_0x0203_hbi_ovr = 0;
    if (interp == 4) self->regs->reg_0x0203_hbi_ovr = 1;
    if (interp == 8) self->regs->reg_0x0203_hbi_ovr = 2;
    if (interp == 16) self->regs->reg_0x0203_hbi_ovr = 3;
    if (interp == 32) self->regs->reg_0x0203_hbi_ovr = 4;

    LMS7002M_regs_spi_write(self, 0x0203);
}

void LMS7002M_txtsp_set_freq(LMS7002M_t *self, const LMS7002M_chan_t channel, const double freqRel)
{
    LMS7002M_set_mac_ch(self, channel);
    self->regs->reg_0x0208_cmix_byp = (freqRel==0.0)?1:0;
    LMS7002M_regs_spi_write(self, 0x0208);
    LMS7002M_set_nco_freq(self, LMS_TX, channel, freqRel);
}

double LMS7002M_txtsp_get_freq(LMS7002M_t *self, const LMS7002M_chan_t channel)
{
    LMS7002M_set_mac_ch(self, channel);
    LMS7002M_regs_spi_read(self, 0x0208);
    if (self->regs->reg_0x0208_cmix_byp == 1)
        return 0.0;
    return LMS7002M_get_nco_freq(self, LMS_TX, channel);
}


void LMS7002M_txtsp_tsg_const(LMS7002M_t *self, const LMS7002M_chan_t channel, const int valI, const int valQ)
{
    LMS7002M_set_mac_ch(self, channel);

    //muxes
    self->regs->reg_0x0200_tsgfc = REG_0X0200_TSGFC_FS;
    self->regs->reg_0x0200_tsgmode = REG_0X0200_TSGMODE_DC;
    self->regs->reg_0x0200_insel = REG_0X0200_INSEL_TEST;
    LMS7002M_regs_spi_write(self, 0x0200);

    //load I
    self->regs->reg_0x020c_dc_reg = valI;
    LMS7002M_regs_spi_write(self, 0x020c);
    self->regs->reg_0x0200_tsgdcldi = 0;
    LMS7002M_regs_spi_write(self, 0x0200);
    self->regs->reg_0x0200_tsgdcldi = 1;
    LMS7002M_regs_spi_write(self, 0x0200);
    self->regs->reg_0x0200_tsgdcldi = 0;
    LMS7002M_regs_spi_write(self, 0x0200);

    //load Q
    self->regs->reg_0x020c_dc_reg = valQ;
    LMS7002M_regs_spi_write(self, 0x020c);
    self->regs->reg_0x0200_tsgdcldq = 0;
    LMS7002M_regs_spi_write(self, 0x0200);
    self->regs->reg_0x0200_tsgdcldq = 1;
    LMS7002M_regs_spi_write(self, 0x0200);
    self->regs->reg_0x0200_tsgdcldq = 0;
    LMS7002M_regs_spi_write(self, 0x0200);
}

void LMS7002M_txtsp_tsg_tone(LMS7002M_t *self, const LMS7002M_chan_t channel)
{
    LMS7002M_set_mac_ch(self, channel);

    //muxes
    self->regs->reg_0x0200_tsgmode = REG_0X0200_TSGMODE_NCO;
    self->regs->reg_0x0200_insel = REG_0X0200_INSEL_TEST;
    LMS7002M_regs_spi_write(self, 0x0200);

    self->regs->reg_0x0200_tsgfcw = REG_0X0200_TSGFCW_DIV8;
    LMS7002M_regs_spi_write(self, 0x0200);
}

void LMS7002M_txtsp_tsg_tone_div(LMS7002M_t *self, const LMS7002M_chan_t channel, int div)
{
    LMS7002M_set_mac_ch(self, channel);

    //muxes
    self->regs->reg_0x0200_tsgmode = REG_0X0200_TSGMODE_NCO;
    self->regs->reg_0x0200_insel = REG_0X0200_INSEL_TEST;
    LMS7002M_regs_spi_write(self, 0x0200);

    switch(div) {
        case 4: self->regs->reg_0x0200_tsgfcw = REG_0X0200_TSGFCW_DIV4; break;
        default: self->regs->reg_0x0200_tsgfcw = REG_0X0200_TSGFCW_DIV8; break;
    }
    LMS7002M_regs_spi_write(self, 0x0200);
}

void LMS7002M_txtsp_set_dc_correction(
    LMS7002M_t *self,
    const LMS7002M_chan_t channel,
    const double valI,
    const double valQ)
{
    LMS7002M_set_mac_ch(self, channel);

    const bool bypass = (valI == 0.0) && (valQ == 0.0);
    self->regs->reg_0x0208_dc_byp = bypass?1:0;
    LMS7002M_regs_spi_write(self, 0x0208);

    self->regs->reg_0x0204_dccorri = (int)(valI*128);
    self->regs->reg_0x0204_dccorrq = (int)(valQ*128);
    LMS7002M_regs_spi_write(self, 0x0204);
}

void LMS7002M_txtsp_get_dc_correction(
    LMS7002M_t *self,
    const LMS7002M_chan_t channel,
    double *valI,
    double *valQ)
{
    LMS7002M_set_mac_ch(self, channel);

    int dccorr = LMS7002M_spi_read(self, 0x0204);
    *valI = (double)(int8_t)((dccorr >> 8) & 0xff);
    *valQ = (double)(int8_t)((dccorr >> 0) & 0xff);
}

void LMS7002M_txtsp_set_iq_correction(
    LMS7002M_t *self,
    const LMS7002M_chan_t channel,
    const double phase,
    const double gain)
{
    LMS7002M_set_mac_ch(self, channel);

    const bool bypassPhase = (phase == 0.0);
    const bool bypassGain = (gain == 1.0) || (gain == 0.0);
    self->regs->reg_0x0208_ph_byp = bypassPhase?1:0;
    self->regs->reg_0x0208_gc_byp = bypassGain?1:0;
    LMS7002M_regs_spi_write(self, 0x0208);

    self->regs->reg_0x0203_iqcorr = (int)(2047*(phase/(M_PI/2)));
    self->regs->reg_0x0202_gcorri = 2047;
    self->regs->reg_0x0201_gcorrq = 2047;
    if (gain > 1.0) self->regs->reg_0x0201_gcorrq = (int)((1.0/gain)*2047);
    if (gain < 1.0) self->regs->reg_0x0202_gcorri = (int)((gain/1.0)*2047);

    LMS7002M_regs_spi_write(self, 0x0203);
    LMS7002M_regs_spi_write(self, 0x0202);
    LMS7002M_regs_spi_write(self, 0x0201);
}

void LMS7002M_txtsp_get_iq_correction(
    LMS7002M_t *self,
    const LMS7002M_chan_t channel,
    double* phase,
    double* gain)
{
    LMS7002M_set_mac_ch(self, channel);

    LMS7002M_regs_spi_read(self, 0x0203);
    LMS7002M_regs_spi_read(self, 0x0202);
    LMS7002M_regs_spi_read(self, 0x0201);
    LMS7002M_regs_spi_read(self, 0x0208);

    int bypassPhase = self->regs->reg_0x0208_ph_byp;
    int bypassGain = self->regs->reg_0x0208_gc_byp;

    *phase = self->regs->reg_0x0203_iqcorr * (M_PI/2)/2047;
    int gaini = self->regs->reg_0x0202_gcorri;
    int gainq = self->regs->reg_0x0201_gcorrq;

    if (gaini == 2047) {
        *gain = 1.0/(gainq/2047.0);
    } else if (gainq == 2047) {
        *gain = gaini/2047.0;
    } else { // likely maxxed intentionally
        *gain = 1.0;
    }

    if (bypassGain)
        *gain = 1.0; // or 0.0
    if (bypassPhase)
        *phase = 0.0;
    return;
}