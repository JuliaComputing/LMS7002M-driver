///
/// \file LMS7002M_trf.c
///
/// TX RF frontend controls for the LMS7002M C driver.
///
/// \copyright
/// Copyright (c) 2015-2016 Fairwaves, Inc.
/// Copyright (c) 2015-2016 Rice University
/// SPDX-License-Identifier: Apache-2.0
/// http://www.apache.org/licenses/LICENSE-2.0
///

#include <stdlib.h>
#include <math.h> //exp
#include "LMS7002M_impl.h"

void LMS7002M_trf_enable(LMS7002M_t *self, const LMS7002M_chan_t channel, const bool enable)
{
    LMS7002M_set_mac_ch(self, channel);
    self->regs->reg_0x0124_en_dir_trf = 1;
    LMS7002M_regs_spi_write(self, 0x0124);

    self->regs->reg_0x0100_en_g_trf = enable?1:0;
    self->regs->reg_0x0100_pd_tlobuf_trf = enable?0:1;
    self->regs->reg_0x0100_pd_txpad_trf = enable?0:1;
    LMS7002M_regs_spi_write(self, 0x0100);

    //the chB LO enable register is a chA register
    if (channel != LMS_CHA)
    {
        LMS7002M_set_mac_ch(self, LMS_CHA);
        self->regs->reg_0x0100_en_nexttx_trf = enable?1:0;
        LMS7002M_regs_spi_write(self, 0x0100);
    }
}

void LMS7002M_trf_select_band(LMS7002M_t *self, const LMS7002M_chan_t channel, const int band)
{
    LMS7002M_set_mac_ch(self, channel);
    self->regs->reg_0x0103_sel_band1_trf = (band == 1)?1:0;
    self->regs->reg_0x0103_sel_band2_trf = (band == 2)?1:0;
    LMS7002M_regs_spi_write(self, 0x0103);
}

int LMS7002M_trf_get_band(LMS7002M_t *self, const LMS7002M_chan_t channel)
{
    LMS7002M_set_mac_ch(self, channel);
    LMS7002M_regs_spi_read(self, 0x0103);
    return self->regs->reg_0x0103_sel_band2_trf << 1 | self->regs->reg_0x0103_sel_band1_trf;
}

void LMS7002M_trf_enable_loopback(LMS7002M_t *self, const LMS7002M_chan_t channel, const bool enable)
{
    LMS7002M_set_mac_ch(self, channel);
    self->regs->reg_0x0101_en_loopb_txpad_trf = enable?1:0;
    LMS7002M_regs_spi_write(self, 0x0101);
}

int LMS7002M_trf_get_loopback(LMS7002M_t *self, const LMS7002M_chan_t channel)
{
    LMS7002M_set_mac_ch(self, channel);
    LMS7002M_regs_spi_read(self, 0x0101);
    return self->regs->reg_0x0101_en_loopb_txpad_trf;
}

double LMS7002M_trf_set_pad(LMS7002M_t *self, const LMS7002M_chan_t channel, const double gain)
{
    const double pmax = 0;
    double loss = pmax-gain;

    //different scaling realm
    if (loss > 10) loss = (loss+10)/2;

    //clip
    if (loss > 31) loss = 31;
    if (loss < 0) loss = 0;

    //integer round
    int loss_int = (int)(loss + 0.5);

    LMS7002M_set_mac_ch(self, channel);
    self->regs->reg_0x0101_loss_lin_txpad_trf = loss_int;
    self->regs->reg_0x0101_loss_main_txpad_trf = loss_int;
    LMS7002M_regs_spi_write(self, 0x0101);

    if (loss_int > 10) return pmax-10-2*(loss_int-10);
    return pmax-loss_int;
}

double LMS7002M_trf_get_pad(LMS7002M_t *self, const LMS7002M_chan_t channel)
{
    LMS7002M_set_mac_ch(self, channel);
    LMS7002M_regs_spi_read(self, 0x0101);
    const int loss_int = self->regs->reg_0x0101_loss_lin_txpad_trf;

    if (loss_int > 10) return -10-2*(loss_int-10);
    return -loss_int;
}


double LMS7002M_trf_set_loopback_pad(LMS7002M_t *self, const LMS7002M_chan_t channel, const double gain)
{
    //there are 4 discrete gain values, use the midpoints
    double actual = 0.0;
    int val = 0;
    if      (gain >= (-1.4-0)/2)   val = 0, actual = 0.0;
    else if (gain >= (-1.4-3.3)/2) val = 1, actual = -1.4;
    else if (gain >= (-3.3-4.3)/2) val = 2, actual = -3.3;
    else                           val = 3, actual = -4.3;

    LMS7002M_set_mac_ch(self, channel);
    self->regs->reg_0x0101_l_loopb_txpad_trf = val;
    LMS7002M_regs_spi_write(self, 0x0101);

    return actual;
}

double LMS7002M_trf_get_loopback_pad(LMS7002M_t *self, const LMS7002M_chan_t channel)
{

    LMS7002M_set_mac_ch(self, channel);
    LMS7002M_regs_spi_read(self, 0x0101);
    const int val = self->regs->reg_0x0101_l_loopb_txpad_trf;
    if (val == 0) return 0.0;
    else if (val == 1) return -1.4;
    else if (val == 2) return -3.3;
    else return -4.3;
}
