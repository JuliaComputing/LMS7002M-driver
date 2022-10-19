///
/// \file LMS7002M_vco.c
///
/// Generalized VCO tuning algorithms
///
/// \copyright
/// Copyright (c) 2016-2016 Skylark Wireless
/// SPDX-License-Identifier: Apache-2.0
/// http://www.apache.org/licenses/LICENSE-2.0
///

#include <stdlib.h>
#include "LMS7002M_impl.h"
#include "LMS7002M_vco.h"
#include <LMS7002M/LMS7002M_logger.h>
#include <unistd.h>
#include <string.h>

static void LMS7002M_read_vco_cmp(LMS7002M_t *self, const int vco_cmp_addr)
{
    //an initial read forces spi writes to be flushed
    //any read will do, the address does not matter
    LMS7002M_regs_spi_read(self, vco_cmp_addr);

    //sleep while the comparator value settles
    usleep(1000);

    //final read of the comparator after settling
    LMS7002M_regs_spi_read(self, vco_cmp_addr);
}

/*!
 * Sweep though the 7 lower CSW positions given a starting point.
 * This function is invoked twice to sweep upper and lower ranges.
 */
static int LMS7002M_tune_vco_sweep(
    LMS7002M_t *self,
    int *vco_csw_reg,
    const int vco_csw_addr,
    int *vco_cmpho_reg,
    int *vco_cmplo_reg,
    const int vco_cmp_addr,
    const int start_point,
    int *lo, int *hi
)
{
    int csw_lowest = start_point+128;
    *vco_csw_reg = start_point;
    for (int i = 6; i >= 0; i--)
    {
        *vco_csw_reg |= 1 << i;
        LMS7002M_regs_spi_write(self, vco_csw_addr);
        LMS7002M_read_vco_cmp(self, vco_cmp_addr);

        LMS7_logf(LMS7_DEBUG, "i=%d, hi=%d, lo=%d", i, *vco_cmpho_reg, *vco_cmplo_reg);
        if (*vco_cmplo_reg != 0)
        {
            *vco_csw_reg &= ~(1 << i); //clear bit i
        }
        if (*vco_cmpho_reg != 0 && *vco_cmplo_reg == 0 && *vco_csw_reg < csw_lowest)
        {
            csw_lowest = *vco_csw_reg;
        }
        LMS7002M_regs_spi_write(self, vco_csw_addr);
    }

    //find the midpoint for the high and low bounds
    int csw_highest = *vco_csw_reg;
    while (csw_lowest <= csw_highest && csw_lowest > start_point)
    {
        csw_lowest--;
        *vco_csw_reg = csw_lowest;
        LMS7002M_regs_spi_write(self, vco_csw_addr);
        LMS7002M_read_vco_cmp(self, vco_cmp_addr);

        if (*vco_cmpho_reg != 0 && *vco_cmplo_reg == 0) continue;
        csw_lowest++;
        break;
    }

    *lo = csw_lowest;
    *hi = csw_highest;
    LMS7_logf(LMS7_DEBUG, "lowest CSW_VCO %i, highest CSW_VCO %i", csw_lowest, csw_highest);
    return 0;
}

void scan_csw(LMS7002M_t *self, int dir, int *vco_csw_reg, int *csw_arr, int *vco_cmpho_reg, int *vco_cmplo_reg, const int vco_csw_addr, const int vco_cmp_addr){
    int init = (dir > 0) ? 0 : 255;
    for (int i = 0; i < 256; i++){
        *vco_csw_reg = init + i*dir;
        LMS7002M_regs_spi_write(self, vco_csw_addr);
        LMS7002M_read_vco_cmp(self, vco_cmp_addr);
        csw_arr[*vco_csw_reg] = *vco_cmplo_reg | *vco_cmpho_reg << 1;
    }
}

int LMS7002M_tune_vco(
    LMS7002M_t *self,
    int *vco_csw_reg,
    const int vco_csw_addr,
    int *vco_cmpho_reg,
    int *vco_cmplo_reg,
    const int vco_cmp_addr
)
{
    int csw_ascending[256];
    int csw_decending[256];

    scan_csw(self, 1, vco_csw_reg, csw_ascending, vco_cmpho_reg, vco_cmplo_reg, vco_csw_addr, vco_cmp_addr);
    
    if (LMS7_get_log_level() >= LMS7_DEBUG){

        char output[1024] = {'\0'};
        // Scan the comparators in the opposite direction to check for hyseresis
        scan_csw(self, -1, vco_csw_reg, csw_decending, vco_cmpho_reg, vco_cmplo_reg, vco_csw_addr, vco_cmp_addr);

        strcat(output, "Ascending comparator vals:\n");
        for (int i = 0; i < 256; i++)
        {
            char* c = csw_ascending[i] == 0 ? "0" : csw_ascending[i] == 2 ? "2" : "3";
            strcat(output+strlen(output), c);
        }
        LMS7_log(LMS7_DEBUG, output);
        memset(output, '\0', sizeof(output));
        strcat(output, "Decending comparator vals:\n");
        for (int i = 0; i < 256; i++)
        {
            char* c = csw_decending[i] == 0 ? "0" : csw_decending[i] == 2 ? "2" : "3";
            strcat(output+strlen(output), c);
        }
        LMS7_log(LMS7_DEBUG, output);
    }

    int high_ct = 0;
    int low_ct = 0;
    for (int i = 0; i < 256; i++) {
        if (csw_ascending[i] == 3) high_ct++;
        if (csw_ascending[i] == 0) low_ct++;
    }
    LMS7_logf(LMS7_DEBUG, "high_ct=%i, low_ct=%i\n", high_ct, low_ct);

    if (high_ct == 256)
    {
        LMS7_log(LMS7_DEBUG, "VCO select FAIL - too high");
        return -1;
    } else if (low_ct == 256)
    {
        LMS7_log(LMS7_DEBUG, "VCO select FAIL - too low");
        return -1;
    }

    int csw_lowest = 0;
    int csw_highest = 0;
    int csw_lowest_fallback = 0;
    int csw_highest_fallback = 0;
    /*
    We need to find the boundary of the deadband range for the CSW register.
    Though the samples may be noisy or have "stuck" values at the start,
    so we try to be robust searching for a sequence like the following:
    002222233
    We in effect debounce the comparator a bit here.
    If the first search does not find a reasonable CSW register, we will
    fallback to a less strict check search and print a warning.
    */
    for (int i = 0; i < 252; i++)
    {
        if (csw_ascending[i] == 0 && csw_ascending[i+1] == 0 && csw_ascending[i+2] == 2 && csw_ascending[i+3] == 2)
            csw_lowest = i+2;
        else if (csw_ascending[i] == 2 && csw_ascending[i+1] == 2 && csw_ascending[i+2] == 3 && csw_ascending[i+3] == 3)
            csw_highest = i+1;

        if (csw_ascending[i] == 0 && csw_ascending[i+1] == 2 && csw_ascending[i+2] == 2)
            csw_lowest_fallback = i+1;
        else if (csw_ascending[i] == 2 && csw_ascending[i+1] == 3 && csw_ascending[i+2] == 3)
            csw_highest_fallback = i;
    }

    //set the midpoint of the search
    *vco_csw_reg = (csw_highest+csw_lowest)/2;
    LMS7002M_regs_spi_write(self, vco_csw_addr);

    //check that the vco selection was successful
    LMS7002M_read_vco_cmp(self, vco_cmp_addr);

    if (*vco_cmpho_reg != 0 && *vco_cmplo_reg == 0)
    {
        LMS7_logf(LMS7_DEBUG, "lowest CSW_VCO %i, highest CSW_VCO %i, CSW_VCO %i", csw_lowest, csw_highest, *vco_csw_reg);
    }
    else
    {
        //set the midpoint of the search
        *vco_csw_reg = (csw_highest_fallback+csw_lowest_fallback)/2;
        LMS7002M_regs_spi_write(self, vco_csw_addr);
        LMS7_logf(LMS7_DEBUG, "Warning: We used a less strict algorithm to find the ideal VCO.");
        LMS7_logf(LMS7_DEBUG, "lowest CSW_VCO %i, highest CSW_VCO %i, CSW_VCO %i", csw_lowest_fallback, csw_highest_fallback, *vco_csw_reg);

        //check that the vco selection was successful
        LMS7002M_read_vco_cmp(self, vco_cmp_addr);
    }

    if (*vco_cmpho_reg != 0 && *vco_cmplo_reg == 0)
    {
        LMS7_log(LMS7_DEBUG, "VCO OK");
    }
    else
    {
        LMS7_log(LMS7_DEBUG, "tune vco: VCO select FAIL");
        return -1;
    }
    return 0;
}
