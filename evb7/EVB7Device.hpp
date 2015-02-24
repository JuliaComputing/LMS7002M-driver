//
// SoapySDR wrapper for the LMS7002M driver.
//
// Copyright (c) 2015-2015 Fairwaves, Inc.
// Copyright (c) 2015-2015 Rice University
// SPDX-License-Identifier: Apache-2.0
// http://www.apache.org/licenses/LICENSE-2.0
//

#include "EVB7Regs.hpp"
#include "twbw_helper.h"
#include "spidev_interface.h"
#include "sysfs_gpio_interface.h"
#include "xilinx_user_gpio.h"
#include "xilinx_user_mem.h"

#include <SoapySDR/Device.hpp>
#include <SoapySDR/Logger.hpp>
#include <pothos_zynq_dma_driver.h>
#include <cstring>
#include <cstdlib>
#include <stdexcept>
#include <iostream>

#include <LMS7002M/LMS7002M.h>
#include <LMS7002M/LMS7002M_impl.h>

class EVB7 : public SoapySDR::Device
{
public:
    EVB7(void);

    ~EVB7(void);

/*******************************************************************
* Identification API
******************************************************************/
    std::string getDriverKey(void) const
    {
        return "EVB7";
    }

    std::string getHardwareKey(void) const
    {
        return "EVB7";
    }

/*******************************************************************
* Channels API
******************************************************************/
    size_t getNumChannels(const int) const
    {
        return 1;
    }

    bool getFullDuplex(const int, const size_t) const
    {
        return true;
    }

/*******************************************************************
* Stream API
******************************************************************/

    SoapySDR::Stream *setupStream(
        const int direction,
        const std::string &format,
        const std::vector<size_t> &channels,
        const SoapySDR::Kwargs &);

    void rxFlush(void);

    void closeStream(SoapySDR::Stream *stream);

    size_t getStreamMTU(SoapySDR::Stream *stream) const;

    int sendControlMessage(const int tag, const bool timeFlag, const bool burstFlag, const int frameSize, const int burstSize, const long long time);

    int convertRemainder(void *outp, const size_t numOutSamps, int &flags);

    int activateStream(
        SoapySDR::Stream *stream,
        const int flags,
        const long long timeNs,
        const size_t numElems);

    int deactivateStream(
        SoapySDR::Stream *stream,
        const int flags,
        const long long timeNs);

    int readStream(
        SoapySDR::Stream *,
        void * const *buffs,
        const size_t numElems,
        int &flags,
        long long &timeNs,
        const long timeoutUs);

    int writeStream(
        SoapySDR::Stream *,
        const void * const *buffs,
        const size_t numElems,
        int &flags,
        const long long timeNs,
        const long timeoutUs
    );

    int readStreamStatus(
        SoapySDR::Stream *stream,
        size_t &chanMask,
        int &flags,
        long long &timeNs,
        const long timeoutUs);

    size_t getNumDirectAccessBuffers(SoapySDR::Stream *stream);

    int acquireReadBuffer(
        SoapySDR::Stream *stream,
        size_t &handle,
        const void **buffs,
        int &flags,
        long long &timeNs,
        const long timeoutUs);

    void releaseReadBuffer(
        SoapySDR::Stream *stream,
        const size_t handle);

    int acquireWriteBuffer(
        SoapySDR::Stream *stream,
        size_t &handle,
        void **buffs,
        const long timeoutUs);

    void releaseWriteBuffer(
        SoapySDR::Stream *stream,
        const size_t handle,
        const size_t numElems,
        int &flags,
        const long long timeNs);

    //rx streaming
    int _remainderHandle;
    size_t _remainderSamps;
    const uint32_t *_remainderBuff;

    //tx streaming
    bool _userHandlesTxStatus;

    //stream configuration
    enum StreamFormat
    {
        SF_CS16,
        SF_CF32,
    };
    StreamFormat _rxFormat;
    StreamFormat _txFormat;

    /*******************************************************************
     * Antenna API
     ******************************************************************/

    /*******************************************************************
     * Frontend corrections API
     ******************************************************************/

    /*******************************************************************
     * Gain API
     ******************************************************************/

    /*******************************************************************
     * Frequency API
     ******************************************************************/
    void setFrequency(const int direction, const size_t channel, const double frequency, const SoapySDR::Kwargs &args);

    double getFrequency(const int direction, const size_t channel) const;

    double getFrequency(const int direction, const size_t channel, const std::string &name) const;

    std::vector<std::string> listFrequencies(const int, const size_t) const;

    SoapySDR::RangeList getFrequencyRange(const int, const size_t) const;

    std::map<int, double> _cachedLOFrequencies;
    std::map<int, std::map<size_t, double> > _cachedBBFrequencies;

    /*******************************************************************
     * Sample Rate API
     ******************************************************************/
    void setSampleRate(const int direction, const size_t, const double rate);

    double getSampleRate(const int direction, const size_t) const;

    std::vector<double> listSampleRates(const int direction, const size_t) const;

    std::map<int, double> _cachedSampleRates;

    /*******************************************************************
     * Clocking API
     ******************************************************************/
    double getTSPRate(const int direction) const
    {
        return (direction == SOAPY_SDR_TX)? _masterClockRate : _masterClockRate/4;
    }

    void setMasterClockRate(const double rate)
    {
        int ret = LMS7002M_set_data_clock(_lms, EXT_REF_CLK, rate, &_masterClockRate);
        if (ret != 0)
        {
            SoapySDR::logf(SOAPY_SDR_ERROR, "LMS7002M_set_data_clock(%f MHz) -> %d", rate/1e6, ret);
            throw std::runtime_error("EVB7 fail LMS7002M_set_data_clock()");
        }
        SoapySDR::logf(SOAPY_SDR_TRACE, "LMS7002M_set_data_clock(%f MHz) -> %f MHz", rate/1e6, _masterClockRate/1e6);
    }

    double getMasterClockRate(void) const
    {
        return _masterClockRate;
    }

    /*******************************************************************
     * Time API
     ******************************************************************/
    long long ticksToTimeNs(const long long ticks) const
    {
        return ticks/(IF_TIME_CLK/1e9);
    }

    long long timeNsToTicks(const long long timeNs) const
    {
        return timeNs/(1e9/IF_TIME_CLK);
    }

    bool hasHardwareTime(const std::string &what) const
    {
        if (what.empty()) return true;
        return EVB7::hasHardwareTime(what);
    }

    long long getHardwareTime(const std::string &) const
    {
        long long timeLo = this->readRegister(FPGA_REG_RD_TIME_LO);
        long long timeHi = this->readRegister(FPGA_REG_RD_TIME_HI);
        return this->ticksToTimeNs((timeHi << 32) | timeLo);
    }

    void setHardwareTime(const long long timeNs, const std::string &)
    {
        long long ticks = this->timeNsToTicks(timeNs);
        this->writeRegister(FPGA_REG_WR_TIME_LO, ticks & 0xffffffff);
        this->writeRegister(FPGA_REG_WR_TIME_HI, ticks >> 32);
        this->writeRegister(FPGA_REG_WR_TIME_LATCH, 1);
        this->writeRegister(FPGA_REG_WR_TIME_LATCH, 0);
    }

    /*******************************************************************
     * Sensor API
     ******************************************************************/

    /*******************************************************************
     * Register API
     ******************************************************************/
    void writeRegister(const unsigned addr, const unsigned value)
    {
        xumem_write32(_regs, addr, value);
    }

    unsigned readRegister(const unsigned addr) const
    {
        return xumem_read32(_regs, addr);
    }

    /*******************************************************************
     * Settings API
     ******************************************************************/

    /*******************************************************************
     * GPIO API
     ******************************************************************/

    /*******************************************************************
     * I2C API
     ******************************************************************/

    /*******************************************************************
     * SPI API
     ******************************************************************/

    /*******************************************************************
     * UART API
     ******************************************************************/

private:
    void *_regs;
    void *_spiHandle;
    LMS7002M_t *_lms;
    pzdud_t *_rx_data_dma;
    pzdud_t *_rx_ctrl_dma;
    pzdud_t *_tx_data_dma;
    pzdud_t *_tx_stat_dma;
    double _masterClockRate;
};
