//
// SoapySDR wrapper for the LMS7002M driver.
//
// Copyright (c) 2015-2015 Fairwaves, Inc.
// Copyright (c) 2015-2015 Rice University
// SPDX-License-Identifier: Apache-2.0
// http://www.apache.org/licenses/LICENSE-2.0
//

// For similar implementations, and to extend the implementation, see:
// https://pothosware.github.io/SoapySDR/doxygen/latest/classSoapySDR_1_1Device.html
// https://github.com/myriadrf/LimeSuite/blob/master/SoapyLMS7/Settings.cpp
// https://github.com/xtrx-sdr/libxtrx/blob/master/soapy/SoapyXTRX.cpp

#include <SoapySDR/Device.hpp>
#include <SoapySDR/Logger.hpp>
#include <SoapySDR/Time.hpp>
#include <mutex>
#include <cstring>
#include <cstdlib>
#include <stdexcept>
#include <iostream>

#include <LMS7002M/LMS7002M.h>

class XTRX : public SoapySDR::Device
{
public:
    XTRX(const SoapySDR::Kwargs &args);

    ~XTRX(void);

/*******************************************************************
* Identification API
******************************************************************/
    std::string getDriverKey(void) const
    {
        return "XTRX";
    }

    std::string getHardwareKey(void) const
    {
        return "XTRX";
    }

/*******************************************************************
* Channels API
******************************************************************/
    size_t getNumChannels(const int) const
    {
        return 2;
    }

    bool getFullDuplex(const int, const size_t) const
    {
        return true;
    }

    /*******************************************************************
     * Cal hooks
     ******************************************************************/
    void loadCalData(void);
    void applyCalData(const int direction, const size_t channel, const double rfFreq);

    /*******************************************************************
     * Antenna API
     ******************************************************************/
    std::vector<std::string> listAntennas(const int direction, const size_t channel) const;
    void setAntenna(const int direction, const size_t channel, const std::string &name);
    std::string getAntenna(const int direction, const size_t channel) const;

    std::map<int, std::map<size_t, std::string>> _cachedAntValues;

    /*******************************************************************
     * Frontend corrections API
     ******************************************************************/
    bool _rxDCOffsetMode;
    std::complex<double> _txDCOffset;

    void setDCOffsetMode(const int direction, const size_t channel, const bool automatic);

    bool getDCOffsetMode(const int direction, const size_t channel) const;

    void setDCOffset(const int direction, const size_t channel, const std::complex<double> &offset);

    std::complex<double> getDCOffset(const int direction, const size_t channel) const;

    void setIQBalance(const int direction, const size_t channel, const std::complex<double> &balance);

    std::complex<double> getIQBalance(const int direction, const size_t channel) const;

    std::map<int, std::map<size_t, std::complex<double>>> _cachedIqBalValues;

    /*******************************************************************
     * Gain API
     ******************************************************************/
    std::vector<std::string> listGains(const int direction, const size_t channel) const;
    void setGain(const int direction, const size_t channel, const std::string &name, const double value);
    double getGain(const int direction, const size_t channel, const std::string &name) const;
    SoapySDR::Range getGainRange(const int direction, const size_t channel, const std::string &name) const;

    std::map<int, std::map<size_t, std::map<std::string, double>>> _cachedGainValues;

    /*******************************************************************
     * Frequency API
     ******************************************************************/
    void setFrequency(const int direction, const size_t channel, const std::string &, const double frequency, const SoapySDR::Kwargs &args = SoapySDR::Kwargs());
    double getFrequency(const int direction, const size_t channel, const std::string &name) const;
    std::vector<std::string> listFrequencies(const int, const size_t) const;
    SoapySDR::RangeList getFrequencyRange(const int, const size_t, const std::string &) const;

    std::map<int, std::map<size_t, std::map<std::string, double>>> _cachedFreqValues;

    /*******************************************************************
     * Sample Rate API
     ******************************************************************/
    void setSampleRate(const int direction, const size_t, const double rate);
    double getSampleRate(const int direction, const size_t) const;
    std::vector<double> listSampleRates(const int direction, const size_t) const;

    std::map<int, double> _cachedSampleRates;

    /*******************************************************************
     * BW filter API
     ******************************************************************/
    void setBandwidth(const int direction, const size_t channel, const double bw);
    double getBandwidth(const int direction, const size_t channel) const;
    std::vector<double> listBandwidths(const int direction, const size_t channel) const;

    std::map<int, std::map<size_t, double>> _cachedFilterBws;

    /*******************************************************************
     * Clocking API
     ******************************************************************/
    double getTSPRate(const int direction) const;
    void setMasterClockRate(const double rate);
    double getMasterClockRate(void) const;

    /*******************************************************************
     * Sensor API
     ******************************************************************/

    /*******************************************************************
     * Register API
     ******************************************************************/
    void writeRegister(const unsigned addr, const unsigned value);
    unsigned readRegister(const unsigned addr) const;

    /*******************************************************************
     * Settings API
     ******************************************************************/

    /*!
     * Write arbitrary settings in the form of key/value pairs
     * that are otherwise not covered by the other API calls.
     * Example: setting loopback and test modes.
     *
     * - RXTSP_ENABLE(TRUE/FALSE) - call the RX TSP enable routine.
     *   Call with TRUE (enable) to reapply default settings.
     *
     * - TXTSP_ENABLE(TRUE/FALSE) - call the TX TSP enable routine.
     *   Call with TRUE (enable) to reapply default settings.
     *
     * - RBB_ENABLE(TRUE/FALSE) - call the RX baseband enable routine.
     *   Call with TRUE (enable) to reapply default settings.
     *
     * - TBB_ENABLE(TRUE/FALSE) - call the TX baseband enable routine.
     *   Call with TRUE (enable) to reapply default settings.
     *
     * - RXTSP_TSG_CONST(amplitude) - set the RX digital signal generator
     *   for a constant valued output.
     *
     * - TXTSP_TSG_CONST(amplitude) - set the TX digital signal generator
     *   for a constant valued output.
     *
     * - TBB_ENABLE_LOOPBACK(path) - enable TX baseband loopback.
     *   Use LB_DISCONNECTED, LB_DAC_CURRENT, LB_LB_LADDER, or LB_MAIN_TBB for the path.
     *
     * - RBB_SET_PATH(path) set the RX baseband input path.
     *   Use BYP, LBF, HBF for bypassing or filter path.
     *   Use LB_BYP, LB_LBF, LB_HBF for loopback versions.
     */
    void writeSetting(const std::string &key, const std::string &value);

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

    LMS7002M_dir_t dir2LMS(const int direction) const
    {
        return (direction == SOAPY_SDR_RX)?LMS_RX:LMS_TX;
    }

    LMS7002M_chan_t ch2LMS(const size_t channel) const
    {
        return (channel == 0)?LMS_CHA:LMS_CHB;
    }

    const char *dir2Str(const int direction) const
    {
        return (direction == SOAPY_SDR_RX)?"RX":"TX";
    }

    int _fd;
    LMS7002M_t *_lms;
    double _masterClockRate;

    //calibration data
    std::vector<std::map<std::string, std::string>> _calData;

    //register protection
    std::mutex _mutex;
};
