//
// SoapySDR driver for the LMS7002M-based Fairwaves XTRX.
//
// Copyright (c) 2021 Julia Computing.
// Copyright (c) 2015-2015 Fairwaves, Inc.
// Copyright (c) 2015-2015 Rice University
// SPDX-License-Identifier: Apache-2.0
// http://www.apache.org/licenses/LICENSE-2.0
//

#include "XTRXDevice.hpp"

#include <chrono>
#include <thread>

// XXX: these functions are stubs, just there so that we can read data
//      to verify the digital interface

SoapySDR::Stream *SoapyXTRX::setupStream(const int direction,
                                         const std::string &format,
                                         const std::vector<size_t> &channels,
                                         const SoapySDR::Kwargs &) {
    std::lock_guard<std::mutex> lock(_mutex);

    if (direction == SOAPY_SDR_RX) {
        return RX_STREAM;
    } else if (direction == SOAPY_SDR_TX) {
        return TX_STREAM;
    } else {
        throw std::runtime_error("Invalid direction");
    }
}

void SoapyXTRX::closeStream(SoapySDR::Stream *stream) { return; }

int SoapyXTRX::activateStream(SoapySDR::Stream *stream, const int flags,
                              const long long timeNs, const size_t numElems) {
    return 0;
}

int SoapyXTRX::deactivateStream(SoapySDR::Stream *stream, const int flags,
                                const long long timeNs) {
    return 0;
}

int SoapyXTRX::acquireReadBuffer(SoapySDR::Stream *stream, size_t &handleOut,
                                 const void **buffs, int &flags,
                                 long long &timeNs, const long timeoutUs) {
    if (stream != RX_STREAM)
        return SOAPY_SDR_STREAM_ERROR;

    // calculate when the loop should exit
    const auto timeout = std::chrono::duration_cast<
        std::chrono::high_resolution_clock::duration>(
        std::chrono::microseconds(timeoutUs));
    const auto exitTime = std::chrono::high_resolution_clock::now() + timeout;

    // poll for status events until the timeout expires
    while (true) {
        litepcie_dma_process(&_dma);
        char *buf_rd = litepcie_dma_next_read_buffer(&_dma);
        if (buf_rd) {
            buffs[0] = buf_rd;
            return DMA_BUFFER_SIZE;
        }

        // sleep for a fraction of the total timeout
        const auto sleepTimeUs = std::min<long>(1000, timeoutUs / 10);
        std::this_thread::sleep_for(std::chrono::microseconds(sleepTimeUs));

        // check for timeout expired
        const auto timeNow = std::chrono::high_resolution_clock::now();
        if (exitTime < timeNow)
            return SOAPY_SDR_TIMEOUT;
    }
}
