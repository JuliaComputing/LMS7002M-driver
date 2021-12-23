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
#include <cassert>
#include <thread>
#include <sys/mman.h>

// XXX: these functions are stubs, just there so that we can read data
//      to verify the digital interface

SoapySDR::Stream *SoapyXTRX::setupStream(const int direction,
                                         const std::string &format,
                                         const std::vector<size_t> &channels,
                                         const SoapySDR::Kwargs &) {
    std::lock_guard<std::mutex> lock(_mutex);

    if (direction == SOAPY_SDR_RX) {
        if (_rx_stream.opened)
            throw std::runtime_error("RX stream already opened");

        // configure the file descriptor watcher
        _rx_stream.fds.fd = _fd;
        _rx_stream.fds.events = POLLIN;

        // initialize the DMA engine
        if ((litepcie_request_dma(_fd, 0, 1) == 0))
            throw std::runtime_error("DMA not available");

        // mmap the DMA buffers
        _rx_stream.buf =
            mmap(NULL, DMA_BUFFER_TOTAL_SIZE, PROT_READ | PROT_WRITE,
                 MAP_SHARED, _fd, _mmap_dma_info.dma_rx_buf_offset);
        if (_rx_stream.buf == MAP_FAILED)
            throw std::runtime_error("MMAP failed");

        _rx_stream.opened = true;
        return RX_STREAM;
    } else if (direction == SOAPY_SDR_TX) {
        if (_tx_stream.opened)
            throw std::runtime_error("TX stream already opened");

        // configure the file descriptor watcher
        _tx_stream.fds.fd = _fd;
        _tx_stream.fds.events = POLLOUT;

        // initialize the DMA engine
        if ((litepcie_request_dma(_fd, 1, 0) == 0))
            throw std::runtime_error("DMA not available");

        // mmap the DMA buffers
        _tx_stream.buf =
            mmap(NULL, DMA_BUFFER_TOTAL_SIZE, PROT_WRITE, MAP_SHARED, _fd,
                 _mmap_dma_info.dma_tx_buf_offset);
        if (_tx_stream.buf == MAP_FAILED)
            throw std::runtime_error("MMAP failed");

        _tx_stream.opened = true;
        return TX_STREAM;
    } else {
        throw std::runtime_error("Invalid direction");
    }
}

void SoapyXTRX::closeStream(SoapySDR::Stream *stream) {
    std::lock_guard<std::mutex> lock(_mutex);

    if (stream == RX_STREAM) {
        // release the DMA engine
        litepcie_release_dma(_fd, 0, 1);

        munmap(_rx_stream.buf, _mmap_dma_info.dma_tx_buf_size *
                                   _mmap_dma_info.dma_tx_buf_count);
        _rx_stream.opened = false;
    } else if (stream == TX_STREAM) {
        // release the DMA engine
        litepcie_release_dma(_fd, 1, 0);

        munmap(_tx_stream.buf, _mmap_dma_info.dma_rx_buf_size *
                                   _mmap_dma_info.dma_rx_buf_count);
        _tx_stream.opened = false;
    }
}

int SoapyXTRX::activateStream(SoapySDR::Stream *stream, const int flags,
                              const long long timeNs, const size_t numElems) {
    if (stream == RX_STREAM) {
        // enable the DMA engine
        litepcie_dma_writer(_fd, 1, &_rx_stream.hw_count, &_rx_stream.sw_count);
        _rx_stream.user_count = 0;
    } else if (stream == TX_STREAM) {
        // enable the DMA engine
        litepcie_dma_reader(_fd, 0, &_tx_stream.hw_count, &_tx_stream.sw_count);
    }

    // TODO: set-up the LMS7002M

    return 0;
}

int SoapyXTRX::deactivateStream(SoapySDR::Stream *stream, const int flags,
                                const long long timeNs) {
    if (stream == RX_STREAM) {
        // disable the DMA engine
        litepcie_dma_writer(_fd, 0, &_rx_stream.hw_count, &_rx_stream.sw_count);
    } else if (stream == TX_STREAM) {
        // disable the DMA engine
        litepcie_dma_reader(_fd, 1, &_tx_stream.hw_count, &_tx_stream.sw_count);
    }
    return 0;
}

int SoapyXTRX::acquireReadBuffer(SoapySDR::Stream *stream, size_t &handleOut,
                                 const void **buffs, int &flags,
                                 long long &timeNs, const long timeoutUs) {
    if (stream != RX_STREAM)
        return SOAPY_SDR_STREAM_ERROR;

    // get the DMA counters
    litepcie_dma_writer(_fd, 1, &_rx_stream.hw_count, &_rx_stream.sw_count);

    // check if buffers available
    int buffers_available = _rx_stream.hw_count - _rx_stream.user_count;

    // wait, if necessary
    assert(buffers_available >= 0);
    if (buffers_available == 0) {
        int ret = poll(&_rx_stream.fds, 1, timeoutUs / 1000);
        if (ret < 0)
            throw std::runtime_error(
                "SoapyXTRX::acquireReadBuffer(): poll failed, " +
                std::string(strerror(errno)));
        else if (ret == 0)
            return SOAPY_SDR_TIMEOUT;

        // get new DMA counters
        litepcie_dma_writer(_fd, 1, &_rx_stream.hw_count,
                            &_rx_stream.user_count);
        buffers_available = _rx_stream.hw_count - _rx_stream.user_count;
        assert(buffers_available > 0);
    }

    // detect overflows
    if ((_rx_stream.hw_count - _rx_stream.sw_count) > DMA_BUFFER_COUNT / 2)
        return SOAPY_SDR_OVERFLOW;

    // get the buffer
    handleOut = _rx_stream.user_count;
    int buf_offset = _rx_stream.user_count % DMA_BUFFER_COUNT;
    buffs[0] = _rx_stream.buf + buf_offset * DMA_BUFFER_SIZE;

    // update the DMA counters
    _rx_stream.user_count += 1;

    return DMA_BUFFER_SIZE;
}

void SoapyXTRX::releaseReadBuffer(SoapySDR::Stream *stream, size_t handle) {
    if (handle != _rx_stream.sw_count)
        throw std::runtime_error(
            "SoapyXTRX::releaseReadBuffer(): handles should be released in order");

    // XXX: does it even matter we only bump the sw_count upon 'release'?
    //      the DMA engine can overflow anyway.
    struct litepcie_ioctl_mmap_dma_update mmap_dma_update;
    mmap_dma_update.sw_count = _rx_stream.sw_count + 1;
    checked_ioctl(_fd, LITEPCIE_IOCTL_MMAP_DMA_WRITER_UPDATE, &mmap_dma_update);
}
