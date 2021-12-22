#include "XTRXDevice.hpp"

#include <chrono>
#include <thread>

// XXX: these functions are stubs, just there so that we can read data
//      to verify the digital interface

SoapySDR::Stream *XTRX::setupStream(
    const int direction,
    const std::string &format,
    const std::vector<size_t> &channels,
    const SoapySDR::Kwargs &)
{
	std::lock_guard<std::mutex> lock(_mutex);

	if(direction==SOAPY_SDR_RX){
        return RX_STREAM;
	} else if(direction==SOAPY_SDR_TX){
		return TX_STREAM;
	} else {
		throw std::runtime_error("Invalid direction");
	}
}

void XTRX::closeStream(SoapySDR::Stream *stream)
{
    return;
}

int XTRX::activateStream(
    SoapySDR::Stream *stream,
    const int flags,
    const long long timeNs,
    const size_t numElems)
{
    // XXX: initialize once, globally
    _dma = {.use_writer = 1};
    if (litepcie_dma_init(&_dma, "/dev/litepcie0", 0))
        throw std::runtime_error("Failed to initialize DMA");

    return 0;
}

int XTRX::deactivateStream(
    SoapySDR::Stream *stream,
    const int flags,
    const long long timeNs)
{
    litepcie_dma_cleanup(&_dma);
    return 0;
}

int XTRX::acquireReadBuffer(
    SoapySDR::Stream *stream,
    size_t &handleOut,
    const void **buffs,
    int &flags,
    long long &timeNs,
    const long timeoutUs)
{
    if (stream != RX_STREAM)
        return SOAPY_SDR_STREAM_ERROR;

	//calculate when the loop should exit
	const auto timeout = std::chrono::duration_cast<std::chrono::high_resolution_clock::duration>(std::chrono::microseconds(timeoutUs));
	const auto exitTime = std::chrono::high_resolution_clock::now() + timeout;

	//poll for status events until the timeout expires
	while (true) {
        litepcie_dma_process(&_dma);
        char *buf_rd = litepcie_dma_next_read_buffer(&_dma);
        if (buf_rd) {
            buffs[0] = buf_rd;
            return DMA_BUFFER_SIZE;
        }

		//sleep for a fraction of the total timeout
		const auto sleepTimeUs = std::min<long>(1000, timeoutUs/10);
		std::this_thread::sleep_for(std::chrono::microseconds(sleepTimeUs));

		//check for timeout expired
		const auto timeNow = std::chrono::high_resolution_clock::now();
		if (exitTime < timeNow) return SOAPY_SDR_TIMEOUT;
    }
}
