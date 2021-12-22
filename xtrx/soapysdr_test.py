import SoapySDR
from SoapySDR import * #SOAPY_SDR_ constants
import numpy #use numpy for buffers

#create device instance
#args can be user defined or from the enumeration result
args = dict(driver="XTRX")
sdr = SoapySDR.Device(args)

#enable the TX pattern generator
sdr.writeSetting("TX_PATTERN", "1")

#record some data
import subprocess
subprocess.check_call([r"/home/tim/Julia/src/litex/xtrx_julia/software/user/litepcie_test", "record", "test.bin", "1024"])

# TODO: do this natively

#setup a stream (complex floats)
rxStream = sdr.setupStream(SOAPY_SDR_RX, SOAPY_SDR_CF32)
sdr.activateStream(rxStream) #start streaming

#create a re-usable buffer for rx samples
# buff = numpy.array([0]*1024, numpy.complex64)

#receive some samples
for i in range(10):
    handle = 0
    buff = 0
    flags = 0
    timeNs = 0
    sr = sdr.acquireReadBuffer(rxStream, handle, [buff], flags, timeNs)
    # sr = sdr.readStream(rxStream, [buff], len(buff))
    if sr.ret < 0:
        raise Exception(SoapySDR.errToStr(sr.ret))
    # print(sr.ret) #num samples or error code
    # print(sr.flags) #flags set by receive operation
    # print(sr.timeNs) #timestamp for receive buffer

#shutdown the stream
sdr.deactivateStream(rxStream) #stop streaming
sdr.closeStream(rxStream)
