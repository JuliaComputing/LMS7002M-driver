using SoapySDR

# open the first device
devs = Devices()
dev = open(devs[1])

# get the RX channel
chan = dev.rx[1]

# enable pattern generator
SoapySDR.SoapySDRDevice_writeSetting(dev, "TX_PATTERN", "1")

# open RX stream
stream = SoapySDR.Stream(ComplexF32, [chan])

function process_buffers(stream)
    SoapySDR.activate!(stream)

    try
        # acquire buffers using the low-level API
        buffs = Ptr{UInt32}[C_NULL]
        bytes = 0
        total_bytes = 0

        println("Receiving data")
        time = @elapsed for i in 1:100
            bytes, handle, flags, timeNs = SoapySDR.SoapySDRDevice_acquireReadBuffer(dev, stream, buffs)
            SoapySDR.SoapySDRDevice_releaseReadBuffer(dev, stream, handle)
            total_bytes += bytes
        end
        println("Data rate: $(Base.format_bytes(total_bytes / time))/s")

        # print last array, for verification
        arr = unsafe_wrap(Array, buffs[1], bytes)
        display(arr[1:10])
        println("\n ...")
    finally
        SoapySDR.deactivate!(stream)
    end
end
process_buffers(stream)

# close everything
close(stream)
close(dev)
