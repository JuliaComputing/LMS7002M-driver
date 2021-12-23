using SoapySDR

# open the first device
devs = Devices()
dev = open(devs[1])

# get the RX channel
chan = dev.rx[1]

# enable loopback
SoapySDR.SoapySDRDevice_writeSetting(dev, "TX_PATTERN", "1")

# open RX streams
stream = SoapySDR.Stream(ComplexF32, [chan])

function process_buffers(stream)
    SoapySDR.activate!(stream)

    try
        # acquire buffers using the low-level API
        buffs = Ptr{UInt32}[C_NULL]
        bytes = 0
        for i in 1:100
            bytes, handle, flags, timeNs = SoapySDR.SoapySDRDevice_acquireReadBuffer(dev, stream, buffs)
            println("Got buffer of size: $(Base.format_bytes(bytes))")
            SoapySDR.SoapySDRDevice_releaseReadBuffer(dev, stream, handle)
        end

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
