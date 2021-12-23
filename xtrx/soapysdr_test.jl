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

function doit(stream)
    SoapySDR.activate!(stream)

    try
        # acquire a single read buffer using the low-level API
        buffs = Ptr{UInt32}[C_NULL]
        for i in 1:10
            bytes, handle, flags, timeNs = SoapySDR.SoapySDRDevice_acquireReadBuffer(dev, stream, buffs)
            arr = unsafe_wrap(Array, buffs[1], bytes)
            println("Got buffer of size: $(Base.format_bytes(bytes))")
            # display(arr[1:10])
            # println("\n ...")
            SoapySDR.SoapySDRDevice_releaseReadBuffer(dev, stream, handle)
        end
    finally
        SoapySDR.deactivate!(stream)
    end
end
doit(stream)

# close everything
close(stream)
close(dev)
