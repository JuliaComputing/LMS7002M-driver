using SoapySDR

# Get the Devices
devs = Devices()

# Now open the first device and get the first RX and TX channel streams
dev = open(devs[1])
c_tx = dev.tx[1]
c_rx = dev.rx[1]

# enable loopback
SoapySDR.SoapySDRDevice_writeSetting(dev, "TX_PATTERN", "1")

# Open both RX and TX streams
s_tx = SoapySDR.Stream(ComplexF32, [c_tx])
s_rx = SoapySDR.Stream(ComplexF32, [c_rx])

buffs = Ptr{UInt32}[C_NULL]
SoapySDR.activate!(s_rx)
bytes, handle, flags, timeNs = SoapySDR.SoapySDRDevice_acquireReadBuffer(dev, s_rx, buffs)
arr = unsafe_wrap(Array, buffs[1], bytes)
@show arr
SoapySDR.deactivate!(s_rx)

# Make sure we close the Streams and Devices where we are done with them
close.((s_tx, s_rx))
close(dev)
