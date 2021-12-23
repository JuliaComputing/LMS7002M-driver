# Control driver for LMS7002M dual transceiver

## The LMS7002M transceiver

The LMS7002M is Lime’s second-generation field programmable RF (FPRF) transceiver IC
with extended functionality to cover a greater range of frequencies and applications...
http://www.limemicro.com/products/field-programmable-rf-ics-lms7002m/

This branch is a WIP to interface with Fairwave's XTRX using LiteX-based gateware:

```
$ LITEPCIE_ROOT=/path/to/xtrx_julia/software cmake -GNinja -DCMAKE_BUILD_TYPE=Debug ../
$ ninja && ./xtrx/xtrx_main /dev/litepcie0
=========================================================
== Test LMS7002M access
=========================================================
...
```

To build and test the SoapySDR integration:

- ensure SoapySDR is installed before configuring the build directory
- build as above, which should result in `xtrx/libSoapyXTRX.so`
- link that library into `/usr/local/lib/SoapySDR/modules0.8`

```
$ SoapySDRUtil --make="driver=XTRX"
ninja: no work to do.
######################################################
##     Soapy SDR -- the SDR abstraction library     ##
######################################################

Make device driver=XTRX
...
[INFO] Initialization complete
  driver=XTRX
  hardware=XTRX
[INFO] Power down and cleanup
```

For similar implementations, and to extend the implementation, see:

- https://pothosware.github.io/SoapySDR/doxygen/latest/classSoapySDR_1_1Device.html
- https://github.com/myriadrf/LMS7002M-driver/blob/master/evb7/
- https://github.com/myriadrf/LimeSuite/blob/master/SoapyLMS7/
- https://github.com/xtrx-sdr/libxtrx/blob/master/soapy/
- https://github.com/pothosware/SoapyHackRF/

## Driver functionality

This project contains a C driver for control of the LMS7002M transceiver.
The driver provides user APIs for tuning frequencies, setting gains, setting filters,
setting sample rates, setting stream modes, configuring switches, and calibration.
Although this driver can select the streaming mode of the LMS7002M,
it does not directly interact with receive or transmit baseband data.

## Hardware independent

End applications may access the SPI bus for the LMS7002M in a variety of ways.
To ensure that the driver can talk to the LMS7002M independent of the hardware,
the caller provides the driver instance with SPI access callback functions.
These functions implement the hardware-dependent SPI register access routines.

## Using LMS7002M API

The C sources in the src/ directory can be compiled into any C or C++ project.
As an alternative, the CMake build exists in the root directory
to build a LMS7002M static library and to install header files.
A client project can link against this library in standard fashion.

### Installation instructions

```
git clone https://github.com/myriadrf/LMS7002M-driver.git
cd LMS7002M-driver
mkdir build; cd build
cmake ../
make -j4
sudo make install
```

## Licensing information

Use, modification and distribution is subject to the Apache License
Version 2.0. (See accompanying file LICENSE-2.0.txt or copy at
https://www.apache.org/licenses/LICENSE-2.0.txt)

## Project layout

* regs/ - LMS7002M register map and register header generator
* include/ - LMS7002M headers for C driver interface
* src/ - LMS7002M C sources for implementation details
* interfaces/ - example SPI interfaces for use in LMS7002M
* evb7/ - example SoapySDR wrapper using the LMS7002M C driver
* test/ - simple register access test using the LMS7002M C driver

## References

* Driver based on Programming and Calibration Guide v2.24.0
