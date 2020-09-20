# Copyright (c) 2016, NVIDIA CORPORATION.  All rights reserved.

@page dwx_canbus_logger_sample CAN Bus Message Logger Sample

The CAN Bus Message Logger sample is a simple CAN bus listener sample. All
messages received over the CAN bus are printed on the console. A valid SocketCAN
(or AurixCAN applicable to DRIVE PX 2 only) device must be present in the system.
DRIVE PX 2 provides up to 6 CAN connections, where
CAN-5 and CAN-6 are routed directly to the both Tegras and CAN-1..CAN-4 are passed through Aurix.

#### Usage

	./sample_canbus_logger

#### Testing

Test the applications using either a real CAN device or a virtual one. Create a
virtual device using the following commands:

    sudo modprobe vcan
    sudo ip link add dev vcan0 type vcan
    sudo ip link set up vcan0

In order to send data from the console to the virtual CAN bus, use the `cansend`
tool (from the `can-utils` package).

    cansend vcan0 30B#1122334455667788

This example sends a CAN message with ID 0x30B and with 8-bytes containing:
0x11, ..., 0x88

If using the virtual CAN interface, the sample can be started with
`--driver=can.socket --params=device=vcan0` to listen on the virtual CAN
interface. Any message sent with `cansend vcan0` is displayed on the console.


#### Real Hardware

If using real-hardware connected to the CAN bus, set the bitrate to 500 KB in
order to test the sample. The sample does not support changing of the bitrate.
Set the bitrate on the CAN device by executing:

    sudo ip link set can0 type can bitrate 500000
    sudo ip link set can0 up

The CAN interface used to listen on a real CAN bus must have SocketCAN driver
implementation in the system. A SocketCAN based driver can be identified easily
if `:> sudo ifconfig -a` returns the default network interfaces **and** a CAN
based interface (i.e., canX, slcanX, etc).

##### SocketCAN
To execute the sample to listen on the SocketCAN connectors, i.e., CAN-5 and CAN-6, use:

    --driver=can.socket --params=device=can0

Per default, CAN-5 is routed to `can0` SocketCAN interface on TegraA and CAN-6 to `can1` SocketCAN
interface on TegraB.

##### AurixCAN
On DRIVE PX 2 and ACR, the CAN connectors marked with CAN-1..CAN-4 are reachable throught Aurix by the Tegras.
For this to work, a proper setup of the Aurix needs to be made prior to running the application. Please
refer to the EasyCAN user guide provided as part of PDK documentation to set up Aurix to filter
and pass a selected subset of CAN messages. In order to connect to AurixCAN, use the following arguments:

On DRIVE PX2:
    --driver=can.aurix --params=ip=10.42.0.83,bus=a

On ACR:
    --driver=can.aurix --params=ip=127.0.0.1,aport=50000,bport=50103,bus=a

Where `ip` is the IP address from which the CAN messages are forwarded (by default, set to 10.42.0.83)
and `bus` points to the corresponding CAN bus connector, i.e., CAN-1->a, ..., CAN-4->d.

The connection to AurixCAN happens using UDP sockets. Additional arguments for the remote (`aport`) and
local (`bport`) UDP port can be specified if AurixCAN is working in a non-default configuration. By
default, on DRIVE PX2, AurixCAN is reachable over `aport=50000` and communicates with the Tegra over
`bport=60395`. On ACR however, AurixCAN is reachable over `aport=50000` and communicates with
the Tegra over `bport=50103`.

##### Hardware Timestamping
DriveWorks supports HW timestamps of the CAN messages. Tegra provides a hardware-based counter
timestamps all CAN messages on arrival directly on the SoC. For the case of AurixCAN,
the messages are timestamped by Aurix on arrival and passed together with the CAN messages over UDP
to DriveWorks. In order for HW timestamps to work for SocketCAN, the application must run with
root rights.


#### Message Sending
Additionally, the sample can be used as a CAN message generator. When starting
it, the following parameter can be passed:
`--send_i_understand_implications=100`, which generates a random CAN message
each 100ms. Note that sending might interfere with real hardware; therefore, only
use it if you know what you are doing.


#### Filtering
The sample provides support to filter (i.e., pass) messages of certain IDs. A
filter can be specified  by adding comma separated `id:mask` values to the
`--filter=` argument. A message is considered passing if `<received_can_id> &
mask == id & mask`.

    Example 1 - pass all messages between 201-2FF: `--filter=200:F00`
    Example 2 - pass only 100,110 and 301-31F: `--filter=100:FFF,110:FFF,300:FE0`

