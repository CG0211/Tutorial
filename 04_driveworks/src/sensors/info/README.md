# Copyright (c) 2016, NVIDIA CORPORATION.  All rights reserved.

@page dwx_sensor_enum_sample Sensor Enumeration Sample

The Sensor Enumeration sample is a minimal sample using the sensor abstraction
layer to enumerate all available sensors drivers in SDK. The sample does not
expect any user input. 

./sample_sensors_info

On execution, the sample outputs a list of sensors in the
following format:

    Platform: OS_LINUX: 
       Sensor [0] : can.socket ? device=can0
       Sensor [1] : can.virtual ? file=/path/to/file.can[,create_seek]
       Sensor [2] : camera.virtual ? video/file=filepath.{h264,raw}[,timestamp=file.txt][,create_seek]
       Sensor [3] : camera.usb ? device=0
       Sensor [4] : camera.pointgrey ? device=0[,fps=30]
       Sensor [5] : gps.uart ? device=/dev/ttyXXX[,baud={1200,2400,4800,9600,19200,38400,57600,115200}[,format=nmea0183]]
       Sensor [6] : gps.virtual ? file=filepath.bins[,create_seek]
       Sensor [7] : gps.xsens ? device=0[,frequency=100]
       Sensor [8] : gps.novatel ? 
       Sensor [9] : imu.uart ? device=/dev/ttyXXX[,baud={1200,2400,4800,9600,19200,38400,57600,115200}[,format=xsens_nmea]]
       Sensor [10] : imu.xsens ? device=0[,frequency=100]
       Sensor [11] : imu.virtual ? file=filepath.bin[,create_seek]
       Sensor [12] : imu.novatel ? 
       Sensor [13] : lidar.virtual ? file=filepath.bin[,create_seek]
       Sensor [14] : lidar.socket ? ip=X.X.X.X,port=XXXX,device={QUAN_M81A, IBEO_LUX, VELO_VLP16, VELO_HDL32E, VELO_HDL64E},scan-frequency=XX.X
       Sensor [15] : radar.virtual ? file=filepath.bin[,create_seek]
       Sensor [16] : radar.socket ? ip=X.X.X.X,port=XXXX,device={DELPHI_ESR2_5, CONTINENTAL_ARS430}

    Platform: OS_DRIVE_V4L - CURRENT: 
       Sensor [0] : can.socket ? device=can0
       Sensor [1] : can.aurix ? ip=10.0.0.1,bus={a,b,c,d,e,f}[,aport=50000,bport=60395]
       Sensor [2] : can.virtual ? file=/path/to/file.can[,create_seek]
       Sensor [3] : camera.gmsl ? csi-port={ab,cd,ef},camera-count={1,2,3,4},camera-type={ov10635,c-ov10640-b1,ov10640-svc210,ov10640-svc212,ar0231,ar0231-rccb,ar0231-rccb-ssc,ar0231-rccb-bae,ar0231-rccb-ss3322,ar0231-rccb-ss3323},output-format={yuv+raw+data}[,slave={0,1}][,fifo-size={3..20}][,cross-csi-sync={0,1}][,custom-board=0][,camera-mask={0001|0010|0011|..|1111}][,required-framerate={20,30,36}]
       Sensor [4] : camera.virtual ? video/file=filepath.{h264/raw}[,timestamp=file.txt][,create_seek]
       Sensor [5] : camera.usb ? device=0
       Sensor [6] : camera.pointgrey ? device=0
       Sensor [7] : gps.uart ? device=/dev/ttyXXX[,baud={1200,2400,4800,9600,19200,38400,57600,115200}[,format=nmea0183]]
       Sensor [8] : gps.virtual ? file=filepath.gps[,create_seek]
       Sensor [9] : gps.xsens ? device=0[,frequency=100]
       Sensor [10] : gps.novatel ? 
       Sensor [11] : imu.uart ? device=/dev/ttyXXX[,baud={1200,2400,4800,9600,19200,38400,57600,115200}[,format=xsens_nmea]]
       Sensor [12] : imu.xsens ? device=0[,frequency=100]
       Sensor [13] : imu.virtual ? file=filepath.imu[,create_seek]
       Sensor [14] : imu.novatel ? 
       Sensor [15] : lidar.virtual ? file=yourfilename.bin
       Sensor [16] : lidar.socket ? ip=xxx.xxx.xxx.xxx,port=x,device={QUAN_M81A, IBEO_LUX, VELO_VLP16, VELO_HDL32E, VELO_HDL64E},scan-frequency=10
       Sensor [17] : radar.virtual ? file=yourfilename.bin,device={DELPHI_ESR2_5, CONTINENTAL_ARS430}
       Sensor [18] : radar.socket ? ip=xxx.xxx.xxx.xxx,port=x,device={DELPHI_ESR2_5, CONTINENTAL_ARS430}


The list of available sensors is grouped by the underlying platform
(Linux or NVIDIA DRIVE<sup>&trade;</sup> PX 2) and on the sensor drivers currently
available/implemented in DriveWorks. A list indicates the name of the sensor and
the underlying protocol, as well as a set of string-based key-value pairs that
you can pass to a sensor as additional arguments.

#### Example

In the example below, there is a sensor `camera.gmsl` available that expects as
a parameter the csi-port, which can be any value from the set {ab,cd,ef}, a
number of cameras available at this port (i.e., 1, 2, 3 or 4), and the camera
type (i.e., `ov10635` or `c-ov10640-b1`).

    Sensor [1] : camera.gmsl ? csi-port={ab,cd,ef},camera-count={1,2,3,4},camera-type={ov10635,c-ov10640-b1}
