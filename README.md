
# Radar setup

There is a command called cansend, that belongs to can-utils, used for sending configuration messages to the radar. Here are some messages proposed, but other messages can be sent (watch the manual ARS40X_Technical_Documentation_V 1.8_18.10.2017 inside documentation folder).

Installation of can-utils
```bash
sudo apt-get install can-utils
candump can0 // Watch the raw data received once the peak CAN bus is installed and connected to Radar
```

Configuration messages to choose between cluster detection or object detection
```bash
cansend can0 200#F8000000089C0000 // Objects detection with all extended properties
cansend can0 200#F8000000109C0000 // Clusters detection with all extended properties
```
```bash
cansend can0 200#8200000001800000 // Objects id 1
cansend can0 200#8200000002800000 // Objects id 2
cansend can0 200#8200000003800000 // Objects id 3
cansend can0 200#8200000004800000 // Objects id 4
cansend can0 200#8200000005800000 // Objects id 5
cansend can0 200#8200000006800000 // Objects id 6
cansend can0 200#8200000007800000 // Objects id 7
```

Configuration messages for applying different filters.
```bash
cansend can0 202#8C0000012C // Maximum distance of objects detected 30 meters
cansend can0 202#AE06800FFF // Minimum value of object RCS -10 dBm2
cansend can0 202#C600030007 // Minimum value of objects probability of existence 75%
```

Other option to configure the radar is to modify the values of configuration_vars.h inside the header folder of socketcan brige. Please remember that is necessary to build the package again in order to save the values chosen in the configuration header file.

Start ros package
```bash
source devel/setup.bash
roslaunch radar_ros run.launch calib_radar:=true //if need calib Radar, both cluster and object are ok
```

