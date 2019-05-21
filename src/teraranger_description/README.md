# teraranger_description
# ROS package for URDF description of TeraRanger products by Terabee

This package is a collection of URDF files of teraranger products:
* [TeraRanger Tower](https://www.terabee.com/shop/lidar-tof-multi-directional-arrays/teraranger-tower/)
* [TeraRanger Tower Evo](https://www.terabee.com/shop/lidar-tof-multi-directional-arrays/teraranger-tower-evo/)

It is required for using these packages:
* [teraranger_array_converter](https://github.com/Terabee/teraranger_array_converter)

## Building and running the package from source

To clone and build the package in your workspace follow these steps:

* If you have ssh key setup for your github account:

```
cd ~/ros_ws/src
git clone git@github.com:Terabee/teraranger_description.git
cd ~/ros_ws
catkin_make
source devel/setup.bash
```

* If you prefer to use https use this set of commands:

```
cd ~/ros_ws/src
git clone https://github.com/Terabee/teraranger_description.git
cd ~/ros_ws
catkin_make
source devel/setup.bash
```

## Using teraranger_description

To add easily a tower to your robot description, first include the xacro (xml macro) file :
```
<xacro:include filename="$(find teraranger_description)/urdf/teraranger_tower.urdf.xacro"/>
```
Then you will be able to call the xacro from your main URDF file:
```
<teraranger_tower multi_hub="false" hub_id="0" parent="base_link" x="0.000" y="0.000" z="0.500" roll="0.0" pitch="0.0" yaw="0.0" />
```
If you want a custom setup, just include the base_hub and teraranger_one xacro files and build your own setup:
```
<xacro:include filename="$(find teraranger_description)/urdf/base_hub.urdf.xacro"/>
<xacro:include filename="$(find teraranger_description)/urdf/teraranger_one.urdf.xacro"/>
```
Custom setup example (1 TeraRanger Hub + 1 TeraRanger One sensor):

```
<base_hub multi_hub="0" hub_id="0" parent="base_link" x="0.060" y="0.000" z="0.000" roll="0.0" pitch="0.0" yaw="0.0"/>
<teraranger_one multi_hub="0" hub_id="0" id="0" x="0.060" y="0.000" z="0.000" roll="0.0" pitch="0.0" yaw="0.0" gaussian_noise="0.06"/>
```
INFOS: By settings the multihub parameter to _true_ or _1_ you will enable auto-namespacing of the frame_id with the following convention:
* sensor frame = hub\_${hub_id}\_base_range\_${id}
* hub frame = base\_hub\_${hub_id}

___To enable compatibility, The ROS driver for the tower (from the [teraranger\_array](https://github.com/Terabee/teraranger_array) package) will append its namespace to the frame_id inside the RangeArray and Range messages, thus it is strongly recommended to use hub\_${hub_id} as node namespace for the driver.___
