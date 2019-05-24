Write and build gazebo plugins as in http://gazebosim.org/tutorials?tut=plugins_model&cat=write_plugin 


Place the built .so files into catkin_ws/src/Firmware/build/px4_sitl_default/build_gazebo


Combine the plugins into the model by simply adding <plugin name="model_movement" filename="libmodel_movement.so"/>   before </model>  in the .sdf file