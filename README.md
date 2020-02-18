# Luos ROS example: Bike sharing example


This is a Luos example using ROS2: the bike sharing setup works as is: TODO

## Get started
Make sure you first downloaded package `luos_interface` from `luos_ros2` repository and this example package and that you have compiled and sourced your ROS2 workspace:
```
~/ros2_ws/src/$ git clone https://github.com/aubrune/luos_ros2.git
~/ros2_ws/src/$ git clone https://github.com/aubrune/luos_bike_alarm_example.git

~/ros2_ws/$ colcon build --symlink-install    # Build the ROS workspace
~/ros2_ws/$ source ~/.bashrc                  # Source all new launches messages and resources
```

Plug at least a Luos Imu node and gate to your computer. The expected alias is `Imu_mod`.

Then, start the bike example from its launchfile:
```
~/ros2_ws/$ ros2 launch luos_bike_alarm_example example.launch.py
```

RViz2 will pop up and show a bike, agitate the Luo Imu node in order to update the RViz2 view in real time. If the bike shows but does not actuate, make sure that Imu data comes from the expected topic `/Imu_mod/imu`, or change the topic name.