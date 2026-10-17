# External IMU dependency

This repository does **not** redistribute the NTRexLAB MW-AHRS driver/SDK or its binary libraries.

The project was developed with the manufacturer's ROS/Linux package:

- NTRexLAB `2th_NtrexAHRS_lib_ROS`
- Repository: https://github.com/ntrexlab/2th_NtrexAHRS_lib_ROS
- Branch used by the manufacturer repository: `ver_2.0`

The integrated launch expects the vendor ROS 2 package to provide:

```text
package:    stella_ahrs
executable: stella_ahrs_node
output:     /imu/data (sensor_msgs/msg/Imu)
```

`mmr_status` contains the project-specific `tilt_monitor_node`, which consumes `/imu/data` and publishes the MMR tilt/status topics used by the rest of the system.

Install the vendor package separately in the ROS 2 environment before enabling IMU integration.
