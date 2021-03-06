## Overview

  - A Precise Strapdown Inertial Navigation System (PSINS) C++ algorithm and Integrated Navigation (GNSS/INS/Odometry) algorithm based on adaptive Kalman Filter for ROS

## Usage

  ```
mkdir PSINS
cd PSINS
mkdir src
cd src
catkin_init_workspace
git clone https://github.com/BohemianRhapsodyz/PSINS-ROS.git
cd ..
catkin_make
source devel/setup.bash
roslaunch od_sins_realtime od_sins_realtime.launch
```

## Note

  - Replace your imu.txt path in file readimu.cpp
  - Check ROS topic imu/encoder/nav 
  - the file imu.txt provides a dataset of IMU/Odometry. You can test your own GNSS/IMU/Odometry dataset.

## Acknowledgement

  - This project is modified from PSINS C++ toolbox from Prof.Gongmin Yan of Northwestern Polytechnical University.
