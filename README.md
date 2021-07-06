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
catkin_make
cd ..
source devel/setup.bash
roslaunch od_sins_realtime od_sins_realtime.launch
```

## Acknowledgement

  - This project is modified from PSINS C++ toolbox from Prof.Gongmin Yan of Northwestern Polytechnical University.
