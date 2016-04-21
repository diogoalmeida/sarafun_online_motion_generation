# SARAFun

## Overview

This is the repository storing the software that CERTH/ITI/AUTH develops for
the SARAFUN project.

The repository contains ROS wrapper for HQP solver, with the following tasks added
* Joint limit avoidance
* Obstacle avoidance, currently only spheres w.r.t to robot's end effector
* VITE dynamical system for task reaching, see. S. Bullock, Daniel Grossberg "Neural dynamics of planned arm movements: Emergent invariants and
            speed-accuracy properties during trajectory formation", Psychol. Rev., 95 (1) (1988), pp. 49-90 

* Component packages:
  - `sarafun_hqp_omg`: The ROS wrapper of the HQP solver, used for Online Motion Generation
  - `lwr_description`: Description for KUKA LWR robot (obtained from here: https://github.com/gpldecha/LWR_KUKA_ROS)

## Dependencies

* Ubuntu 14.04
* ROS Indigo  ...
* git
* The `abb_irb14000_support` package
* You need the main HQP solver from https://github.com/stack-of-tasks/

To build 
  
```bash
mkdir ~/sarafun
cd ~/sarafun
git clone https://github.com/stack-of-tasks/soth.git
cd soth/cmake
git clone https://github.com/jrl-umi3218/jrl-cmakemodules.git ./

cd ..
mkdir build
cd build
cmake ..
make -j4
sudo make install
LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib

```

## How to use

Setup your workspace and clone and build the packages:

```bash
cd ~/sarafun
mkdir ros_ws
cd ros_ws 
mkdir src
cd src

git clone https://github.com/auth-arl/sarafun_online_motion_generation.git ./sarafun_hqp_omg
cd ~/sarafun/ros_ws
catkin_make
source ~/sarafun/ros_ws/devel/setup.bash
```

Run the launch file:

```bash
roslaunch sarafun_hqp_omg hqp.launch simmode:=true safetyDst:=0.1
```

And in another terminal run the [`sarafun_hqp_omg/run.sh`](sarafun_hqp_omg/run.sh) script.

## Documentation
* See comments in launch file



