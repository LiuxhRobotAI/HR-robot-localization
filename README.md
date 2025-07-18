<div align="center">
    <h2><strong>High-order regularization dealing with ill-conditioned robot localization problems</strong></h2>
</div>

<div align="center">
    <a href="https://github.com/LiuxhRobotAI" target='_blank'>Xinghua Liu</a><sup> </sup>&nbsp;&nbsp;&nbsp;
    <a href="" target='_blank'>Ming Cao</a><sup> </sup>&nbsp;&nbsp;&nbsp;
    </br></br>
    <sup> </sup>University of Groningen, the Netherlands&nbsp;&nbsp;&nbsp;
    </br></br>
</div>

## About
In this work, we propose a high-order regularization method to solve the ill-conditioned problems. Numerical solutions to robot localization problems are often unstable when the problems are ill-conditioned. A typical way to solve ill-conditioned problems is regularization, and a classical regularization method is the Tikhonov regularization. It is shown that the Tikhonov regularization can be seen as a low-order case of our method. We find that the proposed method is superior to the Tikhonov regularization in approximating some ill-conditioned inverse problems, such as robot localization problems. The proposed method overcomes the over-smoothing problem in the Tikhonov regularization as it can use more than one term in the approximation of the matrix inverse, and an explanation for the over-smoothing of the Tikhonov regularization is given. Moreover, one a priori criterion which improves the numerical stability of the ill-conditioned problem is proposed to obtain an optimal regularization matrix. As most of the regularization solutions are biased, we also provide two bias-correction techniques for the proposed high-order regularization. The simulation and experiment results using a sensor network in a 3D environment are discussed, demonstrating the performance of the proposed method.

## Restricted robot localization
A demo for such a restricted environment and experiments can be found in the following video:

<!-- [![Video](uwb_localization_dwm/materials/High-order_regularization_robot_localization.gif)](uwb_localization_dwm/materials/High-order_regularization_robot_localization.gif) -->

<a href="https://youtu.be/2Apt2UHLVNQ" target="_blank"><img src="uwb_localization_dwm/materials/High-order_regularization_robot_localization.gif" alt="HRRL" width="600" height="350" border="10" /></a>

Or find it on:

[Youtube](https://youtu.be/2Apt2UHLVNQ)

[Bilibili](https://www.bilibili.com/video/BV15huCzWE6V/?pop_share=1)

## Updates
<!-- - \[2025.10\] - Presenting at IROS 2025. -->
- \[2025.04\] - Published.
- \[2025.03\] - Accepted by T-RO.


# HR-robot-localization
The high-order regularization method for ill-conditioned problems. Some robot localization problems in restricted situations are ill-conditioned problems.

## uwb_localization_dwm
An implementation of the high-order regularization method for robot localization to deal with the ill-conditioned situations. The robot is equipped with some UWB(Ultra-wideband) sensors with DWM1001c.

## Information of platform

OS: `Ubuntu 20.04 LTS` Or `Ubuntu 20.04` under `WSL2 (and Windows 11)`

ROS: `noetic`

## Dependency
To build and run the package, the following dependencies should be included.
```
pip install pyserial
```

## Get Started

### UWB sensors setup
Follow the steps in [The MDEK1001 User Manual](https://www.qorvo.com/products/d/da007995) or [Decawave's DRTLS Guide](https://www.qorvo.com/products/d/da007996) to set up a UWB network with anchors and active tags using a [DRTLS application](https://www.qorvo.com/products/d/da007984) provided by [Decawave(Qorvo)](https://www.qorvo.com/products/p/MDEK1001#documents). Note that the DRTLS application is not necessary according to the MDEK1001 user manual, which means that you can set up the UWB sensor with a COM APP, such as Tera Term (and cutecom with CR mode if inputs are required).

### UWB sensors connection
If you are using Ubuntu directly or can connect the sensors already, you can skip this step.

```
# Install usbipd-win to connection USB ports in WSL2 (Run in Powershell)
winget install --interactive --exact dorssel.usbipd-win

# Install USBIP tools and hardware database (Run in WSL)
sudo apt install linux-tools-generic hwdata
sudo update-alternatives --install /usr/local/bin/usbip usbip /usr/lib/linux-tools/*-generic/usbip 20

# Connection USB port to Ubuntu (Run in Powershell)
usbipd wsl list
usbipd wsl attach --busid <busid>

# Reconnection USB port in Ubuntu (Run in WSL2)
sudo usbip attach -r $HOSTNAME.local -b <busip> # Run in WSL2 to reconnect <busip>.
```

## Test DWM1001-DEV
### Test UWB with a script
Run the demo to get UWB data from the active tag:

```
python3 get_uwb_data.py
```

## Create ROS package
To download this source code
```
cd ~/catkin_ws/src
git clone https://github.com/LiuxhRobotAI/HR-robot-localization.git

# built
cd ~/catkin_ws
catkin_make
```

## Run ROS package for publishing tags' and anchors' information calculated by DRTLS for DWM1001C
Run the demo to publish the raw data with ROS.
```
roslaunch uwb_localization_dwm demo_pub_dis.launch
```
Or run with a specific port (USB port for UWB tag sensor, e.g. ttyACM0)
```
rosrun uwb_localization_dwm dwm1001_publish_distances.py _port:=/dev/ttyACM0 _tag_name:="tag0"
```

If everything goes well, you will get the following (an example) output.
```
[INFO] [1688393248.888293]: Anchor AN0:  x: 1.0 y: 0.0 z: 0.0
[INFO] [1688393248.889265]: Anchor AN1:  x: 0.0 y: 1.0 z: 0.0
[INFO] [1688393248.890024]: Anchor AN2:  x: 0.0 y: 0.0 z: 0.0
[INFO] [1688393248.890024]: Anchor AN3:  x: 0.0 y: 0.0 z: 1.0
[INFO] [1688393248.890642]: Tag tag:  x: 0.95 y: 0.92 z: 0.32
```

## Run ROS package for publishing tags' localization calculated by lsm and hrm with at least four anchors and one tag
Run the localization.launch to publish the positions of the tag.
```
roslaunch uwb_localization_dwm localization.launch
```
Or run with a specific port (USB port for UWB tag sensor)
```
rosrun uwb_localization_dwm uwb_localization.py _port:=/dev/ttyACM0 _tag_name:="tag0"
```

## Robot control
If you are using a `Jackal` robot to carry the tag and running a simulation [Jackal](http://docs.ros.org/en/latest-available/api/jackal_tutorials/html/simulation.html), you can use the following command to control the robot
```
rosrun uwb_localization_dwm jackal_teleop_key
```

## Calibration of two paths(Odometry)
The `simulation` variable in the main() of calibration.py is used to show a simulation of calibration. Set the `simulation` to False and input two paths to be calibrated. To show the result, please run
```
roslaunch uwb_localization_dwm calibration.launch
```

## Distance data saving and analyzing
The `save_bag_topic_dis.py`  can be used to analyze the statistical features of UWB distance measurements. Follow a `bag_name` to analyze a specific dataset. Example to run
```
python save_bag_topic_dis.py [bag_name] # Analyze the defaut dataset if no [bag_name].
```

## Path data saving and analyzing
The `plot_path_results.py` can be used to save and plot the paths of the robot using different methods. Follow a `bag_name` to analyze a specific dataset. Example to run
```
python save_plot_path_results.py [bag_name] # Analyze the defaut dataset if no [bag_name].
```

## Localization using stored distance data
The `uwb_fuhry_tihkionov.py` can be used to localize and get the paths of the robot using stored distance data with different methods. Example to run
```
python uwb_fuhry_tihkionov.py [bag_name_prefix height]
```

## Localization using stored distance data
The `path_result_analysis.py` can be used to analyze the performance of two methods. The paths of the robot stored different methods. Right now, there are `HRM`, `LS` and `FTR`. The reference is provided by a Lidar using an `A-LOAM` localization method, and the matched reference and estimation paths are aligned by Umeyama's method similar to the `evo`, an evaluation tool. Follow a `bag_name` to analyze a specific dataset. Example to run
```
python path_result_analysis.py [bag_name]

```

## Simulation
In these similations, robot localization problems are used to present the solutions provided by the high-order regularization. Some other regularization methods are also included.

```
cd simulation
[run] HR_RLmain.m
```

## Publications

We kindly recommend to cite [our paper](https://arxiv.org/abs/2410.01919) if you find this high-order regularization method useful:

```latex
@ARTICLE{liu2025high,
  author={Liu, Xinghua and Cao, Ming},
  journal={IEEE Transactions on Robotics},
  title={High-Order Regularization Dealing With ILL-Conditioned Robot Localization Problems},
  year={2025},
  volume={41},
  number={},
  pages={3539-3555},
  doi={10.1109/TRO.2025.3562487}
}
```
