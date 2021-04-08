# ISP2021-cad2cav
This is the github project for the F1Tenth Independent Study Projects 2021. In this project we want to plan the paths for multi-agent given a prior of an environment like a blueprint of the building’s floor plan.

![](docs/img/unreal.gif)
![](docs/img/automapper.gif)

## System Requirements
- Linux Ubuntu (tested on 20.04 LTS)
- ROS Noetic

## Software Requirements
- Boost 1.71
- LibConfig++ 1.5.0
- [OpenCV 4.4.0](https://github.com/opencv/opencv/tree/4.4.0)
- [Google OR-Tools 8.2](https://github.com/google/or-tools/releases/tag/v8.2)
- [OSQP 0.6.2](https://github.com/oxfordcontrol/osqp/releases/tag/v0.6.2)
- [OSQP-Eigen 0.6.3](https://github.com/robotology/osqp-eigen/releases/tag/v0.6.3)
- [METIS 5.1.0](http://glaros.dtc.umn.edu/gkhome/metis/metis/overview)

This project also depends on [another UE software](https://github.com/shineyruan/unreal_levine_4) that provides initial landmark locations for the planner to plan. One should also have the following:
- Autodesk Revit 2019
- Autodesk AutoCAD 2019
- Unreal Engine 4.23.1

## Installation
1. This repo serves as a collection of ROS packages and you can plug it directly into any Catkin workspaces.
```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin_init_workspace
cd src && git clone --recursive https://github.com/mlab-upenn/ISP2021-cad2cav.git
cd ..
catkin_make
source devel/setup.bash
```

You can install LibConfig++, Boost and METIS library through apt repository:
```bash
sudo apt install libconfig++-dev libboost-all-dev libmetis-dev
```

Other dependencies (OpenCV, Google OR-Tools, OSQP, OSQP-Eigen) has to be built from source manually. 

**Note. In order to integrate OR-Tools libraries into the CMake build system, OR-Tools itself needs to be built with CMake. You can find instructions on how to build OR-Tools with CMake [here.](https://github.com/google/or-tools/tree/v8.2/cmake)**

2. Developers of this project should also have Unreal Engine 4.23.1 installed in the system. From this point onwards, we assume that your UE4 is cloned and installed in directory `${UE4_ROOT}`.
```bash
cd ~
git clone --recursive https://github.com/shineyruan/unreal_levine_4
```

One can check the installation of UE4 by trying to open the project in UE4 Editor:
```bash
cd ${UE4_ROOT}/Engine/Binaries/Linux
./UE4Editor ~/unreal_levine_4/Levine_4.uproject
```

## Running the code
1. **Get landmark locations from UE4 gameplay.** Open the UE4 project and hit "Play". During gameplay, one should right-click on the landmarks in UE4 scene and the game would automatically save their locations in `~/unreal_levine_4/csv/actorLocation.csv`.
2. **Copy the CSV file into ROS project.**
```bash
cp ~/unreal_levine_4/csv/actorLocation.csv ~/catkin_ws/src/ISP2021-cad2cav/auto_mapping_ros/csv
```
3. **Run Automapper to generate waypoint sequences** for vehicles to follow. 
```bash
rosrun auto_mapping_ros coverage_sequence_creator_cad
```
Click on the pop-up window to specify the location of the first-3 landmarks in UE4 scene. This step is meant to calculate the transformation of coordinates from UE4 scene to ROS map. When the program finishes, you should have multiple files `csv/sequence_1.csv`, `csv/sequence_2.csv`, depending on how many vehicles you have specified.

4. **Run the Automapper simulations.**
In Terminal 1:
```bash
roslaunch f110_simulator simulator.launch
```
In Terminal 2:
```bash
roslaunch auto_mapping_ros auto_mapping_ros.launch
```





## Folder Structure
To be continued...
<!-- All main scripts depend on the following subfolders:

1. Folder 1 contains the files for xxx...
2. Folder 2 contains the files for... -->


## Files
To be continued...
| <!--    |                                | File | Description |
| ------- | ------------------------------ |
| main.py | Is used to start the algorithm |
| test.py | Is used to create the results  |      | -->         |
