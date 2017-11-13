# Related Publications:

[1] Chen X., Lu H., Xiao J., Zhang H., Wang P. (2017) Robust Relocalization Based on Active Loop Closure for Real-Time Monocular SLAM. In: Liu M., Chen H., Vincze M. (eds) Computer Vision Systems. ICVS 2017. Lecture Notes in Computer Science, vol 10528. Springer, Cham. **[PDF](https://www.researchgate.net/publication/318307400_Robust_relocalization_based_on_active_loop_closure_for_real-time_monocular_SLAM)**.

# ORB-SLAM2

**website:** [ORB_SLAM2](https://github.com/raulmur/ORB_SLAM2)
**Authors:** [Raul Mur-Artal](http://webdiis.unizar.es/~raulmur/), [Juan D. Tardos](http://webdiis.unizar.es/~jdtardos/), [J. M. M. Montiel](http://webdiis.unizar.es/~josemari/) and [Dorian Galvez-Lopez](http://doriangalvez.com/) ([DBoW2](https://github.com/dorian3d/DBoW2))



# Building ORB-SLAM2 library

We provide a script `build.sh` to build the *Thirdparty* libraries and *ORB-SLAM2*. Please make sure you have installed all required dependencies (see section 2). Execute:
```
cd ORB_SLAM2
chmod +x build.sh
./build.sh
```

This will create **libORB_SLAM2.so**  at *lib* folder and the executables **mono_tum**, **mono_kitti**, **rgbd_tum**, **stereo_kitti** in *Examples* folder.

# KITTI Dataset  

1. Download the dataset (grayscale images) from http://www.cvlibs.net/datasets/kitti/eval_odometry.php 

2. Execute the following command. Change `KITTIX.yaml`by KITTI00-02.yaml, KITTI03.yaml or KITTI04-12.yaml for sequence 0 to 2, 3, and 4 to 12 respectively. Change `PATH_TO_DATASET_FOLDER` to the uncompressed dataset folder. Change `SEQUENCE_NUMBER` to 00, 01, 02,.., 11. 
```
./Examples/Monocular/mono_kitti Vocabulary/ORBvoc.txt Examples/Monocular/KITTIX.yaml PATH_TO_DATASET_FOLDER/dataset/sequences/SEQUENCE_NUMBER
```

# ROS Examples

### Building the nodes for mono, stereo and RGB-D
1. Add the path including *Examples/ROS/ORB_SLAM2* to the ROS_PACKAGE_PATH environment variable. Open .bashrc file and add at the end the following line. Replace PATH by the folder where you cloned ORB_SLAM2:

  ```
  export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:PATH/ORB_SLAM2/Examples/ROS
  ```
  
2. Go to *Examples/ROS/ORB_SLAM2* folder and execute:

  ```
  mkdir build
  cd build
  cmake .. -DROS_BUILD_TYPE=Release
  make -j
  ```
  
### Running Monocular Node
For a monocular input from topic `/camera/image_raw` run node ORB_SLAM2/Mono. You will need to provide the vocabulary file and a settings file. See the monocular examples above.

  ```
  rosrun ORB_SLAM2 Mono PATH_TO_VOCABULARY PATH_TO_SETTINGS_FILE
  ```
  
### Using roslaunch

For usb-camera:

  ```
  roslaunch SLAM+CAMERA.launch
  ```
For ros-bag:

  ```
  roslaunch SLAM+BAG.launch
  ```
