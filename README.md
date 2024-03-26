# TUTORIEL: IAI_KINECTv2

### Droit d'administrateur pour le bus USB
```
sudo chmod 777 -R /dev/bus/usb
```

### Changer la carte graphique a utiliser:
```
sudo prime-select nvidia
sudo prime-select intel
```

### Tester la caméra:
```
sudo libfreenect2/build/bin/Protonect
```

## Commande pour lancer iai:
#### 1) Sur un terminal, lancer ros:
```
source /opt/ros/noetic/setup.bash && roscore
```

#### 2) Sur un second terminal, lancer le bridge: 
```
cd catkin_ws && source devel/setup.bash
sudo chmod 777 -R /dev/bus/usb
roslaunch kinect2_bridge kinect2_bridge.launch publish_tf:=true reg_method:=cpu depth_method:=opengl _fps_limit:=1
```

#### 3) Sur un troisième terminal, lancer la calibration:
```
cd catkin_ws && source devel/setup.bash && cd ~/kinect_cal_data
mkdir ~/kinect_cal_data && cd ~/kinect_cal_data
```

##### Couleur:
```
rosrun kinect2_calibration kinect2_calibration chess5x7x0.03 record color
rosrun kinect2_calibration kinect2_calibration chess5x7x0.03 calibrate color
```

##### Infrarouge:
```
rosrun kinect2_calibration kinect2_calibration chess5x7x0.03 record ir
rosrun kinect2_calibration kinect2_calibration chess5x7x0.03 calibrate ir
```

##### Synchronisé:
```
rosrun kinect2_calibration kinect2_calibration chess5x7x0.03 record sync
rosrun kinect2_calibration kinect2_calibration chess5x7x0.03 calibrate sync
rosrun kinect2_calibration kinect2_calibration chess5x7x0.03 calibrate depth
```

#### Stocker les données de configuration
- Trouver le numéro de série.	
- roscd kinect2_bridge/data; mkdir [numéro de série]


#### 4/ Voir les résultats:
rosrun kinect2_viewer kinect2_viewer image

rosrun kinect2_viewer kinect2_viewer cloud

rosrun kinect2_viewer kinect2_viewer hd both




## Résultats
```
     avg: -0.0666907
     var: 0.000507086
  stddev: 0.0225186
     rms: 0.0703899
  median: -0.0722807
```

- Color
```
[0.01741028210295193, -0.2166430421664149, -0.001386609164906177, -0.04721894724647839, 0.1518956436721509]
```

- IR
```
[0.05082085099111906, -0.1192824255262204, -0.01048585440482947, 0.01465124113965962, 0.03637082571849185]
```

- Sync
```
[ INFO] [CameraCalibration::calibrateExtrinsics] Camera Matrix Color:
[3108.365648632343, 0, 2059.294624809378;
 0, 988.7132860413669, 537.8098456037209;
 0, 0, 1]
[ INFO] [CameraCalibration::calibrateExtrinsics] Distortion Coeeficients Color:
[0.01741028210295193, -0.2166430421664149, -0.001386609164906177, -0.04721894724647839, 0.1518956436721509]

[ INFO] [CameraCalibration::calibrateExtrinsics] Camera Matrix Ir:
[326.9108752938313, 0, 271.0901672601053;
 0, 324.8649090527477, 206.5844701290807;
 0, 0, 1]
[ INFO] [CameraCalibration::calibrateExtrinsics] Distortion Coeeficients Ir:
[0.05082085099111906, -0.1192824255262204, -0.01048585440482947, 0.01465124113965962, 0.03637082571849185]
```

- Depth
```
[ INFO] [CameraCalibration::calibrateExtrinsics] calibrating Color and Ir extrinsics...
[ INFO] [CameraCalibration::calibrateExtrinsics] re-projection error: 0

[ INFO] [CameraCalibration::calibrateExtrinsics] Rotation:
[]
[ INFO] [CameraCalibration::calibrateExtrinsics] Translation:
[]
[ INFO] [CameraCalibration::calibrateExtrinsics] Essential:
[]
[ INFO] [CameraCalibration::calibrateExtrinsics] Fundamental:
[]
```