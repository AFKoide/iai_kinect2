Droit d'administrateur pour le bus USB
sudo chmod 777 -R /dev/bus/usb

Commande pour lancer le programme :
roslaunch kinect2_bridge kinect2_bridge.launch publish_tf:=true reg_method:=cpu depth_method:=opengl