#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>
#include <mutex>
#include <thread>
#include <chrono>

// ROS
#include <ros/ros.h>
#include <ros/spinner.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

#include <cv_bridge/cv_bridge.h>

// DEFINITION COMMUNE AU PROJET
#include <kinect2_bridge/kinect2_definitions.h>


class Record
{
    public:
    bool running;
    enum Camera
    {
        COLOR,
        IR,
        BOTH
    };
};

void main()