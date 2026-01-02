#pragma once
#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>

extern int ROW;
extern int COL;
extern int FOCAL_LENGTH;
const int NUM_OF_CAM = 1;


extern std::string IMAGE_TOPIC;
extern std::string IMU_TOPIC;
extern std::string FISHEYE_MASK;
extern std::vector<std::string> CAM_NAMES;
extern int MAX_CNT;
extern int MIN_DIST;
extern int WINDOW_SIZE;
extern int FREQ;
extern double F_THRESHOLD;
extern int SHOW_TRACK;
extern int STEREO_TRACK;
extern int EQUALIZE;
extern int FISHEYE;
extern bool PUB_THIS_FRAME;
extern int FAST_THRESHOLD;
extern int USE_BIDIRECTIONAL_FLOW;

// Adaptive feature tracking for high-speed scenarios
extern double MAX_VELOCITY_THRESHOLD;  // m/s - threshold to detect high-speed motion
extern int VELOCITY_BOOST_FEATURES;    // extra features to add at high speed
extern double MIN_PARALLAX_THRESHOLD;  // minimum parallax to accept new keyframe
extern int ENABLE_VELOCITY_CHECK;      // enable velocity-based adaptive tracking

void readParameters(ros::NodeHandle &n);
