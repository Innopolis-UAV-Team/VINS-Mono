#include "parameters.h"

std::string IMAGE_TOPIC;
std::string IMU_TOPIC;
std::vector<std::string> CAM_NAMES;
std::string FISHEYE_MASK;
int MAX_CNT;
int MIN_DIST;
int WINDOW_SIZE;
int FREQ;
double F_THRESHOLD;
int SHOW_TRACK;
int STEREO_TRACK;
int EQUALIZE;
int ROW;
int COL;
int FOCAL_LENGTH;
int FISHEYE;
bool PUB_THIS_FRAME;
int FAST_THRESHOLD;
int USE_BIDIRECTIONAL_FLOW;

// Adaptive feature tracking
double MAX_VELOCITY_THRESHOLD = 5.0;  // m/s
int VELOCITY_BOOST_FEATURES = 50;     // extra features at high speed
double MIN_PARALLAX_THRESHOLD = 5.0;  // pixels
int ENABLE_VELOCITY_CHECK = 1;

template <typename T>
T readParam(ros::NodeHandle &n, std::string name)
{
    T ans;
    if (n.getParam(name, ans))
    {
        ROS_INFO_STREAM("Loaded " << name << ": " << ans);
    }
    else
    {
        ROS_ERROR_STREAM("Failed to load " << name);
        n.shutdown();
    }
    return ans;
}

void readParameters(ros::NodeHandle &n)
{
    std::string config_file;
    config_file = readParam<std::string>(n, "config_file");
    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
    }
    std::string VINS_FOLDER_PATH = readParam<std::string>(n, "vins_folder");

    fsSettings["image_topic"] >> IMAGE_TOPIC;
    fsSettings["imu_topic"] >> IMU_TOPIC;
    MAX_CNT = fsSettings["max_cnt"];
    MIN_DIST = fsSettings["min_dist"];
    ROW = fsSettings["image_height"];
    COL = fsSettings["image_width"];
    FREQ = fsSettings["freq"];
    F_THRESHOLD = fsSettings["F_threshold"];
    SHOW_TRACK = fsSettings["show_track"];
    EQUALIZE = fsSettings["equalize"];
    FISHEYE = fsSettings["fisheye"];
    
    // Optional parameter for FAST threshold, default to 20 if not set
    if (!fsSettings["fast_threshold"].empty())
        FAST_THRESHOLD = fsSettings["fast_threshold"];
    else
        FAST_THRESHOLD = 20;
    
    // Optional parameter for bidirectional flow, default to 1 (enabled) if not set
    if (!fsSettings["use_bidirectional_flow"].empty())
        USE_BIDIRECTIONAL_FLOW = fsSettings["use_bidirectional_flow"];
    else
        USE_BIDIRECTIONAL_FLOW = 1;
    
    // Adaptive tracking parameters
    ENABLE_VELOCITY_CHECK = fsSettings["enable_velocity_check"].empty() ? 1 : (int)fsSettings["enable_velocity_check"];
    if (!fsSettings["max_velocity_threshold"].empty())
        MAX_VELOCITY_THRESHOLD = (double)fsSettings["max_velocity_threshold"];
    if (!fsSettings["velocity_boost_features"].empty())
        VELOCITY_BOOST_FEATURES = (int)fsSettings["velocity_boost_features"];
    if (!fsSettings["min_parallax_threshold"].empty())
        MIN_PARALLAX_THRESHOLD = (double)fsSettings["min_parallax_threshold"];
    
    if (ENABLE_VELOCITY_CHECK)
        ROS_INFO("Velocity-adaptive tracking enabled: max_vel=%.1f m/s, boost=%d features",
                 MAX_VELOCITY_THRESHOLD, VELOCITY_BOOST_FEATURES);

    if (FISHEYE == 1)
        FISHEYE_MASK = VINS_FOLDER_PATH + "config/fisheye_mask.jpg";
    CAM_NAMES.push_back(config_file);

    WINDOW_SIZE = 20;
    STEREO_TRACK = false;
    FOCAL_LENGTH = 460;
    PUB_THIS_FRAME = false;

    if (FREQ == 0)
        FREQ = 100;

    fsSettings.release();


}
