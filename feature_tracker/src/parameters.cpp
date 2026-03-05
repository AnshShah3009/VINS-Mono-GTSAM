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

template <typename T>
T readParam(std::shared_ptr<rclcpp::Node> n, std::string name)
{
    T ans;
    if (n->has_parameter(name))
    {
        n->get_parameter(name, ans);
        RCLCPP_INFO_STREAM(n->get_logger(), "Loaded " << name << ": " << ans);
    }
    else
    {
        ans = n->declare_parameter<T>(name, T());
        if (n->get_parameter(name, ans))
        {
             RCLCPP_INFO_STREAM(n->get_logger(), "Loaded " << name << ": " << ans);
        }
        else
        {
            RCLCPP_ERROR_STREAM(n->get_logger(), "Failed to load " << name);
            throw std::runtime_error("Failed to load parameter: " + name);
        }
    }
    return ans;
}

void readParameters(std::shared_ptr<rclcpp::Node> n)
{
    std::string config_file;
    try {
        config_file = readParam<std::string>(n, "config_file");
    } catch (const std::exception& e) {
         RCLCPP_ERROR(n->get_logger(), "Please provide config_file path as a parameter");
         return;
    }
    
    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
    }
    
    std::string VINS_FOLDER_PATH;
    try {
        VINS_FOLDER_PATH = readParam<std::string>(n, "vins_folder");
    } catch (...) {
        VINS_FOLDER_PATH = "./"; 
    }

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

