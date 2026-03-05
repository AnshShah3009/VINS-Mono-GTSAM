#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/bool.hpp>
#include <cv_bridge/cv_bridge.hpp>

#include "feature_tracker.h"

#define SHOW_UNDISTORTION 0

class FeatureTrackerNode : public rclcpp::Node
{
public:
    FeatureTrackerNode() : Node("feature_tracker")
    {
        readParameters(shared_from_this());

        for (int i = 0; i < NUM_OF_CAM; i++)
            trackerData[i].readIntrinsicParameter(CAM_NAMES[i]);

        if (FISHEYE)
        {
            for (int i = 0; i < NUM_OF_CAM; i++)
            {
                trackerData[i].fisheye_mask = cv::imread(FISHEYE_MASK, 0);
                if (!trackerData[i].fisheye_mask.data)
                {
                    RCLCPP_ERROR(this->get_logger(), "load mask fail");
                    rclcpp::shutdown();
                }
                else
                    RCLCPP_INFO(this->get_logger(), "load mask success");
            }
        }

        sub_img = this->create_subscription<sensor_msgs::msg::Image>(
            IMAGE_TOPIC, 100, std::bind(&FeatureTrackerNode::img_callback, this, std::placeholders::_1));

        pub_img = this->create_publisher<sensor_msgs::msg::PointCloud>("feature", 1000);
        pub_match = this->create_publisher<sensor_msgs::msg::Image>("feature_img", 1000);
        pub_restart = this->create_publisher<std_msgs::msg::Bool>("restart", 1000);
    }

private:
    void img_callback(const sensor_msgs::msg::Image::SharedPtr img_msg)
    {
        double timestamp = rclcpp::Time(img_msg->header.stamp).seconds();
        if (first_image_flag)
        {
            first_image_flag = false;
            first_image_time = timestamp;
            last_image_time = timestamp;
            return;
        }
        // detect unstable camera stream
        if (timestamp - last_image_time > 1.0 || timestamp < last_image_time)
        {
            RCLCPP_WARN(this->get_logger(), "image discontinue! reset the feature tracker!");
            first_image_flag = true;
            last_image_time = 0;
            pub_count = 1;
            std_msgs::msg::Bool restart_flag;
            restart_flag.data = true;
            pub_restart->publish(restart_flag);
            return;
        }
        last_image_time = timestamp;
        // frequency control
        if (round(1.0 * pub_count / (timestamp - first_image_time)) <= FREQ)
        {
            PUB_THIS_FRAME = true;
            // reset the frequency control
            if (abs(1.0 * pub_count / (timestamp - first_image_time) - FREQ) < 0.01 * FREQ)
            {
                first_image_time = timestamp;
                pub_count = 0;
            }
        }
        else
            PUB_THIS_FRAME = false;

        cv_bridge::CvImageConstPtr ptr;
        if (img_msg->encoding == "8UC1")
        {
            sensor_msgs::msg::Image img;
            img.header = img_msg->header;
            img.height = img_msg->height;
            img.width = img_msg->width;
            img.is_bigendian = img_msg->is_bigendian;
            img.step = img_msg->step;
            img.data = img_msg->data;
            img.encoding = "mono8";
            ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
        }
        else
            ptr = cv_bridge::toCvCopy(*img_msg, sensor_msgs::image_encodings::MONO8);

        cv::Mat show_img = ptr->image;
        TicToc t_r;
        for (int i = 0; i < NUM_OF_CAM; i++)
        {
            RCLCPP_DEBUG(this->get_logger(), "processing camera %d", i);
            if (i != 1 || !STEREO_TRACK)
                trackerData[i].readImage(ptr->image.rowRange(ROW * i, ROW * (i + 1)), timestamp);
            else
            {
                if (EQUALIZE)
                {
                    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
                    clahe->apply(ptr->image.rowRange(ROW * i, ROW * (i + 1)), trackerData[i].cur_img);
                }
                else
                    trackerData[i].cur_img = ptr->image.rowRange(ROW * i, ROW * (i + 1));
            }

#if SHOW_UNDISTORTION
            trackerData[i].showUndistortion("undistrotion_" + std::to_string(i));
#endif
        }

        for (unsigned int i = 0;; i++)
        {
            bool completed = false;
            for (int j = 0; j < NUM_OF_CAM; j++)
                if (j != 1 || !STEREO_TRACK)
                    completed |= trackerData[j].updateID(i);
            if (!completed)
                break;
        }

        if (PUB_THIS_FRAME)
        {
            pub_count++;
            auto feature_points = std::make_shared<sensor_msgs::msg::PointCloud>();
            sensor_msgs::msg::ChannelFloat32 id_of_point;
            sensor_msgs::msg::ChannelFloat32 u_of_point;
            sensor_msgs::msg::ChannelFloat32 v_of_point;
            sensor_msgs::msg::ChannelFloat32 velocity_x_of_point;
            sensor_msgs::msg::ChannelFloat32 velocity_y_of_point;

            feature_points->header = img_msg->header;
            feature_points->header.frame_id = "world";

            for (int i = 0; i < NUM_OF_CAM; i++)
            {
                auto &un_pts = trackerData[i].cur_un_pts;
                auto &cur_pts = trackerData[i].cur_pts;
                auto &ids = trackerData[i].ids;
                auto &pts_velocity = trackerData[i].pts_velocity;
                for (unsigned int j = 0; j < ids.size(); j++)
                {
                    if (trackerData[i].track_cnt[j] > 1)
                    {
                        int p_id = ids[j];
                        geometry_msgs::msg::Point32 p;
                        p.x = un_pts[j].x;
                        p.y = un_pts[j].y;
                        p.z = 1;

                        feature_points->points.push_back(p);
                        id_of_point.values.push_back(p_id * NUM_OF_CAM + i);
                        u_of_point.values.push_back(cur_pts[j].x);
                        v_of_point.values.push_back(cur_pts[j].y);
                        velocity_x_of_point.values.push_back(pts_velocity[j].x);
                        velocity_y_of_point.values.push_back(pts_velocity[j].y);
                    }
                }
            }
            feature_points->channels.push_back(id_of_point);
            feature_points->channels.push_back(u_of_point);
            feature_points->channels.push_back(v_of_point);
            feature_points->channels.push_back(velocity_x_of_point);
            feature_points->channels.push_back(velocity_y_of_point);
            
            RCLCPP_DEBUG(this->get_logger(), "publish %f, at %f", timestamp, this->now().seconds());
            
            if (!init_pub)
            {
                init_pub = 1;
            }
            else
                pub_img->publish(*feature_points);

            if (SHOW_TRACK)
            {
                auto bgr_ptr = cv_bridge::cvtColor(ptr, sensor_msgs::image_encodings::BGR8);
                cv::Mat stereo_img = bgr_ptr->image;

                for (int i = 0; i < NUM_OF_CAM; i++)
                {
                    cv::Mat tmp_img = stereo_img.rowRange(i * ROW, (i + 1) * ROW);
                    cv::cvtColor(show_img, tmp_img, cv::COLOR_GRAY2RGB);

                    for (unsigned int j = 0; j < trackerData[i].cur_pts.size(); j++)
                    {
                        double len = std::min(1.0, 1.0 * trackerData[i].track_cnt[j] / WINDOW_SIZE);
                        cv::circle(tmp_img, trackerData[i].cur_pts[j], 2, cv::Scalar(255 * (1 - len), 0, 255 * len), 2);
                    }
                }
                pub_match->publish(*(bgr_ptr->toImageMsg()));
            }
        }
        RCLCPP_INFO(this->get_logger(), "whole feature tracker processing costs: %f", t_r.toc());
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_img;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr pub_img;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_match;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_restart;

    FeatureTracker trackerData[NUM_OF_CAM];
    double first_image_time;
    int pub_count = 1;
    bool first_image_flag = true;
    double last_image_time = 0;
    bool init_pub = 0;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FeatureTrackerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}



// new points velocity is 0, pub or not?
// track cnt > 1 pub?