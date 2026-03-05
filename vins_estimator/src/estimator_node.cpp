#include <stdio.h>
#include <queue>
#include <map>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <std_msgs/msg/bool.hpp>

#include "estimator.h"
#include "parameters.h"
#include "utility/visualization.h"

Estimator estimator;

std::condition_variable con;
double current_time = -1;
std::queue<sensor_msgs::msg::Imu::SharedPtr> imu_buf;
std::queue<sensor_msgs::msg::PointCloud::SharedPtr> feature_buf;
std::queue<sensor_msgs::msg::PointCloud::SharedPtr> relo_buf;
int sum_of_wait = 0;

std::mutex m_buf;
std::mutex m_state;
std::mutex i_buf;
std::mutex m_estimator;

double latest_time;
Eigen::Vector3d tmp_P;
Eigen::Quaterniond tmp_Q;
Eigen::Vector3d tmp_V;
Eigen::Vector3d tmp_Ba;
Eigen::Vector3d tmp_Bg;
Eigen::Vector3d acc_0;
Eigen::Vector3d gyr_0;
bool init_feature = 0;
bool init_imu = 1;
double last_imu_t = 0;

void predict(const sensor_msgs::msg::Imu::SharedPtr imu_msg)
{
    double t = rclcpp::Time(imu_msg->header.stamp).seconds();
    if (init_imu)
    {
        latest_time = t;
        init_imu = 0;
        return;
    }
    double dt = t - latest_time;
    latest_time = t;

    double dx = imu_msg->linear_acceleration.x;
    double dy = imu_msg->linear_acceleration.y;
    double dz = imu_msg->linear_acceleration.z;
    Eigen::Vector3d linear_acceleration{dx, dy, dz};

    double rx = imu_msg->angular_velocity.x;
    double ry = imu_msg->angular_velocity.y;
    double rz = imu_msg->angular_velocity.z;
    Eigen::Vector3d angular_velocity{rx, ry, rz};

    Eigen::Vector3d un_acc_0 = tmp_Q * (acc_0 - tmp_Ba) - estimator.g;

    Eigen::Vector3d un_gyr = 0.5 * (gyr_0 + angular_velocity) - tmp_Bg;
    tmp_Q = tmp_Q * Utility::deltaQ(un_gyr * dt);

    Eigen::Vector3d un_acc_1 = tmp_Q * (linear_acceleration - tmp_Ba) - estimator.g;

    Eigen::Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);

    tmp_P = tmp_P + dt * tmp_V + 0.5 * dt * dt * un_acc;
    tmp_V = tmp_V + dt * un_acc;

    acc_0 = linear_acceleration;
    gyr_0 = angular_velocity;
}

void update()
{
    latest_time = current_time;
    tmp_P = estimator.Ps[WINDOW_SIZE];
    tmp_Q = estimator.Rs[WINDOW_SIZE];
    tmp_V = estimator.Vs[WINDOW_SIZE];
    tmp_Ba = estimator.Bas[WINDOW_SIZE];
    tmp_Bg = estimator.Bgs[WINDOW_SIZE];
    acc_0 = estimator.acc_0;
    gyr_0 = estimator.gyr_0;

    std::queue<sensor_msgs::msg::Imu::SharedPtr> tmp_imu_buf = imu_buf;
    while (!tmp_imu_buf.empty()) {
        predict(tmp_imu_buf.front());
        tmp_imu_buf.pop();
    }
}

std::vector<std::pair<std::vector<sensor_msgs::msg::Imu::SharedPtr>, sensor_msgs::msg::PointCloud::SharedPtr>>
getMeasurements(rclcpp::Logger logger)
{
    std::vector<std::pair<std::vector<sensor_msgs::msg::Imu::SharedPtr>, sensor_msgs::msg::PointCloud::SharedPtr>> measurements;

    while (true)
    {
        if (imu_buf.empty() || feature_buf.empty())
            return measurements;

        if (!(rclcpp::Time(imu_buf.back()->header.stamp).seconds() > rclcpp::Time(feature_buf.front()->header.stamp).seconds() + estimator.td))
        {
            sum_of_wait++;
            return measurements;
        }

        if (!(rclcpp::Time(imu_buf.front()->header.stamp).seconds() < rclcpp::Time(feature_buf.front()->header.stamp).seconds() + estimator.td))
        {
            RCLCPP_WARN(logger, "throw img, only should happen at the beginning");
            feature_buf.pop();
            continue;
        }
        sensor_msgs::msg::PointCloud::SharedPtr img_msg = feature_buf.front();
        feature_buf.pop();

        std::vector<sensor_msgs::msg::Imu::SharedPtr> IMUs;
        while (rclcpp::Time(imu_buf.front()->header.stamp).seconds() < rclcpp::Time(img_msg->header.stamp).seconds() + estimator.td)
        {
            IMUs.emplace_back(imu_buf.front());
            imu_buf.pop();
        }
        IMUs.emplace_back(imu_buf.front());
        if (IMUs.empty())
            RCLCPP_WARN(logger, "no imu between two image");
        measurements.emplace_back(IMUs, img_msg);
    }
    return measurements;
}

class EstimatorNode : public rclcpp::Node
{
public:
    EstimatorNode() : Node("vins_estimator")
    {
        readParameters(shared_from_this());
        estimator.setParameter();
        
        RCLCPP_WARN(this->get_logger(), "waiting for image and imu...");
        registerPub(shared_from_this());

        sub_imu = this->create_subscription<sensor_msgs::msg::Imu>(
            IMU_TOPIC, rclcpp::QoS(2000).best_effort().durability_volatile(),
            std::bind(&EstimatorNode::imu_callback, this, std::placeholders::_1));

        sub_image = this->create_subscription<sensor_msgs::msg::PointCloud>(
            "/feature_tracker/feature", 2000,
            std::bind(&EstimatorNode::feature_callback, this, std::placeholders::_1));

        sub_restart = this->create_subscription<std_msgs::msg::Bool>(
            "/feature_tracker/restart", 2000,
            std::bind(&EstimatorNode::restart_callback, this, std::placeholders::_1));

        sub_relo_points = this->create_subscription<sensor_msgs::msg::PointCloud>(
            "/pose_graph/match_points", 2000,
            std::bind(&EstimatorNode::relocalization_callback, this, std::placeholders::_1));

        measurement_process = std::thread(&EstimatorNode::process, this);
    }

    ~EstimatorNode()
    {
        if (measurement_process.joinable())
            measurement_process.join();
    }

private:
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr imu_msg)
    {
        double timestamp = rclcpp::Time(imu_msg->header.stamp).seconds();
        if (timestamp <= last_imu_t)
        {
            RCLCPP_WARN(this->get_logger(), "imu message in disorder!");
            return;
        }
        last_imu_t = timestamp;

        {
            std::unique_lock<std::mutex> lk(m_buf);
            imu_buf.push(imu_msg);
        }
        con.notify_one();

        {
            std::lock_guard<std::mutex> lg(m_state);
            predict(imu_msg);
            std_msgs::msg::Header header = imu_msg->header;
            header.frame_id = "world";
            if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
                pubLatestOdometry(tmp_P, tmp_Q, tmp_V, header);
        }
    }

    void feature_callback(const sensor_msgs::msg::PointCloud::SharedPtr feature_msg)
    {
        if (!init_feature)
        {
            init_feature = 1;
            return;
        }
        {
            std::unique_lock<std::mutex> lk(m_buf);
            feature_buf.push(feature_msg);
        }
        con.notify_one();
    }

    void restart_callback(const std_msgs::msg::Bool::SharedPtr restart_msg)
    {
        if (restart_msg->data == true)
        {
            RCLCPP_WARN(this->get_logger(), "restart the estimator!");
            std::unique_lock<std::mutex> lk(m_buf);
            while(!feature_buf.empty()) feature_buf.pop();
            while(!imu_buf.empty()) imu_buf.pop();
            lk.unlock();
            
            std::lock_guard<std::mutex> lg(m_estimator);
            estimator.clearState();
            estimator.setParameter();
            current_time = -1;
            last_imu_t = 0;
        }
    }

    void relocalization_callback(const sensor_msgs::msg::PointCloud::SharedPtr points_msg)
    {
        std::unique_lock<std::mutex> lk(m_buf);
        relo_buf.push(points_msg);
    }

    void process()
    {
        while (rclcpp::ok())
        {
            std::vector<std::pair<std::vector<sensor_msgs::msg::Imu::SharedPtr>, sensor_msgs::msg::PointCloud::SharedPtr>> measurements;
            std::unique_lock<std::mutex> lk(m_buf);
            con.wait(lk, [&] {
                return (measurements = getMeasurements(this->get_logger())).size() != 0 || !rclcpp::ok();
            });
            if (!rclcpp::ok()) break;
            lk.unlock();

            m_estimator.lock();
            for (auto &measurement : measurements)
            {
                auto img_msg = measurement.second;
                double dx = 0, dy = 0, dz = 0, rx = 0, ry = 0, rz = 0;
                for (auto &imu_msg : measurement.first)
                {
                    double t = rclcpp::Time(imu_msg->header.stamp).seconds();
                    double img_t = rclcpp::Time(img_msg->header.stamp).seconds() + estimator.td;
                    if (t <= img_t)
                    { 
                        if (current_time < 0)
                            current_time = t;
                        double dt = t - current_time;
                        current_time = t;
                        dx = imu_msg->linear_acceleration.x;
                        dy = imu_msg->linear_acceleration.y;
                        dz = imu_msg->linear_acceleration.z;
                        rx = imu_msg->angular_velocity.x;
                        ry = imu_msg->angular_velocity.y;
                        rz = imu_msg->angular_velocity.z;
                        estimator.processIMU(dt, Eigen::Vector3d(dx, dy, dz), Eigen::Vector3d(rx, ry, rz));
                    }
                    else
                    {
                        double dt_1 = img_t - current_time;
                        double dt_2 = t - img_t;
                        current_time = img_t;
                        double w1 = dt_2 / (dt_1 + dt_2);
                        double w2 = dt_1 / (dt_1 + dt_2);
                        dx = w1 * dx + w2 * imu_msg->linear_acceleration.x;
                        dy = w1 * dy + w2 * imu_msg->linear_acceleration.y;
                        dz = w1 * dz + w2 * imu_msg->linear_acceleration.z;
                        rx = w1 * rx + w2 * imu_msg->angular_velocity.x;
                        ry = w1 * ry + w2 * imu_msg->angular_velocity.y;
                        rz = w1 * rz + w2 * imu_msg->angular_velocity.z;
                        estimator.processIMU(dt_1, Eigen::Vector3d(dx, dy, dz), Eigen::Vector3d(rx, ry, rz));
                    }
                }
                
                sensor_msgs::msg::PointCloud::SharedPtr relo_msg = nullptr;
                while (!relo_buf.empty())
                {
                    relo_msg = relo_buf.front();
                    relo_buf.pop();
                }
                if (relo_msg != nullptr)
                {
                    std::vector<Eigen::Vector3d> match_points;
                    double frame_stamp = rclcpp::Time(relo_msg->header.stamp).seconds();
                    for (unsigned int i = 0; i < relo_msg->points.size(); i++)
                    {
                        Eigen::Vector3d u_v_id;
                        u_v_id.x() = relo_msg->points[i].x;
                        u_v_id.y() = relo_msg->points[i].y;
                        u_v_id.z() = relo_msg->points[i].z;
                        match_points.push_back(u_v_id);
                    }
                    Eigen::Vector3d relo_t(relo_msg->channels[0].values[0], relo_msg->channels[0].values[1], relo_msg->channels[0].values[2]);
                    Eigen::Quaterniond relo_q(relo_msg->channels[0].values[3], relo_msg->channels[0].values[4], relo_msg->channels[0].values[5], relo_msg->channels[0].values[6]);
                    Eigen::Matrix3d relo_r = relo_q.toRotationMatrix();
                    int frame_index = static_cast<int>(relo_msg->channels[0].values[7]);
                    estimator.setReloFrame(frame_stamp, frame_index, match_points, relo_t, relo_r);
                }

                RCLCPP_DEBUG(this->get_logger(), "processing vision data with stamp %f \n", rclcpp::Time(img_msg->header.stamp).seconds());

                TicToc t_s;
                std::map<int, std::vector<std::pair<int, Eigen::Matrix<double, 7, 1>>>> image;
                for (unsigned int i = 0; i < img_msg->points.size(); i++)
                {
                    int v = img_msg->channels[0].values[i] + 0.5;
                    int feature_id = v / NUM_OF_CAM;
                    int camera_id = v % NUM_OF_CAM;
                    double x = img_msg->points[i].x;
                    double y = img_msg->points[i].y;
                    double z = img_msg->points[i].z;
                    double p_u = img_msg->channels[1].values[i];
                    double p_v = img_msg->channels[2].values[i];
                    double velocity_x = img_msg->channels[3].values[i];
                    double velocity_y = img_msg->channels[4].values[i];
                    Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
                    xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
                    image[feature_id].emplace_back(camera_id,  xyz_uv_velocity);
                }
                estimator.processImage(image, img_msg->header);

                double whole_t = t_s.toc();
                printStatistics(estimator, whole_t);
                std_msgs::msg::Header header = img_msg->header;
                header.frame_id = "world";

                pubOdometry(estimator, header);
                pubKeyPoses(estimator, header);
                pubCameraPose(estimator, header);
                pubPointCloud(estimator, header);
                pubTF(estimator, header);
                pubKeyframe(estimator);
                if (relo_msg != nullptr)
                    pubRelocalization(estimator);
            }
            m_estimator.unlock();
            
            {
                std::lock_guard<std::mutex> lk_buf(m_buf);
                std::lock_guard<std::mutex> lk_state(m_state);
                if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
                    update();
            }
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud>::SharedPtr sub_image;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_restart;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud>::SharedPtr sub_relo_points;

    std::thread measurement_process;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<EstimatorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

