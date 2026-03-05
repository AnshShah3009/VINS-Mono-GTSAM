#include <cstdio>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <fstream>
#include <eigen3/Eigen/Dense>

using namespace std;
using namespace Eigen;

const int SKIP = 50;
string benchmark_output_path;
string estimate_output_path;

struct Data
{
    Data(FILE *f)
    {
        if (fscanf(f, " %lf,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f", &t,
               &px, &py, &pz,
               &qw, &qx, &qy, &qz,
               &vx, &vy, &vz,
               &wx, &wy, &wz,
               &ax, &ay, &az) != EOF)
        {
            t /= 1e9;
        }
    }
    double t;
    float px, py, pz;
    float qw, qx, qy, qz;
    float vx, vy, vz;
    float wx, wy, wz;
    float ax, ay, az;
};

class BenchmarkPublisher : public rclcpp::Node
{
public:
    BenchmarkPublisher() : Node("benchmark_publisher")
    {
        this->declare_parameter<string>("data_name", "");
        string csv_file;
        this->get_parameter("data_name", csv_file);

        if (csv_file.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "No data_name parameter provided!");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Loading ground truth: %s", csv_file.c_str());
        FILE *f = fopen(csv_file.c_str(), "r");
        if (f == NULL)
        {
            RCLCPP_ERROR(this->get_logger(), "Can't load ground truth; wrong path: %s", csv_file.c_str());
            return;
        }

        char tmp[10000];
        if (fgets(tmp, 10000, f) == NULL)
        {
            RCLCPP_WARN(this->get_logger(), "Can't load ground truth; no data available");
        }
        while (!feof(f))
            benchmark.emplace_back(f);
        fclose(f);
        if (!benchmark.empty())
            benchmark.pop_back();

        RCLCPP_INFO(this->get_logger(), "Data loaded: %d", (int)benchmark.size());

        pub_odom = this->create_publisher<nav_msgs::msg::Odometry>("odometry", 1000);
        pub_path = this->create_publisher<nav_msgs::msg::Path>("path", 1000);

        sub_odom = this->create_subscription<nav_msgs::msg::Odometry>(
            "estimated_odometry", 1000, std::bind(&BenchmarkPublisher::odom_callback, this, std::placeholders::_1));
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom_msg)
    {
        double odom_time = rclcpp::Time(odom_msg->header.stamp).seconds();

        if (benchmark.empty() || odom_time > benchmark.back().t)
            return;

        for (; idx < static_cast<int>(benchmark.size()) && benchmark[idx].t <= odom_time; idx++)
            ;

        if (init++ < SKIP)
        {
            baseRgt = Quaterniond(odom_msg->pose.pose.orientation.w,
                                  odom_msg->pose.pose.orientation.x,
                                  odom_msg->pose.pose.orientation.y,
                                  odom_msg->pose.pose.orientation.z) *
                      Quaterniond(benchmark[idx - 1].qw,
                                  benchmark[idx - 1].qx,
                                  benchmark[idx - 1].qy,
                                  benchmark[idx - 1].qz).inverse();
            baseTgt = Vector3d{odom_msg->pose.pose.position.x,
                               odom_msg->pose.pose.position.y,
                               odom_msg->pose.pose.position.z} -
                      baseRgt * Vector3d{benchmark[idx - 1].px, benchmark[idx - 1].py, benchmark[idx - 1].pz};
            return;
        }

        nav_msgs::msg::Odometry odometry;
        odometry.header.stamp = rclcpp::Time(static_cast<int64_t>(benchmark[idx - 1].t * 1e9));
        odometry.header.frame_id = "world";
        odometry.child_frame_id = "world";

        Vector3d tmp_T = baseTgt + baseRgt * Vector3d{benchmark[idx - 1].px, benchmark[idx - 1].py, benchmark[idx - 1].pz};
        odometry.pose.pose.position.x = tmp_T.x();
        odometry.pose.pose.position.y = tmp_T.y();
        odometry.pose.pose.position.z = tmp_T.z();

        Quaterniond tmp_R = baseRgt * Quaterniond{benchmark[idx - 1].qw,
                                                  benchmark[idx - 1].qx,
                                                  benchmark[idx - 1].qy,
                                                  benchmark[idx - 1].qz};
        odometry.pose.pose.orientation.w = tmp_R.w();
        odometry.pose.pose.orientation.x = tmp_R.x();
        odometry.pose.pose.orientation.y = tmp_R.y();
        odometry.pose.pose.orientation.z = tmp_R.z();

        Vector3d tmp_V = baseRgt * Vector3d{benchmark[idx - 1].vx,
                                            benchmark[idx - 1].vy,
                                            benchmark[idx - 1].vz};
        odometry.twist.twist.linear.x = tmp_V.x();
        odometry.twist.twist.linear.y = tmp_V.y();
        odometry.twist.twist.linear.z = tmp_V.z();
        pub_odom->publish(odometry);

        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header = odometry.header;
        pose_stamped.pose = odometry.pose.pose;
        path.header = odometry.header;
        path.poses.push_back(pose_stamped);
        pub_path->publish(path);
    }

    int idx = 1;
    vector<Data> benchmark;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom;
    nav_msgs::msg::Path path;

    int init = 0;
    Quaterniond baseRgt;
    Vector3d baseTgt;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BenchmarkPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
