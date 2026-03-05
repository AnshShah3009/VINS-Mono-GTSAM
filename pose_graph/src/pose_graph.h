#pragma once

#include <thread>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>
#include <string>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <queue>
#include <assert.h>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <stdio.h>
#include <rclcpp/rclcpp.hpp>
#include "keyframe.h"
#include "utility/tic_toc.h"
#include "utility/utility.h"
#include "utility/CameraPoseVisualization.h"
#include "utility/tic_toc.h"
#include "ThirdParty/DBoW/DBoW2.h"
#include "ThirdParty/DVision/DVision.h"
#include "ThirdParty/DBoW/TemplatedDatabase.h"
#include "ThirdParty/DBoW/TemplatedVocabulary.h"

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/slam/dataset.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/ISAM2.h>

#define SHOW_S_EDGE false
#define SHOW_L_EDGE true
#define SAVE_LOOP_PATH true

using namespace DVision;
using namespace DBoW2;

class PoseGraph
{
public:
	PoseGraph();
	~PoseGraph();
	void registerPub(rclcpp::Node::SharedPtr n);						// to register publishers
	void addKeyFrame(KeyFrame *cur_kf, bool flag_detect_loop);	// to add keyframe to keyframelist, also adds factors to factor graph
	void loadKeyFrame(KeyFrame *cur_kf, bool flag_detect_loop); // to load keyframe to keyframelist, also adds factors to factor graph
	void loadVocabulary(std::string voc_path);
	void updateKeyFrameLoop(int index, Eigen::Matrix<double, 8, 1> &_loop_info);
	KeyFrame *getKeyFrame(int index);
	nav_msgs::msg::Path path[10];
	nav_msgs::msg::Path base_path;
	CameraPoseVisualization *posegraph_visualization;
	void savePoseGraph();
	void loadPoseGraph();
	void publish();
	Vector3d t_drift;
	double yaw_drift;
	Matrix3d r_drift;
	// world frame( base sequence or first sequence)<----> cur sequence frame
	Vector3d w_t_vio;
	Matrix3d w_r_vio;

private:
	//  variables for gtsam
	gtsam::ISAM2 *isam2;						   // gtsam isam2 optimizer
	gtsam::ISAM2Params isam2_params;		   // gtsam isam2 parameters
	gtsam::GaussNewtonParams params;					   // gtsam GNC optimizer
	gtsam::NonlinearFactorGraph graph;					   // factor graph pointer for gtsam
	std::string g20File;								   // file name for g2o file
	gtsam::Values initial;								   // to store dafault values
	gtsam::Values optimized;							   // to store optimized values
	gtsam::noiseModel::Diagonal::shared_ptr priorModel;	   // prior model error for gtsam
	gtsam::noiseModel::Diagonal::shared_ptr gravityPriorModel; // model to enforce 4-DoF (lock roll/pitch)
	gtsam::noiseModel::Diagonal::shared_ptr odometryModel; // odometry model error for gtsam
	gtsam::noiseModel::Diagonal::shared_ptr infiniteModel; // to Add Prior for new sequences
	gtsam::noiseModel::Diagonal::shared_ptr loopModel;	   // loop model error for gtsam
	gtsam::noiseModel::Diagonal::shared_ptr infiModel;	   // infi model error for gtsam
	bool is3D = true;									   // parameter for file reading and writing

	int detectLoop(KeyFrame *keyframe, int frame_index); // detect loop function to check for loop detects and set flag, also call Pnp function to set positions between frames
	void addKeyFrameIntoVoc(KeyFrame *keyframe);		 // adds keyframe into vocabulary
	void optimize4DoF();
	void updatePath();

	std::list<KeyFrame *> keyframelist; // all keyframe

	std::mutex m_keyframelist; // mutex lock for keyframelist
	std::mutex m_optimize_buf; // mutext lock for for optimize_buf
	std::mutex m_path;		   // mutex lock for path
	std::mutex m_drift;		   // mutext lock for drift
	std::mutex m_posegraph;	   // mutex lock for posegraph

	std::thread t_optimization; // thread for optimization

	// for information sharing between mutex locks
	std::queue<int> optimize_buf; // for optimization
	int addedFactorsTill;		  // to keep track of last keyframe added to factorgraph

	int global_index;
	int sequence_cnt;
	std::vector<bool> sequence_loop; // keeps track if the sequence has detected a loop clousre with any other sequence
	std::map<int, cv::Mat> image_pool;
	int earliest_loop_index;
	int base_sequence;

	BriefDatabase db;
	BriefVocabulary *voc;

	// Publishers for paths
	rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_pg_path;
	rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_base_path;
	rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_pose_graph;
	rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path[10];
};

// To normalize angle and get value between 0 to 360
template <typename T>
T NormalizeAngle(const T &angle_degrees)
{
	if (angle_degrees > T(180.0))
		return angle_degrees - T(360.0);
	else if (angle_degrees < T(-180.0))
		return angle_degrees + T(360.0);
	else
		return angle_degrees;
};

// Yaw Pitch Roll to Rotation Matrix
template <typename T>
void YawPitchRollToRotationMatrix(const T yaw, const T pitch, const T roll, T R[9])
{

	T y = yaw / T(180.0) * T(M_PI);
	T p = pitch / T(180.0) * T(M_PI);
	T r = roll / T(180.0) * T(M_PI);

	R[0] = cos(y) * cos(p);
	R[1] = -sin(y) * cos(r) + cos(y) * sin(p) * sin(r);
	R[2] = sin(y) * sin(r) + cos(y) * sin(p) * cos(r);
	R[3] = sin(y) * cos(p);
	R[4] = cos(y) * cos(r) + sin(y) * sin(p) * sin(r);
	R[5] = -cos(y) * sin(r) + sin(y) * sin(p) * cos(r);
	R[6] = -sin(p);
	R[7] = cos(p) * sin(r);
	R[8] = cos(p) * cos(r);
};

// Rotation Matrix transpose
template <typename T>
void RotationMatrixTranspose(const T R[9], T inv_R[9])
{
	inv_R[0] = R[0];
	inv_R[1] = R[3];
	inv_R[2] = R[6];
	inv_R[3] = R[1];
	inv_R[4] = R[4];
	inv_R[5] = R[7];
	inv_R[6] = R[2];
	inv_R[7] = R[5];
	inv_R[8] = R[8];
};

// Point after Rotation
template <typename T>
void RotationMatrixRotatePoint(const T R[9], const T t[3], T r_t[3])
{
	r_t[0] = R[0] * t[0] + R[1] * t[1] + R[2] * t[2];
	r_t[1] = R[3] * t[0] + R[4] * t[1] + R[5] * t[2];
	r_t[2] = R[6] * t[0] + R[7] * t[1] + R[8] * t[2];
};

