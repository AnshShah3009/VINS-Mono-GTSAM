#include "pose_graph.h"

PoseGraph::PoseGraph()
{
    posegraph_visualization = new CameraPoseVisualization(1.0, 0.0, 1.0, 1.0);
    posegraph_visualization->setScale(0.1);
    posegraph_visualization->setLineWidth(0.01);

    // requires edit for gtsam
    priorModel = gtsam::noiseModel::Diagonal::Sigmas((Eigen::VectorXd(6) << 0, 0, 0, 0, 0, 0).finished());
    gravityPriorModel = gtsam::noiseModel::Diagonal::Sigmas((Eigen::VectorXd(6) << 0.01, 0.01, 10000.0, 10000.0, 10000.0, 10000.0).finished()); // Lock roll/pitch tightly
    odometryModel = gtsam::noiseModel::Diagonal::Sigmas((Eigen::VectorXd(6) << 1e-5, 1e-5, 1e-5, 1e-3, 1e-3, 1e-3).finished());
    loopModel = gtsam::noiseModel::Diagonal::Sigmas((Eigen::VectorXd(6) << 0.001, 0.001, 0.001, 0.05, 0.05, 0.05).finished());
    infiModel = gtsam::noiseModel::Isotropic::Sigmas((Eigen::VectorXd(6) << 6.283, 6.283, 6.283, 10000, 10000, 10000).finished());
    addedFactorsTill = -1;

    // setting up params for gtsam optimizer
    // isam2_params.reloginTol = 1e-6;
    isam2_params.factorization = gtsam::ISAM2Params::CHOLESKY;
    isam2 = new gtsam::ISAM2(isam2_params);

    t_optimization = std::thread(&PoseGraph::optimize4DoF, this); // thread to run optimization

    earliest_loop_index = -1;              // Index of earliest loop closure to optimize only after that
    t_drift = Eigen::Vector3d(0, 0, 0);    // Invariant for translation drift Vector
    yaw_drift = 0;                         // ZerInvarianto for yaw drift
    r_drift = Eigen::Matrix3d::Identity(); // Invariant for rotation drift Matrix
    w_t_vio = Eigen::Vector3d(0, 0, 0);    // Invariant for VIO Vector
    w_r_vio = Eigen::Matrix3d::Identity(); // Invariant for VIO Matrix
    global_index = 0;                      // global index of all keyframe globally
    sequence_cnt = 0;                      // sequence count, 0 for base sequence, 1 for first sequence, 2 for second sequence, etc.
    sequence_loop.push_back(0);
    base_sequence = 1;
}

// destructor for Posegraph
PoseGraph::~PoseGraph()
{
    t_optimization.join(); // empty the thread
}

// for Ros Publishers to advertise topics : pose_graph_path, base_path, path_1, path_3 .... till 9
void PoseGraph::registerPub(rclcpp::Node::SharedPtr n)
{
    pub_pg_path = n->create_publisher<nav_msgs::msg::Path>("pose_graph_path", 1000);
    pub_base_path = n->create_publisher<nav_msgs::msg::Path>("base_path", 1000);
    pub_pose_graph = n->create_publisher<visualization_msgs::msg::MarkerArray>("pose_graph", 1000);
    for (int i = 1; i < 10; i++)
        pub_path[i] = n->create_publisher<nav_msgs::msg::Path>("path_" + std::to_string(i), 1000);
}


// To load descriptor vocabulary (can use anytype of descriptor)
void PoseGraph::loadVocabulary(std::string voc_path)
{
    voc = new BriefVocabulary(voc_path);
    db.setVocabulary(*voc, false, 0);
}

// To add keyframe into pose graph (gtsam implementation here)
void PoseGraph::addKeyFrame(KeyFrame *cur_kf, bool flag_detect_loop)
{
    Vector3d vio_P_cur;
    Matrix3d vio_R_cur;
    printf("\n keyframe added %d ", cur_kf->index);

    int new_seq = 0; // flag for new sequence to add prior factor

    // initiliaze world transformation coordinates for new sequence
    if (sequence_cnt != cur_kf->sequence)
    {
        sequence_cnt++;
        sequence_loop.push_back(0);
        // setting world rotation coordinates
        w_t_vio = Eigen::Vector3d(0, 0, 0);
        w_r_vio = Eigen::Matrix3d::Identity();
        m_drift.lock();
        // setting drift for new sequence as 0,0,0 with no rotation
        t_drift = Eigen::Vector3d(0, 0, 0);
        r_drift = Eigen::Matrix3d::Identity();
        m_drift.unlock();
        new_seq = 1; // added flag for new sequence addition to check if we need to add prior node in factor graph
    }

    cur_kf->getVioPose(vio_P_cur, vio_R_cur);
    vio_P_cur = w_r_vio * vio_P_cur + w_t_vio;
    vio_R_cur = w_r_vio * vio_R_cur;
    cur_kf->updateVioPose(vio_P_cur, vio_R_cur);
    // updates VioPose of current keyframe before using it for calculation in factor graph

    cur_kf->index = global_index;
    global_index++;
    int loop_index = -1;
    int loop_flag = -1; // to add only factor edge

    // m_posegraph.lock();
    // flag_detect_loop is to check if loop detection is turned on or not
    if (flag_detect_loop)
    {
        TicToc tmp_t; // time start before loop detection and ending of frame addition
        loop_index = detectLoop(cur_kf, cur_kf->index);
    }
    else
    {
        // when flag is off, just add keyframe into for data recording for offline slam maybe
        addKeyFrameIntoVoc(cur_kf);
    }

    // if any loop was detected only then process the loop before adding new keyframe
    if (loop_index != -1)
    {
        // printf(" %d detect loop with %d \n", cur_kf->index, loop_index);
        KeyFrame *old_kf = getKeyFrame(loop_index); // to get keyframe with which loop has been detected

        if (cur_kf->findConnection(old_kf))
        {
            if (earliest_loop_index > loop_index || earliest_loop_index == -1)
                earliest_loop_index = loop_index;

            Vector3d w_P_old, w_P_cur, vio_P_cur;
            Matrix3d w_R_old, w_R_cur, vio_R_cur;
            old_kf->getPose(w_P_old, w_R_old); // refer to globally optimized path instead of local VIO path (fixed)
            cur_kf->getVioPose(vio_P_cur, vio_R_cur);

            Vector3d relative_t;
            Quaterniond relative_q;
            relative_t = cur_kf->getLoopRelativeT();
            relative_q = (cur_kf->getLoopRelativeQ()).toRotationMatrix();
            w_P_cur = w_R_old * relative_t + w_P_old;
            w_R_cur = w_R_old * relative_q;
            double shift_yaw;
            Matrix3d shift_r;
            Vector3d shift_t;
            shift_yaw = Utility::R2ypr(w_R_cur).x() - Utility::R2ypr(vio_R_cur).x(); // can be made 6DOF
            shift_r = Utility::ypr2R(Vector3d(shift_yaw, 0, 0));
            shift_t = w_P_cur - w_R_cur * vio_R_cur.transpose() * vio_P_cur; // shift is the difference between vio and world frame wrt this loop closure

            // 4DOF math can be updated to 6DOF math for loop detection, might be because the PNP is not accurate in other angles
            Vector3d gtsam_rel_t;
            Quaterniond gtsam_rel_q;
            double gtsam_rel_yaw;
            gtsam_rel_t = cur_kf->getLoopRelativeT();
            // Matrix3d rela_q = (cur_kf->getLoopRelativeQ()).toRotationMatrix(); // to do 6DOF instead of 4DOF
            gtsam_rel_yaw = cur_kf->getLoopRelativeYaw();
            gtsam_rel_q = Utility::ypr2R(Vector3d(gtsam_rel_yaw, 0, 0));
            Matrix3d gtsam_rel_r = gtsam_rel_q.toRotationMatrix();
            m_posegraph.lock();
            Matrix3d RI;
            Vector3d PI;
            cur_kf->getVioPose(PI, RI);
            double lock = gtsam_rel_t.norm() / 5;
            gtsam::noiseModel::Diagonal::shared_ptr loop_noise = gtsam::noiseModel::Diagonal::Sigmas((Eigen::VectorXd(6) << 0.05, 0.05, 0.05, lock, lock, lock).finished());
            graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(old_kf->index, cur_kf->index, gtsam::Pose3(gtsam::Rot3(gtsam_rel_r), gtsam::Point3(-gtsam_rel_t)), loop_noise);
            initial.insert(cur_kf->index, gtsam::Pose3(gtsam::Rot3(RI), gtsam::Point3(PI)));
            loop_flag = 1;
            m_posegraph.unlock();
            // shift vio pose of whole sequence to the world frame wrt the first loop closure
            if (old_kf->sequence != cur_kf->sequence && sequence_loop[cur_kf->sequence] == 0)
            {
                w_r_vio = shift_r;
                w_t_vio = shift_t;
                vio_P_cur = w_r_vio * vio_P_cur + w_t_vio;
                vio_R_cur = w_r_vio * vio_R_cur;
                cur_kf->updateVioPose(vio_P_cur, vio_R_cur);
                list<KeyFrame *>::iterator it = keyframelist.begin();
                for (; it != keyframelist.end(); it++)
                {
                    if ((*it)->sequence == cur_kf->sequence)
                    {
                        Vector3d vio_P_cur;
                        Matrix3d vio_R_cur;
                        (*it)->getVioPose(vio_P_cur, vio_R_cur);
                        vio_P_cur = w_r_vio * vio_P_cur + w_t_vio;
                        vio_R_cur = w_r_vio * vio_R_cur;
                        (*it)->updateVioPose(vio_P_cur, vio_R_cur);
                    }
                }
                sequence_loop[cur_kf->sequence] = 1;
            }
        }
    }

    m_keyframelist.lock();

    Vector3d P;
    Matrix3d R;

    cur_kf->getVioPose(P, R);
    P = r_drift * P + t_drift;
    R = r_drift * R;
    cur_kf->updatePose(P, R);
    Quaterniond Q{R};
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header.stamp = rclcpp::Time(static_cast<uint64_t>(cur_kf->time_stamp * 1e9));
    pose_stamped.header.frame_id = "world";
    pose_stamped.pose.position.x = P.x() + VISUALIZATION_SHIFT_X;
    pose_stamped.pose.position.y = P.y() + VISUALIZATION_SHIFT_Y;
    pose_stamped.pose.position.z = P.z();
    pose_stamped.pose.orientation.x = Q.x();
    pose_stamped.pose.orientation.y = Q.y();
    pose_stamped.pose.orientation.z = Q.z();
    pose_stamped.pose.orientation.w = Q.w();
    path[sequence_cnt].poses.push_back(pose_stamped);
    path[sequence_cnt].header = pose_stamped.header;


    if (SAVE_LOOP_PATH)
    {
        ofstream loop_path_file(VINS_RESULT_PATH, ios::app);
        loop_path_file.setf(ios::fixed, ios::floatfield);
        loop_path_file.precision(0);
        loop_path_file << cur_kf->time_stamp * 1e9 << ",";
        loop_path_file.precision(5);
        loop_path_file << P.x() << ","
                       << P.y() << ","
                       << P.z() << ","
                       << Q.w() << ","
                       << Q.x() << ","
                       << Q.y() << ","
                       << Q.z()
                       << endl;
        loop_path_file.close();
    }
    // draw local connection
    if (SHOW_S_EDGE)
    {
        list<KeyFrame *>::reverse_iterator rit = keyframelist.rbegin();
        for (int i = 0; i < 4; i++)
        {
            if (rit == keyframelist.rend())
                break;
            Vector3d conncected_P;
            Matrix3d connected_R;
            if ((*rit)->sequence == cur_kf->sequence)
            {
                (*rit)->getPose(conncected_P, connected_R);
                posegraph_visualization->add_edge(P, conncected_P);
            }
            rit++;
        }
    }

    if (SHOW_L_EDGE)
    {
        if (cur_kf->has_loop)
        {
            // printf("has loop \n");
            KeyFrame *connected_KF = getKeyFrame(cur_kf->loop_index);
            Vector3d connected_P, P0;
            Matrix3d connected_R, R0;
            connected_KF->getPose(connected_P, connected_R);
            // cur_kf->getVioPose(P0, R0);
            cur_kf->getPose(P0, R0);
            if (cur_kf->sequence > 0)
            {
                // printf("add loop into visual \n");
                posegraph_visualization->add_loopedge(P0, connected_P + Vector3d(VISUALIZATION_SHIFT_X, VISUALIZATION_SHIFT_Y, 0));
            }
        }
    }

    // posegraph_visualization->add_pose(P + Vector3d(VISUALIZATION_SHIFT_X, VISUALIZATION_SHIFT_Y, 0), Q);
    keyframelist.push_back(cur_kf);

    m_posegraph.lock();

    addedFactorsTill = cur_kf->index;

    if (loop_flag != 1)
    {
        Matrix3d R0;
        Vector3d P0;
        cur_kf->getVioPose(P0, R0);
        // initial.insert(cur_kf->index, gtsam::Pose3(gtsam::Rot3(R0), gtsam::Point3(P0)));
        initial.insert(cur_kf->index, gtsam::Pose3(gtsam::Rot3(R0), gtsam::Point3(P0)));
    }
    if (cur_kf->index == 0)
    {
        Matrix3d R0;
        Vector3d P0;
        cur_kf->getVioPose(P0, R0);

        graph.addPrior(cur_kf->index, gtsam::Pose3(gtsam::Rot3(R0), gtsam::Point3(P0)), priorModel);
    }
    else if (new_seq == 1)
    {
        // add prior for the new sequence
        graph.addPrior(cur_kf->index, gtsam::Pose3(gtsam::Rot3(1, 0, 0, 0, 1, 0, 0, 0, 1), gtsam::Point3(0, 0, 0)), infiModel);
    }
    else
    {
        // normal edge
        Vector3d P0, P1, Pf;
        Matrix3d R0, R1, Rf;
        cur_kf->getVioPose(P1, R1);
        getKeyFrame(cur_kf->index - 1)->getVioPose(P0, R0);

        gtsam::Pose3 T0 = gtsam::Pose3(gtsam::Rot3(R0), gtsam::Point3(P0));
        gtsam::Pose3 T1 = gtsam::Pose3(gtsam::Rot3(R1), gtsam::Point3(P1));
        gtsam::Pose3 Tf = T0.inverse() * T1;
        
        graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(cur_kf->index - 1, cur_kf->index, Tf, odometryModel);
        
        // Add gravity/4-DoF prior to lock roll and pitch to the VIO's extremely accurate gravity alignment
        Matrix3d R_vio;
        Vector3d P_vio;
        cur_kf->getVioPose(P_vio, R_vio);
        gtsam::Pose3 T_gravity_prior(gtsam::Rot3(R_vio), gtsam::Point3(0,0,0)); 
        graph.addPrior(cur_kf->index, T_gravity_prior, gravityPriorModel);
    }

    m_posegraph.unlock();

    publish();

    m_optimize_buf.lock();
    optimize_buf.push(cur_kf->index); // to give optimize function the current keyframe index
    m_optimize_buf.unlock();

    m_keyframelist.unlock();
}

// will need change for gtsam 4.0
void PoseGraph::loadKeyFrame(KeyFrame *cur_kf, bool flag_detect_loop)
{
    cur_kf->index = global_index;
    global_index++;
    int loop_index = -1;
    int loop_flag = -1;
    
    if (flag_detect_loop)
    {
        loop_index = detectLoop(cur_kf, cur_kf->index);
    }
    else
    {
        addKeyFrameIntoVoc(cur_kf);
    }
    if (loop_index != -1)
    {
        printf(" %d detect loop with %d \n", cur_kf->index, loop_index);
        KeyFrame *old_kf = getKeyFrame(loop_index);
        if (cur_kf->findConnection(old_kf))
        {
            if (earliest_loop_index > loop_index || earliest_loop_index == -1)
                earliest_loop_index = loop_index;
                
            Vector3d w_P_old, w_P_cur, vio_P_cur;
            Matrix3d w_R_old, w_R_cur, vio_R_cur;
            old_kf->getPose(w_P_old, w_R_old);
            cur_kf->getVioPose(vio_P_cur, vio_R_cur);

            Vector3d relative_t;
            Quaterniond relative_q;
            relative_t = cur_kf->getLoopRelativeT();
            relative_q = (cur_kf->getLoopRelativeQ()).toRotationMatrix();
            w_P_cur = w_R_old * relative_t + w_P_old;
            w_R_cur = w_R_old * relative_q;
            
            // GTSAM ISAM2 Factor setup
            Vector3d gtsam_rel_t = cur_kf->getLoopRelativeT();
            double gtsam_rel_yaw = cur_kf->getLoopRelativeYaw();
            Quaterniond gtsam_rel_q(Utility::ypr2R(Vector3d(gtsam_rel_yaw, 0, 0)));
            Matrix3d gtsam_rel_r = gtsam_rel_q.toRotationMatrix();
            
            m_posegraph.lock();
            double lock = gtsam_rel_t.norm() / 5;
            gtsam::noiseModel::Diagonal::shared_ptr loop_noise = gtsam::noiseModel::Diagonal::Sigmas((Eigen::VectorXd(6) << 0.05, 0.05, 0.05, lock, lock, lock).finished());
            graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(old_kf->index, cur_kf->index, gtsam::Pose3(gtsam::Rot3(gtsam_rel_r), gtsam::Point3(-gtsam_rel_t)), loop_noise);
            loop_flag = 1;
            m_posegraph.unlock();

            m_optimize_buf.lock();
            optimize_buf.push(cur_kf->index);
            m_optimize_buf.unlock();
        }
    }

    m_keyframelist.lock();

    m_posegraph.lock();
    addedFactorsTill = cur_kf->index;
    
    Matrix3d R0;
    Vector3d P0;
    if (loop_flag != 1) {
        cur_kf->getPose(P0, R0);
        initial.insert(cur_kf->index, gtsam::Pose3(gtsam::Rot3(R0), gtsam::Point3(P0)));
    }
    
    if (cur_kf->index == 0)
    {
        cur_kf->getPose(P0, R0);
        graph.addPrior(cur_kf->index, gtsam::Pose3(gtsam::Rot3(R0), gtsam::Point3(P0)), priorModel);
    }
    else
    {
        Vector3d P1, Pf;
        Matrix3d R1, Rf;
        cur_kf->getPose(P1, R1);
        getKeyFrame(cur_kf->index - 1)->getPose(P0, R0);

        gtsam::Pose3 T0 = gtsam::Pose3(gtsam::Rot3(R0), gtsam::Point3(P0));
        gtsam::Pose3 T1 = gtsam::Pose3(gtsam::Rot3(R1), gtsam::Point3(P1));
        gtsam::Pose3 Tf = T0.inverse() * T1;
        
        graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(cur_kf->index - 1, cur_kf->index, Tf, odometryModel);
        
        // Add 4-DoF Gravity Prior for loaded frame
        Matrix3d R_vio;
        Vector3d P_vio;
        cur_kf->getVioPose(P_vio, R_vio);
        gtsam::Pose3 T_gravity_prior(gtsam::Rot3(R_vio), gtsam::Point3(0,0,0)); 
        graph.addPrior(cur_kf->index, T_gravity_prior, gravityPriorModel);
    }
    m_posegraph.unlock();

    Vector3d P;
    Matrix3d R;
    cur_kf->getPose(P, R);
    Quaterniond Q{R};
    geometry_msgs::msg::PoseStamped pose_stamped;
    // setting up message
    pose_stamped.header.stamp = rclcpp::Time(static_cast<uint64_t>(cur_kf->time_stamp * 1e9));
    pose_stamped.header.frame_id = "world";
    pose_stamped.pose.position.x = P.x() + VISUALIZATION_SHIFT_X;
    pose_stamped.pose.position.y = P.y() + VISUALIZATION_SHIFT_Y;
    pose_stamped.pose.position.z = P.z();
    pose_stamped.pose.orientation.x = Q.x();
    pose_stamped.pose.orientation.y = Q.y();
    pose_stamped.pose.orientation.z = Q.z();
    pose_stamped.pose.orientation.w = Q.w();
    base_path.poses.push_back(pose_stamped);
    base_path.header = pose_stamped.header;


    // draw local connection
    if (SHOW_S_EDGE)
    {
        list<KeyFrame *>::reverse_iterator rit = keyframelist.rbegin();
        for (int i = 0; i < 1; i++)
        {
            if (rit == keyframelist.rend())
                break;
            Vector3d conncected_P;
            Matrix3d connected_R;
            if ((*rit)->sequence == cur_kf->sequence)
            {
                (*rit)->getPose(conncected_P, connected_R);
                posegraph_visualization->add_edge(P, conncected_P);
            }
            rit++;
        }
    }
    /*
    if (cur_kf->has_loop)
    {
        KeyFrame* connected_KF = getKeyFrame(cur_kf->loop_index);
        Vector3d connected_P;
        Matrix3d connected_R;
        connected_KF->getPose(connected_P,  connected_R);
        posegraph_visualization->add_loopedge(P, connected_P, SHIFT);
    }
    */

    keyframelist.push_back(cur_kf);
    // publish();
    m_keyframelist.unlock();
}

// get Keyframe by index (no change)
KeyFrame *PoseGraph::getKeyFrame(int index)
{
    //    unique_lock<mutex> lock(m_keyframelist);
    list<KeyFrame *>::iterator it = keyframelist.begin();
    for (; it != keyframelist.end(); it++)
    {
        if ((*it)->index == index)
            break;
    }
    if (it != keyframelist.end())
        return *it;
    else
        return NULL;
}

// detect any loop with a keyframe(no change)
int PoseGraph::detectLoop(KeyFrame *keyframe, int frame_index)
{
    // put image into image_pool; for visualization
    cv::Mat compressed_image;
    if (DEBUG_IMAGE)
    {
        int feature_num = keyframe->keypoints.size();
        cv::resize(keyframe->image, compressed_image, cv::Size(376, 240));
        putText(compressed_image, "feature_num:" + to_string(feature_num), cv::Point2f(10, 10), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255));
        image_pool[frame_index] = compressed_image;
    }
    TicToc tmp_t;
    // first query; then add this frame into database!
    QueryResults ret;
    TicToc t_query;
    db.query(keyframe->brief_descriptors, ret, 4, frame_index - 50);
    // printf("query time: %f", t_query.toc());
    // cout << "Searching for Image " << frame_index << ". " << ret << endl;

    TicToc t_add;
    db.add(keyframe->brief_descriptors);
    // printf("add feature time: %f", t_add.toc());
    //  ret[0] is the nearest neighbour's score. threshold change with neighour score
    bool find_loop = false;
    cv::Mat loop_result;
    if (DEBUG_IMAGE)
    {
        loop_result = compressed_image.clone();
        if (ret.size() > 0)
            putText(loop_result, "neighbour score:" + to_string(ret[0].Score), cv::Point2f(10, 50), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255));
    }
    // visual loop result
    if (DEBUG_IMAGE)
    {
        for (unsigned int i = 0; i < ret.size(); i++)
        {
            int tmp_index = ret[i].Id;
            auto it = image_pool.find(tmp_index);
            cv::Mat tmp_image = (it->second).clone();
            putText(tmp_image, "index:  " + to_string(tmp_index) + "loop score:" + to_string(ret[i].Score), cv::Point2f(10, 50), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255));
            cv::hconcat(loop_result, tmp_image, loop_result);
        }
    }
    // a good match with its nerghbour
    if (ret.size() >= 1 && ret[0].Score > 0.05)
        for (unsigned int i = 1; i < ret.size(); i++)
        {
            // if (ret[i].Score > ret[0].Score * 0.3)
            if (ret[i].Score > 0.015)
            {
                find_loop = true;
                int tmp_index = ret[i].Id;
                if (DEBUG_IMAGE && 0)
                {
                    auto it = image_pool.find(tmp_index);
                    cv::Mat tmp_image = (it->second).clone();
                    putText(tmp_image, "loop score:" + to_string(ret[i].Score), cv::Point2f(10, 50), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255));
                    cv::hconcat(loop_result, tmp_image, loop_result);
                }
            }
        }
    /*
        if (DEBUG_IMAGE)
        {
            cv::imshow("loop_result", loop_result);
            cv::waitKey(20);
        }
    */
    if (find_loop && frame_index > 50)
    {
        int min_index = -1;
        for (unsigned int i = 0; i < ret.size(); i++)
        {
            if (min_index == -1 || (ret[i].Id < min_index && ret[i].Score > 0.015))
                min_index = ret[i].Id;
        }
        return min_index;
    }
    else
        return -1;
}

// Just adding keyframe into database (no change)
void PoseGraph::addKeyFrameIntoVoc(KeyFrame *keyframe)
{
    // put image into image_pool; for visualization
    cv::Mat compressed_image;
    if (DEBUG_IMAGE)
    {
        int feature_num = keyframe->keypoints.size();
        cv::resize(keyframe->image, compressed_image, cv::Size(376, 240));
        putText(compressed_image, "feature_num:" + to_string(feature_num), cv::Point2f(10, 10), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255));
        image_pool[keyframe->index] = compressed_image;
    }

    db.add(keyframe->brief_descriptors);
}

//  optimizes the graph after every 2 seconds (gtsam implementation here)
void PoseGraph::optimize4DoF()
{
    while (true)
    {
        int cur_index = -1;
        int first_looped_index = -1;

        m_optimize_buf.lock();

        while (!optimize_buf.empty())
        {
            cur_index = optimize_buf.front();
            first_looped_index = earliest_loop_index;
            optimize_buf.pop();
        }

        m_optimize_buf.unlock();

        // if we have any keyframes in optimize buff
        if (cur_index != -1)
        {
            TicToc tmp_t; // time at start of optimization

            m_posegraph.lock();
            // GTSAM ISAM2 optimization
            isam2->update(graph, initial);
            isam2->update();
            
            graph.resize(0);
            initial.clear();
            
            gtsam::Values result = isam2->calculateEstimate();
            m_posegraph.unlock();

            m_keyframelist.lock();
            KeyFrame *cur_kf = getKeyFrame(cur_index);

            list<KeyFrame *>::iterator it;
            for (it = keyframelist.begin(); it != keyframelist.end(); it++)
            {
                if(result.exists((*it)->index)){
                    Vector3d tmp_t = result.at<gtsam::Pose3>((*it)->index).translation();
                    auto tmp_q = result.at<gtsam::Pose3>((*it)->index).rotation();
                    Matrix3d tmp_r = tmp_q.matrix();
                    (*it)->updatePose(tmp_t, tmp_r);
                }

                if ((*it)->index == cur_index)
                    break;
            }
            printf("pose optimization time: %f \n", tmp_t.toc());

            Vector3d cur_t, vio_t;
            Matrix3d cur_r, vio_r;
            cur_kf->getPose(cur_t, cur_r);
            cur_kf->getVioPose(vio_t, vio_r);
            m_drift.lock();
            yaw_drift = Utility::R2ypr(cur_r).x() - Utility::R2ypr(vio_r).x();
            r_drift = Utility::ypr2R(Vector3d(yaw_drift, 0, 0));
            t_drift = cur_t - r_drift * vio_t;
            m_drift.unlock();
            // cout << "t_drift " << t_drift.transpose() << endl;
            // cout << "r_drift " << Utility::R2ypr(r_drift).transpose() << endl;
            // cout << "yaw drift " << yaw_drift << endl;

            it++;
            for (; it != keyframelist.end(); it++)
            {
                Vector3d P;
                Matrix3d R;
                (*it)->getVioPose(P, R);
                P = r_drift * P + t_drift;
                R = r_drift * R;
                (*it)->updatePose(P, R);
            }
            m_keyframelist.unlock();
            updatePath();
        }
        std::chrono::milliseconds dura(1000);
        std::this_thread::sleep_for(dura);
        // std::cout << "pose graph thread finish" << std::endl;
    }
}

// update the path for visualization (no change)
void PoseGraph::updatePath()
{
    m_keyframelist.lock();
    m_posegraph.lock();

    list<KeyFrame *>::iterator it;
    for (int i = 1; i <= sequence_cnt; i++)
    {
        path[i].poses.clear();
    }
    base_path.poses.clear();
    posegraph_visualization->reset();

    if (SAVE_LOOP_PATH)
    {
        ofstream loop_path_file_tmp(VINS_RESULT_PATH, ios::out);
        loop_path_file_tmp.close();
    }

    for (it = keyframelist.begin(); it != keyframelist.end(); it++)
    {
        Vector3d P;
        Matrix3d R;
        (*it)->getPose(P, R);
        Quaterniond Q;
        Q = R;
        // printf("path p: %f, %f, %f\n", P.x(), P.z(), P.y());

        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header.stamp = rclcpp::Time(static_cast<uint64_t>((*it)->time_stamp * 1e9));
        pose_stamped.header.frame_id = "world";
        pose_stamped.pose.position.x = P.x() + VISUALIZATION_SHIFT_X;
        pose_stamped.pose.position.y = P.y() + VISUALIZATION_SHIFT_Y;
        pose_stamped.pose.position.z = P.z();
        pose_stamped.pose.orientation.x = Q.x();
        pose_stamped.pose.orientation.y = Q.y();
        pose_stamped.pose.orientation.z = Q.z();
        pose_stamped.pose.orientation.w = Q.w();

        if ((*it)->sequence == 0)
        {
            base_path.poses.push_back(pose_stamped);
            base_path.header = pose_stamped.header;
        }
        else
        {
            path[(*it)->sequence].poses.push_back(pose_stamped);
            path[(*it)->sequence].header = pose_stamped.header;
        }

        if (SAVE_LOOP_PATH)
        {
            ofstream loop_path_file(VINS_RESULT_PATH, ios::app);
            loop_path_file.setf(ios::fixed, ios::floatfield);
            loop_path_file.precision(0);
            loop_path_file << (*it)->time_stamp * 1e9 << ",";
            loop_path_file.precision(5);
            loop_path_file << P.x() << ","
                           << P.y() << ","
                           << P.z() << ","
                           << Q.w() << ","
                           << Q.x() << ","
                           << Q.y() << ","
                           << Q.z()
                           << endl;
            loop_path_file.close();
        }
        // draw local connection
        if (SHOW_S_EDGE)
        {
            list<KeyFrame *>::reverse_iterator rit = keyframelist.rbegin();
            list<KeyFrame *>::reverse_iterator lrit;
            for (; rit != keyframelist.rend(); rit++)
            {
                if ((*rit)->index == (*it)->index)
                {
                    lrit = rit;
                    lrit++;
                    for (int i = 0; i < 4; i++)
                    {
                        if (lrit == keyframelist.rend())
                            break;
                        if ((*lrit)->sequence == (*it)->sequence)
                        {
                            Vector3d conncected_P;
                            Matrix3d connected_R;
                            (*lrit)->getPose(conncected_P, connected_R);
                            posegraph_visualization->add_edge(P, conncected_P);
                        }
                        lrit++;
                    }
                    break;
                }
            }
        }
        // draw loop connection
        if (SHOW_L_EDGE)
        {
            if ((*it)->has_loop && (*it)->sequence == sequence_cnt)
            {

                KeyFrame *connected_KF = getKeyFrame((*it)->loop_index);
                Vector3d connected_P;
                Matrix3d connected_R;
                connected_KF->getPose(connected_P, connected_R);
                //(*it)->getVioPose(P, R);
                (*it)->getPose(P, R);
                if ((*it)->sequence > 0)
                {
                    posegraph_visualization->add_loopedge(P, connected_P + Vector3d(VISUALIZATION_SHIFT_X, VISUALIZATION_SHIFT_Y, 0));
                }
            }
        }
    }
    publish();
    m_posegraph.unlock();
    m_keyframelist.unlock();
}

// will require change for gtsam
void PoseGraph::savePoseGraph()
{
    m_keyframelist.lock();
    TicToc tmp_t;
    FILE *pFile;
    printf("pose graph path: %s\n", POSE_GRAPH_SAVE_PATH.c_str());
    printf("pose graph saving... \n");
    string file_path = POSE_GRAPH_SAVE_PATH + "pose_graph.txt";
    pFile = fopen(file_path.c_str(), "w");
    fprintf(pFile, "index time_stamp Tx Ty Tz Qw Qx Qy Qz loop_index loop_info\n");
    list<KeyFrame *>::iterator it;
    for (it = keyframelist.begin(); it != keyframelist.end(); it++)
    {
        std::string image_path, descriptor_path, brief_path, keypoints_path;
        if (DEBUG_IMAGE)
        {
            image_path = POSE_GRAPH_SAVE_PATH + to_string((*it)->index) + "_image.png";
            imwrite(image_path.c_str(), (*it)->image);
        }
        Quaterniond VIO_tmp_Q{(*it)->vio_R_w_i};
        Quaterniond PG_tmp_Q{(*it)->R_w_i};
        Vector3d VIO_tmp_T = (*it)->vio_T_w_i;
        Vector3d PG_tmp_T = (*it)->T_w_i;

        fprintf(pFile, " %d %f \n %f %f %f \n %f %f %f \n %f %f %f %f \n %f %f %f %f \n %d %f %f %f %f %f %f %f %f %d \n\n\n", (*it)->index, (*it)->time_stamp,
                VIO_tmp_T.x(), VIO_tmp_T.y(), VIO_tmp_T.z(),
                PG_tmp_T.x(), PG_tmp_T.y(), PG_tmp_T.z(),
                VIO_tmp_Q.w(), VIO_tmp_Q.x(), VIO_tmp_Q.y(), VIO_tmp_Q.z(),
                PG_tmp_Q.w(), PG_tmp_Q.x(), PG_tmp_Q.y(), PG_tmp_Q.z(),
                (*it)->loop_index,
                (*it)->loop_info(0), (*it)->loop_info(1), (*it)->loop_info(2), (*it)->loop_info(3),
                (*it)->loop_info(4), (*it)->loop_info(5), (*it)->loop_info(6), (*it)->loop_info(7),
                (int)(*it)->keypoints.size());

        // fprintf(pFile, " %d %f %f %f %f %f %f %f %f %d \n", (*it)->index,
        //         PG_tmp_T.x(), PG_tmp_T.y(), PG_tmp_T.z(),
        //         PG_tmp_Q.w(), PG_tmp_Q.x(), PG_tmp_Q.y(), PG_tmp_Q.z(),

        //         VIO_tmp_Q.w(), VIO_tmp_Q.x(), VIO_tmp_Q.y(), VIO_tmp_Q.z(),

        // write keypoints, brief_descriptors   vector<cv::KeyPoint> keypoints vector<BRIEF::bitset> brief_descriptors;
        assert((*it)->keypoints.size() == (*it)->brief_descriptors.size());
        brief_path = POSE_GRAPH_SAVE_PATH + to_string((*it)->index) + "_briefdes.dat";
        std::ofstream brief_file(brief_path, std::ios::binary);
        keypoints_path = POSE_GRAPH_SAVE_PATH + to_string((*it)->index) + "_keypoints.txt";
        FILE *keypoints_file;
        keypoints_file = fopen(keypoints_path.c_str(), "w");
        // for (int i = 0; i < (int)(*it)->keypoints.size(); i++)
        // {
        //     brief_file << (*it)->brief_descriptors[i] << endl;
        //     fprintf(keypoints_file, "%f %f %f %f\n", (*it)->keypoints[i].pt.x, (*it)->keypoints[i].pt.y,
        //             (*it)->keypoints_norm[i].pt.x, (*it)->keypoints_norm[i].pt.y);
        // }
        brief_file.close();
        fclose(keypoints_file);
    }
    fclose(pFile);

    printf("save pose graph time: %f s\n", tmp_t.toc() / 1000);
    m_keyframelist.unlock();
}

// will require change for gtsam
void PoseGraph::loadPoseGraph()
{
    TicToc tmp_t;
    FILE *pFile;
    string file_path = POSE_GRAPH_SAVE_PATH + "pose_graph.txt";

    printf("lode pose graph from: %s \n", file_path.c_str());
    printf("pose graph loading...\n");

    pFile = fopen(file_path.c_str(), "r");

    if (pFile == NULL)
    {
        printf("lode previous pose graph error: wrong previous pose graph path or no previous pose graph \n the system will start with new pose graph \n");
        return;
    }

    int index;
    double time_stamp;
    double VIO_Tx, VIO_Ty, VIO_Tz;
    double PG_Tx, PG_Ty, PG_Tz;
    double VIO_Qw, VIO_Qx, VIO_Qy, VIO_Qz;
    double PG_Qw, PG_Qx, PG_Qy, PG_Qz;
    double loop_info_0, loop_info_1, loop_info_2, loop_info_3;
    double loop_info_4, loop_info_5, loop_info_6, loop_info_7;
    int loop_index;
    int keypoints_num;
    Eigen::Matrix<double, 8, 1> loop_info;
    int cnt = 0;

    while (fscanf(pFile, "%d %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %d %lf %lf %lf %lf %lf %lf %lf %lf %d", &index, &time_stamp,
                  &VIO_Tx, &VIO_Ty, &VIO_Tz,
                  &PG_Tx, &PG_Ty, &PG_Tz,
                  &VIO_Qw, &VIO_Qx, &VIO_Qy, &VIO_Qz,
                  &PG_Qw, &PG_Qx, &PG_Qy, &PG_Qz,
                  &loop_index,
                  &loop_info_0, &loop_info_1, &loop_info_2, &loop_info_3,
                  &loop_info_4, &loop_info_5, &loop_info_6, &loop_info_7,
                  &keypoints_num) != EOF)
    {
        /*
        printf("I read: %d %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %d %lf %lf %lf %lf %lf %lf %lf %lf %d\n", index, time_stamp,
                                    VIO_Tx, VIO_Ty, VIO_Tz,
                                    PG_Tx, PG_Ty, PG_Tz,
                                    VIO_Qw, VIO_Qx, VIO_Qy, VIO_Qz,
                                    PG_Qw, PG_Qx, PG_Qy, PG_Qz,
                                    loop_index,
                                    loop_info_0, loop_info_1, loop_info_2, loop_info_3,
                                    loop_info_4, loop_info_5, loop_info_6, loop_info_7,
                                    keypoints_num);
        */
        cv::Mat image;
        std::string image_path, descriptor_path;
        if (DEBUG_IMAGE)
        {
            image_path = POSE_GRAPH_SAVE_PATH + to_string(index) + "_image.png";
            image = cv::imread(image_path.c_str(), 0);
        }

        Vector3d VIO_T(VIO_Tx, VIO_Ty, VIO_Tz);
        Vector3d PG_T(PG_Tx, PG_Ty, PG_Tz);
        Quaterniond VIO_Q;
        VIO_Q.w() = VIO_Qw;
        VIO_Q.x() = VIO_Qx;
        VIO_Q.y() = VIO_Qy;
        VIO_Q.z() = VIO_Qz;
        Quaterniond PG_Q;
        PG_Q.w() = PG_Qw;
        PG_Q.x() = PG_Qx;
        PG_Q.y() = PG_Qy;
        PG_Q.z() = PG_Qz;
        Matrix3d VIO_R, PG_R;
        VIO_R = VIO_Q.toRotationMatrix();
        PG_R = PG_Q.toRotationMatrix();
        Eigen::Matrix<double, 8, 1> loop_info;
        loop_info << loop_info_0, loop_info_1, loop_info_2, loop_info_3, loop_info_4, loop_info_5, loop_info_6, loop_info_7;

        if (loop_index != -1)
            if (earliest_loop_index > loop_index || earliest_loop_index == -1)
            {
                earliest_loop_index = loop_index;
            }

        // load keypoints, brief_descriptors
        string brief_path = POSE_GRAPH_SAVE_PATH + to_string(index) + "_briefdes.dat";
        std::ifstream brief_file(brief_path, std::ios::binary);
        string keypoints_path = POSE_GRAPH_SAVE_PATH + to_string(index) + "_keypoints.txt";
        FILE *keypoints_file;
        keypoints_file = fopen(keypoints_path.c_str(), "r");
        vector<cv::KeyPoint> keypoints;
        vector<cv::KeyPoint> keypoints_norm;
        vector<BRIEF::bitset> brief_descriptors;
        for (int i = 0; i < keypoints_num; i++)
        {
            BRIEF::bitset tmp_des;
            brief_file >> tmp_des;
            brief_descriptors.push_back(tmp_des);
            cv::KeyPoint tmp_keypoint;
            cv::KeyPoint tmp_keypoint_norm;
            double p_x, p_y, p_x_norm, p_y_norm;
            if (!fscanf(keypoints_file, "%lf %lf %lf %lf", &p_x, &p_y, &p_x_norm, &p_y_norm))
                printf(" fail to load pose graph \n");
            tmp_keypoint.pt.x = p_x;
            tmp_keypoint.pt.y = p_y;
            tmp_keypoint_norm.pt.x = p_x_norm;
            tmp_keypoint_norm.pt.y = p_y_norm;
            keypoints.push_back(tmp_keypoint);
            keypoints_norm.push_back(tmp_keypoint_norm);
        }
        brief_file.close();
        fclose(keypoints_file);

        KeyFrame *keyframe = new KeyFrame(time_stamp, index, VIO_T, VIO_R, PG_T, PG_R, image, loop_index, loop_info, keypoints, keypoints_norm, brief_descriptors);
        loadKeyFrame(keyframe, 0);
        if (cnt % 20 == 0)
        {
            publish();
        }
        cnt++;
    }
    fclose(pFile);
    printf("load pose graph time: %f s\n", tmp_t.toc() / 1000);
    base_sequence = 0;
}

// to publish paths (no change)
void PoseGraph::publish()
{
    for (int i = 1; i <= sequence_cnt; i++)
    {
        // if (sequence_loop[i] == true || i == base_sequence)
        if (1 || i == base_sequence)
        {
            pub_pg_path->publish(path[i]);
            pub_path[i]->publish(path[i]);
            posegraph_visualization->publish_by(pub_pose_graph, path[sequence_cnt].header);
        }
    }
    base_path.header.frame_id = "world";
    pub_base_path->publish(base_path);
    // posegraph_visualization->publish_by(pub_pose_graph, path[sequence_cnt].header);
}


// to update keyframe loop like shift, given recolisatoin 0 no not needed for now
void PoseGraph::updateKeyFrameLoop(int index, Eigen::Matrix<double, 8, 1> &_loop_info)
{
    KeyFrame *kf = getKeyFrame(index);
    kf->updateLoop(_loop_info);
    if (abs(_loop_info(7)) < 30.0 && Vector3d(_loop_info(0), _loop_info(1), _loop_info(2)).norm() < 20.0)
    {
        if (FAST_RELOCALIZATION)
        {
            KeyFrame *old_kf = getKeyFrame(kf->loop_index);
            Vector3d w_P_old, w_P_cur, vio_P_cur;
            Matrix3d w_R_old, w_R_cur, vio_R_cur;
            old_kf->getPose(w_P_old, w_R_old);
            kf->getVioPose(vio_P_cur, vio_R_cur);

            Vector3d relative_t;
            Quaterniond relative_q;
            relative_t = kf->getLoopRelativeT();
            relative_q = (kf->getLoopRelativeQ()).toRotationMatrix();
            w_P_cur = w_R_old * relative_t + w_P_old;
            w_R_cur = w_R_old * relative_q;
            double shift_yaw;
            Matrix3d shift_r;
            Vector3d shift_t;
            shift_yaw = Utility::R2ypr(w_R_cur).x() - Utility::R2ypr(vio_R_cur).x();
            shift_r = Utility::ypr2R(Vector3d(shift_yaw, 0, 0));
            shift_t = w_P_cur - w_R_cur * vio_R_cur.transpose() * vio_P_cur;

            m_drift.lock();
            yaw_drift = shift_yaw;
            r_drift = shift_r;
            t_drift = shift_t;
            m_drift.unlock();
        }
    }
}