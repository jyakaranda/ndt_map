/**
 * @file ndt_map.cpp
 * @author heng zhang (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2019-05-20
 * 
 * @copyright Copyright (c) 2019
 * 
 */

#include "ndt_map/ndt_map.h"

NDTMap::NDTMap(ros::NodeHandle nh, ros::NodeHandle pnh) : nh_(nh), pnh_(pnh)
{
  pnh_.param<float>("scan_period", scan_period_, 0.2);
  pnh_.param<float>("keyframe_dist", keyframe_dist_, 0.3);
  keyframe_dist_ *= keyframe_dist_;
  pnh_.param<float>("surround_search_radius", surround_search_radius_, 20.);
  pnh_.param<int>("surround_search_num", surround_search_num_, 50);
  pnh_.param<float>("voxel_leaf_size", voxel_leaf_size_, 2.);
  pnh_.param<float>("min_scan_range", min_scan_range_, 2);
  min_scan_range_ *= min_scan_range_;
  pnh_.param<float>("max_scan_range", max_scan_range_, 70);
  max_scan_range_ *= max_scan_range_;
  pnh_.param<bool>("use_odom", use_odom_, true);
  pnh_.param<bool>("use_imu", use_imu_, true);
  pnh_.param<bool>("loop_closure_enabled", loop_closure_enabled_, true);

  pnh_.param<double>("trans_eps", trans_eps_, 0.01);
  pnh_.param<double>("step_size", step_size, 0.1);
  pnh_.param<double>("ndt_res", ndt_res_, 1.);
  pnh_.param<int>("max_iters", max_iters_, 30);

  pnh_.param<float>("history_search_radius", history_search_radius_, 10.);
  pnh_.param<int>("history_search_num", history_search_num_, 20);
  pnh_.param<float>("history_fitness_score", history_fitness_score_, 0.3);
  pnh_.param<float>("ds_history_size", ds_history_size_, 1.);

  pnh_.param<std::string>("save_dir", save_dir_, "");

  tf_b2l_ = Eigen::Matrix4f::Identity();
  float roll, pitch, yaw;
  if (!nh_.getParam("tf_b2l_x", tf_b2l_(0, 3)) || !nh_.getParam("tf_b2l_y", tf_b2l_(1, 3)) || !nh_.getParam("tf_b2l_z", tf_b2l_(2, 3)) || !nh_.getParam("tf_b2l_roll", roll) || !nh_.getParam("tf_b2l_pitch", pitch) || !nh_.getParam("tf_b2l_yaw", yaw))
  {
    ROS_ERROR("transform between /base_link to /laser not set.");
    exit(-1);
  }
  Eigen::AngleAxisf rx(roll, Eigen::Vector3f::UnitX());
  Eigen::AngleAxisf ry(pitch, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf rz(yaw, Eigen::Vector3f::UnitZ());
  tf_b2l_.block(0, 0, 3, 3) = (rz * ry * rx).matrix();

  if (!init())
  {
    exit(-1);
  }

  pub_keyposes_ = nh_.advertise<sensor_msgs::PointCloud2>("/keyposes", 1);
  pub_laser_cloud_surround_ = nh_.advertise<sensor_msgs::PointCloud2>("/laser_cloud_surround", 1);
  pub_undistorted_pc_ = nh_.advertise<sensor_msgs::PointCloud2>("/undistorted_pc", 1);
  pub_predict_pose_ = nh_.advertise<nav_msgs::Odometry>("/predict_pose", 1);
  pub_updated_pose_ = nh_.advertise<nav_msgs::Odometry>("/updated_pose", 1);

  pub_history_keyframes_ = nh_.advertise<sensor_msgs::PointCloud2>("/history_keyframes", 1);
  pub_recent_keyframes_ = nh_.advertise<sensor_msgs::PointCloud2>("/recent_keyframes", 1);
  pub_icp_keyframes_ = nh_.advertise<sensor_msgs::PointCloud2>("/icp_keyframes", 1);

  srv_save_map_ = nh_.advertiseService("/save_map", &NDTMap::saveMapCB, this);

  sub_pc_ = nh_.subscribe<sensor_msgs::PointCloud2>("/lslidar_point_cloud", 5, boost::bind(&NDTMap::pcCB, this, _1));
  sub_imu_ = nh_.subscribe<sensor_msgs::Imu>("/imu/data", 5, boost::bind(&NDTMap::imuCB, this, _1));
  sub_odom_ = nh_.subscribe<nav_msgs::Odometry>("/odom/imu", 5, boost::bind(&NDTMap::odomCB, this, _1));
}

bool NDTMap::init()
{
  ISAM2Params params;
  params.relinearizeThreshold = 0.01;
  params.relinearizeSkip = 1;
  isam = new ISAM2(params);
  gtsam::Vector vector6(6);
  vector6 << 1e-6, 1e-6, 1e-6, 1e-8, 1e-8, 1e-6;
  prior_noise_ = noiseModel::Diagonal::Variances(vector6);
  odom_noise_ = noiseModel::Diagonal::Variances(vector6);

  cpu_ndt_.setTransformationEpsilon(trans_eps_);
  cpu_ndt_.setResolution(ndt_res_);
  cpu_ndt_.setStepSize(step_size);
  cpu_ndt_.setMaximumIterations(max_iters_);

  voxel_filter_.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
  ds_source_.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
  ds_history_keyframes_.setLeafSize(ds_history_size_, ds_history_size_, ds_history_size_);

  imu_ptr_front_ = odom_ptr_front_ = 0;
  imu_ptr_last_ = odom_ptr_last_ = -1;
  imu_ptr_last_iter_ = odom_ptr_last_iter_ = 0;

  pre_pose_m_ = cur_pose_m_ = pre_pose_o_ = cur_pose_o_ = Eigen::Matrix4f::Identity();

  tf_m2o_.setIdentity();

  loop_closed_ = false;

  ground_filter.setIfClipHeight(false);
  ground_filter.setMinDistance(1.0);

  pc_source_.reset(new pcl::PointCloud<PointT>());
  pc_target_.reset(new pcl::PointCloud<PointT>());

  cloud_keyposes_3d_.reset(new pcl::PointCloud<PointT>());
  cloud_keyposes_6d_.reset(new pcl::PointCloud<PointXYZIRPYT>());

  latest_keyframe_.reset(new pcl::PointCloud<PointT>());
  near_history_keyframes_.reset(new pcl::PointCloud<PointT>());

  kdtree_poses_.reset(new pcl::KdTreeFLANN<PointT>());

  imu_time_.fill(0);
  imu_roll_.fill(0);
  imu_pitch_.fill(0);
  imu_yaw_.fill(0);

  imu_acc_x_.fill(0);
  imu_acc_y_.fill(0);
  imu_acc_z_.fill(0);
  imu_velo_x_.fill(0);
  imu_velo_y_.fill(0);
  imu_velo_z_.fill(0);
  imu_shift_x_.fill(0);
  imu_shift_y_.fill(0);
  imu_shift_z_.fill(0);

  imu_angular_velo_x_.fill(0);
  imu_angular_velo_y_.fill(0);
  imu_angular_velo_z_.fill(0);
  imu_angular_rot_x_.fill(0);
  imu_angular_rot_y_.fill(0);
  imu_angular_rot_z_.fill(0);

  ROS_INFO("init.");
  return true;
}

void NDTMap::run() {}

void NDTMap::visualThread()
{
  ros::Duration duration(2.5);
  while (ros::ok())
  {
    publishKeyposesAndFrames();
    duration.sleep();
  }
}

/**
 * @brief 参考 loam 的点云去运动畸变（基于匀速运动假设）
 * 
 */
void NDTMap::adjustDistortion(pcl::PointCloud<PointT>::Ptr &cloud, double scan_time)
{
  bool half_passed = false;
  int cloud_size = cloud->points.size();

  float start_ori = -std::atan2(cloud->points[0].y, cloud->points[0].x);
  float end_ori = -std::atan2(cloud->points[cloud_size - 1].y, cloud->points[cloud_size - 1].x);
  if (end_ori - start_ori > 3 * M_PI)
  {
    end_ori -= 2 * M_PI;
  }
  else if (end_ori - start_ori < M_PI)
  {
    end_ori += 2 * M_PI;
  }
  float ori_diff = end_ori - start_ori;

  Eigen::Vector3f rpy_start, shift_start, velo_start, rpy_cur, shift_cur, velo_cur;
  Eigen::Vector3f shift_from_start;
  Eigen::Matrix3f r_s_i, r_c;
  Eigen::Vector3f adjusted_p;
  float ori_h;
  for (int i = 0; i < cloud_size; ++i)
  {
    PointT &p = cloud->points[i];
    ori_h = -std::atan2(p.y, p.x);
    if (!half_passed)
    {
      if (ori_h < start_ori - M_PI * 0.5)
      {
        ori_h += 2 * M_PI;
      }
      else if (ori_h > start_ori + M_PI * 1.5)
      {
        ori_h -= 2 * M_PI;
      }

      if (ori_h - start_ori > M_PI)
      {
        half_passed = true;
      }
    }
    else
    {
      ori_h += 2 * M_PI;
      if (ori_h < end_ori - 1.5 * M_PI)
      {
        ori_h += 2 * M_PI;
      }
      else if (ori_h > end_ori + 0.5 * M_PI)
      {
        ori_h -= 2 * M_PI;
      }
    }

    float rel_time = (ori_h - start_ori) / ori_diff * scan_period_;

    if (imu_ptr_last_ > 0)
    {
      imu_ptr_front_ = imu_ptr_last_iter_;
      while (imu_ptr_front_ != imu_ptr_last_)
      {
        if (scan_time + rel_time < imu_time_[imu_ptr_front_])
        {
          break;
        }
        imu_ptr_front_ = (imu_ptr_front_ + 1) % imu_queue_len_;
      }
      if (std::abs(scan_time + rel_time - imu_time_[imu_ptr_front_]) > scan_period_)
      {
        ROS_WARN_COND(i < 10, "unsync imu and pc msg");
        continue;
      }

      if (scan_time + rel_time > imu_time_[imu_ptr_front_])
      {
        rpy_cur(0) = imu_roll_[imu_ptr_front_];
        rpy_cur(1) = imu_pitch_[imu_ptr_front_];
        rpy_cur(2) = imu_yaw_[imu_ptr_front_];
        shift_cur(0) = imu_shift_x_[imu_ptr_front_];
        shift_cur(1) = imu_shift_y_[imu_ptr_front_];
        shift_cur(2) = imu_shift_z_[imu_ptr_front_];
        velo_cur(0) = imu_velo_x_[imu_ptr_front_];
        velo_cur(1) = imu_velo_y_[imu_ptr_front_];
        velo_cur(2) = imu_velo_z_[imu_ptr_front_];
      }
      else
      {
        int imu_ptr_back = (imu_ptr_front_ - 1 + imu_queue_len_) % imu_queue_len_;
        float ratio_front = (scan_time + rel_time - imu_time_[imu_ptr_back]) / (imu_time_[imu_ptr_front_] - imu_time_[imu_ptr_back]);
        float ratio_back = 1. - ratio_front;
        rpy_cur(0) = imu_roll_[imu_ptr_front_] * ratio_front + imu_roll_[imu_ptr_back] * ratio_back;
        rpy_cur(1) = imu_pitch_[imu_ptr_front_] * ratio_front + imu_pitch_[imu_ptr_back] * ratio_back;
        rpy_cur(2) = imu_yaw_[imu_ptr_front_] * ratio_front + imu_yaw_[imu_ptr_back] * ratio_back;
        shift_cur(0) = imu_shift_x_[imu_ptr_front_] * ratio_front + imu_shift_x_[imu_ptr_back] * ratio_back;
        shift_cur(1) = imu_shift_y_[imu_ptr_front_] * ratio_front + imu_shift_y_[imu_ptr_back] * ratio_back;
        shift_cur(2) = imu_shift_z_[imu_ptr_front_] * ratio_front + imu_shift_z_[imu_ptr_back] * ratio_back;
        velo_cur(0) = imu_velo_x_[imu_ptr_front_] * ratio_front + imu_velo_x_[imu_ptr_back] * ratio_back;
        velo_cur(1) = imu_velo_y_[imu_ptr_front_] * ratio_front + imu_velo_y_[imu_ptr_back] * ratio_back;
        velo_cur(2) = imu_velo_z_[imu_ptr_front_] * ratio_front + imu_velo_z_[imu_ptr_back] * ratio_back;
      }

      r_c = (Eigen::AngleAxisf(rpy_cur(2), Eigen::Vector3f::UnitZ()) * Eigen::AngleAxisf(rpy_cur(1), Eigen::Vector3f::UnitY()) * Eigen::AngleAxisf(rpy_cur(0), Eigen::Vector3f::UnitX())).toRotationMatrix();

      if (i == 0)
      {
        rpy_start = rpy_cur;
        shift_start = shift_cur;
        velo_start = velo_cur;
        r_s_i = r_c.inverse();
      }
      else
      {
        shift_from_start = shift_cur - shift_start - velo_start * rel_time;
        adjusted_p = r_s_i * (r_c * Eigen::Vector3f(p.x, p.y, p.z) + shift_from_start);
        p.x = adjusted_p.x();
        p.y = adjusted_p.y();
        p.z = adjusted_p.z();
      }
    }
    imu_ptr_last_iter_ = imu_ptr_front_;
  }

  if (pub_undistorted_pc_.getNumSubscribers() > 0)
  {
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*cloud, msg);
    msg.header.stamp.fromSec(scan_time);
    msg.header.frame_id = "/laser";
    pub_undistorted_pc_.publish(msg);
  }
}

/**
 * @brief 提取附近点云作为 target map。
 * 在 loop_closure_enabled_ = false 的情况下目前不需要提取，因为可以通过 updateVoxelGrid 更新 target map
 * 
 */
void NDTMap::extractSurroundKeyframes()
{
  if (cloud_keyframes_.empty())
  {
    return;
  }

  bool target_updated = false;
  if (loop_closure_enabled_)
  {
    if (recent_keyframes_.size() < surround_search_num_)
    {
      recent_keyframes_.clear();
      for (int i = cloud_keyposes_3d_->points.size() - 1; i >= 0; --i)
      {
        int this_key_id = int(cloud_keyposes_3d_->points[i].intensity);
        pcl::PointCloud<PointT>::Ptr tf_cloud(new pcl::PointCloud<PointT>());
        tf_cloud = transformPointCloud(cloud_keyframes_[this_key_id], cloud_keyposes_6d_->points[this_key_id]);
        recent_keyframes_.push_back(tf_cloud);
        if (recent_keyframes_.size() >= surround_search_num_)
        {
          break;
        }
      }
      target_updated = true;
    }
    else
    {
      static int latest_frame_id = cloud_keyframes_.size() - 1;
      if (latest_frame_id != cloud_keyframes_.size() - 1)
      {
        latest_frame_id = cloud_keyframes_.size() - 1;
        recent_keyframes_.pop_back();
        pcl::PointCloud<PointT>::Ptr tf_cloud(new pcl::PointCloud<PointT>());
        tf_cloud = transformPointCloud(cloud_keyframes_[latest_frame_id], cloud_keyposes_6d_->points[latest_frame_id]);
        recent_keyframes_.push_front(tf_cloud);
        target_updated = true;
      }
    }
  }
  else
  {
  }

  if (target_updated)
  {
    pc_target_->clear();
    for (auto keyframe : recent_keyframes_)
    {
      *pc_target_ += *keyframe;
    }
    cpu_ndt_.setInputTarget(pc_target_);
    ROS_INFO("new ndt target set");
  }

  if (pub_recent_keyframes_.getNumSubscribers() > 0)
  {
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*pc_target_, msg);
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "map";
    pub_recent_keyframes_.publish(msg);
  }
}

/**
 * @brief 保存关键帧及其对应位姿，更新位姿图
 * 移动距离作为关键帧选取标准
 * 
 */
bool NDTMap::saveKeyframesAndFactor()
{
  // 此处的当前位姿(cur_pose_ndt_)为 ndt 匹配后的 final_transformation
  double roll, pitch, yaw;
  tf::Matrix3x3(tf::Quaternion(cur_pose_ndt_.pose.pose.orientation.x, cur_pose_ndt_.pose.pose.orientation.y, cur_pose_ndt_.pose.pose.orientation.z, cur_pose_ndt_.pose.pose.orientation.w)).getRPY(roll, pitch, yaw);
  if (cloud_keyposes_3d_->points.empty())
  {
    // gtSAMgraph_.add(PriorFactor<Pose3>(0, Pose3(Rot3::Quaternion(cur_pose_ndt_.pose.pose.orientation.w, cur_pose_ndt_.pose.pose.orientation.x, cur_pose_ndt_.pose.pose.orientation.y, cur_pose_ndt_.pose.pose.orientation.z), Point3(cur_pose_ndt_.pose.pose.position.x, cur_pose_ndt_.pose.pose.position.y, cur_pose_ndt_.pose.pose.position.z)), prior_noise_));
    gtSAMgraph_.add(PriorFactor<Pose3>(0, Pose3(Rot3::RzRyRx(roll, pitch, yaw), Point3(cur_pose_ndt_.pose.pose.position.x, cur_pose_ndt_.pose.pose.position.y, cur_pose_ndt_.pose.pose.position.z)), prior_noise_));
    initial_estimate_.insert(0, Pose3(Rot3::Quaternion(cur_pose_ndt_.pose.pose.orientation.w, cur_pose_ndt_.pose.pose.orientation.x, cur_pose_ndt_.pose.pose.orientation.y, cur_pose_ndt_.pose.pose.orientation.z), Point3(cur_pose_ndt_.pose.pose.position.x, cur_pose_ndt_.pose.pose.position.y, cur_pose_ndt_.pose.pose.position.z)));
    pre_keypose_ = cur_pose_ndt_;
  }
  else
  {
    const auto &pre_pose = cloud_keyposes_6d_->points[cloud_keyposes_3d_->points.size() - 1];
    if (std::pow(cur_pose_ndt_.pose.pose.position.x - pre_pose.x, 2) +
            std::pow(cur_pose_ndt_.pose.pose.position.y - pre_pose.y, 2) +
            std::pow(cur_pose_ndt_.pose.pose.position.z - pre_pose.z, 2) <
        keyframe_dist_)
    {
      // too close
      return false;
    }

    // gtsam::Pose3 pose_from = Pose3(Rot3::Quaternion(pre_keypose_.pose.pose.orientation.w, pre_keypose_.pose.pose.orientation.x, pre_keypose_.pose.pose.orientation.y, pre_keypose_.pose.pose.orientation.z), Point3(pre_keypose_.pose.pose.position.x, pre_keypose_.pose.pose.position.y, pre_keypose_.pose.pose.position.z));
    gtsam::Pose3 pose_from = Pose3(Rot3::RzRyRx(pre_pose.roll, pre_pose.pitch, pre_pose.yaw), Point3(pre_pose.x, pre_pose.y, pre_pose.z));
    // gtsam::Pose3 pose_to = Pose3(Rot3::Quaternion(cur_pose_ndt_.pose.pose.orientation.w, cur_pose_ndt_.pose.pose.orientation.x, cur_pose_ndt_.pose.pose.orientation.y, cur_pose_ndt_.pose.pose.orientation.z), Point3(cur_pose_ndt_.pose.pose.position.x, cur_pose_ndt_.pose.pose.position.y, cur_pose_ndt_.pose.pose.position.z));
    gtsam::Pose3 pose_to = Pose3(Rot3::RzRyRx(roll, pitch * 0, yaw), Point3(cur_pose_ndt_.pose.pose.position.x, cur_pose_ndt_.pose.pose.position.y, cur_pose_ndt_.pose.pose.position.z * 0));
    gtSAMgraph_.add(BetweenFactor<Pose3>(cloud_keyposes_3d_->points.size() - 1, cloud_keyposes_3d_->points.size(), pose_from.between(pose_to), odom_noise_));
    initial_estimate_.insert(cloud_keyposes_3d_->points.size(), Pose3(Rot3::RzRyRx(roll, pitch * 0, yaw), Point3(cur_pose_ndt_.pose.pose.position.x, cur_pose_ndt_.pose.pose.position.y, cur_pose_ndt_.pose.pose.position.z * 0)));
  }

  isam->update(gtSAMgraph_, initial_estimate_);
  isam->update();

  gtSAMgraph_.resize(0);
  initial_estimate_.clear();

  PointT this_pose_3d;
  PointXYZIRPYT this_pose_6d;
  Pose3 latest_estimate;
  isam_current_estimate_ = isam->calculateEstimate();
  latest_estimate = isam_current_estimate_.at<Pose3>(isam_current_estimate_.size() - 1);

  this_pose_6d.x = this_pose_3d.x = latest_estimate.translation().x();
  this_pose_6d.y = this_pose_3d.y = latest_estimate.translation().y();
  this_pose_6d.z = this_pose_3d.z = latest_estimate.translation().z();
  this_pose_6d.intensity = this_pose_3d.intensity = cloud_keyposes_3d_->points.size();
  this_pose_6d.roll = latest_estimate.rotation().roll();
  this_pose_6d.pitch = latest_estimate.rotation().pitch();
  this_pose_6d.yaw = latest_estimate.rotation().yaw();
  this_pose_6d.time = cur_pose_ndt_.header.stamp.toSec();
  cloud_keyposes_3d_->points.push_back(this_pose_3d);
  cloud_keyposes_6d_->points.push_back(this_pose_6d);

  // std::cout << "pre_keypose: (" << pre_keypose_.pose.pose.position.x << ", " << pre_keypose_.pose.pose.position.y << ", " << pre_keypose_.pose.pose.position.z << "; " << std::endl;
  std::cout << "cur_pose_ndt: (" << cur_pose_ndt_.pose.pose.position.x << ", " << cur_pose_ndt_.pose.pose.position.y << ", " << cur_pose_ndt_.pose.pose.position.z << "; "
            << roll << ", " << pitch << ", " << yaw << ")" << std::endl;
  std::cout << "this_pose_6d: (" << this_pose_6d.x << ", " << this_pose_6d.y << ", " << this_pose_6d.z << "; "
            << this_pose_6d.roll << ", " << this_pose_6d.pitch << ", " << this_pose_6d.yaw << ")" << std::endl;

  if (cloud_keyposes_3d_->points.size() > 1)
  {
    pre_keypose_.pose.pose.position.x = this_pose_3d.x;
    pre_keypose_.pose.pose.position.y = this_pose_3d.y;
    pre_keypose_.pose.pose.position.z = this_pose_3d.z;
    pre_keypose_.pose.pose.orientation.w = latest_estimate.rotation().toQuaternion().w();
    pre_keypose_.pose.pose.orientation.x = latest_estimate.rotation().toQuaternion().x();
    pre_keypose_.pose.pose.orientation.y = latest_estimate.rotation().toQuaternion().y();
    pre_keypose_.pose.pose.orientation.z = latest_estimate.rotation().toQuaternion().z();
    pre_keypose_.header.stamp = cur_pose_ndt_.header.stamp;
  }

  cur_pose_m_.block<3, 3>(0, 0) = Eigen::Quaternionf(pre_keypose_.pose.pose.orientation.w, pre_keypose_.pose.pose.orientation.x, pre_keypose_.pose.pose.orientation.y, pre_keypose_.pose.pose.orientation.z).toRotationMatrix();
  cur_pose_m_(0, 3) = pre_keypose_.pose.pose.position.x;
  cur_pose_m_(1, 3) = pre_keypose_.pose.pose.position.y;
  cur_pose_m_(2, 3) = pre_keypose_.pose.pose.position.z;

  pcl::PointCloud<PointT>::Ptr cur_keyframe(new pcl::PointCloud<PointT>());
  pcl::copyPointCloud(*pc_source_, *cur_keyframe);
  for (auto &p : cur_keyframe->points)
  {
    p.intensity = this_pose_3d.intensity;
  }

  cloud_keyframes_.push_back(cur_keyframe);

  ROS_INFO("saveKeyframesAndFactor: %d points", cur_keyframe->points.size());

  return true;
}

void NDTMap::publishKeyposesAndFrames()
{
  if (pub_keyposes_.getNumSubscribers() > 0)
  {
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*cloud_keyposes_3d_, msg);
    msg.header.frame_id = "map";
    msg.header.stamp = ros::Time::now();
    pub_keyposes_.publish(msg);
  }
  if (pub_laser_cloud_surround_.getNumSubscribers() > 0)
  {
    int num_points = 0;
    sensor_msgs::PointCloud2 msg;
    pcl::PointCloud<PointT>::Ptr map(new pcl::PointCloud<PointT>());
    Eigen::Matrix4f T(Eigen::Matrix4f::Identity());
    for (int i = 0; i < cloud_keyframes_.size(); ++i)
    {
      pcl::PointCloud<PointT>::Ptr tmp(new pcl::PointCloud<PointT>());
      tmp = transformPointCloud(cloud_keyframes_[i], cloud_keyposes_6d_->points[i]);
      *map += *tmp;
      num_points += tmp->points.size();
    }
    pcl::toROSMsg(*map, msg);
    msg.header.frame_id = "map";
    msg.header.stamp = ros::Time::now();
    pub_laser_cloud_surround_.publish(msg);
    ROS_INFO("Map Size: %d points", num_points);
  }
}

void NDTMap::odomCB(const nav_msgs::OdometryConstPtr &msg)
{
  odom_ptr_last_ = (odom_ptr_last_ + 1) % imu_queue_len_;
  if ((odom_ptr_last_ + 1) % imu_queue_len_ == odom_ptr_front_)
  {
    odom_ptr_front_ = (odom_ptr_front_ + 1) % imu_queue_len_;
  }
  odom_queue_[odom_ptr_last_] = *msg;
}

void NDTMap::pcCB(const sensor_msgs::PointCloud2ConstPtr &msg)
{
  std::lock_guard<std::mutex> lock(mtx_);
  auto start = std::chrono::system_clock::now();

  extractSurroundKeyframes();

  // 去运动畸变，距离和 voxel 滤波
  pcl::PointCloud<PointT>::Ptr tmp_cloud(new pcl::PointCloud<PointT>());

  pc_source_->clear();
  pcl::fromROSMsg(*msg, *tmp_cloud);
  // 点云转换到 /base_link 坐标系中处理
  pcl::transformPointCloud(*tmp_cloud, *pc_source_, tf_b2l_);

  if (use_imu_)
  {
    adjustDistortion(pc_source_, msg->header.stamp.toSec());
  }

  tmp_cloud->clear();
  ROS_INFO("before ds: %d points", pc_source_->points.size());
  voxel_filter_.setInputCloud(pc_source_);
  voxel_filter_.filter(*tmp_cloud);
  pc_source_->clear();
  float r;
  for (const auto &p : tmp_cloud->points)
  {
    r = p.x * p.x + p.y * p.y;
    if (r > min_scan_range_ && r < max_scan_range_)
    {
      pc_source_->points.push_back(p);
    }
  }
  ROS_INFO("after ds: %d points", pc_source_->points.size());

  // 第一帧点云，存为 target，初始化起始位置
  if (cloud_keyframes_.empty())
  {
    ROS_INFO("first laser frame.");
    *pc_target_ += *pc_source_;
    cpu_ndt_.setInputTarget(pc_target_);
    if (use_odom_ && odom_ptr_last_ != -1)
    {
      int odom_ptr = odom_ptr_front_;
      while (odom_ptr != odom_ptr_last_)
      {
        if (odom_queue_[odom_ptr].header.stamp > msg->header.stamp)
        {
          break;
        }
        odom_ptr = (odom_ptr + 1) % imu_queue_len_;
      }
      pre_pose_o_.block<3, 3>(0, 0) = Eigen::Quaternionf(odom_queue_[odom_ptr].pose.pose.orientation.w, odom_queue_[odom_ptr].pose.pose.orientation.x, odom_queue_[odom_ptr].pose.pose.orientation.y, odom_queue_[odom_ptr].pose.pose.orientation.z).toRotationMatrix();
      pre_pose_o_(0, 3) = odom_queue_[odom_ptr].pose.pose.position.x;
      pre_pose_o_(1, 3) = odom_queue_[odom_ptr].pose.pose.position.y;
      pre_pose_o_(2, 3) = odom_queue_[odom_ptr].pose.pose.position.z;
      cur_pose_o_ = pre_pose_o_;
      odom_ptr_front_ = odom_ptr; // 更新指针
    }
  }

  nav_msgs::Odometry predict_msg;
  // 配准初始位姿估计
  if (use_odom_ && odom_ptr_last_ != -1)
  {
    pre_pose_o_ = cur_pose_o_;
    int odom_ptr = odom_ptr_front_;
    while (odom_ptr != odom_ptr_last_)
    {
      if (odom_queue_[odom_ptr].header.stamp > msg->header.stamp)
      {
        break;
      }
      odom_ptr = (odom_ptr + 1) % imu_queue_len_;
    }
    cur_pose_o_.block<3, 3>(0, 0) = Eigen::Quaternionf(odom_queue_[odom_ptr].pose.pose.orientation.w, odom_queue_[odom_ptr].pose.pose.orientation.x, odom_queue_[odom_ptr].pose.pose.orientation.y, odom_queue_[odom_ptr].pose.pose.orientation.z).toRotationMatrix();
    cur_pose_o_(0, 3) = odom_queue_[odom_ptr].pose.pose.position.x;
    cur_pose_o_(1, 3) = odom_queue_[odom_ptr].pose.pose.position.y;
    cur_pose_o_(2, 3) = odom_queue_[odom_ptr].pose.pose.position.z;
    odom_ptr_front_ = odom_ptr; // 更新指针

    Eigen::Quaternionf tmp_q(pre_pose_m_.block<3, 3>(0, 0));
    double roll, pitch, yaw;
    tf::Matrix3x3(tf::Quaternion(tmp_q.x(), tmp_q.y(), tmp_q.z(), tmp_q.w())).getRPY(roll, pitch, yaw);
    pre_pose_m_.block<3, 3>(0, 0) = Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()).toRotationMatrix();

    Eigen::Matrix4f r_m2o = Eigen::Matrix4f::Identity();
    r_m2o.block<3, 3>(0, 0) = Eigen::Quaternionf(tf_m2o_.getRotation().w(), tf_m2o_.getRotation().x(), tf_m2o_.getRotation().y(), tf_m2o_.getRotation().z()).toRotationMatrix();
    cur_pose_m_ = pre_pose_m_ * pre_pose_o_.inverse() * cur_pose_o_; // 见笔记“预测当前位姿”

    tmp_q = Eigen::Quaternionf(cur_pose_m_.block<3, 3>(0, 0));
    tf::Matrix3x3(tf::Quaternion(tmp_q.x(), tmp_q.y(), tmp_q.z(), tmp_q.w())).getRPY(roll, pitch, yaw);
    std::cout << "initial guess: "
              << "(" << cur_pose_m_(0, 3) << ", " << cur_pose_m_(1, 3) << ", " << cur_pose_m_(2, 3) << ", " << roll
              << ", " << pitch << ", " << yaw << ")" << std::endl;
    predict_msg.header.stamp = msg->header.stamp;
    predict_msg.header.frame_id = "map";
    predict_msg.pose.pose.position.x = cur_pose_m_(0, 3);
    predict_msg.pose.pose.position.y = cur_pose_m_(1, 3);
    predict_msg.pose.pose.position.z = cur_pose_m_(2, 3);
    predict_msg.pose.pose.orientation.w = tmp_q.w();
    predict_msg.pose.pose.orientation.x = tmp_q.x();
    predict_msg.pose.pose.orientation.y = tmp_q.y();
    predict_msg.pose.pose.orientation.z = tmp_q.z();
    pub_predict_pose_.publish(predict_msg);
  }
  else
  {
    // TODO: 改进
    predict_pose_ = pre_pose_ndt_;
    cur_pose_m_ = pre_pose_m_;
  }

  // ndt 匹配迭代
  cpu_ndt_.setInputSource(pc_source_);
  cpu_ndt_.align(cur_pose_m_);
  fitness_score_ = cpu_ndt_.getFitnessScore();
  final_transformation_ = cpu_ndt_.getFinalTransformation();
  has_converged_ = cpu_ndt_.hasConverged();
  final_iters_ = cpu_ndt_.getFinalNumIteration();

  Eigen::Quaternionf tmp_q(final_transformation_.block<3, 3>(0, 0));
  double roll, pitch, yaw;
  tf::Matrix3x3(tf::Quaternion(tmp_q.x(), tmp_q.y(), tmp_q.z(), tmp_q.w())).getRPY(roll, pitch, yaw);

  cur_pose_ndt_.pose.pose.position.x = final_transformation_(0, 3);
  cur_pose_ndt_.pose.pose.position.y = final_transformation_(1, 3);
  cur_pose_ndt_.pose.pose.position.z = final_transformation_(2, 3);
  cur_pose_ndt_.pose.pose.orientation.w = tmp_q.w();
  cur_pose_ndt_.pose.pose.orientation.x = tmp_q.x();
  cur_pose_ndt_.pose.pose.orientation.y = tmp_q.y();
  cur_pose_ndt_.pose.pose.orientation.z = tmp_q.z();
  cur_pose_ndt_.header.stamp = msg->header.stamp;
  cur_pose_m_ = final_transformation_;
  nav_msgs::Odometry updated_msg;
  updated_msg.header.stamp = msg->header.stamp;
  updated_msg.header.frame_id = "map";
  updated_msg.pose = cur_pose_ndt_.pose;
  pub_updated_pose_.publish(updated_msg);

  tf::Transform tf_m2b;
  tf_m2b.setOrigin(tf::Vector3(final_transformation_(0, 3), final_transformation_(1, 3), final_transformation_(2, 3)));
  tf_m2b.setRotation(tf::Quaternion(tmp_q.x(), tmp_q.y(), tmp_q.z(), tmp_q.w()));
  if (use_odom_ && odom_ptr_last_ != -1)
  {
    tf_o2b_.setOrigin(tf::Vector3(odom_queue_[odom_ptr_front_].pose.pose.position.x, odom_queue_[odom_ptr_front_].pose.pose.position.y, odom_queue_[odom_ptr_front_].pose.pose.position.z));
    tf_o2b_.setRotation(tf::Quaternion(odom_queue_[odom_ptr_front_].pose.pose.orientation.x, odom_queue_[odom_ptr_front_].pose.pose.orientation.y, odom_queue_[odom_ptr_front_].pose.pose.orientation.z, odom_queue_[odom_ptr_front_].pose.pose.orientation.w));
    tf_m2o_ = tf_m2b * tf_o2b_.inverse();
    // tf::StampedTransform tmp;
    // tf_listener_.waitForTransform("/odom", "/base_link", ros::Time(0), ros::Duration(0.05));
    // tf_listener_.lookupTransform("/odom", "/base_link", ros::Time(0), tmp);
    // tf_m2o_ = tf_m2b * tmp.inverse();
    tf_broadcaster_.sendTransform(tf::StampedTransform(tf_m2o_, msg->header.stamp, "map", "/odom"));
  }
  else
  {
    tf_broadcaster_.sendTransform(tf::StampedTransform(tf_m2b, msg->header.stamp, "map", "/base_link"));
  }

  auto end = std::chrono::system_clock::now();
  std::chrono::duration<double> elapsed = end - start;
  std::cout << "-----------------------------------------------------------------" << std::endl;
  std::cout << "Sequence number: " << msg->header.seq << "; time elapsed: " << elapsed.count() << std::endl;
  std::cout << "Number of scan points: " << pc_source_->size() << " points." << std::endl;
  std::cout << "map: " << pc_target_->points.size() << " points." << std::endl;
  std::cout << "NDT has converged: " << has_converged_ << std::endl;
  std::cout << "Fitness score: " << fitness_score_ << std::endl;
  std::cout << "Number of iteration: " << final_iters_ << std::endl;
  std::cout << "(x,y,z,roll,pitch,yaw):" << std::endl;
  std::cout << "(" << final_transformation_(0, 3) << ", " << final_transformation_(1, 3) << ", " << final_transformation_(2, 3) << ", " << roll
            << ", " << pitch << ", " << yaw << ")" << std::endl;
  std::cout << "Transformation Matrix:" << std::endl;
  std::cout << final_transformation_ << std::endl;
  std::cout << "shift: " << std::sqrt(std::pow(final_transformation_(0, 3) - pre_pose_m_(0, 3), 2) + std::pow(final_transformation_(1, 3) - pre_pose_m_(1, 3), 2) + std::pow(final_transformation_(2, 3) - pre_pose_m_(2, 3), 2)) << std::endl;
  std::cout << "-----------------------------------------------------------------" << std::endl;

  if (saveKeyframesAndFactor())
  {
    pcl::PointCloud<PointT>::Ptr pc_m(new pcl::PointCloud<PointT>());
    pcl::transformPointCloud(*pc_source_, *pc_m, cur_pose_m_);
    cpu_ndt_.updateVoxelGrid(pc_m);
  }
  else
  {
    std::cout << "too close" << std::endl;
  }

  pre_pose_m_ = cur_pose_m_;
  pre_pose_ndt_ = cur_pose_ndt_;

  correctPoses();
}

void NDTMap::imuCB(const sensor_msgs::ImuConstPtr &msg)
{
  double roll, pitch, yaw;
  tf::Quaternion orientation;
  tf::quaternionMsgToTF(msg->orientation, orientation);
  tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
  float acc_x = msg->linear_acceleration.x + 9.81 * sin(pitch);
  float acc_y = msg->linear_acceleration.y - 9.81 * cos(pitch) * sin(roll);
  float acc_z = msg->linear_acceleration.z - 9.81 * cos(pitch) * cos(roll);

  imu_ptr_last_ = (imu_ptr_last_ + 1) % imu_queue_len_;

  if ((imu_ptr_last_ + 1) % imu_queue_len_ == imu_ptr_front_)
  {
    imu_ptr_front_ = (imu_ptr_front_ + 1) % imu_queue_len_;
  }

  imu_time_[imu_ptr_last_] = msg->header.stamp.toSec();
  imu_roll_[imu_ptr_last_] = roll;
  imu_pitch_[imu_ptr_last_] = pitch;
  imu_yaw_[imu_ptr_last_] = yaw;
  imu_acc_x_[imu_ptr_last_] = acc_x;
  imu_acc_y_[imu_ptr_last_] = acc_y;
  imu_acc_z_[imu_ptr_last_] = acc_z;
  imu_angular_velo_x_[imu_ptr_last_] = msg->angular_velocity.x;
  imu_angular_velo_y_[imu_ptr_last_] = msg->angular_velocity.y;
  imu_angular_velo_z_[imu_ptr_last_] = msg->angular_velocity.z;

  // 转换到 imu 的全局坐标系中
  Eigen::Matrix3f rot = Eigen::Quaternionf(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z).toRotationMatrix();
  Eigen::Vector3f acc = rot * Eigen::Vector3f(acc_x, acc_y, acc_z);
  // TODO: lego_loam 里没有对角速度转换，是否需要尚且存疑
  // Eigen::Vector3f angular_velo = rot * Eigen::Vector3f(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
  Eigen::Vector3f angular_velo(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);

  int imu_ptr_back = (imu_ptr_last_ - 1 + imu_queue_len_) % imu_queue_len_;
  double time_diff = imu_time_[imu_ptr_last_] - imu_time_[imu_ptr_back];
  if (time_diff < 1.)
  {
    imu_shift_x_[imu_ptr_last_] = imu_shift_x_[imu_ptr_back] + imu_velo_x_[imu_ptr_back] * time_diff + acc(0) * time_diff * time_diff * 0.5;
    imu_shift_y_[imu_ptr_last_] = imu_shift_y_[imu_ptr_back] + imu_velo_y_[imu_ptr_back] * time_diff + acc(1) * time_diff * time_diff * 0.5;
    imu_shift_z_[imu_ptr_last_] = imu_shift_z_[imu_ptr_back] + imu_velo_z_[imu_ptr_back] * time_diff + acc(2) * time_diff * time_diff * 0.5;

    imu_velo_x_[imu_ptr_last_] = imu_velo_x_[imu_ptr_back] + acc(0) * time_diff;
    imu_velo_y_[imu_ptr_last_] = imu_velo_y_[imu_ptr_back] + acc(1) * time_diff;
    imu_velo_z_[imu_ptr_last_] = imu_velo_z_[imu_ptr_back] + acc(2) * time_diff;

    imu_angular_rot_x_[imu_ptr_last_] = imu_angular_rot_x_[imu_ptr_back] + angular_velo(0) * time_diff;
    imu_angular_rot_y_[imu_ptr_last_] = imu_angular_rot_y_[imu_ptr_back] + angular_velo(1) * time_diff;
    imu_angular_rot_z_[imu_ptr_last_] = imu_angular_rot_z_[imu_ptr_back] + angular_velo(2) * time_diff;
  }
}

void NDTMap::loopClosureThread()
{
  if (!loop_closure_enabled_)
  {
    return;
  }
  ros::Duration duration(1);
  while (ros::ok())
  {
    performLoopClosure();
    duration.sleep();
  }
}

/**
 * @brief 回环检测及位姿图更新
 * ICP 匹配添加回环约束
 */
void NDTMap::performLoopClosure()
{
  if (cloud_keyposes_3d_->points.empty())
  {
    return;
  }

  if (!detectLoopClosure())
  {
    return;
  }
  else
  {
    ROS_WARN("detected loop closure");
  }

  auto start = std::chrono::system_clock::now();

  pcl::IterativeClosestPoint<PointT, PointT> icp;
  icp.setMaxCorrespondenceDistance(100);
  icp.setMaximumIterations(100);
  icp.setTransformationEpsilon(1e-6);
  icp.setEuclideanFitnessEpsilon(1e-6);
  icp.setRANSACIterations(0);

  icp.setInputSource(latest_keyframe_);
  icp.setInputTarget(near_history_keyframes_);
  pcl::PointCloud<PointT>::Ptr unused_result(new pcl::PointCloud<PointT>());
  Eigen::Matrix4f initial_guess(Eigen::Matrix4f::Identity());
  initial_guess.block<3, 3>(0, 0) = (Eigen::AngleAxisf(cloud_keyposes_6d_->points[latest_history_frame_id_].yaw, Eigen::Vector3f::UnitZ()) *
                                     Eigen::AngleAxisf(cloud_keyposes_6d_->points[latest_history_frame_id_].pitch, Eigen::Vector3f::UnitY()) *
                                     Eigen::AngleAxisf(cloud_keyposes_6d_->points[latest_history_frame_id_].roll, Eigen::Vector3f::UnitX()))
                                        .toRotationMatrix();
  initial_guess(0, 3) = cloud_keyposes_6d_->points[latest_history_frame_id_].x;
  initial_guess(1, 3) = cloud_keyposes_6d_->points[latest_history_frame_id_].y;
  initial_guess(2, 3) = cloud_keyposes_6d_->points[latest_history_frame_id_].z;
  icp.align(*unused_result);

  bool has_converged = icp.hasConverged();
  float fitness_score = icp.getFitnessScore();
  Eigen::Matrix4f correction_frame = icp.getFinalTransformation();
  Eigen::Quaternionf tmp_q(correction_frame.block<3, 3>(0, 0));
  double roll, pitch, yaw;
  tf::Matrix3x3(tf::Quaternion(tmp_q.x(), tmp_q.y(), tmp_q.z(), tmp_q.w())).getRPY(roll, pitch, yaw);

  auto end = std::chrono::system_clock::now();
  std::chrono::duration<double> elapsed = end - start;

  if (has_converged == false || fitness_score > history_fitness_score_)
  {
    ROS_WARN("loop cannot closed");
    return;
  }
  else
  {
    ROS_WARN("loop closed");
    if (pub_icp_keyframes_.getNumSubscribers() > 0)
    {
      pcl::PointCloud<PointT>::Ptr closed_cloud(new pcl::PointCloud<PointT>());
      pcl::transformPointCloud(*latest_keyframe_, *closed_cloud, correction_frame);
      sensor_msgs::PointCloud2 msg;
      pcl::toROSMsg(*closed_cloud, msg);
      msg.header.stamp.fromSec(cloud_keyposes_6d_->points[latest_history_frame_id_].time);
      msg.header.frame_id = "map";
      pub_icp_keyframes_.publish(msg);
    }
  }

  Eigen::Matrix4f t_wrong = initial_guess;
  Eigen::Matrix4f t_correct = correction_frame * t_wrong;
  // Eigen::Matrix4f t_correct = correction_frame;
  Eigen::Quaternionf r_correct(t_correct.block<3, 3>(0, 0));
  gtsam::Pose3 pose_from = Pose3(Rot3::Quaternion(r_correct.w(), r_correct.x(), r_correct.y(), r_correct.z()),
                                 Point3(t_correct(0, 3), t_correct(1, 3), t_correct(2, 3)));
  gtsam::Pose3 pose_to = Pose3(Rot3::RzRyRx(cloud_keyposes_6d_->points[closest_history_frame_id_].roll, cloud_keyposes_6d_->points[closest_history_frame_id_].pitch, cloud_keyposes_6d_->points[closest_history_frame_id_].yaw),
                               Point3(cloud_keyposes_6d_->points[closest_history_frame_id_].x, cloud_keyposes_6d_->points[closest_history_frame_id_].y, cloud_keyposes_6d_->points[closest_history_frame_id_].z));
  float noise_score = fitness_score;
  gtsam::Vector vector6(6);
  vector6 << noise_score, noise_score, noise_score, noise_score, noise_score, noise_score;
  constraint_noise_ = noiseModel::Diagonal::Variances(vector6);

  std::lock_guard<std::mutex> lock(mtx_);
  gtSAMgraph_.add(BetweenFactor<Pose3>(latest_history_frame_id_, closest_history_frame_id_, pose_from.between(pose_to), constraint_noise_));
  isam->update(gtSAMgraph_);
  isam->update();
  gtSAMgraph_.resize(0);

  std::cout << "-----------------------------------------------------------------" << std::endl;
  std::cout << "Time elapsed: " << elapsed.count() << std::endl;
  std::cout << "Number of source points: " << latest_keyframe_->size() << " points." << std::endl;
  std::cout << "target: " << near_history_keyframes_->points.size() << " points." << std::endl;
  std::cout << "ICP has converged: " << has_converged << std::endl;
  std::cout << "Fitness score: " << fitness_score << std::endl;
  std::cout << "initial (x,y,z,roll,pitch,yaw):" << std::endl;
  std::cout << "(" << initial_guess(0, 3) << ", " << initial_guess(1, 3) << ", " << initial_guess(2, 3) << ", "
            << cloud_keyposes_6d_->points[latest_history_frame_id_].roll << ", " << cloud_keyposes_6d_->points[latest_history_frame_id_].pitch << ", " << cloud_keyposes_6d_->points[latest_history_frame_id_].yaw << ")" << std::endl;
  std::cout << "final (x,y,z,roll,pitch,yaw):" << std::endl;
  std::cout << "(" << t_correct(0, 3) << ", " << t_correct(1, 3) << ", " << t_correct(2, 3) << ", "
            << roll << ", " << pitch << ", " << yaw << ")" << std::endl;
  std::cout << "Transformation Matrix:" << std::endl;
  std::cout << t_correct << std::endl;
  std::cout << "shift: " << std::sqrt(std::pow(correction_frame(0, 3) - initial_guess(0, 3), 2) + std::pow(correction_frame(1, 3) - initial_guess(1, 3), 2) + std::pow(correction_frame(2, 3) - initial_guess(2, 3), 2)) << std::endl;
  std::cout << "-----------------------------------------------------------------" << std::endl;

  loop_closed_ = true;
}

/**
 * @brief 基于里程计的回环检测，直接提取当前位姿附近(radiusSearch)的 keyframe 作为 icp 的 target
 * 
 */
bool NDTMap::detectLoopClosure()
{
  latest_keyframe_->clear();
  near_history_keyframes_->clear();

  std::lock_guard<std::mutex> lock(mtx_);

  PointT cur_pose;
  cur_pose.x = cur_pose_m_(0, 3);
  cur_pose.y = cur_pose_m_(1, 3);
  cur_pose.z = cur_pose_m_(2, 3);
  kdtree_poses_->setInputCloud(cloud_keyposes_3d_);
  kdtree_poses_->radiusSearch(cur_pose, history_search_radius_, search_idx_, search_dist_);

  latest_history_frame_id_ = cloud_keyframes_.size() - 1;
  closest_history_frame_id_ = -1;
  for (int i = 0; i < search_idx_.size(); ++i)
  {
    if (cur_pose_ndt_.header.stamp.toSec() - cloud_keyposes_6d_->points[search_idx_[i]].time > 30.)
    {
      closest_history_frame_id_ = search_idx_[i];
      break;
    }
  }
  // 时间太短不做回环
  if (closest_history_frame_id_ == -1)
  {
    return false;
  }

  pcl::copyPointCloud(*transformPointCloud(cloud_keyframes_[latest_history_frame_id_], cloud_keyposes_6d_->points[latest_history_frame_id_]), *latest_keyframe_);

  pcl::PointCloud<PointT>::Ptr tmp_cloud(new pcl::PointCloud<PointT>());
  for (int i = -history_search_num_, j; i <= history_search_num_; ++i)
  {
    j = closest_history_frame_id_ + i;
    if (j < 0 || j >= latest_history_frame_id_)
    {
      continue;
    }
    *tmp_cloud += *transformPointCloud(cloud_keyframes_[j], cloud_keyposes_6d_->points[j]);
  }

  ds_history_keyframes_.setInputCloud(tmp_cloud);
  ds_history_keyframes_.filter(*near_history_keyframes_);

  if (pub_history_keyframes_.getNumSubscribers() > 0)
  {
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*near_history_keyframes_, msg);
    msg.header.stamp = cur_pose_ndt_.header.stamp;
    msg.header.frame_id = "map";
    pub_history_keyframes_.publish(msg);
  }

  return true;
}

/**
 * @brief 检测出回环后，更新位姿及 target map
 * 
 */
void NDTMap::correctPoses()
{
  if (loop_closed_)
  {
    recent_keyframes_.clear();
    ROS_WARN("correctPoses");
    int num_poses = isam_current_estimate_.size();
    for (int i = 0; i < num_poses; ++i)
    {
      // if (std::abs(cloud_keyposes_3d_->points[i].z - isam_current_estimate_.at<Pose3>(i).translation().z()) > 1)
      // {
      //   ROS_WARN("aaaa");
      // }
      // else
      // {
      //   ROS_WARN("bbbb");
      // }
      cloud_keyposes_6d_->points[i].x = cloud_keyposes_3d_->points[i].x = isam_current_estimate_.at<Pose3>(i).translation().x();
      cloud_keyposes_6d_->points[i].y = cloud_keyposes_3d_->points[i].y = isam_current_estimate_.at<Pose3>(i).translation().y();
      cloud_keyposes_6d_->points[i].z = cloud_keyposes_3d_->points[i].z = isam_current_estimate_.at<Pose3>(i).translation().z();
      cloud_keyposes_6d_->points[i].roll = isam_current_estimate_.at<Pose3>(i).rotation().roll();
      cloud_keyposes_6d_->points[i].pitch = isam_current_estimate_.at<Pose3>(i).rotation().pitch();
      cloud_keyposes_6d_->points[i].yaw = isam_current_estimate_.at<Pose3>(i).rotation().yaw();
    }

    loop_closed_ = false;
  }
}

bool NDTMap::saveMapCB(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  std::stringstream ss;
  ss << int(ros::Time::now().toSec());
  std::string stamp = ss.str();

  int num_points = 0, num_points1 = 0;
  pcl::PointCloud<PointT>::Ptr map(new pcl::PointCloud<PointT>());
  pcl::PointCloud<PointT>::Ptr map_no_ground(new pcl::PointCloud<PointT>());
  for (int i = 0; i < cloud_keyframes_.size(); ++i)
  {
    pcl::PointCloud<PointT>::Ptr tmp(new pcl::PointCloud<PointT>());
    tmp = transformPointCloud(cloud_keyframes_[i], cloud_keyposes_6d_->points[i]);
    *map += *tmp;
    num_points += tmp->points.size();

    pcl::PointCloud<PointT>::Ptr tmp1(new pcl::PointCloud<PointT>());
    ground_filter.convert(cloud_keyframes_[i], tmp1);
    tmp1 = transformPointCloud(tmp1, cloud_keyposes_6d_->points[i]);
    *map_no_ground += *tmp1;
    num_points1 += tmp1->points.size();
  }

  map->width = map->points.size();
  map->height = 1;
  map->is_dense = false;

  map_no_ground->width = map_no_ground->points.size();
  map_no_ground->height = 1;
  map_no_ground->is_dense = false;

  pcl::PointCloud<PointT>::Ptr poses(new pcl::PointCloud<PointT>());
  pcl::copyPointCloud(*cloud_keyposes_3d_, *poses);
  poses->width = poses->points.size();
  poses->height = 1;
  poses->is_dense = false;

  pcl::io::savePCDFile(save_dir_ + "pose3d_" + stamp + ".pcd", *poses);
  pcl::io::savePCDFile(save_dir_ + "frames_" + stamp + ".pcd", *map);
  pcl::io::savePCDFile(save_dir_ + "frames_no_ground_" + stamp + ".pcd", *map_no_ground);

  ROS_WARN("Save map. pose size: %d, cloud size: %d, cloud no ground size: %d", poses->points.size(), num_points, num_points1);
}