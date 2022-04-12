#pragma once

#include "raisim/math.hpp"
/// do not change the name of the method
inline Eigen::Vector3d getFootLinearVelocity (const Eigen::VectorXd& gc, const Eigen::VectorXd& gv) {

  /// Local Coordinates
  //world_to_root
  Eigen::Vector3d r_world_root(gc(0), gc(1), gc(2));
  Eigen::Matrix3d R_world_root;
  R_world_root = Eigen::Quaternion<double>(gc(3), gc(4), gc(5), gc(6)).toRotationMatrix();

  //root_to_hip
  Eigen::Vector3d r_root_hip(-0.183f, 0.047f, 0.0f);
  Eigen::Matrix3d R_root_hip(Eigen::AngleAxisd(gc(16), Eigen::Vector3d::UnitX()));

  //hip_to_thigh
  Eigen::Vector3d r_hip_thigh(0.0f, 0.08505f, 0.0f);
  Eigen::Matrix3d R_hip_thigh(Eigen::AngleAxisd(gc(17), Eigen::Vector3d::UnitY()));

  //thigh_to_calf
  Eigen::Vector3d r_thigh_calf(0.0f, 0.0f, -0.2f);
  Eigen::Matrix3d R_thigh_calf(Eigen::AngleAxisd(gc(18), Eigen::Vector3d::UnitY()));

  //calf_to_ee
  Eigen::Vector3d r_calf_ee(0.0f, 0.0f, -0.2f);

  /// Forward Kinematics
  // world to hip
  Eigen::Vector3d r_world_hip = r_world_root + R_world_root * r_root_hip;
  // world to thigh
  Eigen::Vector3d r_world_thigh = r_world_root + R_world_root * (r_root_hip + (R_root_hip * r_hip_thigh));
  // world to calf
  Eigen::Vector3d r_world_calf = r_world_root + R_world_root * (r_root_hip + (R_root_hip * (r_hip_thigh + R_hip_thigh * r_thigh_calf)));
  // world to ee
  Eigen::Vector3d r_world_ee = r_world_root + R_world_root * (r_root_hip + R_root_hip * (r_hip_thigh + R_hip_thigh * ((r_thigh_calf + R_thigh_calf * (r_calf_ee)))));
  
  // loal orientations to world coordinate 
  Eigen::Matrix3d R_world_hip = R_world_root * R_root_hip;
  Eigen::Matrix3d R_world_thigh = R_world_hip * R_hip_thigh;
  Eigen::Matrix3d R_world_calf = R_world_thigh * R_thigh_calf;

  // angular velocity of each joint w.r.t. world coordinate
  Eigen::Vector3d wW_root = Eigen::Vector3d{gv(3), gv(4), gv(5)};
  Eigen::Vector3d wW_hip = R_world_hip * Eigen::Vector3d::UnitX() * gv(15);
  Eigen::Vector3d wW_thigh = R_world_thigh * Eigen::Vector3d::UnitY() * gv(16);
  Eigen::Vector3d wW_calf = R_world_calf * Eigen::Vector3d::UnitY() * gv(17);

  // position vector from each joint to ee w.r.t. world position
  Eigen::Vector3d wpos_root_ee = r_world_ee - r_world_root;
  Eigen::Vector3d wpos_hip_ee = r_world_ee - r_world_hip;
  Eigen::Vector3d wpos_thigh_ee = r_world_ee - r_world_thigh;
  Eigen::Vector3d wpos_calf_ee = r_world_ee - r_world_calf;

  // linear velocity
  Eigen::Vector3d lin_vel;

  // linear velocity of root
  lin_vel = Eigen::Vector3d{gv(0), gv(1), gv(2)} 
          + wW_root.cross(wpos_root_ee)
          + wW_hip.cross(wpos_hip_ee)
          + wW_thigh.cross(wpos_thigh_ee)
          + wW_calf.cross(wpos_calf_ee);
  
    return lin_vel;
}

/// do not change the name of the method
inline Eigen::Vector3d getFootAngularVelocity (const Eigen::VectorXd& gc, const Eigen::VectorXd& gv) {

  /// Local Coordinates
  //world_to_root
  Eigen::Matrix3d R_world_root;
  R_world_root = Eigen::Quaternion<double>(gc(3), gc(4), gc(5), gc(6)).toRotationMatrix();

  //root_to_hip
  Eigen::Matrix3d R_root_hip(Eigen::AngleAxisd(gc(16), Eigen::Vector3d::UnitX()));

  //hip_to_thigh
  Eigen::Matrix3d R_hip_thigh(Eigen::AngleAxisd(gc(17), Eigen::Vector3d::UnitY()));

  //thigh_to_calf
  Eigen::Matrix3d R_thigh_calf(Eigen::AngleAxisd(gc(18), Eigen::Vector3d::UnitY()));

  // w.r.t. World Coordinate loal orientations
  Eigen::Matrix3d R_world_hip = R_world_root * R_root_hip;
  Eigen::Matrix3d R_world_thigh = R_world_hip * R_hip_thigh;
  Eigen::Matrix3d R_world_calf = R_world_thigh * R_thigh_calf;

  // angular velocity of each joint w.r.t. world coordinate
  Eigen::Vector3d wW_hip = R_world_hip * Eigen::Vector3d::UnitX() * gv(15);
  Eigen::Vector3d wW_thigh = R_world_thigh * Eigen::Vector3d::UnitY() * gv(16);
  Eigen::Vector3d wW_calf = R_world_calf * Eigen::Vector3d::UnitY() * gv(17);
  
  // angular velocity
  Eigen::Vector3d ang_vel;
  ang_vel = Eigen::Vector3d{gv(3), gv(4), gv(5)} + wW_hip + wW_thigh + wW_calf;



    return ang_vel;
}