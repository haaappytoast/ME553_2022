//
// Created by Jemin Hwangbo on 2022/03/17.
//

#ifndef ME553_2022_SOLUTIONS_EXERCISE1_H_
#define ME553_2022_SOLUTIONS_EXERCISE1_H_

#include <Eigen/Core>
#include <iostream>
#include <Eigen/Geometry> 

/// do not change the name of the method
inline Eigen::Vector3d getEndEffectorPosition (const Eigen::VectorXd& gc) {

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

  Eigen::Vector3d r_world_ee;
  
  Eigen::Vector3d r_thigh_ee = r_thigh_calf + R_thigh_calf * (r_calf_ee);
  Eigen::Vector3d r_hip_ee = r_hip_thigh + R_hip_thigh * (r_thigh_ee);
  Eigen::Vector3d r_root_ee = r_root_hip + R_root_hip * r_hip_ee;
  r_world_ee = r_world_root + R_world_root * r_root_ee;
  
  /* just added for debugging
  /// Forward Kinematics
  // world to hip
  Eigen::Vector3d r_world_hip = r_world_root + R_world_root * r_root_hip;

  // world to thigh
  Eigen::Vector3d r_world_thigh = r_world_root + R_world_root * (r_root_hip + (R_root_hip * r_hip_thigh));

  // world to calf
  Eigen::Vector3d r_world_calf = r_world_root + R_world_root * (r_root_hip + (R_root_hip * (r_hip_thigh + R_hip_thigh * r_thigh_calf)));

  // world to ee
  Eigen::Vector3d r_world_ee = r_world_root + R_world_root * (r_root_hip + R_root_hip * (r_hip_thigh + R_hip_thigh * ((r_thigh_calf + R_thigh_calf * (r_calf_ee)))));
  */
 
  return r_world_ee;
}

#endif //ME553_2022_SOLUTIONS_EXERCISE1_H_
