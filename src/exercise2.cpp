//
// Created by Jemin Hwangbo on 2022/03/17.
//


#define _MAKE_STR(x) __MAKE_STR(x)
#define __MAKE_STR(x) #x

#include "raisim/RaisimServer.hpp"
#include "exercise1_20223353.hpp"
#include "exercise2_20223353.hpp"

int main(int argc, char* argv[]) {
  auto binaryPath = raisim::Path::setFromArgv(argv[0]);

  // create raisim world
  raisim::World world; // physics world
  raisim::RaisimServer server(&world); // visualization server
  world.addGround();
  world.setTimeStep(0.0001);

  // a1
  auto a1 = world.addArticulatedSystem(std::string(_MAKE_STR(RESOURCE_DIR)) + "/a1/urdf/a1.urdf");
  a1->setName("a1");
  server.focusOn(a1);

  // a1 configuration
  Eigen::VectorXd gc(a1->getGeneralizedCoordinateDim());
  Eigen::VectorXd gv(a1->getDOF());

  //(r_x, r_y, r_z / 
  // q_w, q_x, q_y, q_z / 
  // FR_hip, FR_thigh, FR_calf / 
  // FL_hip, FL_thigh, FL_calf / 
  // RR_hip, RR_thigh, RR_calf / 
  // RL_hip, RL_thigh, RL_calf)
  gc << 0, 0, 10.54, 1.0, 0.0, 0.0, 0.0, 0.03, 0.4, -0.8, -0.03, 0.4, -0.8, 0.03, -0.4, 0.8, -0.03, -0.4, 0.8; //19개
  
  /////////////////////////////////
  // wVb_x, wVb_y, wVb_z 0 1 2
  // wWb_x, wWb_y, wWb_z 3 4 5
  // FR_hip, FR_thigh, FR_calf / // 6 7 8 theta_dot
  // FL_hip, FL_thigh, FL_calf / // 9 10 11 theta_dot
  // RR_hip, RR_thigh, RR_calf / // 12 13 14 theta_dot
  // RL_hip, RL_thigh, RL_calf)  // 15 16 17 theta_dot
  gv << 0.1, 0.2, 0.3, 0.1, 0.4, 0.3, 0.1, 0.1, 0.1, 0.2, 0.2, 0.2, 0.3, 0.3, 0.3, 0.4, 0.4, 0.4; //18개
  a1->setState(gc, gv);

  // visualization
  server.launchServer();
  raisim::Vec<3> footVel, footAngVel;

  raisim::Vec<3> root, temp_vel;
  bool answerCorrect = true;

  // hip: (1, 0, 0)
  // thigh: (0, 0.99955, -0.0299955)

  // debug sphere
  auto debug_Sphere = server.addVisualSphere("debug_sphere", 0.05);

  for (int i=0; i<20000; i++) {
    std::this_thread::sleep_for(std::chrono::microseconds(1000));
    world.integrate1();

    a1->getFrameVelocity("RL_foot_fixed", footVel);
    a1->getFrameAngularVelocity("RL_foot_fixed", footAngVel);

    if((footVel.e() - getFootLinearVelocity(gc, gv)).norm() < 1e-8) {
      std::cout<<"the linear velocity is correct "<<std::endl;
    } else {
      std::cout<<"the linear velocity is not correct "<<std::endl;
      answerCorrect = false;
    }

    if((footAngVel.e() - getFootAngularVelocity(gc, gv)).norm() < 1e-8) {
      std::cout<<"the angular velocity is correct "<<std::endl;
    } else {
      std::cout<<"the angular velocity is not correct "<<std::endl;
      answerCorrect = false;
    }
    world.integrate2();
    a1->getState(gc, gv);


    debug_Sphere->setColor(1,0,0,1);
    debug_Sphere->setPosition(getEndEffectorPosition(gc));
  }

  server.killServer();

  if(answerCorrect) {
    std::cout<<"The solution is correct "<<std::endl;
  } else {
    std::cout<<"The solution is not correct "<<std::endl;
    answerCorrect = false;
  }
/////////////////////////////////

  return 0;
}
