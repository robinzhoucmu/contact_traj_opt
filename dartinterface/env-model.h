#include "dart/dart.h"
struct ContactInfo3d{
  // typedef Eigen::Matrix<double, 6, Eigen::Dynamic> Jacobian;
  dart::math::Jacobian Jc;
  Eigen::Vector3d pt_1;
  Eigen::Vector3d pt_2;
  // Pointing from body2 to body 1.
  Eigen::Vector3d normal;
  std::vector<Eigen::Vector3d> friction_basis;
}

// Set up the environment and create the initial robot,object.
class EnvModel {
 public:
  EnvModel(){};
  
  // Configuration set() and get().
  void SetRobotQ(const Eigen::VectorXd &_positions);
  void SetObjectQ(const Eigen::VectorXd &_positions);
  Eigen::VectorXd GetRobotQ() const;
  Eigen::VectorXd GetObjectQ() const;
  
  // Extract Jacobian matrix 

  // Extract distance between two object and robot.
  
  
 private:
  dart::dynamics::SkeletonPtr mRobot;
  dart::dynamics::SkeletonPtr mObject; 
  
};

