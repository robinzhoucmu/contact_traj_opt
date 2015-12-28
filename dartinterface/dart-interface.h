#include "env-model.h"
// Minimum interface between outside trajectory optimizer and dart environment. 
// Think of this interface provides only the mathematical quantities, being agnostic to the fact that it's optimizing through contact. All the distance computation logic, contact information/jacobian should be hidden away from the optimizer. 

class DartInterface {
 public:
  // [GL] should the constructor take in world, robot, and obj? or take in env_model that has all the info? 
  DartInterface(){};
  void SetRobotQ(const Eigen::VectorXd &_positions);
  void SetObjectQ(const Eigen::VectorXd &_positions);
  
  // Interface to equation of motion parameters.
  const Eigen::MatrixXd& GetObjectMass() const;
  const Eigen::VectorXd& GetCoriolisAndGravityForce() const;

  // Todo(Jiaji): Merge into one Jacobian matrix??
  // [GL] With K contacts, the vector has K elements 
  // and each element is a (mxn) matrix where m = q (generalized coordinate), n = 6. 
  // Transforms K world contact wrenches to generalized force. 
  // *q = objectDOF + robotDOF
  // or can we separate q and have GetAllRobotJacobians() and GetAllObjectJacobians()?
  std::vector< Eigen::Matrix<double, 6, Eigen::Dynamic> > GetAllJacobians() const;
  
  std::vector<double> GetAllContactDistances() const;

  //  [GL] In world coordinate, right?
  std::vector< Eigen::Vector3d > GetAllNormals() const;
  
  // [GL] a vector of K (number of contacts) elements where 
  // each element is a vector of d(number of basis) elements  
  std::vector< std::vector<Eigen::Vector3d> > GetAllFrictionBasis() const; 
  
  // Asking mEnvModel to extract contact informations.
  void ComputeContact();

  const int obj_dof; 
  const int robot_dof;
  const int num_contacts;

 private:
  std::shared_ptr<EnvModel> mEnvModel;
  
};
