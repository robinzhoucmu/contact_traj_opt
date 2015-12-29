#ifndef DART_INTERFACE_H
#define DART_INTERFACE_H

#include "env-model.h"
// Minimum interface between outside trajectory optimizer and dart environment. 
// Think of this interface provides only the mathematical quantities, being agnostic to the fact that it's optimizing through contact. All the distance computation logic, contact information/jacobian should be hidden away from the optimizer. 

class DartInterface {
 public:
  DartInterface(dart::dynamics::SkeletonPtr _robot, 
		dart::dynamics::SkeletonPtr _object, 
		std::vector<bool> _dim_selector = std::vector<bool>(6, true));

  DartInterface(dart::dynamics::SkeletonPtr _robot, 
		dart::dynamics::SkeletonPtr _object,	   
		std::vector< dart::dynamics::SkeletonPtr > _extContacts,
		std::vector<bool> _dim_selector = std::vector<bool>(6, true));
  DartInterface(){};
  
  bool SetRelatedDimension(std::vector<bool> _dim_selector);
  
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
  // [Jiaji]: should be. 
  std::vector< Eigen::Vector3d > GetAllNormals() const;
  
  // Form the normal wrenches for the unit normal forces.
  // By multiplying the Linear Jacobian transpose and select the related indices.
  std::vector< Eigen::VectorXd > GetAllNormalWrenches() const;

  // [GL] a vector of K (number of contacts) elements where 
  // each element is a vector of d(number of basis) elements  
  std::vector< std::vector<Eigen::Vector3d> > GetAllFrictionBasis() const; 
  
  // Form the friction wrenches. 
  // By multiplying the Linear Jacobian transpose and select the related indices.
  std::vector< std::vector<Eigen::VectorXd> > GetAllFrictionWrenches() const;
 
  int obj_dof() {return mobj_dof;}
  int robot_dof() {return mrobot_dof;}
  int num_contacts() {return mnum_contacts;}
  // Asking mEnvModel to extract contact informations.
  void ComputeContact();

 private:
  std::shared_ptr<EnvModel> mEnvModel;
  std::vector<int> mRelatedDims;
  int mobj_dof; 
  int mrobot_dof;
  int mnum_contacts;
};

#endif
