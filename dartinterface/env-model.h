#include "dart/dart.h"
struct ContactInfo3d{
  // First body node (robot body node)
  dart::dynamics::WeakBodyNodePtr bodyNode1;

  // Second body node (object body node)
  dart::dynamics::WeakBodyNodePtr bodyNode2;  

  // typedef Eigen::Matrix<double, 6, Eigen::Dynamic> Jacobian;
  dart::math::Jacobian Jc;
  
  // Closet point of body 1 to body 2.
  Eigen::Vector3d pt_1;
  
  // Closet point of body 2 to body 1. 
  // When in contact, pt_1 and pt_2 should be the same.
  Eigen::Vector3d pt_2;
  
  // Distance (away or penetrated)
  double distance;

  // Pointing from body2 to body 1.
  Eigen::Vector3d normal;
  
  // Friciton basis in the tangent plane.
  std::vector<Eigen::Vector3d> friction_basis;
  
  // Initialize with 2 bodies of interest.
  ContactInfo3d(dart::dynamics::WeakBodyNodePtr _bd1, dart::dynamics::WeakBodyNodePtr _bd2) {
    bodyNode1 = _bd1;
    bodyNode2 = _bd2;
  }
  void GenerateFrictionBasis(int _nBasis) {
  
  }
};

// Set up the environment and create the initial robot,object.
// For now, we are assuming there is only one rigid body object. The robot
// could be a multi-link robot. 
class EnvModel {
 public:
  EnvModel(){};
  
  // Configuration set() and get().
  void SetRobotQ(const Eigen::VectorXd &_positions);
  void SetObjectQ(const Eigen::VectorXd &_positions);
  Eigen::VectorXd GetRobotQ() const;
  Eigen::VectorXd GetObjectQ() const;
  
  // Extract Jacobian matrix (world frame) of a point 
  // on the object.
  dart::math::Jacobian GetObjectPointJacobian(Eigen::Vector3d _pt);
  
  // Extract all pairs of body node contact information between robot and object.
  void ExtractObjectRobotContactPairs();
  
  // Extract all pairs of body node contact information between object and 
  // external environment contacts.
  void ExtractObjectEnvironmentContactPairs();

 private:

  // Add ContactInfo among a body of the robot (_br) and the object body (_bo).
  void AddTwoBodiesContactInfo(dart::dynamics::BodyNodePtr _br,
			       dart::dynamics::BodyNodePtr _bo,
			       std::vector<ContactInfo3d>* _contactInfos);
 
  void AddTwoBodiesContactInfoFromDetector(dart::dynamics::BodyNodePtr _br,
					   dart::dynamics::BodyNodePtr _bo,
					   std::vector<ContactInfo3d>* _contactInfos);

  ContactInfo3d GetSingleCollisionInfo(dart::collision::CollisionDetector* _detector, int _contactId);

  dart::dynamics::SkeletonPtr mRobot;
  dart::dynamics::SkeletonPtr mObject; 
  std::vector<ContactInfo3d> mContactInfos;
  dart::collision::CollisionDetector* mDetector;
};

