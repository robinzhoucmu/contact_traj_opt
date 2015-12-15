#include "env-model.h"
#include <assert.h>

void EnvModel::SetRobotQ(const Eigen::VectorXd& _positions) {
  mRobot->setPosition(_positions);
}

void EnvModel::SetObjectQ(const Eigen::VectorXd& _positions) {
  mObject->setPositions(_positions);
}

Eigen::VectorXd EnvModel::GetRobotQ() const {
  return mRobot->getPositions();
}

Eigen::VectorXd EnvModel::GetObjectQ() const {
  return mObject->getPositions();
}

dart::math::Jacobian EnvModel::GetObjectPointJacobian(Eigen::Vector3d _pt) {
  // Todo(Jiaji): Pass in body node as a parameter when the object is no longer
  // a single rigid body.
  return mObject->
    getWorldJacobian(mObject->getBodyNode(mObject->getNumBodyNodes() - 1), _pt);
}

void EnvModel::AddTwoBodiesContactInfo(dart::dynamics::BodyNode _br,
				       dart::dynamics::BodyNode _bo,
				       std::vector<ContactInfo3d>* _contactInfos) {
  // First compute the distance information.
  dart::collision::DistancePair = mDetector->computeDistancePair(_br, _bo);
  // Check if already in collision (distance <=0)
  // Todo (Jiaji): use dart's eps?
  if (DistancePair.distance <=0) {
    AddTwoBodiesContactInfoFromDetector(_br, _bo, _contactInfos);
  } else {  
    // Todo(Jiaji): This is a total hack...
    // When the two bodies are away from each other, we move the base of the robot
    // such that they are definitely in collision and subsequently get the contactInfo.
    
    // Pointing from the closet point on the robot to the closet point on the object.
    Eigen::Vector3d trans = DistancePair.pt_2 - DistancePair.pt_1;
    Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
    tf.translation() = trans;
    // Move the robot base.
    mRobot->getJoint(0)->setTransformFromParentBodyNode(tf);
    // Call collision detector now.
    AddTwoBodiesContactInfoFromDetector(_br, _bo, _contactInfos);
  }
}

void EnvModel::AddTwoBodiesContactInfoFromDetector(dart::dynamics::BodyNodePtr _br,
						   dart::dynamics::BodyNodePtr _bo,
						   std::vector<ContactInfo3d>* _contactInfos) {
    mDetector->detectCollision(_br, _bo);
    // Todo(Jiaji): only allow single contact between two bodies?
    int num_collisions = mDetector->getNumContacts();
    for (int i = 0; i < num_collisions; ++i ) {
      ContactInfo3d contact_i = GetSingleCollisionInfo(mDetector, i);
      _contactInfos->push_back(contact_i);
    }
}

ContactInfo3d EnvModel::GetSingleCollisionInfo(
    dart::collision::CollisionDetector* _detector, int _contactId) {
  // Check for contact id.
  assert(_contactId < _detector->getNumContacts());

  const dart::collision::Contact &contact_dart = _detector->getContact(_contactId);
  ContactInfo3d contact_res(contact_dart.bodyNode1, contact_dart.bodyNode2);

  // When in contact, the two points on each body are the same.
  contact_res.pt_1 = contact_dart.point;
  contact_res.pt_2 = contact_dart.point;

  // Get the normal.
  contact_res.normal = contact_dart.normal;

  // When in contact, assign the distance as penetration depth.
  contact_res.distance = contact_dart.penetrationDepth;
  return contact_res;
}
