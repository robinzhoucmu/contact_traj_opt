#include "dart-interface.h"
DartInterface::DartInterface(dart::dynamics::SkeletonPtr _robot, 
			     dart::dynamics::SkeletonPtr _object) {
  mEnvModel = std::make_shared<EnvModel> (_robot, _object);
}

DartInterface::DartInterface(dart::dynamics::SkeletonPtr _robot, 
			     dart::dynamics::SkeletonPtr _object,	   
			     std::vector< dart::dynamics::SkeletonPtr > _extContacts) {
  mEnvModel = std::make_shared<EnvModel> (_robot, _object, _extContacts);
}

const Eigen::MatrixXd& DartInterface::GetObjectMass() const {
  return mEnvModel->GetObjectMass();
}

const Eigen::VectorXd& DartInterface::GetCoriolisAndGravityForce() const {
  return mEnvModel->GetCoriolisAndGravityForce();
}

void DartInterface::SetRobotQ(const Eigen::VectorXd &_positions) {
  mEnvModel->SetRobotQ(_positions);
}

void DartInterface::SetObjectQ(const Eigen::VectorXd &_positions) {
  mEnvModel->SetObjectQ(_positions);
}

std::vector< Eigen::Matrix<double, 6, Eigen::Dynamic> > DartInterface::GetAllJacobians() const {
  std::vector<dart::math::Jacobian> Jcs;
  const std::vector<ContactInfo3d>& contact_infos = mEnvModel->ContactInfos();
  for (int i = 0; i < contact_infos.size(); ++i) {
    Jcs.push_back(contact_infos[i].Jc);
  }
  return Jcs;
}

std::vector<double> DartInterface::GetAllContactDistances() const {
  std::vector<double> distances;
  const std::vector<ContactInfo3d>& contact_infos = mEnvModel->ContactInfos();
  for (int i = 0; i < contact_infos.size(); ++i) {
    distances.push_back(contact_infos[i].distance);
  }
  return distances;
}

std::vector<Eigen::Vector3d> DartInterface::GetAllNormals() const {
  std::vector<Eigen::Vector3d> normals;
  const std::vector<ContactInfo3d>& contact_infos = mEnvModel->ContactInfos();
  for (int i = 0; i < contact_infos.size(); ++i) {
    normals.push_back(contact_infos[i].normal);
  }
  return normals;
}

std::vector< std::vector<Eigen::Vector3d> > DartInterface::GetAllFrictionBasis() const {
  std::vector< std::vector<Eigen::Vector3d> > frictionBasis;
  const std::vector<ContactInfo3d>& contact_infos = mEnvModel->ContactInfos();
  for (int i = 0; i < contact_infos.size(); ++i) {
    frictionBasis.push_back(contact_infos[i].friction_basis);
  }
  return frictionBasis;
}

void DartInterface::ComputeContact() {
  mEnvModel->ExtractObjectRobotContactPairs();
  mEnvModel->ExtractObjectEnvironmentContactPairs();
}



