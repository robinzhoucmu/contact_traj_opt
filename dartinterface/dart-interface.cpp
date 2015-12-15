#include "dart-interface.h"

Eigen::MatrixXd& DartInterface::GetObjectMass() const {
  return mEnvModel->GetObjectMass();
}

Eigen::VectorXd& DartInterface::GetCoriolisAndGravityForce() const {
  return mEnvModel->GetCoriolisAndGravityForce();
}

void DartInterface::SetRobotQ(const Eigen::VectorXd &_positions) {
  mEnvModel->SetRobotQ(_positions);
}

void DartInterface::SetObjectQ(const Eigen::VectorXd &_positions) {
  mEnvModel->SetObjectQ(_positions);
}

std::vector<dart::math::Jacobian> DartInterface::GetAllJacobians() const {
  std::vector<dart::math::Jacobian> Jcs;
  const std::vector<ContactInfo3d>& contact_infos = mEnvModel->ContactInfos();
  for (int i = 0; i < contact_infos.size(); ++i) {
    Jcs.push_back(contact_infos.Jc);
  }
  return Jcs;
}

std::vector<double> DartInterface::GetAllContactDistances() const {
  std::vector<double> distances;
  const std::vector<ContactInfo3d>& contact_infos = mEnvModel->ContactInfos();
  for (int i = 0; i < contact_infos.size(); ++i) {
    distances.push_back(contact_infos.distance);
  }
  return distances;
}

void DartInterface::ComputeContact() {
  mEnvModel->ExtractObjectRobotContactPairs();
  mEnvModel->ExtractObjectEnviromentContactPairs();
}

