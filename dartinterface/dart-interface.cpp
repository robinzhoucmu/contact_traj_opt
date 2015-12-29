#include "dart-interface.h"
DartInterface::DartInterface(dart::dynamics::SkeletonPtr _robot, 
			     dart::dynamics::SkeletonPtr _object,
			     std::vector<bool> _dim_selector) {
  mEnvModel = std::make_shared<EnvModel> (_robot, _object);
}

DartInterface::DartInterface(dart::dynamics::SkeletonPtr _robot, 
			     dart::dynamics::SkeletonPtr _object,	   
			     std::vector< dart::dynamics::SkeletonPtr > _extContacts,
			     std::vector<bool> _dim_selector) {
  mEnvModel = std::make_shared<EnvModel> (_robot, _object, _extContacts);
}

bool DartInterface::SetRelatedDimension(std::vector<bool> _dim_selector) {
  // Todo(Jiaji): For now, we are doing this check for single object rigid body.
  if (_dim_selector.size() == 6) {
    mRelatedDims.clear();
    for (int i = 0; i < _dim_selector.size(); ++i) {
      if (_dim_selector[i]) {
	mRelatedDims.push_back(i);
      }
    }
    return true;
  } else {
    return false;
  }
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

std::vector< Eigen::VectorXd > DartInterface::GetAllNormalWrenches() const {
  std::vector<Eigen::VectorXd> wrenches;
  const std::vector<ContactInfo3d>& contact_infos = mEnvModel->ContactInfos();
  for (int i = 0; i < contact_infos.size(); ++i) {
    // Get Linear Jacobian (first 3 rows) at the contact point (transpose) and then multiply with normal vector.
    Eigen::VectorXd w = 
      ((contact_infos[i].Jc).block(0,0,3,contact_infos[i].Jc.cols())).transpose() * contact_infos[i].normal;
    Eigen::VectorXd w_related(mRelatedDims.size());
    for (int j = 0; j < mRelatedDims.size(); ++j) {
      w_related(j) = w(mRelatedDims[j]);
    }
    wrenches.push_back(w_related);
  }
  return wrenches;
}

std::vector< std::vector<Eigen::Vector3d> > DartInterface::GetAllFrictionBasis() const {
  std::vector< std::vector<Eigen::Vector3d> > frictionBasis;
  const std::vector<ContactInfo3d>& contact_infos = mEnvModel->ContactInfos();
  for (int i = 0; i < contact_infos.size(); ++i) {
    frictionBasis.push_back(contact_infos[i].friction_basis);
  }
  return frictionBasis;
}

// Form the friction wrenches. 
// By multiplying the Linear Jacobian transpose and select the related indices.
std::vector< std::vector<Eigen::VectorXd> > DartInterface::GetAllFrictionWrenches() const {
  std::vector<std::vector<Eigen::VectorXd>> wrench_groups;
  const std::vector<ContactInfo3d>& contact_infos = mEnvModel->ContactInfos();
  for (int i = 0; i < contact_infos.size(); ++i) {
    std::vector<Eigen::VectorXd> wrenches;
    for (int k = 0; k < contact_infos[i].friction_basis.size(); ++k) {
      // Get Linear Jacobian (first 3 rows) at the contact point (transpose) and then multiply with normal vector.
      Eigen::VectorXd w = 
	((contact_infos[i].Jc).block(0,0,3,contact_infos[i].Jc.cols())).transpose() 
	* contact_infos[i].friction_basis[k];

      // Only get the related dimensions.
      Eigen::VectorXd w_related(mRelatedDims.size());
      for (int j = 0; j < mRelatedDims.size(); ++j) {
	w_related(j) = w(mRelatedDims[j]);
      }
      wrenches.push_back(w_related);
    }
    // Add all friction wrench for this contact.
    wrench_groups.push_back(wrenches);
  }
  return wrench_groups;
}
 

void DartInterface::ComputeContact() {
  mEnvModel->ExtractObjectRobotContactPairs();
  mEnvModel->ExtractObjectEnvironmentContactPairs();
}



