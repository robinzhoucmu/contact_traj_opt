#include "env-model.h"
// Minimum interface between outside trajectory optimizer and dart environment. 
// Think of this interface provides only the mathematical quantities, being agnostic to the fact that it's optimizing through contact. All the distance computation logic, contact information/jacobian should be hide away from the optimizer. 

class DartInterface {
 public:
  DartInterface(){};
  void SetRobotQ(const Eigen::VectorXd &_positions);
  void SetObjectQ(const Eigen::VectorXd &_positions);
  
  // Interface to equation of motion parameters.
  Eigen::MatrixXd& GetObjectMass() const;
  Eigen::VectorXd& GetCoriolisAndGravityForce() const;

  // Todo(Jiaji): Merge into one Jacobian matrix??
  std::vector<dart::math::Jacobian> GetAllJacobians() const;
  
  std::vector<double> GetAllContactDistances() const;
  
  // Asking mEnvModel to extract contact informations.
  void ComputeContact();

 private:
  EnvModel mEnvModel;
  
};
