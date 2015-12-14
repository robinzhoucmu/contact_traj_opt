#include "env-model.h"
// Minimum interface between outside trajectory optimizer and dart environment. 
// Think of this interface provides only the mathematical quantities, being agnostic to the fact that it's optimizing through contact. All the distance computation logic, contact information/jacobian should be hide away from the optimizer. 

class DartInterface {
 public:
  DartInterface(){};
  void SetUpRobot();
  void SetUpObject();
  
  // Interface to equation of motion parameters.
  Eigen::MatrixXd& GetObjectMass();
  Eigen::VectorXd& GetCoriolisAndGravityForce();
 private:
  EnvModel mEnvModel;
  
};
