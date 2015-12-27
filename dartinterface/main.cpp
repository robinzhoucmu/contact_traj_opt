#include <string> 
#include "env-model.h"

#include "IpIpoptApplication.hpp"
#include "opt-problem.hpp" 
#include "dart-simulation.hpp"
#include <iostream>

int main(int argc, char* argv[])
{

  /*********************************************************
  ** env-model setup                                      **
  **********************************************************/
  string robot_urdf = "robot.urdf";
  string object_urdf = "object.urdf";

  dart::utils::DartLoader loader; 
  SkeletonPtr robot = loader.parseSkeleton(robot_urdf);
  SkeletonPtr object = loader.parseSkeleton(object_urdf); 

  WorldPtr world = std::make_shared<World>(); 
  world->setGravity(Eigen::Vector3d::Zero());
  world->addSkeleton(robot); 
  world->addSkeleton(object); 

  EnvModel env_model(world, robot, object);



  /*********************************************************
  ** Run optimization problem                             **
  **********************************************************/
  double timestep = 0.1; 
  int num_timestep = 20;

  Eigen::VectorXd t(num_timestep); 
  Eigen::MatrixXd obj_traj(object.getNumDofs()*2,num_timestep);
  Eigen::MatrixXd robot_traj(robot.getNumDofs()*2,num_timestep);

  SmartPtr<TNLP> mynlp = new OptProblem(env_model, timestep, num_timestep, t, obj_traj, robot_traj);
  // Create a new instance of IpoptApplication
  //  (use a SmartPtr, not raw)
  // We are using the factory, since this allows us to compile this
  // example with an Ipopt Windows DLL
  SmartPtr<IpoptApplication> app = IpoptApplicationFactory();
  app->RethrowNonIpoptException(true);

  // Change some options
  // Note: The following choices are only examples, they might not be
  //       suitable for your optimization problem.
  app->Options()->SetNumericValue("tol", 1e-7);
  app->Options()->SetStringValue("output_file", "ipopt.out");
  app->Options()->SetStringValue("linear_solver", "ma57");
  app->Options()->SetStringValue("hessian_approximation", "limited-memory");
  app->Options()->SetStringValue("derivative_test", "first-order");

  // Initialize the IpoptApplication and process the options
  ApplicationReturnStatus status;
  status = app->Initialize();
  if (status != Solve_Succeeded) {
    std::cout << std::endl << std::endl << "*** Error during initialization!" << std::endl;
    return (int) status;
  }

  // Ask Ipopt to solve the problem
  status = app->OptimizeTNLP(mynlp);

  if (status == Solve_Succeeded) {
    std::cout << std::endl << std::endl << "*** The problem solved!" << std::endl;
  }
  else {
    std::cout << std::endl << std::endl << "*** The problem FAILED!" << std::endl;
    return (int) status; 
  }


  /*********************************************************
  ** Simulate Optimized Trajectory                        **
  **********************************************************/

 
  MyWindow window(world, robot, object, robot_traj, object_traj);

  glutInit(&argc, argv);
  window.initWindow(640,480, "Optimization Result");
  glutMainLoop();

}