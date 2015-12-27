#ifndef __OPTPROBLEM_HPP__
#define __OPTPROBLEM_HPP__

#include "IpTNLP.hpp"
#include <Eigen/Dense>
#include <vector>
#include <tuple>
#include "env-model.h"

using namespace Ipopt;

/**
Robot is velocity-controlled.
**/
class OptProblem : public TNLP
{
public:
  /** in - envModel, timestep,  numTimesteps,
  out - t, obj_traj, robot_traj **/
  OptProblem(EnvModel *envModel, double timestep, int numTimesteps, 
             Eigen::VectorXd &t, Eigen::MatrixXd &obj_traj, Eigen::MatrixXd &robot_traj);

  /** default destructor */
  virtual ~OptProblem();

  /**@name Overloaded from TNLP */
  //@{
  /** Method to return some info about the nlp */
  virtual bool get_nlp_info(Index& n, Index& m, Index& nnz_jac_g,
                            Index& nnz_h_lag, IndexStyleEnum& index_style);

  /** Method to return the bounds for my problem */
  virtual bool get_bounds_info(Index n, Number* x_l, Number* x_u,
                               Index m, Number* g_l, Number* g_u);

  /** Method to return the starting point for the algorithm */
  virtual bool get_starting_point(Index n, bool init_x, Number* x,
                                  bool init_z, Number* z_L, Number* z_U,
                                  Index m, bool init_lambda,
                                  Number* lambda);

  /** Method to return the objective value */
  virtual bool eval_f(Index n, const Number* x, bool new_x, Number& obj_value);

  /** Method to return the gradient of the objective */
  virtual bool eval_grad_f(Index n, const Number* x, bool new_x, Number* grad_f);

  /** Method to return the constraint residuals */
  virtual bool eval_g(Index n, const Number* x, bool new_x, Index m, Number* g);

  /** Method to return:
   *   1) The structure of the jacobian (if "values" is NULL)
   *   2) The values of the jacobian (if "values" is not NULL)
   */
  virtual bool eval_jac_g(Index n, const Number* x, bool new_x,
                          Index m, Index nele_jac, Index* iRow, Index *jCol,
                          Number* values);

  /** Method to return:
   *   1) The structure of the hessian of the lagrangian (if "values" is NULL)
   *   2) The values of the hessian of the lagrangian (if "values" is not NULL)
   */
  virtual bool eval_h(Index n, const Number* x, bool new_x,
                      Number obj_factor, Index m, const Number* lambda,
                      bool new_lambda, Index nele_hess, Index* iRow,
                      Index* jCol, Number* values);

  //@}

  /** @name Solution Methods */
  //@{
  /** This method is called when the algorithm is complete so the TNLP can store/write the solution */
  virtual void finalize_solution(SolverReturn status,
                                 Index n, const Number* x, const Number* z_L, const Number* z_U,
                                 Index m, const Number* g, const Number* lambda,
                                 Number obj_value,
				 const IpoptData* ip_data,
				 IpoptCalculatedQuantities* ip_cq);
  //@}

private:
  /**@name Methods to block default compiler methods.
   * The compiler automatically generates the following three methods.
   *  Since the default compiler implementation is generally not what
   *  you want (for all but the most simple classes), we usually 
   *  put the declarations of these methods in the private section
   *  and never implement them. This prevents the compiler from
   *  implementing an incorrect "default" behavior without us
   *  knowing. (See Scott Meyers book, "Effective C++")
   *  
   */
  //@{

  OptProblem(const OptProblem&);
  OptProblem& operator=(const OptProblem&);

  EnvModel *env_model;

  
  Eigen::MatrixXd objectMass;
  Eigen::MatrixXd coriolis_and_gravityForce;
  Eigen::Vector2d initObjState, initRobotState; 
  Eigen::Vector2d goalObjState, goalRobotState;

  int numTimesteps; 
  double timestep; 
  int numVarsPerStep;

  int numRobotDOF, numObjectDOF, numContacts;
  int numTotalConstr, numTotalVar;
  int numDistanceConstr, numLCPConstr, numDynamicConstr;
  int numInitialConstr, numFinalConstr;

  Eigen::MatrixXd Jacobian; 
  vector<tuple<int, int>> indexToJacobian;

  void getObjState(const Number* x, Eigen::MatrixXd& objState);  // (2objectDOF)*timesteps 
  void getRobotState(const Number* x, Eigen::MatrixXd& robotState); // (2robotDOF)*timesteps
  void getContactForce(const Number* x, Eigen::MatrixXd& lambda); // numContacts *timesteps 

  void initializeObjectState(Number *x);
  void initializeRobotState(Number *x); 
  void initializeControl(Number *x); 
  void initializeContactForce(Number *x);
  
  int eval_distance(const Number*x, Number*g, int index); 
  int eval_lcp(const Number*x, Number*g, int index); 
  int eval_objstate(const Number*x, Number*g, int index); 
  int eval_robotstate(const Number*x, Number*g, int index); 
  
  int eval_init_obj_state(const Number*x, Number*g, int index); 
  int eval_final_obj_state(const Number*x, Number*g, int index); 
  int eval_init_robot_state(const Number*x, Number*g, int index); 
  int eval_final_robot_state(const Number*x, Number*g, int index);

  // set lower and upper bound on variables 
  int setObjectBound(Number* x_l, Number* x_u, int index);
  int setRobotBound(Number* x_l, Number* x_u, int index); 
  int setContactForceBound(Number* x_l, Number *x_u, int index); 

  // set lower and upper bounds on constraints 
  int setLCPBound(Number* g_l, Number* g_u, int index); 
  int setDistanceBound(Number* g_l, Number* g_u, int index);
  int setDynamicsEqualityBound(Number* g_l, Number* g_u, int index);


  //@}
};


#endif
