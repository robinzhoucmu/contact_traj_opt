// Copyright (C) 2005, 2006 International Business Machines and others.
// All Rights Reserved.
// This code is published under the Eclipse Public License.
//
// $Id: OptProblem.cpp 2005 2011-06-06 12:55:16Z stefan $
//
// Authors:  Carl Laird, Andreas Waechter     IBM    2005-08-16

#include "opt-problem.hpp"

#include <cassert>
#include <iostream>
#include <stdio.h> 
#include <stdlib.h>
#include <math.h>
#include <fstream>

using namespace Ipopt;
using namespace std;


/**
f = min (qo[T]-Goal)'(qo[T]-Goal)

subject to 
Mqa = J'*contactForce 
distance>=0
contactForce*distance = 0 

// robot is velocity controlled
dqr = u 
**/

OptProblem::OptProblem(EnvModel *envModel, double timestep, int numTimesteps)
{
  env_model = envModel;
  objectMass = envModel->GetObjectMass();
  this->timestep = timestep;
  this->numTimesteps = numTimesteps;

  numRobotDOF = envModel->GetRobotQ().rows(); 
  numObjectDOF = envModel->GetObjectQ().rows(); 
  numContacts = envModel->ContactInfos().size(); 

  numDistanceConstr = numTimesteps*numContacts;
  numLCPConstr = numTimesteps*numContacts;
  numDynamicConstr = (numTimesteps-1)*(numRobotDOF+numObjectDOF*2); 

  numVarsPerStep = numRobotDOF*2 + numObjectDOF*2 + numContacts;
  numTotalConstr = numDistanceConstr + numLCPConstr + numDynamicConstr;
  numTotalVar = numVarsPerStep*numTimesteps;
}

//destructor
OptProblem::~OptProblem(){}

// returns the size of the problem
bool OptProblem::get_nlp_info(Index& n, Index& m, Index& nnz_jac_g,
                             Index& nnz_h_lag, IndexStyleEnum& index_style)
{
  // number of variables  = state_variables * numTimesteps
  n = numTotalVar;

  // number of constraints
  m = numTotalConstr;

  nnz_jac_g = numTotalConstr*numTotalVar;
  nnz_h_lag = numTotalVar*numTotalVar;

  // use the C style indexing (0-based)
  index_style = TNLP::C_STYLE;

  cout << "number of variables:  " << n << endl; 
  cout << "number of constraints: " << m << endl; 

  return true;
}

// returns the variable bounds
bool OptProblem::get_bounds_info(Index n, Number* x_l, Number* x_u,
                                Index m, Number* g_l, Number* g_u)
{

  cout << "Set bounds" << endl;
  // here, the n and m we gave IPOPT in get_nlp_info are passed back to us.
  // If desired, we could assert to make sure they are what we think they are.
  assert(n == numTotalVar);
  assert(m == numTotalConstr);

  // set lower and upper bound on variables 
  setObjectBound(x_l,x_u);
  setRobotBound(x_l, x_u); 
  setControlBound(x_l, x_u); 
  setContactForceBound(x_l, x_u); 

  // set lower and upper bounds on constraints 
  setLCPBound(g_l, g_u); 
  setDistanceBound(g_l, g_u);
  setDynamicsEqualityBound(g_l, g_u);

  return true;
}

// returns the initial point for the problem
bool OptProblem::get_starting_point(Index n, bool init_x, Number* x,
                                   bool init_z, Number* z_L, Number* z_U,
                                   Index m, bool init_lambda,
                                   Number* lambda)
{
  cout << "initialize" << endl;

  // Here, we assume we only have starting values for x, if you code
  // your own NLP, you can provide starting values for the dual variables
  // if you wish
  assert(init_x == true);
  assert(init_z == false);
  assert(init_lambda == false);

  // initialize to the given starting point
  initializeObjectState(x);
  initializeRobotState(x); 
  initializeControl(x);
  initializeContactForce(x);

  cout << "initialization complete" << endl;
  return true;
}

// returns the value of the objective function
bool OptProblem::eval_f(Index n, const Number* x, bool new_x, Number& obj_value)
{
  assert(n == numTotalVar);

  Eigen::MatrixXd objState(numObjectDOF*2, numTimesteps);
  getObjState(x, objState);
  Eigen::VectorXd endObjState = objState.rightCols(1);
  obj_value = pow(endObjState(0) - 7,2);

  return true;
}

// return the gradient of the objective function grad_{x} f(x)
bool OptProblem::eval_grad_f(Index n, const Number* x, bool new_x, Number* grad_f)
{
  assert(n == numTotalVar);

  Eigen::MatrixXd objState(numObjectDOF*2, numTimesteps);
  getObjState(x, objState);
  Eigen::VectorXd endObjState = objState.rightCols(1); 

  for(int i=0;i<numTotalVar;i++){
    grad_f[i] = 0;
  }
  grad_f[numTimesteps-1] = 2*(endObjState-7);

  return true;
}

// return the value of the constraints: g(x)
bool OptProblem::eval_g(Index n, const Number* x, bool new_x, Index m, Number* g)
{
  assert(n == numTotalVar);
  assert(m == numTotalConstr);

  int index = 0; 
  index = eval_distance(x, g, index);
  index = eval_lcp(x, g, index); 
  index = eval_objstate(x, g, index);
  index = eval_robotstate(x,g, index); 

  index = eval_init_obj_state(x,g, index); 
  index = eval_init_robot_state(x,g, index); 
  index = eval_final_obj_state(x,g, index); 
  index = eval_final_robot_state(x,g, index);

  return true;
}

int OptProblem::eval_distance(const Number*x, Number*g, int index){
  Eigen::MatrixXd objState(numObjectDOF*2, numTimesteps);
  Eigen::MatrixXd robotState(2*numRobotDOF, numTimesteps); 
  getObjState(x, objState);
  getRobotState(x, robotState);

  for(int i=0;i<numTimesteps;i++){
    // Get robot and object's pose @ timestep i
    Eigen::VectorXd objAtI = objState(i); 
    Eigen::VectorXd robotAtI = robotState(i); 

    // Set envModel's pose to match above pose 
    envModel->SetObjectQ(objAtI); 
    envModel->SetRobotQ(robotAtI); 
    
    // Get contact info at current pose 
    vector<ContactInfo3d> contactInfos = envModel->ContactInfos();
    for(int j=0;j<contactInfos.size();j++){
      g[index++] = contactInfos[j].distance;
    }
  }

  return index; 

}

int OptProblem::eval_lcp(const Number*x, Number*g, int index){ 
  Eigen::MatrixXd objState(numObjectDOF*2, numTimesteps);
  Eigen::MatrixXd robotState(2*numRobotDOF, numTimesteps); 
  getObjState(x, objState);
  getRobotState(x, robotState);

  Eigen::MatrixXd contactForce(numContacts, numTimesteps); 
  getContactForce(x, contactForce); 

  for(int i=0;i<numTimesteps;i++){
    // Get robot and object's pose @ timestep i
    Eigen::VectorXd objAtI = objState(i); 
    Eigen::VectorXd robotAtI = robotState(i); 

    // Set envModel's pose to match above pose 
    env_model->SetObjectQ(objAtI); 
    env_model->SetRobotQ(robotAtI); 
    
    // Get contact info at current pose 
    vector<ContactInfo3d> contactInfos = env_model->ContactInfos();
    assert(contactInfos.size() == numContacts);

    for(int j=0;j<numContacts;j++){
      double distance = contactInfos[j].distance;
      g[index++] = contactForce(j,i)*distance;
    }
  }

  return index;
}

int OptProblem::eval_objstate(const Number*x, Number*g, int index){

  // qo[t+1] = qo[t]+dqo[t+1]*h
  // mO*(dq[t+1]-dq[t]) = h*contactForce
  int ts = numTimesteps;
  Eigen::MatrixXd objState(numObjectDOF*2, ts);
  Eigen::MatrixXd contactForce(numContacts, ts); 
  getObjState(x, objState);
  getContactForce(x, contactForce); 


  Eigen::MatrixXd q = objState.topRows(numObjectDOF); 
  Eigen::MatrixXd v = objState.bottomRows(numObjectDOF); 


  Eigen::MatrixXd generalized_contact_force = Eigen::MatrixXd::Zeros(numObjectDOF,ts); 

  for(int i=0;i<ts;i++){

    // Get robot and object's pose @ timestep i
    Eigen::VectorXd objAtI = objState(i); 
    Eigen::VectorXd robotAtI = robotState(i); 

    // Set envModel's pose to match above pose 
    env_model->SetObjectQ(objAtI); 
    env_model->SetRobotQ(robotAtI); 
    
    // Get contact info at current pose 
    vector<ContactInfo3d> contactInfos = env_model->ContactInfos();
    assert(contactInfos.size() == numContacts);


    for(int j=0;j<contactInfos.size();j++){
      Eigen::Vector3d contact_normal_world = contactInfos[j].normal; 
      Eigen::Vector3d contact_force_world = contactForce(j,i)*contact_normal_world;
      Eigen::VectorXd contact_wrench_world(6); 
      contact_wrench_world << contact << Eigen::Vector3d::Zero();

      Eigen::VectorXd generalized_force = contactInfos[j].Jc*contact_wrench_world;
      generalized_contact_force.col(i) += generalized_force;
    }    
  }

  Eigen::MatrixXd q_constr = q.rightCols(ts-1) - (q.leftCols(ts-1)  + v.rightCols(ts-1)*timestep);
  Eigen::MatrixXd v_constr = objectMass*(v.rightCols(ts-1) - v.leftCols(ts-1))
                             - timestep*generalized_contact_force.rightCols(ts-1);

  for(int i=0;i<q_constr.rows();i++){
    for(int j=0;j<q_constr.cols();j++){
      g[index++] = q_constr(i,j); 
    }
  }
  for(int i=0;i<v_constr.rows();i++){
    for(int j=0;j<v_constr.cols();j++){
      g[index++] = v_constr(i,j); 
    }
  }
  return index;
}

int OptProblem::eval_robotstate(const Number*x, Number*g, int index){
  // qr[t+1] = qr[t]+dqr[t+1]*h

  int ts = numTimesteps;
  Eigen::MatrixXd robotState(2*numRobotDOF, ts); 
  getRobotState(x, robotState); 

  Eigen::MatrixXd q = robotState.topRows(numRobotDOF);
  Eigen::MatrixXd v = robotState.bottomRows(numRobotDOF); 

  Eigen::MatrixXd q_constr = q.rightCols(ts-1) 
                             - (q.leftCols(ts-1) + v.rightCols(ts-1)*timestep);

  for(int i=0;i<q_constr.rows();i++){
    for(int j=0;j<q_constr.cols();j++){
      g[index++] = q_constr(i,j); 
    }
  }
  return index; 
}

int OptProblem::eval_init_obj_state(const Number*x, Number*g, int index){

  int ts = numTimesteps;
  Eigen::MatrixXd objState(numObjectDOF*2, ts);
  getObjState(x, objState);
  Eigen::VectorXd initial = objState.leftCols(1);

  // g[index++] = initial - initObjState(0); 
  // g[index++] = objState(numTimesteps) - initObjState(1);

  return index;
}



int OptProblem::eval_init_robot_state(const Number*x, Number*g, int index){
  Eigen::MatrixXd robotState(numRobotDOF*2, numTimesteps);
  getRobotState(x, robotState);
  Eigen::VectorXd initial = robotState.leftCols(1);

  g[index++] = initial(0) - initRobotState(0);
  g[index++] = initRobotState(1);

  return index; 
}

int OptProblem::eval_final_obj_state(const Number*x, Number *g, int index){
  Eigen::MatrixXd objState(numObjectDOF*2, numTimesteps);
  getObjState(x, objState);

  Eigen::VectorXd final = objState.rightCols(1); 
  g[index++] = final(0) - finalObjState(0);

  return index;  
}

int OptProblem::eval_final_robot_state(const Number*x, Number*g, int index){
  Eigen::MatrixXd robotState(numRobotDOF*2, numTimesteps);
  getRobotState(x, robotState); 

  Eigen::VectorXd final = robotState.rightCols(1); 
  g[index++] = final(0) - finalRobotState(0);

  return index+1;    
}

void OptProblem::finalize_solution(SolverReturn status,
                                  Index n, const Number* x, const Number* z_L, const Number* z_U,
                                  Index m, const Number* g, const Number* lambda,
                                  Number obj_value,
				  const IpoptData* ip_data,
				  IpoptCalculatedQuantities* ip_cq)
{
  // here is where we would store the solution to variables, or write to a file, etc
  // so we could use the solution.

  // For this example, we write the solution to the console
  std::cout << std::endl << std::endl << "Solution of the primal variables, x" << std::endl;

  for (Index i=0; i<n; i++) {
     std::cout << "x[" << i << "] = " << x[i] << std::endl;
  }

  std::cout << std::endl << std::endl << "Solution of the bound multipliers, z_L and z_U" << std::endl;
  for (Index i=0; i<n; i++) {
    std::cout << "z_L[" << i << "] = " << z_L[i] << std::endl;
  }
  for (Index i=0; i<n; i++) {
    std::cout << "z_U[" << i << "] = " << z_U[i] << std::endl;
  }

  std::cout << std::endl << std::endl << "Objective value" << std::endl;
  std::cout << "f(x*) = " << obj_value << std::endl;

  std::cout << std::endl << "Final value of the constraints:" << std::endl;
  for (Index i=0; i<m ;i++) {
    std::cout << "g(" << i << ") = " << g[i] << std::endl;
  }

  cout << "Writing to traj.out" << endl; 
  Eigen::MatrixXd objState(numObjectDOF*2, numTimesteps); 
  Eigen::MatrixXd robotState(numRobotDOF*2, numTimesteps); 

  getObjState(x, objState);
  getRobotState(x, robotState); 
  
  ofstream objTraj; 
  ofstream.open('objTraj.out'); 

  ofstream robotTraj; 
  ofstream.open('robotTraj.out'); 

  for(int i=0;i<numObjectDOF*2;i++){
    for(int j=0;j<numTimesteps;j++){
      objTraj << j*timestep << " " << objState(i,j) << " ";
    }
    objTraj << endl;
  }

  for(int i=0;i<numRobotDOF*2;i++){
    for(int j=0;j<numTimesteps;j++){
      robotTraj << j*timestep << " " << objState(i,j) << " ";
    }
    robotTraj << endl;
  }

  objTraj.close(); 
  robotTraj.close();
}


void OptProblem::getObjState(const Number* x, Eigen::MatrixXd& objState){
  assert(objState.rows() == 2*numObjectDOF); 
  assert(objState.cols() == numTimesteps); 

  int index=0;
  for(int i=0;i<numObjectDOF*2;i++){
    for(int j=0;j<numTimesteps;j++){
      objState(i,j) = x[index++];
    }
  }
}

void OptProblem::getRobotState(const Number* x, Eigen::MatrixXd& robotState){
  assert(robotState.rows() == 2*numRobotDOF);
  assert(robotState.cols() == numTimesteps); 

  int index = numObjectDOF*2*numTimesteps;
  for (int i=0;i<numRobotDOF*2;i++){
    for (int j=0;j<numTimesteps;j++){
      robotState(i,j) = x[index++];
    }
  }
}

void OptProblem::getContactForce(const Number*x, Eigen::MatrixXd& lambda){
  assert(lambda.rows() == numContacts); 
  assert(lambda.cols() == numTimesteps); 

  int index = numObjectDOF*2*numTimesteps + numRobotDOF*2*numTimesteps;
  for(int i=0;i<numContacts;i++){
    for(int j=0;j<numTimesteps;j++){
      lambda(i,j) = x[index++];
    }
  }
}

  /** Method to return:
   *   1) The structure of the jacobian (if "values" is NULL)
   *   2) The values of the jacobian (if "values" is not NULL)
   */
bool OptProblem::eval_jac_g(Index n, const Number* x, bool new_x,
                          Index m, Index nele_jac, Index* iRow, Index *jCol,
                          Number* values){


  // define Jacobian as dense 
  if (values == NULL) {
    int index =0;
    for(int i=0;i<numTotalConstr;i++){
      for(int j=0;j<numTotalVar;j++){
        iRow[index] = i; 
        jCol[index] = j;
        index++;
      }
    }
  }
  // use finite difference 
  else {
    for(int i=0;i<nele_jac;i++){
      values[i] = 0;
    }
    Number *g_orig = new Number[numTotalConstr];
    eval_g(n, x, new_x, m, g_orig);

    // shift x by 0.01, get new g, and finite-difference 
    Number* x2 = new Number[numTotalVar];
    for(int i=0;i<numTotalVar;i++){
      x2[i] = x[i];
    }

    double h = 0.00001;
    for(int j=0;j<numTotalVar;j++){
      x2[j] = x2[j]+h; 
      Number* g_shift = new Number[numTotalConstr];
      eval_g(n, x2, new_x, m, g_shift);

      for (int i=0;i<numTotalConstr;i++){
        values[i*numTotalVar + j] = (g_shift[i]-g_orig[i])/h;
      }

      delete[] g_shift; 
      x2[j] = x[j]; // shift back 
    }

    delete[] x2; 
    delete[] g_orig;
  }


  return true;
}

  /** Method to return:
   *   1) The structure of the hessian of the lagrangian (if "values" is NULL)
   *   2) The values of the hessian of the lagrangian (if "values" is not NULL)
   */
bool OptProblem::eval_h(Index n, const Number* x, bool new_x,
                      Number obj_factor, Index m, const Number* lambda,
                      bool new_lambda, Index nele_hess, Index* iRow,
                      Index* jCol, Number* values){
  cout << "this should not be called" << endl;
  exit(EXIT_FAILURE);
  return true;
}



int OptProblem::setObjectBound(Number* x_l, Number* x_u, int index){
  //TODO: set initial & final constraint 

  int i=index; 

  // position
  for (i;i<index+numObjectDOF*numTimesteps;i++){
    x_l[i] = -10; 
    x_u[i] = 20;
  }
  index = i; 
  //velocity
  for(i;i<index+numObjectDOF*numTimesteps;i++){
    x_l[i] = -2e19;
    x_u[i] = 2e19;
  }
  return i;
}

int OptProblem::setRobotBound(Number* x_l, Number* x_u, int index){
  // TODO: set initial & final constraint 

  int i = index;

  //position 
  for (i;i<index+numRobotDOF*numTimesteps;i++){
    x_l[i] = -10; 
    x_u[i] = 20;
  }
  index =i; 
  //velocity
  for(i; i<index+numRobotDOF*numTimesteps;i++){
    x_l[i] = -2e19;
    x_u[i] = 2e19;
  }
  return i; 
}

int OptProblem::setContactForceBound(Number* x_l, Number *x_u, int index){
  int i=index;

  for(i;i<index+numContacts*timestep;i++){
    x_l[i] = 0; 
    x_u[i] = 2e19;
  }
  return i;
}

  // set lower and upper bounds on constraints 
int OptProblem::setDistanceBound(Number* g_l, Number* g_u, int index){
  int i=index; 
  for(i;i<index+numDistanceConstr;i++){
    g_l[i] = 0; 
    g_u[i] = 2e19;
  }
  return i; 
}

int OptProblem::setLCPBound(Number* g_l, Number* g_u, int index){
  int i=index; 
  for(i;i<index+numLCPConstr;i++){
    g_l[i] = -2e19;
    g_u[i] = 0;
  }

  return i; 
}

int OptProblem::setDynamicsEqualityBound(Number* g_l, Number* g_u, int index){
  int i=index;
  for(i=index;i<index+numDynamicConstr;i++){
    g_l[i] = 0; 
    g_u[i] = 0;
  }
  return i;
}

int OptProblem::setInitialStateConstraint(Number* g_l, Number* g_u, int index){
  int i=index; 
  for(i;i<index+numInitialConstr;i++){
    // TODO 
  }
  return i; 
}


int OptProblem::setFinalStateConstraint(Number* g_l, Number* g_u, int index){
  int i=index; 
  for(i;i<index+numFinalConstr;i++){
    // TODO
  }
  return i; 
}



