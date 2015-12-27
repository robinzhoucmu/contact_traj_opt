#include "dart/dart.h"
#include <time.h>
#include <fstream>
#include <iostream>
#include <vector>
#include <string>
using namespace dart::dynamics;
using namespace dart::simulation;
using namespace dart::gui;
using namespace std;

const double default_ground_width = 2;
const double default_wall_thickness = 0.1;

class MyWindow: public SimWindow
{
public: 
  MyWindow(const WorldPtr& world, const SkeletonPtr& pusher,
           const SkeletonPtr& slider,
           const vector<double> t, const vector<double> slider_traj, 
           const vector<double> pusher_traj)
  : mPusher(pusher), 
    mSlider(slider),
    mTime(t),
    pusher_traj(pusher_traj), 
    slider_traj(slider_traj),
  {
    setWorld(world);
  }

  void drawSkels() override
  {
    // Make sure lighting is turned on and that polygons get filled in
    glEnable(GL_LIGHTING);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

    SimWindow::drawSkels();
  }

  void displayTimer(int _val) override
  {
    // We remove playback and baking, because we want to be able to add and
    // remove objects during runtime
    int numIter = mDisplayTimeout / (mWorld->getTimeStep() * 1000);
    if (mSimulating)
    {
      for (int i = 0; i < numIter; i++)
        timeStepping();
    }
    glutPostRedisplay();
    glutTimerFunc(mDisplayTimeout, refreshTimer, _val);
  }

  void keyboard(unsigned char key, int x, int y) override
  {
    switch(key)
    {

      case ' ':
        t1 = clock();

    }
    SimWindow::keyboard(key, x, y);

  }

  void timeStepping() override
  {

    double t = mWorld->getTime(); 
    double pusher_x = getPusherPosition(t); 
    double slider_x = getSliderPosition(t); 

    cout << "t" << t << endl;
    cout << "[in] pusher: " << pusher_x << endl; 
    cout << "[in] slider: " << slider_x << endl; 
    mPusher->getDof(3)->setPosition(pusher_x);
    mSlider->getDof(3)->setPosition(slider_x); 

    cout << "[GL] slider" << mSlider->getPositions() << endl;
    cout << "[GL] pusher" << mPusher->getPositions() << endl;
    
    SimWindow::timeStepping();
    
    return;


  }

  double getPusherPosition(double t){
    int i=0;
    for (i=0;i<mTime.size();i++){
      if (mTime[i]>=t) break;
    }
    if (i == 10) i = 9; 

    cout << "i" << i << endl;
    return pusher_traj[i];
  }

  double getSliderPosition(double t){
    int i=0;
    for (i=0;i<mTime.size();i++){
      if (mTime[i]>=t) break;
    }
    if (i == 10) i = 9; 

    return slider_traj[i];
  }


  SkeletonPtr mPusher, mSlider;
  clock_t t1;
  vector<double> mTime, pusher_traj, slider_traj;
};

