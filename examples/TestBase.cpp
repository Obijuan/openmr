#include "TestBase.h"

#include <iostream>
#include <boost/bind.hpp>

//-- Constructor
TestBase::TestBase(string envfile, string controller, bool showgui)
{

  this->showgui = showgui;

  RaveInitialize(true);
  penv = RaveCreateEnvironment();  // create the main environment
  RaveSetDebugLevel(Level_Debug);

  penv->StopSimulation();

  //-- Start the viewer
  if (showgui) {
    pthviewer = new boost::thread(boost::bind(&TestBase::SetViewer, this));
    usleep(200000); // wait for the viewer to init
  }
  
  // load the scene
  if (!penv->Load(envfile)) {
    penv->Destroy();
    return;
  }
  usleep(100000); // wait for the viewer to init


  //-- Get the robot
  std::vector<RobotBasePtr> robots;
  penv->GetRobots(robots);
  probot = robots[0];
  cout << "Robot: " << probot->GetName() << endl;

  // create the controllers, make sure to lock environment!
  {
    EnvironmentMutex::scoped_lock lock(penv->GetMutex()); // lock environment

    //-- Load the controller
    pcontroller=RaveCreateController(penv,controller);
    vector<int> dofindices(probot->GetDOF());
    for(int i = 0; i < probot->GetDOF(); ++i) {
      dofindices[i] = i;
    }
    probot->SetController(pcontroller,dofindices,1);
  }
  
}

//-- Destructor
TestBase::~TestBase()
{
  if (showgui) {
    pthviewer->join();
    delete pthviewer;
  }

  //-- Destroy the environment
  penv->Destroy();
}

void TestBase::SetCamera(dReal q0, dReal q1, dReal q2, dReal q3, dReal tx, dReal ty, dReal tz)
{
  //-- Perform the transformation only if the gui is active
  if (!showgui) return;

  RaveVector<float> rotation(q0,q1,q2,q3);
  RaveVector<float> translation(tx,ty,tz);
  RaveTransform<float> T(rotation,translation);
  viewer->SetCamera(T);
}

void TestBase::SetViewer()
{
  viewer = RaveCreateViewer(penv,"qtcoin");
  BOOST_ASSERT(!!viewer);

  cout << "Viewer!!!!" << endl;

  // attach it to the environment:
  penv->AttachViewer(viewer);

  //-- Set the camera
  SetCamera(0.427, 0.285, 0.47, 0.718, 0.338, 0.18, 0.166);

  // finally you call the viewer's infinite loop
  // (this is why you need a separate thread):
  bool showgui = true;
  viewer->main(showgui);
}




