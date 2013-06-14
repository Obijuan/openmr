#include "TestBase.h"

class Example : public TestBase
{
  public:
    Example(string envfile,string controller,bool showgui=true) :
       TestBase(envfile,controller,showgui) {};
    void run(dReal step, bool realtime=true);
};

void Example::run(dReal step, bool realtime)
{
  stringstream os,is;

  is << "oscillation on ";
  pcontroller->SendCommand(os,is);
  penv->StartSimulation(step,realtime);

  while(1) {

    //-- Sideways movement
    is << "setperiod 1.5 ";
    pcontroller->SendCommand(os,is);
    is << "setoffset 0 0 0 ";
    pcontroller->SendCommand(os,is);
    is << "setinitialphase 0 90 0 ";
    pcontroller->SendCommand(os,is);
    is << "setamplitude 30 30 30 ";
    pcontroller->SendCommand(os,is);
    sleep(10);

      //-- sideways movement (oposite direction)
    is << "setinitialphase 0 -90 0 ";
    pcontroller->SendCommand(os,is);
    sleep(10);

      //-- Debug! Show the viewer transformation
      //RaveTransform<float> t = viewer->GetCameraTransform();
      //cout << "Transform: " << t << endl;

      //-- Rotating
    is << "setinitialphase 0 90 180 ";
    pcontroller->SendCommand(os,is);
    is << "setamplitude 30 60 30 ";
    pcontroller->SendCommand(os,is);
    sleep(20);

      //-- Preparation to rolling
    is << "setamplitude 0 0 0 ";
    pcontroller->SendCommand(os,is);
    is << "setoffset 0 70 0 ";
    pcontroller->SendCommand(os,is);
    sleep(2);

      //-- Rolling
    is << "setperiod 2 ";
    pcontroller->SendCommand(os,is);
    is << "setinitialphase 0 90 0 ";
    pcontroller->SendCommand(os,is);
    is << "setoffset 0 0 0 ";
    pcontroller->SendCommand(os,is);
    is << "setamplitude 70 70 70 ";
    pcontroller->SendCommand(os,is);
    sleep(10);
  }

}

int main(int argc, char ** argv)
{
  string envfile;

  if (argc==1)
    //-- Default file
    envfile="./models/Minicube-II.env.xml";
  else
    envfile = argv[1];

  Example example(envfile,"sinoscontroller");
  usleep(100000);
  example.SetCamera(0.493, 0.286, 0.378, 0.729,0.557, 0.097, 0.352);
  example.run(0.001,true);

  return 0;
}



