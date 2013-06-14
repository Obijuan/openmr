#include "TestBase.h"

class Example : public TestBase
{
  public:
    Example(string envfile,string controller,bool showgui=true) :
       TestBase(envfile,controller,showgui) {};
    void run(dReal step, bool realtime=true);

  private:
    OpenRAVE::EnvironmentBase::GraphHandlePtr flecha;
};

void Example::run(dReal step, bool realtime)
{
  stringstream os,is;

  std::vector<RaveVector<float> > vpoints;
    RaveVector<float> pt0;
    RaveVector<float> color;
    pt0[0] = 0.0;  pt0[1] = 0.0; pt0[2] = 0.0;
    color[0] = 1.0;
    color[1] = 0.0;
    color[2] = 0.0;
    color[3] = 1.0;
    color.w = 1;

    vpoints.push_back(pt0);
    
    flecha = penv->plot3 (vpoints[0], 1, 1, 10.0, color);

  is << "setamplitude 30 30 30 ";
  pcontroller->SendCommand(os,is);

  is << "setinitialphase 0 150 300 ";
  pcontroller->SendCommand(os,is);

  is << "setoffset 0 0 0 ";
  pcontroller->SendCommand(os,is);

  is << "setperiod 2 ";
  pcontroller->SendCommand(os,is);

  is << "oscillation on ";
  pcontroller->SendCommand(os,is);

  penv->StartSimulation(step,realtime);

  while(1);
}

int main(int argc, char ** argv)
{
  string envfile;

  if (argc==1)
    //-- Default file
    envfile="./models/Cube3-I.env.xml";
  else
    envfile = argv[1];

  Example example(envfile,"sinoscontroller");
  usleep(100000);
  example.SetCamera(0.427, 0.285, 0.47, 0.718, 0.59, 0.078, 0.263);
  example.run(0.003,true);

  return 0;
}

