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
    penv->StartSimulation(step, realtime);

    stringstream os,is;
    is << "setamplitude 45 ";
    pcontroller->SendCommand(os,is);

    is << "setinitialphase 0 ";
    pcontroller->SendCommand(os,is);

    is << "setoffset 0 ";
    pcontroller->SendCommand(os,is);

    is << "setperiod 2 ";
    pcontroller->SendCommand(os,is);

    is << "oscillation on ";
    pcontroller->SendCommand(os,is);

    //-- Start recording the servo's angle
    is << "record_on test2.m ";
    pcontroller->SendCommand(os,is);

    const dReal STEP = 0.005;
    penv->StartSimulation(STEP);
    sleep(3);

    is << "record_off ";
    pcontroller->SendCommand(os,is);

    cout << "FIN\n";
    
}

int main(void)
{

  Example example("models/Unimod1.env.xml","sinoscontroller");
  usleep(100000);
  example.SetCamera(0.36697, 0.167263, 0.434483, 0.805345,0.290932, 0.285499, 0.233995);
  example.run(0.005);

  return 0; 
}



