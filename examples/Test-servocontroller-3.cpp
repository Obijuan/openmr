/*------------------------------------------------------------------------*/
/* Test-servocontroller3                                                  */
/*------------------------------------------------------------------------*/
/* (c) Juan Gonzalez. July-2010                                           */
/*------------------------------------------------------------------------*/
/* GPL license                                                            */
/*------------------------------------------------------------------------*/
/* Testing the Servocontroller.                                           */
/* Example of the record_on and record_off commands                       */
/**************************************************************************/

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
  stringstream is, os;

  //-- Initially set the position to 0
  is << "setpos 0 0 ";
  pcontroller->SendCommand(os,is);
  sleep(1);

  //-- Start recording the servo's angle
  is << "record_on test1.m ";
  pcontroller->SendCommand(os,is);
  usleep(100000);

  is << "setpos 45 -45 ";
  pcontroller->SendCommand(os,is);
  sleep(1);

  is << "record_off ";
  pcontroller->SendCommand(os,is);

  cout << "FIN\n";

}

int main(void)
{

  Example example("models/Unimod2.env.xml","servocontroller");
  example.run(0.005);

  return 0;
}


