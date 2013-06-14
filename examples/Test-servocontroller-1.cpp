/*------------------------------------------------------------------------*/
/* Test-servocontroller1                                                  */
/*------------------------------------------------------------------------*/
/* (c) Juan Gonzalez. July-2010                                           */
/*------------------------------------------------------------------------*/
/* GPL license                                                            */
/*------------------------------------------------------------------------*/
/* Testing the Servocontroller.                                           */
/* This example set the servo position to 45 and -45 every second         */
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

    stringstream is,os;

    while(1) {
      //-- Set the position of the servo (servo 0) to 45 degrees
      is << "setpos1 0 45 ";
      pcontroller->SendCommand(os,is);

      //-- Wait one second
      sleep(1);

      //-- Set the position of servo 0 to -45 degrees
      is << "setpos1 0 -45 ";
      pcontroller->SendCommand(os,is);

      //-- Wait one second
      sleep(1);

      //-- Debug! Show the viewer transformation
      //RaveTransform<float> t = viewer->GetCameraTransform();
      //cout << "Transform: " << t << endl;
    }

}

int main(void)
{

  Example example("models/Unimod1.env.xml","servocontroller");
  example.run(0.002);

  return 0;
}


