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
  dReal angle;

  //-- Initially set the position to 0
  is << "setpos1 0 0 ";
  pcontroller->SendCommand(os,is);
  sleep(1);

  is << "setpos1 0 45 ";
  pcontroller->SendCommand(os,is);
  
  //-- Get the servo position every 20ms (during 1 second)
  for (int i=0; i<50; i++) {
    //is << "getpos1 0 ";
    is << "getpos ";
    pcontroller->SendCommand(os,is);
    os >> angle;
    cout << "Servo: " << angle << endl;
    usleep(20000);
  }  

  cout << "END\n";

}

int main(void)
{

  Example example("models/Unimod1.env.xml","servocontroller");
  example.run(0.005);

  return 0;
}


