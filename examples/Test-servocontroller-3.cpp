/*------------------------------------------------------------------------*/
/* Test-servocontroller3                                                  */
/*------------------------------------------------------------------------*/
/* (c) Juan Gonzalez. July-2010                                           */
/* Updated by David Estevez-Fernandez, 2013                               */
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

  char key;
  std::cout << "Press a key to start the simulation" << std::endl;
  std::cin.get(key);

  penv->StartSimulation(step, realtime);
  stringstream is, os;

  //-- Initially set the position to 0
  is << "setpos 0 0 ";
  pcontroller->SendCommand(os,is);
  sleep(1);

  //-- Start recording the servo's angle
  is << "record_on test1 ";
  pcontroller->SendCommand(os,is);
  usleep(100000);

  is << "setpos 45 -45 ";
  pcontroller->SendCommand(os,is);
  sleep(1);

  is << "record_off ";
  pcontroller->SendCommand(os,is);

  cout << "END. Press cntrl+C to exit...\n";

}

int main(int argc, char ** argv)
{
    std::string envfile;

    if (argc==1)
      //-- Default file
      envfile="../models/Unimod2.env.xml";
    else
      envfile = argv[1];

  Example example(envfile, "servocontroller");
  example.run(0.005);

  return 0;
}


