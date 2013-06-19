/*------------------------------------------------------------------------*/
/* Test-servocontroller2                                                  */
/*------------------------------------------------------------------------*/
/* (c) Juan Gonzalez. July-2010                                           */
/* Updated by David Estevez-Fernandez, 2013                               */
/*------------------------------------------------------------------------*/
/* GPL license                                                            */
/*------------------------------------------------------------------------*/
/* Testing the Servocontroller.                                           */
/* This example sets the position of the servos 0 and 1 to 45 and -45      */
/* alternately                                                            */
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

  //-- Main loop
  while(1) {

    //-- Servo positioning example 1: Using the setpos command. The position
    //-- of the two servos is given: Servo 0 to 45 and servo 1 to -45
    is << "setpos 45 -45 ";
    pcontroller->SendCommand(os,is);
    cout << "works ok";
    //-- Wait one second
    sleep(1);

    //-- Servo positioning example 2:
    //-- One method: using the "setpos" command:
    //-- is << "setpos -45 45 ";
    //-- pcontroller->SendCommand(os,is);

    //-- Another method: using the setpos1 command:
    //-- Seting the servo 0 position:
    is << "setpos1 0 -45 ";
    pcontroller->SendCommand(os,is);

    //-- Seting the servo 1 position:
    is << "setpos1 1 45 ";
    pcontroller->SendCommand(os,is);

    //-- Wait one second
    sleep(1);

    //-- Debug! Show the viewer transformation
    //RaveTransform<float> t = viewer->GetCameraTransform();
    //cout << "Transform: " << t << endl;
  }

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

  //example.SetCamera(-0.256, -0.194, 0.54, 0.778, -0.155, 0.23, 0.119);
  example.run(0.005);

  return 0;
}



