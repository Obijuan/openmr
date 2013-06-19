// Copyright (C) 2010 Juan Gonzalez (juan@iearobotics.com)
// Updated by David Estevez-Fernandez, 2013
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program. If not, see <http://www.gnu.org/licenses/>.


//----------------------------------------------------------------------------
//-- This example loads the simplest 3-module snake robot and makes it perform
//-- some test movements: walking, turning, rolling, etc
//----------------------------------------------------------------------------


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
    envfile="../models/Minicube-II.env.xml";
  else
    envfile = argv[1];

  Example example(envfile,"sinoscontroller");
  usleep(100000);
  example.SetCamera(0.493, 0.286, 0.378, 0.729,0.557, 0.097, 0.352);
  example.run(0.001,true);

  return 0;
}



