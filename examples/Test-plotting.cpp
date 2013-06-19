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


//-------------------------------------------------------------------------------
//-- This is a test that plots a point in the origin of the environment
//-------------------------------------------------------------------------------


#include "TestBase.h"

class Example : public TestBase
{
public:
    Example(string envfile,string controller,bool showgui=true) :
	TestBase(envfile,controller,showgui) {};
    void run(dReal step, bool realtime=true);

private:
    OpenRAVE::EnvironmentBase::GraphHandlePtr arrow;
};

void Example::run(dReal step, bool realtime)
{
    stringstream os,is;

    float pt0[3] = { 0.0, 0.0, 0.0};
    RaveVector<float> color;
    color[0] = 1.0;
    color[1] = 0.0;
    color[2] = 0.0;
    color[3] = 1.0;
    color.w = 1;
    
    arrow = penv->plot3 ( pt0, 1, 1, 10.0, color);

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

    char key;
    std::cout << "Press a key to start the simulation" << std::endl;
    std::cin.get(key);

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
    //example.SetCamera(0.427, 0.285, 0.47, 0.718, 0.59, 0.078, 0.263);
    example.run(0.003,true);

    return 0;
}

