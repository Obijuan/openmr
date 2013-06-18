// Copyright (C) 2010 Juan Gonzalez-Gomez (juan@iearobotics.com)
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

#ifndef OPENMR_SINOSCONTROLLER_H
#define OPENMR_SINOSCONTROLLER_H

#include <openrave/openrave.h>
#include <openrave/plugin.h>
#include <boost/bind.hpp>

#include <iostream>
#include <cmath>

using namespace std;
using namespace OpenRAVE;

class SinosController : public ControllerBase
{
public:
    SinosController(EnvironmentBasePtr penv, std::istream& ss) : ControllerBase(penv)
    {
	__description = "Sinusoidal oscillator controller by Juan Gonzalez-Gomez, updated by David Estevez-Fernandez";

	this->env = penv;
    }

    virtual ~SinosController() {}

    virtual bool Init(RobotBasePtr robot, const std::vector<int>& dofindices, int nControlTransformation)
    {
	this->robot = robot;
	this->dofIndices = dofindices;
	this->nControlTransformation = nControlTransformation;

	//-- Initialization of the servocontroller
	servocontroller = RaveCreateController(env,"servocontroller");
	servocontroller->Init( robot, dofindices, nControlTransformation);

	//-- Resize local vectors
	ref_pos.resize(	  robot->GetDOF() );
	amplitude.resize( robot->GetDOF() );
	phase0.resize(	  robot->GetDOF() );
	offset.resize(	  robot->GetDOF() );

	std::cout << "[sinoscontroller] INIT" << endl;

	Reset(0);

	return true;
    }



    virtual void Reset(int options)
    {
	samplingTics=0;
	period=1;
	N=20;
	n=0;
	phase=0;
	cycleTime=0;
	oscillating=false;

	for (int i=0; i<robot->GetDOF(); i++) {
	  ref_pos[i]=0;
	  amplitude[i]=0;
	  phase0[i]=0;
	  offset[i]=0;
	}

	SetRefPos();

	std::cout << "[sinoscontroller] Reset!" << endl;
    }


    virtual void SimulationStep(dReal fTimeElapsed)
    {
	//-- Simulate the servos
	servocontroller->SimulationStep(fTimeElapsed);

	//-- If the oscillating mode is not set, return
	if (!oscillating)
	    return;

	samplingPeriod = round(period/(N*fTimeElapsed));
	samplingTics++;
	//cout << "Sampling tics: " << samplingTics << endl;

	if ( samplingTics == samplingPeriod)
	{
	    samplingTics=0;
	    n++;

	    //-- Calculate the next positions
	    SetRefPos();
	}
    }



    virtual bool SendCommand(std::ostream& os, std::istream& is)
    {
	string cmd;
	is >> cmd;
	std::transform(cmd.begin(), cmd.end(), cmd.begin(), ::tolower);

	//-- Set position command. The joint angles are received in degrees
	if( cmd == "setamplitude" )
	{
	    for(size_t i = 0; i < amplitude.size(); ++i)
	    {
		is >> amplitude[i];

		if( !is )
		    return false;
	    }
	    SetRefPos();
	    return true;
	}
	else if ( cmd == "setinitialphase" )
	{
	    for(size_t i = 0; i < phase0.size(); ++i)
	    {
		is >> phase0[i];

		if( !is )
		    return false;
	    }
	    SetRefPos();
	    return true;
	}
	else if ( cmd == "setoffset" )
	{
	    for(size_t i = 0; i < offset.size(); ++i)
	    {
		is >> offset[i];

		if( !is )
		    return false;
	    }
	    SetRefPos();
	    return true;
	}
	else if ( cmd == "setperiod" )
	{
	    is >> period;
	    samplingPeriod= period/N;
	    SetRefPos();
	    return true;
	}
	else if ( cmd == "oscillation" ) {
	  std::string mode;
	  is >> mode;

	  if (mode=="on")
	      oscillating=true;
	  else
	      oscillating=false;

	  return true;
	}
	else if ( cmd == "record_on" ) {
	  string file;
	  stringstream os2, is2;

	  is >> file;

	  is2 << "record_on " << file << " ";
	  servocontroller->SendCommand(os2,is2);
	}
	else if ( cmd == "record_off" ) {
	  stringstream os2, is2;

	  is2 << "record_off ";
	  servocontroller->SendCommand(os2,is2);
	}
	return true;
    }

    //-- Other functions:
    virtual const std::vector<int>& GetControlDOFIndices() const { return dofIndices; }
    virtual int IsControlTransformation() const { return nControlTransformation; }
    virtual bool SetDesired(const std::vector<dReal>& values, TransformConstPtr trans) { return false; }
    virtual bool SetPath(TrajectoryBaseConstPtr ptraj) { Reset(0); return false; }
    virtual RobotBasePtr GetRobot() const  {return robot;}
    virtual bool IsDone()  { return false; }
    virtual dReal GetTime() const { return 0; }

private:

    //-- Calculate the reference position and send it to the servos
    //-------------------------------------------------------------------------------
    void SetRefPos()
    {
	stringstream os, is;
	is << "setpos ";

	for (size_t i=0; i<ref_pos.size(); i++)
	{
	  ref_pos[i]=amplitude[i]*sin(-(360.0*n)/(float)N *PI/180.0 + phase0[i]*PI/180) + offset[i];
	  is<<ref_pos[i]<<" ";
	}

	//-- Set the new servo reference positions
	servocontroller->SendCommand(os,is);

	//-- Debug
	//std::cout << "n=" << n << " Ref0: " << ref_pos[0] << " Ref1: " << ref_pos[1] << std::endl;
    }

protected:
    EnvironmentBasePtr env;
    RobotBasePtr robot;
    std::vector<int> dofIndices;
    int nControlTransformation;

    ControllerBasePtr servocontroller;
    int samplingTics;
    int samplingPeriod;

    dReal cycleTime;
    bool oscillating;        //-- State of the oscillator: oscillating true/false
    int N;                   //-- Number of samples
    int n;                   //-- Discrete time
    dReal period;            //-- Oscilation period in seconds
    dReal phase;

    std::vector<dReal> ref_pos;   //-- Reference positions for the servos (in degrees)
    std::vector<dReal> amplitude; //-- Oscillation amplitudes
    std::vector<dReal> phase0;    //-- Oscillation initial phase
    std::vector<dReal> offset;    //-- Oscillation offset
};

#endif
