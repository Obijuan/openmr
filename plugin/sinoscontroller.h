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

	RegisterCommand( "Setamplitude", boost::bind(&SinosController::SetAmplitude, this, _1, _2), "Command for setting the aplitude");
	RegisterCommand( "Setinitialphase", boost::bind(&SinosController::SetInitialPhase, this, _1, _2), "Command for setting the initial phase");
	RegisterCommand( "Setoffset", boost::bind(&SinosController::SetOffset, this, _1, _2), "asdf");
	RegisterCommand( "Setperiod", boost::bind(&SinosController::SetPeriod, this, _1, _2), "asdf");
	RegisterCommand( "Oscillation", boost::bind(&SinosController::SetOscillation, this, _1, _2), "asdf");
	RegisterCommand( "Record_on" , boost::bind(&SinosController::RecordOn, this, _1, _2), "asdf");
	RegisterCommand( "Record_off", boost::bind(&SinosController::RecordOff, this, _1, _2), "asdf");

	_penv = penv;
    }

    virtual ~SinosController() {}

    virtual bool Init(RobotBasePtr robot, const std::vector<int>& dofindices, int nControlTransformation)
    {
	_probot = robot;
	_dofindices = dofindices;
	_nControlTransformation = nControlTransformation;

	//-- Initialization of the servocontroller
	_pservocontroller = RaveCreateController(_penv,"servocontroller");
	_pservocontroller->Init( _probot, _dofindices, _nControlTransformation);

	//-- Resize local vectors
	_ref_pos.resize(    _probot->GetDOF() );
	_amplitude.resize(  _probot->GetDOF() );
	_phase0.resize(	    _probot->GetDOF() );
	_offset.resize(	    _probot->GetDOF() );

	std::cout << "[sinoscontroller] INIT" << std::endl;

	Reset(0);

	return true;
    }



    virtual void Reset(int options)
    {
	_samplingTics=0;
	_period=1;
	_N=20;
	_n=0;
	_phase=0;
	_cycleTime=0;
	_oscillating=false;

	for (int i=0; i<_probot->GetDOF(); i++) {
	  _ref_pos[i]=0;
	  _amplitude[i]=0;
	  _phase0[i]=0;
	  _offset[i]=0;
	}

	SetRefPos();

	std::cout << "[sinoscontroller] Reset!" << std::endl;
    }


    virtual void SimulationStep(dReal fTimeElapsed)
    {
	//-- Simulate the servos
	_pservocontroller->SimulationStep(fTimeElapsed);

	//-- If the oscillating mode is not set, return
	if (!_oscillating)
	    return;

	_samplingPeriod = round(_period/(_N*fTimeElapsed));
	_samplingTics++;
	//cout << "Sampling tics: " << _samplingTics << endl;

	if ( _samplingTics == _samplingPeriod)
	{
	    _samplingTics=0;
	    _n++;

	    //-- Calculate the next positions
	    SetRefPos();
	}
    }

    bool SetAmplitude( std::ostream& os, std::istream& is)
    {
	for(size_t i = 0; i < _amplitude.size(); ++i)
	{
	    is >> _amplitude[i];

	    if( !is )
		return false;
	}
	SetRefPos();
	return true;
    }

    bool SetInitialPhase( std::ostream& os, std::istream& is)
    {
	for(size_t i = 0; i < _phase0.size(); ++i)
	{
	    is >> _phase0[i];

	    if( !is )
		return false;
	}
	SetRefPos();
	return true;
    }

    bool SetOffset( std::ostream& os, std::istream& is)
    {
	for(size_t i = 0; i < _offset.size(); ++i)
	{
	    is >> _offset[i];

	    if( !is )
		return false;
	}
	SetRefPos();
	return true;
    }

    bool SetPeriod( std::ostream& os, std::istream& is)
    {
	is >> _period;
	_samplingPeriod= _period/_N;
	SetRefPos();
	return true;
    }

    bool SetOscillation( std::ostream& os, std::istream& is)
    {
	std::string mode;
	is >> mode;

	if (mode=="on")
	    _oscillating=true;
	else
	    _oscillating=false;

	return true;
    }

    bool RecordOn( std::ostream& os, std::istream& is)
    {
	string file;
	stringstream os2, is2;

	is >> file;

	is2 << "record_on " << file << " ";
	_pservocontroller->SendCommand(os2,is2);

	return true;
    }

    bool RecordOff( std::ostream& os, std::istream& is)
    {
	stringstream os2, is2;

	is2 << "record_off ";
	_pservocontroller->SendCommand(os2,is2);

	return true;
    }
/*
    virtual bool SendCommand(std::ostream& os, std::istream& is)
    {
	string cmd;
	is >> cmd;
	std::transform(cmd.begin(), cmd.end(), cmd.begin(), ::tolower);

	//-- Set position command. The joint angles are received in degrees
	if( cmd == "setamplitude" )
	{
	    for(size_t i = 0; i < _amplitude.size(); ++i)
	    {
		is >> _amplitude[i];

		if( !is )
		    return false;
	    }
	    SetRefPos();
	    return true;
	}
	else if ( cmd == "setinitialphase" )
	{
	    for(size_t i = 0; i < _phase0.size(); ++i)
	    {
		is >> _phase0[i];

		if( !is )
		    return false;
	    }
	    SetRefPos();
	    return true;
	}
	else if ( cmd == "setoffset" )
	{
	    for(size_t i = 0; i < _offset.size(); ++i)
	    {
		is >> _offset[i];

		if( !is )
		    return false;
	    }
	    SetRefPos();
	    return true;
	}
	else if ( cmd == "setperiod" )
	{
	    is >> _period;
	    _samplingPeriod= _period/_N;
	    SetRefPos();
	    return true;
	}
	else if ( cmd == "oscillation" ) {
	  std::string mode;
	  is >> mode;

	  if (mode=="on")
	      _oscillating=true;
	  else
	      _oscillating=false;

	  return true;
	}
	else if ( cmd == "record_on" ) {
	  string file;
	  stringstream os2, is2;

	  is >> file;

	  is2 << "record_on " << file << " ";
	  _pservocontroller->SendCommand(os2,is2);
	}
	else if ( cmd == "record_off" ) {
	  stringstream os2, is2;

	  is2 << "record_off ";
	  _pservocontroller->SendCommand(os2,is2);
	}
	return true;
    }
*/
    //-- Other functions:
    virtual const std::vector<int>& GetControlDOFIndices() const { return _dofindices; }
    virtual int IsControlTransformation() const { return _nControlTransformation; }
    virtual bool SetDesired(const std::vector<dReal>& values, TransformConstPtr trans) { return false; }
    virtual bool SetPath(TrajectoryBaseConstPtr ptraj) { Reset(0); return false; }
    virtual RobotBasePtr GetRobot() const  {return _probot;}
    virtual bool IsDone()  { return false; }
    virtual dReal GetTime() const { return 0; }

private:

    //-- Calculate the reference position and send it to the servos
    //-------------------------------------------------------------------------------
    void SetRefPos()
    {
	stringstream os, is;
	is << "setpos ";

	for (size_t i=0; i<_ref_pos.size(); i++)
	{
	  _ref_pos[i]=_amplitude[i]*sin(-(360.0*_n)/(float)_N *PI/180.0 + _phase0[i]*PI/180) + _offset[i];
	  is<<_ref_pos[i]<<" ";
	}

	//-- Set the new servo reference positions
	_pservocontroller->SendCommand(os,is);

	//-- Debug
	//std::cout << "n=" << _n << " Ref0: " << _ref_pos[0] << " Ref1: " << _ref_pos[1] << std::endl;
    }

protected:
    EnvironmentBasePtr _penv;
    RobotBasePtr _probot;
    std::vector<int> _dofindices;
    int _nControlTransformation;

    ControllerBasePtr _pservocontroller;
    int _samplingTics;
    int _samplingPeriod;

    dReal _cycleTime;
    bool _oscillating;        //-- State of the oscillator: oscillating true/false
    int _N;                   //-- Number of samples
    int _n;                   //-- Discrete time
    dReal _period;            //-- Oscilation period in seconds
    dReal _phase;

    std::vector<dReal> _ref_pos;   //-- Reference positions for the servos (in degrees)
    std::vector<dReal> _amplitude; //-- Oscillation amplitudes
    std::vector<dReal> _phase0;    //-- Oscillation initial phase
    std::vector<dReal> _offset;    //-- Oscillation offset
};

#endif
