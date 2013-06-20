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

#ifndef OPENRAVE_MODULAR_ROBOTS_CONTROLLERS_H
#define OPENRAVE_MODULAR_ROBOTS_CONTROLLERS_H

#include <openrave/openrave.h>
#include <openrave/plugin.h>
#include <openrave/kinbody.h>

#include <boost/bind.hpp>

#include <iostream>
#include <fstream>
#include <cmath>

using namespace std;
using namespace OpenRAVE;

//-- Time vector
typedef std::vector<dReal> tvector;

class ServoController : public ControllerBase
{
public:
    ServoController(EnvironmentBasePtr penv, std::istream& ss)  : ControllerBase(penv)
    {
	__description = "Servo controller by Juan Gonzalez-Gomez and Rosen Diankov, updated by David Estevez-Fernandez";

	RegisterCommand("Test",boost::bind(&ServoController::Test,this,_1,_2),"Command for testing and debugging");
	RegisterCommand("Setpos",boost::bind(&ServoController::SetPos,this,_1,_2),"Format: Setpos s1 [s2]. Set the reference position of all the robot joints, in degrees, in the range [-90,90]. If the robot have N joints, there have to be N arguments");
	RegisterCommand("Setpos1",boost::bind(&ServoController::SetPos1,this,_1,_2),"Format: Setpos1 servo pos. Set the reference position of one joint. The argument servo is the servo number, starting from 0. The argument pos is the reference position (in degrees) [-90,90] ");
	RegisterCommand("Getpos",boost::bind(&ServoController::GetPos,this,_1,_2),"Format: Getpos. Get the position of ALL the servos (in degrees)");
	RegisterCommand("Getpos1",boost::bind(&ServoController::GetPos1,this,_1,_2),"Format: Getpos servo. Returns the current servo position (in degrees, in the range [-90,90]. The argument servo is the servo number, starting from 0");
	RegisterCommand("Record_on",boost::bind(&ServoController::RecordOn,this,_1,_2),"Format: Record_on file. Start recording the servo position in the specified file. It will generate an octave file ");
	RegisterCommand("Record_off",boost::bind(&ServoController::RecordOff,this,_1,_2),"Format: Record_off. Stop recording. The octave file is generated ");

	_penv = penv;
    }


    virtual ~ServoController() {}
    
    virtual bool Init(RobotBasePtr robot, const std::vector<int>& dofindices, int nControlTransformation)
    {
	_probot = robot;
	_dofindices = dofindices;
	_nControlTransformation = nControlTransformation;

	//-- Initialize odevelocity controller
	_pvelocitycontroller = RaveCreateController( _penv, "odevelocity");
	_pvelocitycontroller->Init( _probot, _dofindices, _nControlTransformation );

	//-- Get the robot joints. Needed for obtaining the joint angles and maxvelocities.
	std::vector<KinBodyPtr> bodies;
	_penv->GetBodies( bodies );
	_joints = bodies[0]->GetJoints();

	//-- Recording Mode
	//----------------------------------
	_recording = false;

	_phi_tvec.resize( _joints.size() );
	_ref_tvec.resize( _joints.size() );

	std::cout << "[servocontroller] INIT" << std::endl;

	Reset(0);

	return true;
    }

    virtual void Reset(int options)
    {
	//-- Initially, the reference positions should be set to the joints position
	//-- in order for the servos to stay in the initial position
	_ref_pos.resize( _probot->GetDOF());

	std::vector<dReal> angle;

	for (size_t i=0; i< _joints.size(); i++)
	{
	    _joints[i]->GetValues(angle);
	    _ref_pos[i]=angle[0];
	}

	//-- Default value of the Proportional controller KP constant
	_KP=8.3;
    }



    virtual void SimulationStep(dReal fElapsedTime)
    {
	std::vector<dReal> angles;
	std::vector<dReal> error( _probot->GetDOF() );
	std::vector<dReal> velocity( _probot->GetDOF() );

	stringstream is;
	stringstream os;

	is << "setvelocity ";

	//-- K controller for each joint
	for (size_t i=0; i<_joints.size(); i++)
	{
	    //-- Get current joint angles
	    _joints[i]->GetValues(angles);

	    //-- Calculate the distance to the reference position (error)
	    //-- and the desired velocity
	    error[i] = _ref_pos[i] - angles[0];
	    velocity[i] = error[i] * _KP;

	    //-- Limit the velocity to its maximum
	    dReal Maxvel = _joints[i]->GetMaxVel();

	    if (velocity[i] > Maxvel)
		velocity[i] = Maxvel;

	    if (velocity[i] < -Maxvel)
		velocity[i] = -Maxvel;

	    is << velocity[i] << " ";

	    //-- In recording mode, store the current sample
	    if (_recording)
	    {
		_phi_tvec[i].push_back(angles[0]);
		_ref_tvec[i].push_back(_ref_pos[i]);
	    }
	}

	//-- Set the joints velocities
	_pvelocitycontroller->SendCommand(os,is);

    }

    //-- Just a command test for debugging...
    bool Test(std::ostream& os, std::istream& is)
    {
	std::cout<<"[servocontroller] Test..." << std::endl;
	os << "Hello, world!";

	return true;
    }



    //-- Setpos command
    //------------------------------------------------
    //-- Set the reference position of ALL the servos.
    //-- Format: Setpos angle1 angle2 angle3 ... angleN
    //-- Note: joint angles are in degrees

    bool SetPos( std::ostream& os, std::istream& is)
    {
	for (size_t i = 0; i < _ref_pos.size(); i++)
	{
	    dReal pos;
	    is >> pos;

	    if (!is)
		return false;

	    //-- Store the reference positions (in radians)
	    _ref_pos[i]=pos*PI/180;
	}
	return true;
    }



    //-- Setpos1 command
    //-----------------------------------------------
    //-- Set the position of just 1 servo
    //-- Format: Setpos1 servo angle
    //-- Note: servo indices start at 0, and angle is given in degrees

    bool SetPos1(std::ostream& os, std::istream& is)
    {
	int servoIndex;
	dReal angle;

	//-- Get the values:
	is >> servoIndex;
	is >> angle;

	//-- Prevent invalid values
	if (servoIndex < 0 || servoIndex >= (int) _ref_pos.size() )
	    return false;

	//-- Store the reference positions in radians
	_ref_pos[servoIndex]=angle*PI/180;

	return true;
    }



    //-- Getpos command
    //-------------------------------------------------------
    //-- Return the position of all the servos in degrees
    //-- Format: Getpos

    bool GetPos(std::ostream& os, std::istream& is)
    {
      std::vector<dReal> anglesRad; //-- Angle in radians

      for(size_t i = 0; i < _ref_pos.size(); ++i)
      {
	//-- Get the angles of the ith joint in radians
	_joints[i]->GetValues(anglesRad);

	//-- Convert it to degrees and output it
	os << anglesRad[0]*180/PI << " ";
      }

      return true;
    }



    //-- Getpos1 command
    //-----------------------------------------------
    //-- Get the position of just 1 servo
    //-- Format: Setpos1 servo
    //-- Note: servo indices start at 0, and angle is
    //-- returned in degrees

    bool GetPos1(std::ostream& os, std::istream& is)
    {
	int servoIndex;
	std::vector<dReal> anglesRad;

	//-- Get the servo index
	is >> servoIndex;

	//-- Check if index is valid:
	if ( servoIndex > 0 && servoIndex < (int) _joints.size() )
	{
	    //-- Get the current joint angle
	    _joints[servoIndex]->GetValues(anglesRad);

	    //-- Convert it to degrees and output it
	    os << anglesRad[0]*180/PI << " ";
	}
	else
	{
	    return false;
	}

	return true;
    }



    //-- Record On command
    //-----------------------------------------------------
    //-- Start recording the servo position in the specified file.
    //-- Format: Record_on filepath
    //-- Note: It will generate an octave data file

    bool RecordOn(std::ostream& os, std::istream& is)
    {
	std::string fileName;

	//-- Check current status:
	if ( !_recording )
	{
	    if ( is )
	    {
		fileName.erase();
		is >> fileName;

		outDataFilePath = fileName + ".txt";
		outScriptFilePath = fileName + ".m";

		//-- Resize the data vectors
		for (size_t i=0; i<_joints.size(); i++)
		{
		  _phi_tvec[i].resize(0);
		  _ref_tvec[i].resize(0);
		}

		//-- Setting the recording mode on
		_recording=true;

		std::cout << "[servocontroller] RECORDING on: \"" << outDataFilePath << "\"\n";

		return true;

	    }
	}

	return false;

    }


    //-- Record Off command
    //--------------------------------------------------------
    //-- Stops recording the servo position. The output octave
    //-- data file will be actually generated
    //-- Format: Record_off

    bool RecordOff(std::ostream& os, std::istream& is)
    {

	if (_recording)
	{
	    //-- Open the files:
	    ofstream outDataFile( outDataFilePath.c_str() );
	    ofstream outScriptFile( outScriptFilePath.c_str());

	    //-- Restore state
	    _recording=false;

	    if ( outDataFile.is_open() && outScriptFile.is_open() )
	    {
		//-- Write the information in the output file
		generate_octave_data( outDataFile );
		generate_octave_file( outScriptFile, outDataFilePath  );

		//-- Close the file
		outDataFile.close();
		outScriptFile.close();

		std::cout << "[servocontroller] RECORDING off" << std::endl;
		std::cout << "[servocontroller] Max vel: " << _joints[0]->GetMaxVel() << std::endl;
		return true;
	    }
	    else
	    {
		//-- Error message
		std::cerr << "[servocontroller] Error opening the output file." << std::endl;

		std::cout << "[servocontroller] RECORD off" << std::endl;
		std::cout << "[servocontroller] Max vel: " << _joints[0]->GetMaxVel() << std::endl;
		return false;
	    }
	}

	return false;
    }


    //-- Other functions:
    virtual const std::vector<int>& GetControlDOFIndices() const { return _dofindices; }
    virtual int IsControlTransformation() const { return _nControlTransformation; }
    virtual bool SetDesired(const std::vector<dReal>& values, TransformConstPtr trans) { return false; }
    virtual bool SetPath(TrajectoryBaseConstPtr ptraj) { Reset(0); return false; }
    virtual RobotBasePtr GetRobot() const  {return _probot;}
    virtual bool IsDone()  { return false; }
    virtual dReal GetTime() const { return 0; }

private:

    void generate_octave_data(ofstream& outFile)
    {
	//-- Print file header:
	outFile << "# Created by openMR plugin for openRAVE [ http://www.iearobotics.com/wiki/index.php?title=OpenMR:_Modular_Robots_plug-in_for_Openrave ]"
		<< std::endl;

	//-- Data matrix for the phi (actual angle of the joints):
	//-- Each row is a servo, each column is a sample
	outFile << "# name: phi" << std::endl;
	outFile << "# type: matrix" << std::endl;
	outFile << "# rows: " << _phi_tvec.size() << std::endl;
	outFile << "# columns: " << _phi_tvec[0].size() << std::endl;

	for (size_t i =0; i < _phi_tvec.size(); i++)
	{
	    for ( size_t j = 0; j < _phi_tvec[0].size() ; j ++)
	    {
		//-- Save the angles in degrees
		outFile << _phi_tvec[i][j]*180/PI << " ";
	    }

	    outFile << std::endl;
	}

	//-- Data matrix for ref (reference angle of the joints):
	//-- Each row is a servo, each column is a sample
	outFile << "# name: ref" << std::endl;
	outFile << "# type: matrix" << std::endl;
	outFile << "# rows: " << _ref_tvec.size() << std::endl;
	outFile << "# columns: " << _ref_tvec[0].size() << std::endl;

	for (size_t i =0; i < _ref_tvec.size(); i++)
	{
	    for ( size_t j = 0; j < _ref_tvec[0].size() ; j ++)
	    {
		//-- Save the angles in degrees
		outFile << _ref_tvec[i][j]*180/PI << " ";
	    }

	    outFile << std::endl;
	}

	//-- Data matrix for the simulation time
	outFile << "# name: time" << std::endl;
	outFile << "# type: matrix" << std::endl;
	outFile << "# rows: 1" << std::endl;
	outFile << "# columns: " << _phi_tvec[0].size() << std::endl;

	for( size_t i = 0; i < _phi_tvec[0].size(); i++)
	    outFile << i << " ";

    }

    void generate_octave_file( ofstream& outFile, std::string& dataFilePath )
    {
	//-- Print header
	outFile << "# Created by openMR plugin for openRAVE [ http://www.iearobotics.com/wiki/index.php?title=OpenMR:_Modular_Robots_plug-in_for_Openrave ]"
		<< std::endl << std::endl;

	//-- Load data file
	outFile << "# Load data file:" << std::endl;
	outFile << "load('" << dataFilePath << "');" << std::endl << std::endl;


	//-- Plotting things

	//-- Plot the actual positions of the servos
	outFile <<  "# Plot servo actual angles" << std::endl;
	outFile << "plot(";
	for (size_t i=0; i<_phi_tvec.size(); i++)
	{
	    outFile << "time,phi(" << i+1 << ",:), '-'";

	    //-- Add a ',' except for the last element
	    if (i<_phi_tvec.size()-1)
		outFile << ",";
	}
	outFile << ");" << std::endl << endl;

	//-- Plot the reference positions of the servos
	outFile << "# Plot servo reference angles" << std::endl;
	outFile << "hold on;";
	outFile << "plot(";

	for (size_t i=0; i<_ref_tvec.size(); i++)
	{
	    outFile << "time,ref(" << i+1 << ",:),'-'";

	    //-- Add a ',' except for the last element
	    if (i<_ref_tvec.size()-1)
		outFile << ",";
	}

	outFile << ");" << endl << std::endl;


	//-- Add the legends and other nice things
	outFile << "# The following lines make the graph look nice" << std::endl;
	outFile << "legend(";

	for (size_t s=0; s<_phi_tvec.size(); s++)
	{
	    outFile << "'Servo " << s+1 << "'";

	    //-- Add a ',' except for the last element
	    if (s<_phi_tvec.size()-1)
		outFile << ",";
	}
	outFile << ");" << std::endl;

	//-- Formatting the graph
	outFile << "grid on;" << endl;
	outFile << "title('Servo angles')" << endl;
	outFile << "xlabel('Simulation time')" << endl;
	outFile << "ylabel('Angle (degrees)')" << endl;
	outFile << "axis([0," << _phi_tvec[0].size()-1 << ",-90, 90])" << endl;
	outFile << "pause;" << endl;
    }



    //-- CAUTION!!
    //-- This function is no longer used (deprecated)
    void generate_octave_file( ofstream& outFile)
    {

	//-- Size of the sample population
	size_t size = _phi_tvec[0].size();
	std::cout << "[servocontroller] Size: " << size << std::endl;

	//-- Servos angle
	for (size_t s=0; s<_phi_tvec.size(); s++)
	{
	    outFile << "phi" << s <<"=[";

	    for (size_t t=0; t<size; t++)
	    {
		outFile << _phi_tvec[s][t]*180/PI << ",";
	    }

	    outFile << "];" << endl;
	}

	//-- Reference positions
	for (size_t s=0; s<_ref_tvec.size(); s++)
	{
	    outFile << "ref" << s <<"=[";

	    for (size_t t=0; t<size; t++)
	    {
		outFile << _ref_tvec[s][t]*180/PI << ",";
	    }

	    outFile << "];" << endl;
	}

	//-- Time
	outFile << "t=[0:1:" << size-1 << "];" << endl;


	//-- Plot the servo angles
	outFile << "plot(";
	for (size_t s=0; s<_phi_tvec.size(); s++)
	{
	    outFile << "t,phi" << s << ",'-'";

	    //-- Add a ',' except for the last element
	    if (s<_phi_tvec.size()-1)
		outFile << ",";
	}
	outFile << ");" << endl;

	//-- Plot the reference positions
	outFile << "hold on;";
	outFile << "plot(";

	for (size_t s=0; s<_ref_tvec.size(); s++)
	{
	    outFile << "t,ref" << s << ",'-'";

	    //-- Add a ',' except for the last element
	    if (s<_ref_tvec.size()-1)
		outFile << ",";
	}

	outFile << ");" << endl;


	//-- Add the legends
	outFile << "legend(";

	for (size_t s=0; s<_phi_tvec.size(); s++)
	{
	    outFile << "'Servo " << s << "'";

	    //-- Add a ',' except for the last element
	    if (s<_phi_tvec.size()-1)
		outFile << ",";
	}
	outFile << ");" << std::endl;

	//-- Formatting the graph
	outFile << "grid on;" << endl;
	outFile << "title('Servos angle')" << endl;
	outFile << "xlabel('Simulation time')" << endl;
	outFile << "ylabel('Angle (degrees)')" << endl;
	outFile << "axis([0," << size-1 << ",-90, 90])" << endl;
	outFile << "pause;" << endl;
    }

protected:
    EnvironmentBasePtr _penv;
    RobotBasePtr _probot;
    std::vector<int> _dofindices;
    int _nControlTransformation;

    ControllerBasePtr _pvelocitycontroller;
    std::vector<KinBody::JointPtr> _joints;
    std::vector<dReal> _ref_pos;		//-- Reference positions (in radians)
    dReal _KP;				//-- P controller KP constant

    //-- For recording...
    std::string outDataFilePath;         //-- Path to the file for storing the servo positions
    std::string outScriptFilePath;	 //-- Path to the script for printing the beautiful graphs
    bool _recording;			 //-- Recording mode state
    std::vector<tvector> _phi_tvec;	 //-- Temporary storage for the servo's angles in time
    std::vector<tvector> _ref_tvec;       //-- Temporary storage for the servo's reference positions in time

};


#endif
