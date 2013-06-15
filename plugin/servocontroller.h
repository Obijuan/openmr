
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

	this->env = penv;
    }


    virtual ~ServoController() {}
    
    virtual bool Init(RobotBasePtr robot, const std::vector<int>& dofindices, int nControlTransformation)
    {
	this->robot = robot;
	this->dofindices = dofindices;
	this->nControlTransformation = nControlTransformation;

	//-- Initialize odevelocity controller
	velocitycontroller = RaveCreateController( env, "odevelocity");
	velocitycontroller->Init( robot, dofindices, nControlTransformation );

	//-- Get the robot joints. Needed for obtaining the joint angles and maxvelocities.
	std::vector<KinBodyPtr> bodies;
	env->GetBodies( bodies );
	joints = bodies[0]->GetJoints();

	//-- Recording Mode
	//----------------------------------
	recording = false;

	phi_tvec.resize( joints.size() );
	ref_tvec.resize( joints.size() );

	std::cout << "[servocontroller] INIT" << std::endl;

	Reset(0);

	return true;
    }

    virtual void Reset(int options)
    {
	//-- Initially, the reference positions should be set to the joints position
	//-- in order for the servos to stay in the initial position
	ref_pos.resize( robot->GetDOF());

	std::vector<dReal> angle;

	for (size_t i=0; i< joints.size(); i++)
	{
	    joints[i]->GetValues(angle);
	    ref_pos[i]=angle[0];
	}

	//-- Default value of the Proportional controller KP constant
	KP=8.3;
    }



    virtual void SimulationStep(dReal fElapsedTime)
    {
	std::vector<dReal> angles;
	std::vector<dReal> error( robot->GetDOF() );
	std::vector<dReal> velocity( robot->GetDOF() );

	stringstream is;
	stringstream os;

	is << "setvelocity ";

	//-- K controller for each joint
	for (size_t i=0; i<joints.size(); i++)
	{
	    //-- Get current joint angles
	    joints[i]->GetValues(angles);

	    //-- Calculate the distance to the reference position (error)
	    //-- and the desired velocity
	    error[i] = ref_pos[i] - angles[0];
	    velocity[i] = error[i] * KP;

	    //-- Limit the velocity to its maximum
	    dReal Maxvel = joints[i]->GetMaxVel();

	    if (velocity[i] > Maxvel)
		velocity[i] = Maxvel;

	    if (velocity[i] < -Maxvel)
		velocity[i] = -Maxvel;

	    is << velocity[i] << " ";

	    //-- In recording mode, store the current sample
	    if (recording)
	    {
		phi_tvec[i].push_back(angles[0]);
		ref_tvec[i].push_back(ref_pos[i]);
	    }
	}

	//-- Set the joints velocities
	velocitycontroller->SendCommand(os,is);

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
	for (size_t i = 0; i < ref_pos.size(); i++)
	{
	    dReal pos;
	    is >> pos;

	    if (!is)
		return false;

	    //-- Store the reference positions (in radians)
	    ref_pos[i]=pos*PI/180;
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
	if (servoIndex < 0 || servoIndex >= ref_pos.size() )
	    return false;

	//-- Store the reference positions in radians
	ref_pos[servoIndex]=angle*PI/180;

	return true;
    }



    //-- Getpos command
    //-------------------------------------------------------
    //-- Return the position of all the servos in degrees
    //-- Format: Getpos

    bool GetPos(std::ostream& os, std::istream& is)
    {
      std::vector<dReal> anglesRad; //-- Angle in radians

      for(size_t i = 0; i < ref_pos.size(); ++i)
      {
	//-- Get the angles of the ith joint in radians
	joints[i]->GetValues(anglesRad);

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
	if ( servoIndex > 0 && servoIndex < joints.size() )
	{
	    //-- Get the current joint angle
	    joints[servoIndex]->GetValues(anglesRad);

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
	//-- Check current status:
	if ( !recording )
	{
	    if ( is )
	    {
		outFilePath.erase();
		is >> outFilePath;

		//-- Resize the data vectors
		for (size_t i=0; i<joints.size(); i++)
		{
		  phi_tvec[i].resize(0);
		  ref_tvec[i].resize(0);
		}

		//-- Setting the recording mode on
		recording=true;

		std::cout << "[servocontroller] RECORD on:" << outFilePath << "\n";

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

	if (recording)
	{
	    //-- Open the file:
	    ofstream outFile( outFilePath.c_str() );

	    if ( outFile.is_open() )
	    {
		//-- Write the information in the output file
		generate_octave_file( outFile );

		//-- Close the file
		outFile.close();

		//-- Restore state
		recording=false;

		std::cout << "[servocontroller] RECORD off" << std::endl;
		std::cout << "[servocontroller] Max vel: " << joints[0]->GetMaxVel() << std::endl;
		return true;
	    }
	    else
	    {
		//-- Error message
		std::cerr << "[servocontroller] Error opening the output file." << std::endl;

		//-- Restore state
		recording=false;

		std::cout << "[servocontroller] RECORD off" << std::endl;
		std::cout << "[servocontroller] Max vel: " << joints[0]->GetMaxVel() << std::endl;
		return false;
	    }
	}

	return false;
    }


    //-- Other functions:
    virtual const std::vector<int>& GetControlDOFIndices() const { return dofindices; }
    virtual int IsControlTransformation() const { return nControlTransformation; }
    virtual bool SetDesired(const std::vector<dReal>& values, TransformConstPtr trans) { return false; }
    virtual bool SetPath(TrajectoryBaseConstPtr ptraj) { Reset(0); return false; }
    virtual RobotBasePtr GetRobot() const  {return robot;}
    virtual bool IsDone()  { return false; }
    virtual dReal GetTime() const { return 0; }

private:

  void generate_octave_file( ofstream& outFile)
  {
	//-- TODO: meter mano a esto para que saque un archivo de DATOS con el formato de octave

	//-- Size of the sample population
	size_t size = phi_tvec[0].size();
	std::cout << "[servocontroller] Size: " << size << std::endl;

	//-- Servos angle
	for (size_t s=0; s<phi_tvec.size(); s++)
	{
	    outFile << "phi" << s <<"=[";

	    for (size_t t=0; t<size; t++)
	    {
		outFile << phi_tvec[s][t]*180/PI << ",";
	    }

	    outFile << "];" << endl;
	}

	//-- Reference positions
	for (size_t s=0; s<ref_tvec.size(); s++)
	{
	    outFile << "ref" << s <<"=[";

	    for (size_t t=0; t<size; t++)
	    {
		outFile << ref_tvec[s][t]*180/PI << ",";
	    }

	    outFile << "];" << endl;
	 }

	//-- Time
	outFile << "t=[0:1:" << size-1 << "];" << endl;


	//-- Plot the servo angles
	outFile << "plot(";
	for (size_t s=0; s<phi_tvec.size(); s++)
	{
	    outFile << "t,phi" << s << ",'-'";

	    //-- Add a ',' except for the last element
	    if (s<phi_tvec.size()-1)
		outFile << ",";
	}
	outFile << ");" << endl;

	//-- Plot the reference positions
	outFile << "hold on;";
	outFile << "plot(";

	for (size_t s=0; s<ref_tvec.size(); s++)
	{
	    outFile << "t,ref" << s << ",'-'";

	    //-- Add a ',' except for the last element
	    if (s<ref_tvec.size()-1)
		outFile << ",";
	}

	outFile << ");" << endl;


	//-- Add the legends
	outFile << "legend(";

	for (size_t s=0; s<phi_tvec.size(); s++)
	{
	    outFile << "'Servo " << s << "'";

	    //-- Add a ',' except for the last element
	    if (s<phi_tvec.size()-1)
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
    EnvironmentBasePtr env;
    RobotBasePtr robot;
    std::vector<int> dofindices;
    int nControlTransformation;

    ControllerBasePtr velocitycontroller;
    std::vector<KinBody::JointPtr> joints;
    std::vector<dReal> ref_pos;		//-- Reference positions (in radians)
    dReal KP;				//-- P controller KP constant

    //-- For recording...
    std::string outFilePath;             //-- Path to the file for storing the servo positions
    bool recording;			 //-- Recording mode state
    std::vector<tvector> phi_tvec;	 //-- Temporary storage for the servo's angles in time
    std::vector<tvector> ref_tvec;       //-- Temporary storage for the servo's reference positions in time

};


