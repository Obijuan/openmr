
#include <openrave/openrave.h>
#include <openrave/plugin.h>
#include <boost/bind.hpp>

#include "servocontroller.h"
#include "sinoscontroller.h"

using namespace OpenRAVE;

class openMR : public ModuleBase
{
public:
    openMR(EnvironmentBasePtr penv, std::istream& ss) : ModuleBase(penv) {
	RegisterCommand("HelloWorld",boost::bind(&openMR::HelloWorld,this,_1,_2),
			"This is the basic example");
    }
    virtual ~openMR() {}
    
    bool HelloWorld(std::ostream& sout, std::istream& sinput)
    {
        std::string input;
        sinput >> input;
	sout << "Hello World\n";
        return true;
    }
};
