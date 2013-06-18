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

// called to create a new plugin
InterfaceBasePtr CreateInterfaceValidated(InterfaceType type, const std::string& interfacename, std::istream& sinput, EnvironmentBasePtr penv)
{
    if( type == PT_Controller && interfacename == "servocontroller" ) {
	return InterfaceBasePtr(new ServoController(penv,sinput));
    }

    if( type == PT_Controller && interfacename == "sinoscontroller") {
	return InterfaceBasePtr( new SinosController( penv, sinput));
    }

    if( type == PT_Module && interfacename == "openMR" ) {
	return InterfaceBasePtr(new openMR(penv,sinput));
    }

    return InterfaceBasePtr();
}

// called to query available plugins
void GetPluginAttributesValidated(PLUGININFO& info)
{
info.interfacenames[PT_Module].push_back("openMR");
info.interfacenames[PT_Controller].push_back("servocontroller");
info.interfacenames[PT_Controller].push_back("sinoscontroller");

}

// called before plugin is terminated
OPENRAVE_PLUGIN_API void DestroyPlugin()
{
}
