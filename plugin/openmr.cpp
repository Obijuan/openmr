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


// called to create a new plugin
InterfaceBasePtr CreateInterfaceValidated(InterfaceType type, const std::string& interfacename, std::istream& sinput, EnvironmentBasePtr penv)
{
    if( type == PT_Controller && interfacename == "servocontroller" ) {
	return InterfaceBasePtr(new ServoController(penv,sinput));
    }

    if( type == PT_Controller && interfacename == "sinoscontroller") {
	return InterfaceBasePtr( new SinosController( penv, sinput));
    }

    return InterfaceBasePtr();
}

// called to query available plugins
void GetPluginAttributesValidated(PLUGININFO& info)
{
info.interfacenames[PT_Controller].push_back("servocontroller");
info.interfacenames[PT_Controller].push_back("sinoscontroller");

}

// called before plugin is terminated
OPENRAVE_PLUGIN_API void DestroyPlugin()
{
}
