// Copyright (C) 2010 Juan Gonzalez (juan@iearobotics.com)
// 
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
#include "plugindefs.h"
#include <rave/plugin.h>

InterfaceBasePtr CreateInterfaceValidated(InterfaceType type, const std::string& interfacename, std::istream& sinput, EnvironmentBasePtr penv)
{
    switch(type) {
    case PT_Controller:
        if( interfacename == "servocontroller")
            return InterfaceBasePtr(new ServoController(penv));
        else if( interfacename == "sinoscontroller" )
            return InterfaceBasePtr(new SinosController(penv));
        break;
    default:
        break;
    }
    return InterfaceBasePtr();
}


void GetPluginAttributesValidated(PLUGININFO& info)
{
    info.interfacenames[PT_Controller].push_back("ServoController");
    info.interfacenames[PT_Controller].push_back("SinosController");
}


OPENRAVE_PLUGIN_API void DestroyPlugin()
{
}
