#include "openmr.h"

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

