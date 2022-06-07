#include <UnicycleMotionControl/simExtPluginUnicycleMotionControl.h>
#include <UnicycleMotionControl/simLib.h>

#include <UnicycleMotionControl/CoppeliaSimController.hpp>
#include <UnicycleMotionControl/CoppeliaSimP3DXController.hpp>

#include <iostream>
#include <memory>


#ifdef _WIN32
    #ifdef QT_COMPIL
        #include <direct.h>
    #else
        #include <shlwapi.h>
        #pragma comment(lib, "Shlwapi.lib")
    #endif
#endif

#if defined(__linux) || defined(__APPLE__)
    #include <unistd.h>
    #include <string.h>
    #define _stricmp(x,y) strcasecmp(x,y)
#endif

#define CONCAT(x,y,z) x y z
#define strConCat(x,y,z)    CONCAT(x,y,z)

#define PLUGIN_VERSION 5 // 2 since version 3.2.1, 3 since V3.3.1, 4 since V3.4.0, 5 since V3.4.1

static LIBRARY simLib; // the CoppelisSim library that we will dynamically load and bind

std::unique_ptr<labrob::CoppeliaSimController> coppeliasim_controller;

void init() {
  coppeliasim_controller = std::make_unique<labrob::CoppeliaSimP3DXController>();
  coppeliasim_controller->init();
}

void update() {
  coppeliasim_controller->update();
}

// This is the plugin start routine (called just once, just after the plugin was loaded):
SIM_DLLEXPORT unsigned char simStart(void* reservedPointer,int reservedInt)
{
  // Dynamically load and bind CoppelisSim functions:
  // 1. Figure out this plugin's directory:
  char curDirAndFile[1024];
#ifdef _WIN32
  #ifdef QT_COMPIL
    _getcwd(curDirAndFile, sizeof(curDirAndFile));
  #else
    GetModuleFileName(NULL,curDirAndFile,1023);
    PathRemoveFileSpec(curDirAndFile);
  #endif
#else
  getcwd(curDirAndFile, sizeof(curDirAndFile));
#endif

  std::string currentDirAndPath(curDirAndFile);
  // 2. Append the CoppelisSim library's name:
  std::string temp(currentDirAndPath);
#ifdef _WIN32
  temp+="\\coppeliaSim.dll";
#elif defined (__linux)
  temp+="/libcoppeliaSim.so";
#elif defined (__APPLE__)
  temp+="/libcoppeliaSim.dylib";
#endif /* __linux || __APPLE__ */
  // 3. Load the CoppelisSim library:
  simLib=loadSimLibrary(temp.c_str());
  if (simLib==NULL) {
    printf("simExtPluginSkeleton: error: could not find or correctly load the CoppeliaSim library. Cannot start the plugin.\n"); // cannot use simAddLog here.
    return(0); // Means error, CoppelisSim will unload this plugin
  }
  if (getSimProcAddresses(simLib)==0) {
    printf("simExtPluginSkeleton: error: could not find all required functions in the CoppeliaSim library. Cannot start the plugin.\n"); // cannot use simAddLog here.
    unloadSimLibrary(simLib);
    return(0); // Means error, CoppelisSim will unload this plugin
  }

  // Check the version of CoppelisSim:
  int simVer,simRev;
  simGetIntegerParameter(sim_intparam_program_version,&simVer);
  simGetIntegerParameter(sim_intparam_program_revision,&simRev);
  if( (simVer<40000) || ((simVer==40000)&&(simRev<1)) ) {
    simAddLog("PluginSkeleton",sim_verbosity_errors,"sorry, your CoppelisSim copy is somewhat old, CoppelisSim 4.0.0 rev1 or higher is required. Cannot start the plugin.");
    unloadSimLibrary(simLib);
    return(0); // Means error, CoppelisSim will unload this plugin
  }

  // Implicitely include the script lua/simExtPluginSkeleton.lua:
  simRegisterScriptVariable("simSkeleton","require('simExtPluginSkeleton')",0);

  // Register the new function:
  //simRegisterScriptCallbackFunction(strConCat(LUA_GETDATA_COMMAND,"@","PluginSkeleton"),strConCat("...=",LUA_GETDATA_COMMAND,"(string data1,map data2)"),LUA_GETDATA_CALLBACK);

  init();

  return(PLUGIN_VERSION); // initialization went fine, we return the version number of this plugin (can be queried with simGetModuleName)
}

// This is the plugin end routine (called just once, when CoppelisSim is ending, i.e. releasing this plugin):
SIM_DLLEXPORT void simEnd()
{
  // Here you could handle various clean-up tasks

  unloadSimLibrary(simLib); // release the library
}

// This is the plugin messaging routine (i.e. CoppelisSim calls this function very often, with various messages):
SIM_DLLEXPORT void* simMessage(int message,int* auxiliaryData,void* customData,int* replyData)
{ // This is called quite often. Just watch out for messages/events you want to handle
  void* retVal=NULL;

  // Here we can intercept many messages from CoppelisSim. Only the most important messages are listed here.
  // For a complete list of messages that you can intercept/react with, search for "sim_message_eventcallback"-type constants
  // in the CoppelisSim user manual.

  if (message == sim_message_eventcallback_modulehandle) {
    update();
  }

  return(retVal);
}

