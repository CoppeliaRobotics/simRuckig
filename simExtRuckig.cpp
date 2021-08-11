#include "simExtRuckig.h"
#include "simLib.h"
#include <iostream>
#include <vector>

#include <ruckig/ruckig.hpp>
using namespace ruckig;

#ifdef _WIN32
    #include <shlwapi.h>
    #pragma comment(lib, "Shlwapi.lib")
#else
    #include <unistd.h>
#endif

#define PLUGIN_VERSION 1 // 1 since CoppeliaSim V4.3.0

struct SObj
{
	Ruckig<DynamicDOFs> *ruckig;
	InputParameter<DynamicDOFs> *input;
	OutputParameter<DynamicDOFs> *output;

    int scriptHandle;
	int objectHandle;
	int dofs;
	double smallestTimeStep;
};

static int nextObjectHandle=0;
static std::vector<SObj> allObjects;
static LIBRARY simLib;

SIM_DLLEXPORT unsigned char simStart(void* reservedPointer,int reservedInt)
{
	char curDirAndFile[1024];
#ifdef _WIN32
	GetModuleFileName(NULL,curDirAndFile,1023);
	PathRemoveFileSpec(curDirAndFile);
#elif defined (__linux) || defined (__APPLE__)
	getcwd(curDirAndFile, sizeof(curDirAndFile));
#endif
	std::string currentDirAndPath(curDirAndFile);
	std::string temp(currentDirAndPath);
#ifdef _WIN32
	temp+="\\coppeliaSim.dll";
#elif defined (__linux)
	temp+="/libcoppeliaSim.so";
#elif defined (__APPLE__)
	temp+="/libcoppeliaSim.dylib";
#endif
	simLib=loadSimLibrary(temp.c_str());
	if (simLib==NULL)
	{
        printf("simExtRuckig: error: could not find or correctly load the CoppeliaSim library. Cannot start the plugin.\n"); // cannot use simAddLog here.
        return(0);
	}
	if (getSimProcAddresses(simLib)==0)
	{
        printf("simExtRuckig: error: could not find all required functions in the CoppeliaSim library. Cannot start the plugin.\n"); // cannot use simAddLog here.
		unloadSimLibrary(simLib);
        return(0);
	}
    return(PLUGIN_VERSION);
}

SIM_DLLEXPORT void simEnd()
{
    unloadSimLibrary(simLib);
}

SIM_DLLEXPORT void* simMessage(int message,int* auxiliaryData,void* customData,int* replyData)
{
	void* retVal=NULL;

    if (message==sim_message_eventcallback_scriptstatedestroyed)
    {
        for (size_t i=0;i<allObjects.size();i++)
        {
            if (allObjects[i].scriptHandle==auxiliaryData[0])
            {
                delete allObjects[i].ruckig;
                delete allObjects[i].input;
                delete allObjects[i].output;
                allObjects.erase(allObjects.begin()+i);
                break;
            }
        }
    }

    return(retVal);
}

SIM_DLLEXPORT int ruckigPlugin_pos(int scriptHandle,int dofs,double smallestTimeStep,int flags,const double* currentPos,const double* currentVel,const double* currentAccel,const double* maxVel,const double* maxAccel,const double* maxJerk,const unsigned char* selection,const double* targetPos,const double* targetVel)
{
    SObj obj;
    obj.scriptHandle=scriptHandle;
    obj.objectHandle=nextObjectHandle++;

    obj.dofs=dofs;
    obj.smallestTimeStep=smallestTimeStep;

    obj.ruckig = new Ruckig<DynamicDOFs>(dofs,smallestTimeStep);
    obj.input = new InputParameter<DynamicDOFs>(dofs);
    obj.output = new OutputParameter<DynamicDOFs>(dofs);

    for (int i=0;i<dofs;i++)
    {
        obj.input->current_position[i]=currentPos[i];
        obj.input->current_velocity[i]=currentVel[i];
        obj.input->current_acceleration[i]=currentAccel[i];
        obj.input->enabled[i]=selection[i]!=0;
        obj.input->target_position[i]=targetPos[i];
        obj.input->target_velocity[i]=targetVel[i];
    }

    for (int i=0;i<dofs;i++)
    { // for now no min/max values
        obj.input->max_velocity[i]=maxVel[i];
        obj.input->max_acceleration[i]=maxAccel[i];
        obj.input->max_jerk[i]=maxJerk[i];
    }

    if (flags>=0)
    { // we don't have default values!
        if ( ((flags&3)==sim_ruckig_phasesync)||((flags&3)==simrml_only_phase_sync) )
            obj.input->synchronization=Synchronization::Phase; // default
        if ((flags&3)==sim_ruckig_timesync)
            obj.input->synchronization=Synchronization::Time;
        if ((flags&3)==sim_ruckig_nosync)
            obj.input->synchronization=Synchronization::None;
    }

    allObjects.push_back(obj);
    return(obj.objectHandle);
}

SIM_DLLEXPORT int ruckigPlugin_vel(int scriptHandle,int dofs,double smallestTimeStep,int flags,const double* currentPos,const double* currentVel,const double* currentAccel,const double* maxAccel,const double* maxJerk,const unsigned char* selection,const double* targetVel)
{
    SObj obj;
    obj.scriptHandle=scriptHandle;
    obj.objectHandle=nextObjectHandle++;

    obj.dofs=dofs;
    obj.smallestTimeStep=smallestTimeStep;

    obj.ruckig = new Ruckig<DynamicDOFs>(dofs, smallestTimeStep);
    obj.input = new InputParameter<DynamicDOFs>(dofs);
    obj.output = new OutputParameter<DynamicDOFs>(dofs);

    obj.input->control_interface = ControlInterface::Velocity;

    for (int i=0;i<dofs;i++)
    {
        obj.input->current_position[i]=currentPos[i];
        obj.input->current_velocity[i]=currentVel[i];
        obj.input->current_acceleration[i]=currentAccel[i];
        obj.input->enabled[i]=selection[i]!=0;
        obj.input->target_velocity[i]=targetVel[i];
    }

    for (int i=0;i<dofs;i++)
    { // for now no min/max values
        obj.input->max_acceleration[i]=maxAccel[i];
        obj.input->max_jerk[i]=maxJerk[i];
    }

    if (flags>=0)
    { // we don't have default values!
        if ( ((flags&3)==sim_ruckig_phasesync)||((flags&3)==simrml_only_phase_sync) )
            obj.input->synchronization=Synchronization::Phase; // default
        if ((flags&3)==sim_ruckig_timesync)
            obj.input->synchronization=Synchronization::Time;
        if ((flags&3)==sim_ruckig_nosync)
            obj.input->synchronization=Synchronization::None;
    }

    allObjects.push_back(obj);
    return(obj.objectHandle);
}

SIM_DLLEXPORT int ruckigPlugin_step(int objHandle,double timeStep,double* newPos,double* newVel,double* newAccel,double* syncTime)
{
    int index=-1;
    bool ruckigPos=true;
    int retVal=-1;
    for (int i=0;i<int(allObjects.size());i++)
    {
        if (allObjects[i].objectHandle==objHandle)
        {
            ruckigPos=(allObjects[i].input->control_interface == ControlInterface::Position);
            index=i;
            break;
        }
    }
    if (index!=-1)
    {
        int dofs=allObjects[index].dofs;
        int cnt=int((timeStep/allObjects[index].smallestTimeStep)+0.5);
        if (ruckigPos)
        {
            for (int i=0;i<cnt;i++)
            {
                retVal=allObjects[index].ruckig->update(*allObjects[index].input,*allObjects[index].output);

                for (int j=0;j<dofs;j++)
                {
                    allObjects[index].input->current_position[j]=allObjects[index].output->new_position[j];
                    allObjects[index].input->current_velocity[j]=allObjects[index].output->new_velocity[j];
                    allObjects[index].input->current_acceleration[j]=allObjects[index].output->new_acceleration[j];
                }

                if (retVal!=0)
                    break;
            }

            for (int i=0;i<dofs;i++)
            {
                newPos[i]=allObjects[index].output->new_position[i];
                newVel[i]=allObjects[index].output->new_velocity[i];
                newAccel[i]=allObjects[index].output->new_acceleration[i];
            }

            syncTime[0]=allObjects[index].output->trajectory.get_duration();
        }
        else
        {
            for (int i=0;i<cnt;i++)
            {
                retVal=allObjects[index].ruckig->update(*allObjects[index].input,*allObjects[index].output);

                for (int j=0;j<dofs;j++)
                {
                    allObjects[index].input->current_position[j]=allObjects[index].output->new_position[j];
                    allObjects[index].input->current_velocity[j]=allObjects[index].output->new_velocity[j];
                    allObjects[index].input->current_acceleration[j]=allObjects[index].output->new_acceleration[j];
                }

                if (retVal!=0)
                    break;
            }

            for (int i=0;i<dofs;i++)
            {
                newPos[i]=allObjects[index].output->new_position[i];
                newVel[i]=allObjects[index].output->new_velocity[i];
                newAccel[i]=allObjects[index].output->new_acceleration[i];
            }

            syncTime[0]=allObjects[index].output->trajectory.get_duration();
        }
    }
    return(retVal);
}

SIM_DLLEXPORT int ruckigPlugin_remove(int objHandle)
{
    int retVal=-1;
    for (size_t i=0;i<allObjects.size();i++)
    {
        if (allObjects[i].objectHandle==objHandle)
        {
            delete allObjects[i].ruckig;
            delete allObjects[i].input;
            delete allObjects[i].output;
            allObjects.erase(allObjects.begin()+i);
            retVal=1;
            break;
        }
    }
    return(retVal);
}

SIM_DLLEXPORT int ruckigPlugin_dofs(int objHandle)
{
    for (size_t i=0;i<allObjects.size();i++)
    {
        if (allObjects[i].objectHandle==objHandle)
            return(allObjects[i].dofs);
    }
    return(-1);
}
