#include "simExtRuckig.h"
#include "simLib.h"
#include <iostream>
#include <vector>
#include <map>

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
	int dofs;
	double smallestTimeStep;
    double timeLeft;
    bool first;
};

static int nextObjectHandle=0;
static std::map<int,SObj> allObjects;
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
        std::vector<int> toErase;
        auto it=allObjects.begin();
        while (it!=allObjects.end())
        {
            if (it->second.scriptHandle==auxiliaryData[0])
                toErase.push_back(it->first);
            ++it;
        }
        for (size_t i=0;i<toErase.size();i++)
        {
            auto it=allObjects.find(toErase[i]);
            delete it->second.ruckig;
            delete it->second.input;
            delete it->second.output;
            allObjects.erase(it);
        }
    }

    return(retVal);
}

SIM_DLLEXPORT int ruckigPlugin_pos(int scriptHandle,int dofs,double smallestTimeStep,int flags,const double* currentPos,const double* currentVel,const double* currentAccel,const double* maxVel,const double* maxAccel,const double* maxJerk,const unsigned char* selection,const double* targetPos,const double* targetVel)
{
    SObj obj;
    obj.scriptHandle=scriptHandle;

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

    bool usesMinVel=false;
    bool usesMinAccel=false;
    if (flags>=0)
    { // we don't have default values!
        if ( ((flags&3)==sim_ruckig_phasesync)||((flags&3)==simrml_only_phase_sync) )
            obj.input->synchronization=Synchronization::Phase; // default
        if ((flags&3)==sim_ruckig_timesync)
            obj.input->synchronization=Synchronization::Time;
        if ((flags&3)==sim_ruckig_nosync)
            obj.input->synchronization=Synchronization::None;
        usesMinVel=((flags&sim_ruckig_minvel)!=0);
        usesMinAccel=((flags&sim_ruckig_minaccel)!=0);
    }

    std::vector<double> minVel;
    std::vector<double> minAcc;
    for (int i=0;i<dofs;i++)
    {
        obj.input->max_velocity[i]=maxVel[i];
        if (usesMinVel)
            minVel.push_back(maxVel[dofs+i]);
        obj.input->max_acceleration[i]=maxAccel[i];
        if (usesMinAccel)
            minAcc.push_back(maxAccel[dofs+i]);
        obj.input->max_jerk[i]=maxJerk[i];
    }
    if (usesMinVel)
        obj.input->min_velocity=std::make_optional(minVel);
    if (usesMinAccel)
        obj.input->min_acceleration=std::make_optional(minAcc);

    obj.first=true;

    int objectHandle=0;
    if (nextObjectHandle<2000000000)
        objectHandle=nextObjectHandle++;
    else
    {
        while (allObjects.find(objectHandle)!=allObjects.end())
            objectHandle++;
    }
    allObjects[objectHandle]=obj;
    return(objectHandle);
}

SIM_DLLEXPORT int ruckigPlugin_vel(int scriptHandle,int dofs,double smallestTimeStep,int flags,const double* currentPos,const double* currentVel,const double* currentAccel,const double* maxAccel,const double* maxJerk,const unsigned char* selection,const double* targetVel)
{
    SObj obj;
    obj.scriptHandle=scriptHandle;

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

    bool usesMinAccel=false;
    if (flags>=0)
    { // we don't have default values!
        if ( ((flags&3)==sim_ruckig_phasesync)||((flags&3)==simrml_only_phase_sync) )
            obj.input->synchronization=Synchronization::Phase; // default
        if ((flags&3)==sim_ruckig_timesync)
            obj.input->synchronization=Synchronization::Time;
        if ((flags&3)==sim_ruckig_nosync)
            obj.input->synchronization=Synchronization::None;
        usesMinAccel=((flags&sim_ruckig_minaccel)!=0);
    }

    std::vector<double> minAcc;
    for (int i=0;i<dofs;i++)
    {
        obj.input->max_acceleration[i]=maxAccel[i];
        if (usesMinAccel)
            minAcc.push_back(maxAccel[dofs+i]);
        obj.input->max_jerk[i]=maxJerk[i];
    }
    if (usesMinAccel)
        obj.input->min_acceleration=std::make_optional(minAcc);

    obj.first=true;

    int objectHandle=0;
    if (nextObjectHandle<2000000000)
        objectHandle=nextObjectHandle++;
    else
    {
        while (allObjects.find(objectHandle)!=allObjects.end())
            objectHandle++;
    }
    allObjects[objectHandle]=obj;
    return(objectHandle);
}

SIM_DLLEXPORT int ruckigPlugin_step(int objHandle,double timeStep,double* newPos,double* newVel,double* newAccel,double* syncTime)
{
    int retVal=-1;

    auto it=allObjects.find(objHandle);
    if (it!=allObjects.end())
    {
        bool ruckigPos=(it->second.input->control_interface == ControlInterface::Position);
        int dofs=it->second.dofs;
        int cnt=int((timeStep/it->second.smallestTimeStep)+0.5);
        double check=fabs(1.0-((double(cnt)*it->second.smallestTimeStep)/timeStep));
        if (check<0.001)
        { // timeStep should always be a multiple of smallestTimeStep
            if (ruckigPos)
            {
                for (int i=0;i<cnt;i++)
                {
                    retVal=it->second.ruckig->update(*it->second.input,*it->second.output);

                    for (int j=0;j<dofs;j++)
                    {
                        it->second.input->current_position[j]=it->second.output->new_position[j];
                        it->second.input->current_velocity[j]=it->second.output->new_velocity[j];
                        it->second.input->current_acceleration[j]=it->second.output->new_acceleration[j];
                    }

                    if (retVal!=0)
                        break;
                }

                for (int i=0;i<dofs;i++)
                {
                    newPos[i]=it->second.output->new_position[i];
                    newVel[i]=it->second.output->new_velocity[i];
                    newAccel[i]=it->second.output->new_acceleration[i];
                }
            }
            else
            {
                for (int i=0;i<cnt;i++)
                {
                    retVal=it->second.ruckig->update(*it->second.input,*it->second.output);

                    for (int j=0;j<dofs;j++)
                    {
                        it->second.input->current_position[j]=it->second.output->new_position[j];
                        it->second.input->current_velocity[j]=it->second.output->new_velocity[j];
                        it->second.input->current_acceleration[j]=it->second.output->new_acceleration[j];
                    }

                    if (retVal!=0)
                        break;
                }

                for (int i=0;i<dofs;i++)
                {
                    newPos[i]=it->second.output->new_position[i];
                    newVel[i]=it->second.output->new_velocity[i];
                    newAccel[i]=it->second.output->new_acceleration[i];
                }
            }
            if (it->second.first)
                it->second.timeLeft=it->second.output->trajectory.get_duration();
            it->second.timeLeft-=timeStep;
            syncTime[0]=it->second.timeLeft;
            it->second.first=false;
        }
        else
            retVal=-3;
    }
    return(retVal);
}

SIM_DLLEXPORT int ruckigPlugin_remove(int objHandle)
{
    int retVal=-1;
    auto it=allObjects.find(objHandle);
    if (it!=allObjects.end())
    {
        delete it->second.ruckig;
        delete it->second.input;
        delete it->second.output;
        allObjects.erase(it);
        retVal=1;
    }
    return(retVal);
}

SIM_DLLEXPORT int ruckigPlugin_dofs(int objHandle)
{
    int retVal=-1;
    auto it=allObjects.find(objHandle);
    if (it!=allObjects.end())
        retVal=it->second.dofs;
    return(retVal);
}
