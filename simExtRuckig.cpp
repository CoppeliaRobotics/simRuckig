#include "simExtRuckig.h"
#include "simLib.h"
#include <iostream>
#include <boost/lexical_cast.hpp>
#include <vector>

// Following required by the Reflexxes Motion Library:
#include <ruckig/ruckig.hpp>

using namespace ruckig;

#ifdef _WIN32
	#include <shlwapi.h>
	#pragma comment(lib, "Shlwapi.lib")
#endif /* _WIN32 */

#define PLUGIN_VERSION 2 // 2 since CoppeliaSim V3.4.1

struct SObj
{
	Ruckig<0> *ruckig;
	InputParameter<0> *input;
	OutputParameter<0> *output;

	bool destroyAtSimulationEnd;
	int objectHandle;
	int dofs;
	double smallestTimeStep;
};

static int nextObjectHandle=0;

static std::vector<SObj> allObjects;


static LIBRARY simLib; // the CoppeliaSim library that we will dynamically load and bind

SIM_DLLEXPORT unsigned char simStart(void* reservedPointer,int reservedInt)
{
	// 1. Figure out this plugin's directory:
	char curDirAndFile[1024];
#ifdef _WIN32
	GetModuleFileName(NULL,curDirAndFile,1023);
	PathRemoveFileSpec(curDirAndFile);
#elif defined (__linux) || defined (__APPLE__)
	getcwd(curDirAndFile, sizeof(curDirAndFile));
#endif
	std::string currentDirAndPath(curDirAndFile);
    // 2. Append the CoppeliaSim library's name:
	std::string temp(currentDirAndPath);
#ifdef _WIN32
	temp+="\\coppeliaSim.dll";
#elif defined (__linux)
	temp+="/libcoppeliaSim.so";
#elif defined (__APPLE__)
	temp+="/libcoppeliaSim.dylib";
#endif /* __linux || __APPLE__ */
    // 3. Load the CoppeliaSim library:
	simLib=loadSimLibrary(temp.c_str());
	if (simLib==NULL)
	{
        printf("simExtRuckig: error: could not find or correctly load the CoppeliaSim library. Cannot start the plugin.\n"); // cannot use simAddLog here.
        return(0); // Means error, CoppeliaSim will unload this plugin
	}
	if (getSimProcAddresses(simLib)==0)
	{
        printf("simExtRuckig: error: could not find all required functions in the CoppeliaSim library. Cannot start the plugin.\n"); // cannot use simAddLog here.
		unloadSimLibrary(simLib);
        return(0); // Means error, CoppeliaSim will unload this plugin
	}

    // The constants:
    // For backward compatibility (variables/constants):
    simRegisterScriptVariable("simrml_phase_sync_if_possible",(boost::lexical_cast<std::string>(int(simrml_phase_sync_if_possible))).c_str(),-1);
    simRegisterScriptVariable("simrml_only_time_sync",(boost::lexical_cast<std::string>(int(simrml_only_time_sync))).c_str(),-1);
    simRegisterScriptVariable("simrml_only_phase_sync",(boost::lexical_cast<std::string>(int(simrml_only_phase_sync))).c_str(),-1);
    simRegisterScriptVariable("simrml_no_sync",(boost::lexical_cast<std::string>(int(simrml_no_sync))).c_str(),-1);
    simRegisterScriptVariable("simrml_disable_extremum_motion_states_calc",(boost::lexical_cast<std::string>(int(simrml_disable_extremum_motion_states_calc))).c_str(),-1);
    simRegisterScriptVariable("simrml_keep_target_vel",(boost::lexical_cast<std::string>(int(simrml_keep_target_vel))).c_str(),-1);
    simRegisterScriptVariable("simrml_recompute_trajectory",(boost::lexical_cast<std::string>(int(simrml_recompute_trajectory))).c_str(),-1);
    simRegisterScriptVariable("simrml_keep_current_vel_if_fallback_strategy",(boost::lexical_cast<std::string>(int(simrml_keep_current_vel_if_fallback_strategy))).c_str(),-1);

	return(PLUGIN_VERSION); // initialization went fine, we return the version number of this plugin (can be queried with simGetModuleName)
}

SIM_DLLEXPORT void simEnd()
{
//	fclose(file);

	unloadSimLibrary(simLib); // release the library
}

SIM_DLLEXPORT void* simMessage(int message,int* auxiliaryData,void* customData,int* replyData)
{
	void* retVal=NULL;

	if ((message==sim_message_eventcallback_rmlposition))//&&(auxiliaryData[0]!=0)) // if auxiliaryData[0] isn't 0, then we wanna use the type 4 lib!
	{ // the sim_message_eventcallback_rmlposition message is passed when the API function simRMLPosition is called from C/C++ or Lua
		// All input parameters are coded in the data buffer! (i.e. first 4 bytes=DoFs, next 8 bytes=time step, etc.)
		// The values in the data buffer are in meters, not millimeters!
		char* data=(char*)customData;
		int dofs=((int*)(data+0))[0];
		double timeStep=((double*)(data+4))[0];
		int off=12;

		Ruckig<0> ruckig {(size_t)dofs, timeStep};
		InputParameter<0> input {(size_t)dofs};
		OutputParameter<0> output {(size_t)dofs};

		for (int i=0;i<dofs;i++)
			input.current_position[i]=((double*)(data+off))[i];
		off+=dofs*8;
		for (int i=0;i<dofs;i++)
			input.current_velocity[i]=((double*)(data+off))[i];
		off+=dofs*8;
		for (int i=0;i<dofs;i++)
			input.current_acceleration[i]=((double*)(data+off))[i];
		off+=dofs*8;

		for (int i=0;i<dofs;i++)
			input.max_velocity[i]=((double*)(data+off))[i];
		off+=dofs*8;
		for (int i=0;i<dofs;i++)
			input.max_acceleration[i]=((double*)(data+off))[i];
		off+=dofs*8;
		for (int i=0;i<dofs;i++)
			input.max_jerk[i]=((double*)(data+off))[i];
		off+=dofs*8;

		for (int i=0;i<dofs;i++)
			input.enabled[i]=(data[off+i]!=0);
		off+=dofs;

		for (int i=0;i<dofs;i++)
			input.target_position[i]=((double*)(data+off))[i];
		off+=dofs*8;
		for (int i=0;i<dofs;i++)
			input.target_velocity[i]=((double*)(data+off))[i];
		off+=dofs*8;

		// Apply the flags:
		int flags=((int*)(data+off))[0];
		if (flags>=0)
		{ // we don't have default values!
			if ((flags&3)==simrml_phase_sync_if_possible)
				input.synchronization=Synchronization::Phase; // default
			if ((flags&3)==simrml_only_time_sync)
				input.synchronization=Synchronization::Time;
			if ((flags&3)==simrml_only_phase_sync)
				input.synchronization=Synchronization::Phase;
			if ((flags&3)==simrml_no_sync)
				input.synchronization=Synchronization::None;

			// if ((flags&4)==simrml_keep_target_vel)
			// 	Flags.BehaviorAfterFinalStateOfMotionIsReached=RMLPositionFlags::KEEP_TARGET_VELOCITY; // default
			// if ((flags&4)==simrml_recompute_trajectory)
			// 	Flags.BehaviorAfterFinalStateOfMotionIsReached=RMLPositionFlags::RECOMPUTE_TRAJECTORY;

			// if (flags&simrml_disable_extremum_motion_states_calc)
			// 	Flags.EnableTheCalculationOfTheExtremumMotionStates=false;
			// else
			// 	Flags.EnableTheCalculationOfTheExtremumMotionStates=true; // default

			// if (flags&simrml_keep_current_vel_if_fallback_strategy)
			// 	Flags.KeepCurrentVelocityInCaseOfFallbackStrategy=true;
			// else
			// 	Flags.KeepCurrentVelocityInCaseOfFallbackStrategy=false; // default
		}
		off+=4;

		// Check for extension data (not used for now):
		unsigned char extensionBytes=data[off];
		off+=1+extensionBytes;

//		if (file!=NULL)
//			IP->Echo(file);
		// Execute the Reflexxe function!
		replyData[0]=ruckig.update(input, output);
//		printf("ret: %i, sync. time: %f, alphaTime: %f\n",replyData[0],(float)OP->SynchronizationTime,(float)OP->AlphaTime);

		// Next to returning the function return value (just here above), we return a buffer with the new position, velocity and acceleration vector:
		// We also return 8 additional doubles for future extension (1 double is already in use)
		char* retBuff=simCreateBuffer(dofs*8*3+8*8);
		off=0;

		for (int i=0;i<dofs;i++)
			((double*)(retBuff+off))[i]=output.new_position[i];
		off+=dofs*8;
		for (int i=0;i<dofs;i++)
			((double*)(retBuff+off))[i]=output.new_velocity[i];
		off+=dofs*8;
		for (int i=0;i<dofs;i++)
			((double*)(retBuff+off))[i]=output.new_acceleration[i];
		off+=dofs*8;

		((double*)(retBuff+off))[0]=output.trajectory.get_duration();
//		((double*)(retBuff+off))[1]=OP->AlphaTime;
		off+=8*8;

		retVal=retBuff;
	}

	if ((message==sim_message_eventcallback_rmlvelocity))//&&(auxiliaryData[0]!=0)) // if auxiliaryData[0] isn't 0, then we wanna use the type 4 lib!
	{ // the sim_message_eventcallback_rmlvelocity message is passed when the API function simRMLVelocity is called from C/C++ or Lua
		// All input parameters are coded in the data buffer! (i.e. first 4 bytes=DoFs, next 8 bytes=time step, etc.)
		// The values in the data buffer are in meters, not millimeters!
		char* data=(char*)customData;
		int dofs=((int*)(data+0))[0];
		double timeStep=((double*)(data+4))[0];
		int off=12;

		Ruckig<0> ruckig {(size_t)dofs,timeStep};
		InputParameter<0> input {(size_t)dofs};
		OutputParameter<0> output {(size_t)dofs};

		for (int i=0;i<dofs;i++)
			input.current_position[i]=((double*)(data+off))[i];
		off+=dofs*8;
		for (int i=0;i<dofs;i++)
			input.current_velocity[i]=((double*)(data+off))[i];
		off+=dofs*8;
		for (int i=0;i<dofs;i++)
			input.current_acceleration[i]=((double*)(data+off))[i];
		off+=dofs*8;

		for (int i=0;i<dofs;i++)
			input.max_acceleration[i]=((double*)(data+off))[i];
		off+=dofs*8;
		for (int i=0;i<dofs;i++)
			input.max_jerk[i]=((double*)(data+off))[i];
		off+=dofs*8;

		for (int i=0;i<dofs;i++)
			input.enabled[i]=(data[off+i]!=0);
		off+=dofs;

		for (int i=0;i<dofs;i++)
			input.target_velocity[i]=((double*)(data+off))[i];
		off+=dofs*8;

		// Apply the flags:
		int flags=((int*)(data+off))[0];
		if (flags>=0)
		{ // we don't have default values!
			if ((flags&3)==simrml_phase_sync_if_possible)
				input.synchronization=Synchronization::Phase;
			if ((flags&3)==simrml_only_time_sync)
				input.synchronization=Synchronization::Time;
			if ((flags&3)==simrml_only_phase_sync)
				input.synchronization=Synchronization::Phase;
			if ((flags&3)==simrml_no_sync)
				input.synchronization=Synchronization::None; // default

			// if (flags&simrml_disable_extremum_motion_states_calc)
			// 	input.EnableTheCalculationOfTheExtremumMotionStates=false;
			// else
			// 	input.EnableTheCalculationOfTheExtremumMotionStates=true; // default
		}
		off+=4;

		// Check for extension data (not used for now):
		unsigned char extensionBytes=data[off];
		off+=1+extensionBytes;

		// Execute the Reflexxe function!
		replyData[0]=ruckig.update(input,output);

		// Next to returning the function return value (just here above), we return a buffer with the new position, velocity and acceleration vector:
		// We also return 8 additional doubles for future extension (1 double is already in use)
		char* retBuff=simCreateBuffer(dofs*8*3+8*8);
		off=0;

		for (int i=0;i<dofs;i++)
			((double*)(retBuff+off))[i]=output.new_position[i];
		off+=dofs*8;
		for (int i=0;i<dofs;i++)
			((double*)(retBuff+off))[i]=output.new_velocity[i];
		off+=dofs*8;
		for (int i=0;i<dofs;i++)
			((double*)(retBuff+off))[i]=output.new_acceleration[i];
		off+=dofs*8;

		((double*)(retBuff+off))[0]=output.trajectory.get_duration();
//		((double*)(retBuff+off))[1]=OP->AlphaTime;
		off+=8*8;

		retVal=retBuff;
	}

	if (message==sim_message_eventcallback_rmlpos)
	{ // the sim_message_eventcallback_rmlpos message is passed when the API function simRMLPos is called from C/C++ or Lua

		SObj obj;
		obj.destroyAtSimulationEnd=(auxiliaryData[1]!=0);
		obj.objectHandle=nextObjectHandle;

		// All input parameters are coded in the data buffer! (i.e. first 4 bytes=DoFs, next 8 bytes=time step, etc.)
		// The values in the data buffer are in meters, not millimeters!
		char* data=(char*)customData;
		int dofs=((int*)(data+0))[0];
		obj.dofs=dofs;
		double timeStep=((double*)(data+4))[0];
		obj.smallestTimeStep=timeStep;
		int off=12;

		obj.ruckig = new Ruckig<0>(dofs,timeStep);
		obj.input = new InputParameter<0>(dofs);
		obj.output = new OutputParameter<0>(dofs);

		for (int i=0;i<dofs;i++)
			obj.input->current_position[i]=((double*)(data+off))[i];
		off+=dofs*8;
		for (int i=0;i<dofs;i++)
			obj.input->current_velocity[i]=((double*)(data+off))[i];
		off+=dofs*8;
		for (int i=0;i<dofs;i++)
			obj.input->current_acceleration[i]=((double*)(data+off))[i];
		off+=dofs*8;

		for (int i=0;i<dofs;i++)
			obj.input->max_velocity[i]=((double*)(data+off))[i];
		off+=dofs*8;
		for (int i=0;i<dofs;i++)
			obj.input->max_acceleration[i]=((double*)(data+off))[i];
		off+=dofs*8;
		for (int i=0;i<dofs;i++)
			obj.input->max_jerk[i]=((double*)(data+off))[i];
		off+=dofs*8;

		for (int i=0;i<dofs;i++)
			obj.input->enabled[i]=(data[off+i]!=0);
		off+=dofs;

		for (int i=0;i<dofs;i++)
			obj.input->target_position[i]=((double*)(data+off))[i];
		off+=dofs*8;
		for (int i=0;i<dofs;i++)
			obj.input->target_velocity[i]=((double*)(data+off))[i];
		off+=dofs*8;

		// Apply the flags:
		int flags=((int*)(data+off))[0];
		if (flags>=0)
		{ // we don't have default values!
			if ((flags&3)==simrml_phase_sync_if_possible)
				obj.input->synchronization=Synchronization::Phase; // default
			if ((flags&3)==simrml_only_time_sync)
				obj.input->synchronization=Synchronization::Time;
			if ((flags&3)==simrml_only_phase_sync)
				obj.input->synchronization=Synchronization::Phase;
			if ((flags&3)==simrml_no_sync)
				obj.input->synchronization=Synchronization::None;

			// if ((flags&4)==simrml_keep_target_vel)
			// 	obj.PFlags->BehaviorAfterFinalStateOfMotionIsReached=RMLPositionFlags::KEEP_TARGET_VELOCITY; // default
			// if ((flags&4)==simrml_recompute_trajectory)
			// 	obj.PFlags->BehaviorAfterFinalStateOfMotionIsReached=RMLPositionFlags::RECOMPUTE_TRAJECTORY;

			// if (flags&simrml_disable_extremum_motion_states_calc)
			// 	obj.PFlags->EnableTheCalculationOfTheExtremumMotionStates=false;
			// else
			// 	obj.PFlags->EnableTheCalculationOfTheExtremumMotionStates=true; // default

			// if (flags&simrml_keep_current_vel_if_fallback_strategy)
			// 	obj.PFlags->KeepCurrentVelocityInCaseOfFallbackStrategy=true;
			// else
			// 	obj.PFlags->KeepCurrentVelocityInCaseOfFallbackStrategy=false; // default
		}
		off+=4;

		// Check for extension data (not used for now):
		unsigned char extensionBytes=data[off];
		off+=1+extensionBytes;

		allObjects.push_back(obj);
		replyData[0]=0;
		replyData[1]=nextObjectHandle++;
	}

	if (message==sim_message_eventcallback_rmlvel)
	{ // the sim_message_eventcallback_rmlvel message is passed when the API function simRMLVel is called from C/C++ or Lua

		SObj obj;
		obj.destroyAtSimulationEnd=(auxiliaryData[1]!=0);
		obj.objectHandle=nextObjectHandle;

		// All input parameters are coded in the data buffer! (i.e. first 4 bytes=DoFs, next 8 bytes=time step, etc.)
		// The values in the data buffer are in meters, not millimeters!
		char* data=(char*)customData;
		int dofs=((int*)(data+0))[0];
		obj.dofs=dofs;
		double timeStep=((double*)(data+4))[0];
		obj.smallestTimeStep=timeStep;
		int off=12;

		obj.ruckig = new Ruckig<0>(dofs, timeStep);
		obj.input = new InputParameter<0>(dofs);
		obj.output = new OutputParameter<0>(dofs);

		for (int i=0;i<dofs;i++)
			obj.input->current_position[i]=((double*)(data+off))[i];
		off+=dofs*8;
		for (int i=0;i<dofs;i++)
			obj.input->current_velocity[i]=((double*)(data+off))[i];
		off+=dofs*8;
		for (int i=0;i<dofs;i++)
			obj.input->current_acceleration[i]=((double*)(data+off))[i];
		off+=dofs*8;

		for (int i=0;i<dofs;i++)
			obj.input->max_acceleration[i]=((double*)(data+off))[i];
		off+=dofs*8;
		for (int i=0;i<dofs;i++)
			obj.input->max_jerk[i]=((double*)(data+off))[i];
		off+=dofs*8;

		for (int i=0;i<dofs;i++)
			obj.input->enabled[i]=(data[off+i]!=0);
		off+=dofs;

		for (int i=0;i<dofs;i++)
			obj.input->target_velocity[i]=((double*)(data+off))[i];
		off+=dofs*8;

		// Apply the flags:
		int flags=((int*)(data+off))[0];
		if (flags>=0)
		{ // we don't have default values!
			if ((flags&3)==simrml_phase_sync_if_possible)
				obj.input->synchronization=Synchronization::Phase;
			if ((flags&3)==simrml_only_time_sync)
				obj.input->synchronization=Synchronization::Time;
			if ((flags&3)==simrml_only_phase_sync)
				obj.input->synchronization=Synchronization::Phase;
			if ((flags&3)==simrml_no_sync)
				obj.input->synchronization=Synchronization::None; // default

			// if (flags&simrml_disable_extremum_motion_states_calc)
			// 	obj.input.EnableTheCalculationOfTheExtremumMotionStates=false;
			// else
			// 	obj.input.EnableTheCalculationOfTheExtremumMotionStates=true; // default
		}
		off+=4;

		// Check for extension data (not used for now):
		unsigned char extensionBytes=data[off];
		off+=1+extensionBytes;

		allObjects.push_back(obj);
		replyData[0]=0;
		replyData[1]=nextObjectHandle++;
	}

	if (message==sim_message_eventcallback_rmlstep)
	{ // the sim_message_eventcallback_rmlstep message is passed when the API function simRMLStep is called from C/C++ or Lua
		int index=-1;
		bool rmlPos=true;
		for (int i=0;i<int(allObjects.size());i++)
		{
			if (allObjects[i].objectHandle==auxiliaryData[1])
			{
				rmlPos=(allObjects[i].input->interface == Interface::Position);
				index=i;
				break;
			}
		}
		if (index!=-1)
		{
			int dofs=allObjects[index].dofs;
			double timeStep=double(auxiliaryData[2])/100000.0;
			int cnt=int((timeStep/allObjects[index].smallestTimeStep)+0.5);
			if (rmlPos)
			{
				for (int i=0;i<cnt;i++)
				{
					replyData[0]=allObjects[index].ruckig->update(*allObjects[index].input,*allObjects[index].output);

					for (int j=0;j<dofs;j++)
						allObjects[index].input->current_position[j]=allObjects[index].output->new_position[j];
					for (int j=0;j<dofs;j++)
						allObjects[index].input->current_velocity[j]=allObjects[index].output->new_velocity[j];
					for (int j=0;j<dofs;j++)
						allObjects[index].input->current_acceleration[j]=allObjects[index].output->new_acceleration[j];

					if (replyData[0]!=0)
						break;
				}

				// Next to returning the function return value (just here above), we return a buffer with the new position, velocity and acceleration vector:
				// We also return 8 additional doubles for future extension (1 double is already in use)
				char* retBuff=simCreateBuffer(dofs*8*3+8*8);
				int off=0;

				for (int i=0;i<dofs;i++)
					((double*)(retBuff+off))[i]=allObjects[index].output->new_position[i];
				off+=dofs*8;
				for (int i=0;i<dofs;i++)
					((double*)(retBuff+off))[i]=allObjects[index].output->new_velocity[i];
				off+=dofs*8;
				for (int i=0;i<dofs;i++)
					((double*)(retBuff+off))[i]=allObjects[index].output->new_acceleration[i];
				off+=dofs*8;

				((double*)(retBuff+off))[0]=allObjects[index].output->trajectory.get_duration();
				off+=8*8;

				retVal=retBuff;
			}
			else
			{ // RML velocity
				for (int i=0;i<cnt;i++)
				{
					replyData[0]=allObjects[index].ruckig->update(*allObjects[index].input,*allObjects[index].output);

					for (int j=0;j<dofs;j++)
						allObjects[index].input->current_position[j]=allObjects[index].output->new_position[j];
					for (int j=0;j<dofs;j++)
						allObjects[index].input->current_velocity[j]=allObjects[index].output->new_velocity[j];
					for (int j=0;j<dofs;j++)
						allObjects[index].input->current_acceleration[j]=allObjects[index].output->new_acceleration[j];

					if (replyData[0]!=0)
						break;
				}

				// Next to returning the function return value (just here above), we return a buffer with the new position, velocity and acceleration vector:
				// We also return 8 additional doubles for future extension (1 double is already in use)
				char* retBuff=simCreateBuffer(dofs*8*3+8*8);
				int off=0;

				for (int i=0;i<dofs;i++)
					((double*)(retBuff+off))[i]=allObjects[index].output->new_position[i];
				off+=dofs*8;
				for (int i=0;i<dofs;i++)
					((double*)(retBuff+off))[i]=allObjects[index].output->new_velocity[i];
				off+=dofs*8;
				for (int i=0;i<dofs;i++)
					((double*)(retBuff+off))[i]=allObjects[index].output->new_acceleration[i];
				off+=dofs*8;

				((double*)(retBuff+off))[0]=allObjects[index].output->trajectory.get_duration();
				off+=8*8;

				retVal=retBuff;
			}
			replyData[1]=dofs;
		}
	}

	if (message==sim_message_eventcallback_rmlremove)
	{ // the sim_message_eventcallback_rmlremove message is passed when the API function simRMLRemove is called from C/C++ or Lua
		replyData[1]=0;
		for (int i=0;i<int(allObjects.size());i++)
		{
			if (allObjects[i].objectHandle==auxiliaryData[1])
			{
				delete allObjects[i].ruckig;
				delete allObjects[i].input;
				delete allObjects[i].output;
				allObjects.erase(allObjects.begin()+i);
				replyData[1]=1;
				break;
			}
		}
		replyData[0]=0;
	}

	if (message==sim_message_eventcallback_rmlinfo)
    { // the sim_message_eventcallback_rmlinfo is used in CoppeliaSim internally
		if (auxiliaryData[0]==0)
		{ // means: give me the Dofs of this object
			replyData[1]=-1;
			for (int i=0;i<int(allObjects.size());i++)
			{
				if (allObjects[i].objectHandle==auxiliaryData[1])
				{
					replyData[1]=allObjects[i].dofs;
					break;
				}
			}
		}
		replyData[0]=0;
	}


	if (message==sim_message_eventcallback_simulationabouttostart)
	{ // Simulation is about to start

	}

	if (message==sim_message_eventcallback_simulationended)
	{ // Simulation just ended
		for (int i=0;i<int(allObjects.size());i++)
		{
			if (allObjects[i].destroyAtSimulationEnd)
			{
				delete allObjects[i].ruckig;
				delete allObjects[i].input;
				delete allObjects[i].output;
				allObjects.erase(allObjects.begin()+i);
				i--; // reprocess this position
			}
		}
	}

	return(retVal);
}

