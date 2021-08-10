#pragma once

#ifdef _WIN32
	#define SIM_DLLEXPORT extern "C" __declspec(dllexport)
#endif
#if defined (__linux) || defined (__APPLE__)
	#define SIM_DLLEXPORT extern "C"
#endif

SIM_DLLEXPORT unsigned char simStart(void* reservedPointer,int reservedInt);
SIM_DLLEXPORT void simEnd();
SIM_DLLEXPORT void* simMessage(int message,int* auxiliaryData,void* customData,int* replyData);

SIM_DLLEXPORT int ruckigPlugin_pos(int scriptHandle,int dofs,double smallestTimeStep,int flags,const double* currentPos,const double* currentVel,const double* currentAccel,const double* maxVel,const double* maxAccel,const double* maxJerk,const unsigned char* selection,const double* targetPos,const double* targetVel);
SIM_DLLEXPORT int ruckigPlugin_vel(int scriptHandle,int dofs,double smallestTimeStep,int flags,const double* currentPos,const double* currentVel,const double* currentAccel,const double* maxAccel,const double* maxJerk,const unsigned char* selection,const double* targetVel);
SIM_DLLEXPORT int ruckigPlugin_step(int objHandle,double timeStep,double* newPos,double* newVel,double* newAccel,double* syncTime);
SIM_DLLEXPORT int ruckigPlugin_remove(int objHandle);
SIM_DLLEXPORT int ruckigPlugin_dofs(int objHandle);

