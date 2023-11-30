#pragma once

#include <simLib/simTypes.h>
#include <simLib/simExp.h>

SIM_DLLEXPORT int simInit(SSimInit*);
SIM_DLLEXPORT void simCleanup();
SIM_DLLEXPORT void simMsg(SSimMsg*);

SIM_DLLEXPORT int ruckigPlugin_pos(int scriptHandle,int dofs,double smallestTimeStep,int flags,const double* currentPos,const double* currentVel,const double* currentAccel,const double* maxVel,const double* maxAccel,const double* maxJerk,const unsigned char* selection,const double* targetPos,const double* targetVel);
SIM_DLLEXPORT int ruckigPlugin_vel(int scriptHandle,int dofs,double smallestTimeStep,int flags,const double* currentPos,const double* currentVel,const double* currentAccel,const double* maxAccel,const double* maxJerk,const unsigned char* selection,const double* targetVel);
SIM_DLLEXPORT int ruckigPlugin_step(int objHandle,double timeStep,double* newPos,double* newVel,double* newAccel,double* syncTime);
SIM_DLLEXPORT int ruckigPlugin_remove(int objHandle);
SIM_DLLEXPORT int ruckigPlugin_dofs(int objHandle);

