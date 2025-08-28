// GpuPhysicsEngine.cpp
#include "GpuPhysicsEngine.h"

// Include Bullet3 headers here to isolate them
#include "Bullet3OpenCL/Initialize/b3OpenCLUtils.h"
#include "Bullet3OpenCL/BroadphaseCollision/b3GpuBroadphaseInterface.h"
#include "Bullet3OpenCL/RigidBody/b3GpuNarrowPhase.h"
#include "Bullet3OpenCL/RigidBody/b3GpuRigidBodyPipeline.h"

// Implementation of any necessary methods would go here