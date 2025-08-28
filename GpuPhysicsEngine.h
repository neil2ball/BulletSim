// GpuPhysicsEngine.h
#pragma once

// Forward declarations to avoid including Bullet3 headers here
class b3GpuRigidBodyPipeline;
class b3GpuBroadphaseInterface;
class b3GpuNarrowPhase;

// OpenCL types
typedef struct _cl_context* cl_context;
typedef struct _cl_command_queue* cl_command_queue;
typedef struct _cl_device_id* cl_device_id;

struct GpuPhysicsEngine {
    b3GpuRigidBodyPipeline* gpuPipeline;
    b3GpuBroadphaseInterface* gpuBroadphase;
    b3GpuNarrowPhase* gpuNarrowphase;
    cl_context openclContext;
    cl_command_queue openclQueue;
    cl_device_id openclDevice;
};