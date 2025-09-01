/*
 * Copyright (c) Contributors, http://opensimulator.org/
 * See CONTRIBUTORS.TXT for a full list of copyright holders.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyrightD
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the OpenSimulator Project nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE DEVELOPERS ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE CONTRIBUTORS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#pragma once

#ifndef WORLD_DATA_H
#define WORLD_DATA_H
#define CL_TARGET_OPENCL_VERSION 120
#ifdef _WIN32
    #ifdef BULLETSIM_EXPORTS
        #define BULLETSIM_API __declspec(dllexport)
    #else
        #define BULLETSIM_API __declspec(dllimport)
    #endif
#else
    #define BULLETSIM_API
#endif

#include "ArchStuff.h"
#include "APIData.h"
#include <clew/clew.h>
#include <CL/cl.h>

// Keep the Bullet2 includes as they are
#include "Bullet3Common/b3Vector3.h"
#include "Bullet3Common/b3Quaternion.h"
#include "Bullet3Common/b3Transform.h"

#include "BulletDynamics/Dynamics/btDynamicsWorld.h"
#include <btBulletCollisionCommon.h>
#include <btBulletDynamicsCommon.h>

#include <stdarg.h>
#include <map>
#include <cstdarg>
#include <cstdio>

// Forward declarations
class BulletSim;
class ObjectCollection;
class HeightMapData;
class IPhysObject;
class TerrainObject;
class GroundPlaneObject;

class b3GpuRigidBodyPipeline;
class b3GpuBroadphaseInterface;
class b3GpuNarrowPhase;
struct b3RigidBodyData;

// template for debugging call
typedef void DebugLogCallback(const char*);

#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable:4251)
#endif



// Structure to hold the world data that is common to all the objects in the world
struct BULLETSIM_API WorldData
{
	BulletSim* sim;
	
	WorldData(const WorldData&) = delete;
    WorldData& operator=(const WorldData&) = delete;
    WorldData(WorldData&&) = delete;
    WorldData& operator=(WorldData&&) = delete;
	
	WorldData() = default; // allow default construction

	ParamBlock* params;
	
	// The main dynamics world
	btDynamicsWorld* dynamicsWorld;
	
	// The minimum and maximum points in the defined physical space
	btVector3 MinPosition;
	btVector3 MaxPosition;

	// Used to expose updates from Bullet to the BulletSim API
	typedef std::map<IDTYPE, EntityProperties*> UpdatesThisFrameMapType;
	UpdatesThisFrameMapType updatesThisFrame;

	// Some collisionObjects can set themselves up for special collision processing.
	// This is used for ghost objects to be handed in the simulation step.
	typedef std::map<IDTYPE, btCollisionObject*> SpecialCollisionObjectMapType;
	SpecialCollisionObjectMapType specialCollisionObjects;

    // Add Bullet physics components
    btDefaultCollisionConfiguration* collisionConfiguration;
    btCollisionDispatcher* dispatcher;
    btBroadphaseInterface* broadphase;
    btSequentialImpulseConstraintSolver* solver;

	// GPU acceleration members
	cl_context openclContext;
	cl_command_queue openclQueue;
	cl_device_id openclDevice;
	
	
	b3GpuRigidBodyPipeline* gpuPipeline;
	b3GpuNarrowPhase* gpuNarrowphase;
	b3GpuBroadphaseInterface* gpuBroadphaseSap;
	b3DynamicBvhBroadphase* gpuBroadphaseDbvt;
	b3Config gpuConfig;
	
	bool isGpuAvailable = false;

	// DEBUGGGING
	// ============================================================================================
	// Callback to managed code for logging
	// This is a callback into the managed code that writes a text message to the log.
	// This callback is only initialized if the simulator is running in DEBUG mode.
	DebugLogCallback* debugLogCallback;

	void BSLog(const char* msg, ...);
    void BSLog2(const char* msg, va_list argp);
	
	void	dumpAll()
	{
		BSLog("PROFILE LOGGING IS NOT ENABLED IN BULLET");
	}
};

#ifdef _MSC_VER
#pragma warning(pop)
#endif

#endif // WORLD_DATA_H