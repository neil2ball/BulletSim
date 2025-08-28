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

#include "Bullet3OpenCL/Initialize/b3OpenCLUtils.h"
#include "Bullet3OpenCL/BroadphaseCollision/b3GpuBroadphaseInterface.h"
#include "Bullet3OpenCL/RigidBody/b3GpuNarrowPhase.h"
#include "Bullet3OpenCL/RigidBody/b3GpuRigidBodyPipeline.h"
#include "Bullet3Common/b3Vector3.h"
#include "Bullet3Common/b3Quaternion.h"
#include "Bullet3Common/b3Transform.h"

// Forward declarations
class BulletSim;
class ObjectCollection;
class HeightMapData;
class IPhysObject;
class TerrainObject;
class GroundPlaneObject;

// Forward declarations for GPU components
class b3GpuRigidBodyPipeline;
class b3GpuBroadphaseInterface;
class b3GpuNarrowPhase;

#include <stdarg.h>
#include <map>
#include <cstdarg>
#include <cstdio>

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

	ParamBlock* params;
	
	// The main dynamics world
	//btDynamicsWorld* dynamicsWorld;
	
	// The minimum and maximum points in the defined physical space
	b3Vector3 MinPosition;
	b3Vector3 MaxPosition;

	// Used to expose updates from Bullet to the BulletSim API
	typedef std::map<IDTYPE, EntityProperties*> UpdatesThisFrameMapType;
	UpdatesThisFrameMapType updatesThisFrame;

	// Some collisionObjects can set themselves up for special collision processing.
	// This is used for ghost objects to be handed in the simulation step.
	//typedef std::map<IDTYPE, btCollisionObject*> SpecialCollisionObjectMapType;
	//SpecialCollisionObjectMapType specialCollisionObjects;

	// GPU acceleration members
	b3GpuRigidBodyPipeline* gpuPipeline;
	b3GpuBroadphaseInterface* gpuBroadphase;
	b3GpuNarrowPhase* gpuNarrowphase;
	cl_context openclContext;
	cl_command_queue openclQueue;
	cl_device_id openclDevice;

	// DEBUGGGING
	// ============================================================================================
	// Callback to managed code for logging
	// This is a callback into the managed code that writes a text message to the log.
	// This callback is only initialized if the simulator is running in DEBUG mode.
	DebugLogCallback* debugLogCallback;

	void BSLog(const char* format, ...) {
        va_list args;
        va_start(args, format);
        vprintf(format, args);
        va_end(args);
        printf("\n");
    }
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