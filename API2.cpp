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
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE */
#define BT_USE_DOUBLE_PRECISION
#define __CELLOS_LV2__
#define MAX_NUM_PARTS_IN_BITS 10
#ifdef MAX_NUM_PARTS_IN_BITS
#undef MAX_NUM_PARTS_IN_BITS
#endif

#define CL_TARGET_OPENCL_VERSION 120
#include "BulletSim.h"
#include "Util.h"
#include <stdarg.h>

#include "Bullet3OpenCL/RigidBody/b3GpuRigidBodyPipeline.h"
#include "Bullet3Common/b3Vector3.h"
#include "Bullet3Common/b3Quaternion.h"
#include "Bullet3Common/b3Transform.h"
#include "Bullet3Collision/NarrowPhaseCollision/shared/b3Collidable.h"

#include <map>
#include <cstdint>

#if defined(_WIN32) || defined(_WIN64)
    #define DLL_EXPORT __declspec( dllexport )
    #define DLL_IMPORT __declspec( dllimport )
#else
    #define DLL_EXPORT
    #define DLL_IMPORT
#endif

#ifdef __cplusplus
    #define EXTERN_C extern "C"
#else
    #define EXTERN_C extern
#endif

#pragma warning( disable: 4190 ) // Warning about returning Vector3 that we can safely ignore

// The minimum thickness for terrain. If less than this, we correct
#define TERRAIN_MIN_THICKNESS (0.2)

struct VehicleTuning {
    float suspensionStiffness;
    float suspensionDamping;
    float suspensionCompression;
    float suspensionRelaxation;
    float frictionSlip;
    float rollInfluence;
};

struct VehicleWheelInfo {
    b3Transform worldTransform;
    Vector3 chassisConnectionPointCS;
    Vector3 wheelDirectionCS;
    Vector3 wheelAxleCS;
    float suspensionRestLength;
    float maxSuspensionTravelCm;
    float wheelsRadius;
    float suspensionStiffness;
    float wheelsDampingCompression;
    float wheelsDampingRelaxation;
    float frictionSlip;
    bool bIsFrontWheel;
    float rollInfluence;
    float engineForce;
    float brake;
    float steering;
    float rotation;
    float deltaRotation;
    float suspensionRelativeVelocity;
    float clippedInvContactDotSuspension;
    float suspensionForce;
    float skidInfo;
};

/**
 * Returns a pointer to a string that identifies the version of the BulletSim.dll
 * @return pointer to zero terminated static string of format BULLETENGINEVERSION,BULLETSIMVERSION ("3.25,1.3")
 */
/*EXTERN_C DLL_EXPORT const char* GetVersion2()
{
	return BulletSimVersionString.c_str();
}*/

// DEBUG DEBUG DEBUG =========================================================================================
// USE ONLY FOR VITAL DEBUGGING!!!!
static int lastNumberOverlappingPairs;
static BulletSim* staticSim;

static void InitCheckOverlappingPairs(BulletSim* pSim)
{
    staticSim = pSim;
    lastNumberOverlappingPairs = 0;
}

static void CheckOverlappingPairs(char* pReason)
{
    return;
}

// Helper functions for hollow/cut primitives
float CalculateMinOpeningFraction(float primSize)
{
    const float avatarRadius = 0.45f;
    return (avatarRadius * 2.0f) / primSize;
}

// GPU-compatible shape creation functions
EXTERN_C DLL_EXPORT int CreateGpuBoxShape(BulletSim* sim, Vector3 halfExtents)
{
    b3Vector3 he;
    he.setValue(halfExtents.X, halfExtents.Y, halfExtents.Z);
    return sim->registerGpuBoxShape(he);
}

EXTERN_C DLL_EXPORT int CreateGpuSphereShape(BulletSim* sim, float radius)
{
    return sim->registerGpuSphereShape(radius);
}

EXTERN_C DLL_EXPORT int CreateGpuCapsuleShape(BulletSim* sim, float radius, float height)
{
    return sim->registerGpuCapsuleShape(radius, height);
}

EXTERN_C DLL_EXPORT int CreateGpuCylinderShape(BulletSim* sim, Vector3 halfExtents)
{
    b3Vector3 he;
    he.setValue(halfExtents.X, halfExtents.Y, halfExtents.Z);
    return sim->registerGpuCylinderShape(he);
}

EXTERN_C DLL_EXPORT int CreateGpuConvexHullShape(BulletSim* sim, int numPoints, float* points)
{
    return sim->registerGpuConvexHullShape(numPoints, points);
}

EXTERN_C DLL_EXPORT int CreateGpuCompoundShape(BulletSim* sim)
{
    return sim->registerGpuCompoundShape();
}

EXTERN_C DLL_EXPORT void AddChildShapeToGpuCompound(BulletSim* sim, int compoundShapeId, int childShapeId, 
                                                   Vector3 relativePosition, Quaternion relativeRotation)
{
    b3Vector3 pos;
    pos.setValue(relativePosition.X, relativePosition.Y, relativePosition.Z);
    
    b3Quaternion rot;
    rot.setValue(relativeRotation.X, relativeRotation.Y, relativeRotation.Z, relativeRotation.W);
    
    b3Transform transform;
    transform.setRotation(rot);
    transform.setOrigin(pos);
    
    sim->addChildShapeToGpuCompound(compoundShapeId, childShapeId, transform);
}

/**
 * Initializes the physical simulation.
 */
/*EXTERN_C DLL_EXPORT BulletSim* Initialize2(Vector3 maxPosition, ParamBlock* parms,
											int maxCollisions, CollisionDesc* collisionArray,
											int maxUpdates, EntityProperties* updateArray,
											DebugLogCallback* debugLog)
{
	bsDebug_Initialize();

	//BulletSim* sim = new BulletSim(maxPosition.X, maxPosition.Y, maxPosition.Z);
	//sim->getWorldData()->debugLogCallback = debugLog;
	//sim->initPhysics2(parms, maxCollisions, collisionArray, maxUpdates, updateArray);

	//return sim;
}*/

/**
 * Update the internal value of a parameter.
 */
EXTERN_C DLL_EXPORT bool UpdateParameter2(BulletSim* sim, unsigned int localID, const char* parm, float value)
{
	return sim->UpdateParameter2(localID, parm, value);
}

/**
 * Shuts down the physical simulation.
 */
EXTERN_C DLL_EXPORT void Shutdown2(BulletSim* sim)
{
	sim->exitPhysics2();
	bsDebug_AllDone();
	delete sim;
}

// Very low level reset of collision proxy pool
EXTERN_C DLL_EXPORT void ResetBroadphasePool(BulletSim* sim)
{
	// GPU implementation doesn't use this CPU broadphase
}

// Very low level reset of the constraint solver
EXTERN_C DLL_EXPORT void ResetConstraintSolver(BulletSim* sim)
{
	// GPU implementation doesn't use this CPU constraint solver
}

/**
 * Steps the simulation forward a given amount of time and retrieves any physics updates.
 */
/*EXTERN_C DLL_EXPORT int PhysicsStep2(BulletSim* sim, float timeStep, int maxSubSteps, float fixedTimeStep, 
										int* updatedEntityCount, int* collidersCount)
{
	return sim->PhysicsStep2(timeStep, maxSubSteps, fixedTimeStep, updatedEntityCount, collidersCount);
}*/

// GPU-compatible mesh creation
EXTERN_C DLL_EXPORT int CreateGpuMeshShape(BulletSim* sim, int indicesCount, int* indices, int verticesCount, float* vertices)
{
    return sim->registerGpuMeshShape(indicesCount, indices, verticesCount, vertices);
}

// GPU-compatible convex hull creation
EXTERN_C DLL_EXPORT int CreateGpuConvexHullShapeFromPoints(BulletSim* sim, int numPoints, float* points)
{
    return sim->registerGpuConvexHullShape(numPoints, points);
}

// GPU-compatible terrain creation
EXTERN_C DLL_EXPORT int CreateGpuTerrainShape(BulletSim* sim, int width, int length, float* heightData, 
                                             float minHeight, float maxHeight, float scale)
{
    return sim->registerGpuTerrainShape(width, length, heightData, minHeight, maxHeight, scale);
}

// GPU-compatible rigid body creation
EXTERN_C DLL_EXPORT int CreateGpuRigidBody(BulletSim* sim, int shapeId, float mass, 
                                          Vector3 position, Quaternion rotation,
                                          Vector3 linearVelocity, Vector3 angularVelocity)
{
    b3Vector3 pos;
    pos.setValue(position.X, position.Y, position.Z);
    
    b3Quaternion rot;
    rot.setValue(rotation.X, rotation.Y, rotation.Z, rotation.W);
    
    b3Vector3 linVel;
    linVel.setValue(linearVelocity.X, linearVelocity.Y, linearVelocity.Z);
    
    b3Vector3 angVel;
    angVel.setValue(angularVelocity.X, angularVelocity.Y, angularVelocity.Z);
    
    return sim->registerGpuRigidBody(shapeId, mass, pos, rot, linVel, angVel);
}

// GPU-compatible rigid body manipulation
EXTERN_C DLL_EXPORT void SetGpuBodyPosition(BulletSim* sim, int bodyId, Vector3 position)
{
    b3Vector3 pos;
    pos.setValue(position.X, position.Y, position.Z);
    sim->setGpuBodyPosition(bodyId, pos);
}

EXTERN_C DLL_EXPORT void SetGpuBodyRotation(BulletSim* sim, int bodyId, Quaternion rotation)
{
    b3Quaternion rot;
    rot.setValue(rotation.X, rotation.Y, rotation.Z, rotation.W);
    sim->setGpuBodyRotation(bodyId, rot);
}

EXTERN_C DLL_EXPORT void SetGpuBodyLinearVelocity(BulletSim* sim, int bodyId, Vector3 velocity)
{
    b3Vector3 vel;
    vel.setValue(velocity.X, velocity.Y, velocity.Z);
    sim->setGpuBodyLinearVelocity(bodyId, vel);
}

EXTERN_C DLL_EXPORT void SetGpuBodyAngularVelocity(BulletSim* sim, int bodyId, Vector3 velocity)
{
    b3Vector3 vel;
    vel.setValue(velocity.X, velocity.Y, velocity.Z);
    sim->setGpuBodyAngularVelocity(bodyId, vel);
}

EXTERN_C DLL_EXPORT void ApplyGpuBodyCentralForce(BulletSim* sim, int bodyId, Vector3 force)
{
    b3Vector3 f;
    f.setValue(force.X, force.Y, force.Z);
    sim->applyGpuBodyCentralForce(bodyId, f);
}

EXTERN_C DLL_EXPORT void ApplyGpuBodyCentralImpulse(BulletSim* sim, int bodyId, Vector3 impulse)
{
    b3Vector3 imp;
    imp.setValue(impulse.X, impulse.Y, impulse.Z);
    sim->applyGpuBodyCentralImpulse(bodyId, imp);
}

EXTERN_C DLL_EXPORT void ApplyGpuBodyTorque(BulletSim* sim, int bodyId, Vector3 torque)
{
    b3Vector3 t;
    t.setValue(torque.X, torque.Y, torque.Z);
    sim->applyGpuBodyTorque(bodyId, t);
}

// GPU-compatible ray test
EXTERN_C DLL_EXPORT bool GpuRayTest(BulletSim* sim, Vector3 from, Vector3 to, RayResult* result)
{
    b3Vector3 f;
    f.setValue(from.X, from.Y, from.Z);
    
    b3Vector3 t;
    t.setValue(to.X, to.Y, to.Z);
    
    return sim->gpuRayTest(f, t, result);
}

// GPU-compatible convex sweep test
EXTERN_C DLL_EXPORT bool GpuConvexSweepTest(BulletSim* sim, int shapeId, 
                                           Vector3 fromPos, Quaternion fromRot, 
                                           Vector3 toPos, Quaternion toRot, 
                                           SweepResult* result)
{
    b3Vector3 fPos;
    fPos.setValue(fromPos.X, fromPos.Y, fromPos.Z);
    
    b3Quaternion fRot;
    fRot.setValue(fromRot.X, fromRot.Y, fromRot.Z, fromRot.W);
    
    b3Vector3 tPos;
    tPos.setValue(toPos.X, toPos.Y, toPos.Z);
    
    b3Quaternion tRot;
    tRot.setValue(toRot.X, toRot.Y, toRot.Z, toRot.W);
    
    return sim->gpuConvexSweepTest(shapeId, fPos, fRot, tPos, tRot, result);
}

// GPU-compatible contact point query
EXTERN_C DLL_EXPORT int GpuGetContactPoints(BulletSim* sim, int bodyIdA, int bodyIdB, 
                                           ContactPoint* contacts, int maxContacts)
{
    return sim->gpuGetContactPoints(bodyIdA, bodyIdB, contacts, maxContacts);
}

// GPU-compatible AABB query
EXTERN_C DLL_EXPORT bool GpuGetAabb(BulletSim* sim, int bodyId, Vector3* aabbMin, Vector3* aabbMax)
{
    b3Vector3 min, max;
    bool result = sim->gpuGetAabb(bodyId, min, max);
    
    if (result) {
        aabbMin->X = min.x;
        aabbMin->Y = min.y;
        aabbMin->Z = min.z;
        
        aabbMax->X = max.x;
        aabbMax->Y = max.y;
        aabbMax->Z = max.z;
    }
    
    return result;
}

// Utility functions for coordinate conversion
EXTERN_C DLL_EXPORT Vector3 ConvertVectorFromBullet2(Vector3 bulletVec)
{
	return Vector3(bulletVec.X, bulletVec.Z, bulletVec.Y);
}

EXTERN_C DLL_EXPORT Vector3 ConvertVectorToBullet2(Vector3 osVec)
{
	return Vector3(osVec.X, osVec.Z, osVec.Y);
}

EXTERN_C DLL_EXPORT Quaternion ConvertQuaternionFromBullet2(Quaternion bulletQuat)
{
	return Quaternion(-bulletQuat.X, -bulletQuat.Z, -bulletQuat.Y, bulletQuat.W);
}

EXTERN_C DLL_EXPORT Quaternion ConvertQuaternionToBullet2(Quaternion osQuat)
{
	return Quaternion(-osQuat.X, -osQuat.Z, -osQuat.Y, osQuat.W);
}

// GPU-specific debug and information functions
EXTERN_C DLL_EXPORT int GpuGetNumRigidBodies(BulletSim* sim)
{
    return sim->gpuGetNumRigidBodies();
}

EXTERN_C DLL_EXPORT int GpuGetNumCollisionObjects(BulletSim* sim)
{
    return sim->gpuGetNumCollisionObjects();
}

EXTERN_C DLL_EXPORT void GpuDumpWorldState(BulletSim* sim)
{
    sim->gpuDumpWorldState();
}

// GPU-specific constraint functions (limited support)
EXTERN_C DLL_EXPORT int CreateGpuPoint2PointConstraint(BulletSim* sim, int bodyIdA, int bodyIdB, 
                                                      Vector3 pivotInA, Vector3 pivotInB)
{
    b3Vector3 pivotA;
    pivotA.setValue(pivotInA.X, pivotInA.Y, pivotInA.Z);
    
    b3Vector3 pivotB;
    pivotB.setValue(pivotInB.X, pivotInB.Y, pivotInB.Z);
    
    return sim->createGpuPoint2PointConstraint(bodyIdA, bodyIdB, pivotA, pivotB);
}

EXTERN_C DLL_EXPORT void RemoveGpuConstraint(BulletSim* sim, int constraintId)
{
    sim->removeGpuConstraint(constraintId);
}