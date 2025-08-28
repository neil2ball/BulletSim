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

#ifndef BULLET_SIM_H
#define BULLET_SIM_H

// Include Bullet2 headers first
#include <btBulletCollisionCommon.h>
#include <btBulletDynamicsCommon.h>

// Undef conflicting macros
#ifdef MAX_NUM_PARTS_IN_BITS
#undef MAX_NUM_PARTS_IN_BITS
#endif

// Then include your headers
#include "WorldData.h"
#include "VectorConverters.h"

#define CL_TARGET_OPENCL_VERSION 120
#include <CL/cl.h>

#include "DebugLogic.h"
#include "ArchStuff.h"
#include "APIData.h"
#include "GpuPhysicsEngine.h"

#include "Bullet3OpenCL/RigidBody/b3GpuRigidBodyPipeline.h"
#include "Bullet3Common/b3Vector3.h"
#include "Bullet3Common/b3Quaternion.h"
#include "Bullet3Common/b3Transform.h"
#include "Bullet3OpenCL/Initialize/b3OpenCLUtils.h"
#include "Bullet3OpenCL/BroadphaseCollision/b3GpuBroadphaseInterface.h"
#include "Bullet3OpenCL/BroadphaseCollision/b3GpuSapBroadphase.h"
#include "Bullet3OpenCL/RigidBody/b3GpuNarrowPhase.h"
#include "Bullet3Collision/NarrowPhaseCollision/b3RaycastInfo.h"

#include <LinearMath/btScalar.h>

#include <set>
#include <map>
#include <vector>
#include <cstdint>
#include <string>


class b3GpuRigidBodyPipeline;
class b3GpuBroadphaseInterface;
class b3GpuNarrowPhase;
struct b3RigidBodyData;

struct BulletGPUSettings {
    int MaxConvexBodies;
    int MaxConvexShapes;
    int MaxBroadphasePairs;
    int MaxContactCapacity;
    int CompoundPairCapacity;
    int MaxVerticesPerFace;
    int MaxFacesPerShape;
    int MaxConvexVertices;
    int MaxConvexIndices;
    int MaxConvexUniqueEdges;
    int MaxCompoundChildShapes;
    int MaxTriConvexPairCapacity;
    
    BulletGPUSettings() :
        MaxConvexBodies(50000),
        MaxConvexShapes(15000),
        MaxBroadphasePairs(2000000),
        MaxContactCapacity(500000),
        CompoundPairCapacity(100000),
        MaxVerticesPerFace(64),
        MaxFacesPerShape(12),
        MaxConvexVertices(8192),
        MaxConvexIndices(81920),
        MaxConvexUniqueEdges(8192),
        MaxCompoundChildShapes(8192),
        MaxTriConvexPairCapacity(262144) {}
};

struct RayResult {
    bool hasHit;
    b3Vector3 hitNormalWorld;
    b3Vector3 hitPointWorld;
    int collisionObjectId;
    float hitFraction;
};

struct SweepResult {
    bool hasHit;
    b3Vector3 hitNormalWorld;
    b3Vector3 hitPointWorld;
    int collisionObjectId;
    float hitFraction;
};

struct ContactPoint {
    b3Vector3 positionWorldOnA;
    b3Vector3 positionWorldOnB;
    b3Vector3 normalWorldOnB;
    float distance;
    float appliedImpulse;
};

struct ManifoldPoint {
    b3Vector3 localPointA;
    b3Vector3 localPointB;
    b3Vector3 positionWorldOnA;
    b3Vector3 positionWorldOnB;
    b3Vector3 normalWorldOnB;
    float distance;
    float appliedImpulse;
    int lifeTime;
    void* userPersistentData;
};

#define MACRO_AS_STRING1(X) #X
#define MACRO_AS_STRING(X) MACRO_AS_STRING1(X)

static const int    DEFAULT_MAX_SUBSTEPS = 12;
static const float  DEFAULT_CONTACT_IMPULSE_THRESHOLD = 0.2f;

static std::string BulletSimVersionString = MACRO_AS_STRING(BULLETSIMVERSION) "," MACRO_AS_STRING(BULLETVERSION);

class BulletSim
{
private:
    // GPU management
    std::map<IDTYPE, int> gpuBodies;
    std::map<int, int> m_gpuShapeMap;
    std::vector<uint32_t> m_gpuBodyIds;
    std::vector<uint32_t> m_cpuBodyIds;
    
    BulletGPUSettings gpuSettings;
    
    int m_maxSubSteps;
    float m_contactImpulseThreshold;
    
    bool m_gpuAvailable = false;

    int m_dumpStatsCount;

    WorldData m_worldData;

    int m_maxUpdatesPerFrame;
    EntityProperties* m_updatesThisFrameArray;

    CollisionDesc* m_collidersThisFrameArray;
    std::set<COLLIDERKEYTYPE> m_collidersThisFrame;

public:
    // Constructor declaration
    BulletSim(float maxX, float maxY, float maxZ);

    // GPU shape management
    int registerGpuBoxShape(const b3Vector3& halfExtents);
    int registerGpuSphereShape(float radius);
    int registerGpuCapsuleShape(float radius, float height);
    int registerGpuCylinderShape(const b3Vector3& halfExtents);
    int registerGpuConvexHullShape(int numPoints, float* points);
    int registerGpuCompoundShape();
    void addChildShapeToGpuCompound(int compoundShapeId, int childShapeId, const b3Transform& transform);
    int registerGpuMeshShape(int indicesCount, int* indices, int verticesCount, float* vertices);
    int registerGpuTerrainShape(int width, int length, float* heightData, float minHeight, float maxHeight, float scale);

    // GPU rigid body management
    int registerGpuRigidBody(int shapeId, float mass, const b3Vector3& position, const b3Quaternion& rotation,
                            const b3Vector3& linearVelocity, const b3Vector3& angularVelocity);
    void setGpuBodyPosition(int bodyId, const b3Vector3& position);
    void setGpuBodyRotation(int bodyId, const b3Quaternion& rotation);
    void setGpuBodyLinearVelocity(int bodyId, const b3Vector3& velocity);
    void setGpuBodyAngularVelocity(int bodyId, const b3Vector3& velocity);
    void applyGpuBodyCentralForce(int bodyId, const b3Vector3& force);
    void applyGpuBodyCentralImpulse(int bodyId, const b3Vector3& impulse);
    void applyGpuBodyTorque(int bodyId, const b3Vector3& torque);

    // GPU queries
    bool gpuRayTest(const b3Vector3& from, const b3Vector3& to, RayResult* result);
    bool gpuConvexSweepTest(int shapeId, const b3Vector3& fromPos, const b3Quaternion& fromRot,
                           const b3Vector3& toPos, const b3Quaternion& toRot, SweepResult* result);
    int gpuGetContactPoints(int bodyIdA, int bodyIdB, ContactPoint* contacts, int maxContacts);
    bool gpuGetAabb(int bodyId, b3Vector3& aabbMin, b3Vector3& aabbMax);

    // GPU info and debug
    int gpuGetNumRigidBodies();
    int gpuGetNumCollisionObjects();
    void gpuDumpWorldState();

    // GPU constraints
    int createGpuPoint2PointConstraint(int bodyIdA, int bodyIdB, const b3Vector3& pivotInA, const b3Vector3& pivotInB);
    void removeGpuConstraint(int constraintId);

    // Object manipulation methods
    void AddObject2(int gpuBodyId, int group, int mask);
    void RemoveObject2(int gpuBodyId);
    void SetPositionAndOrientation2(int gpuBodyId, b3Vector3 pos, b3Quaternion rot);
    void SetLinearVelocity2(int gpuBodyId, b3Vector3 vel);
    void SetAngularVelocity2(int gpuBodyId, b3Vector3 vel);
    void SetMass2(int gpuBodyId, float mass);
    void SetFriction2(int gpuBodyId, float friction);
    void SetRestitution2(int gpuBodyId, float restitution);
    void SetGravity2(int gpuBodyId, b3Vector3 gravity);
    void SetDamping2(int gpuBodyId, float linearDamping, float angularDamping);
    void SetSleepingThresholds2(int gpuBodyId, float linearThreshold, float angularThreshold);
    void SetCollisionFlags2(int gpuBodyId, int flags);
    void SetCollisionFilter2(int gpuBodyId, int group, int mask);
    void SetActivationState2(int gpuBodyId, int state);
    void ApplyCentralForce2(int gpuBodyId, b3Vector3 force);
    void ApplyCentralImpulse2(int gpuBodyId, b3Vector3 impulse);
    void ApplyTorque2(int gpuBodyId, b3Vector3 torque);
    void ApplyTorqueImpulse2(int gpuBodyId, b3Vector3 impulse);
    void ApplyForce2(int gpuBodyId, b3Vector3 force, b3Vector3 pos);
    void ApplyImpulse2(int gpuBodyId, b3Vector3 impulse, b3Vector3 pos);
    void ClearForces2(int gpuBodyId);

    SweepResult ConvexSweepTest(int shapeId, b3Vector3& fromPos, b3Vector3& targetPos, float extraMargin);
    RayResult RayTest(b3Vector3& from, b3Vector3& to, short filterGroup, short filterMask);
    const b3Vector3 RecoverFromPenetration(IDTYPE id);
    
    // Query methods
    bool RayTest2(b3Vector3 from, b3Vector3 to, RayResult* result);
    bool ConvexSweepTest2(int shapeId, b3Vector3 fromPos, b3Quaternion fromRot, 
                         b3Vector3 toPos, b3Quaternion toRot, SweepResult* result);
    int GetContactPoints2(int bodyIdA, int bodyIdB, 
                         ContactPoint* contacts, int maxContacts);
    int GetManifoldPoints2(int bodyId, ManifoldPoint* points, int maxPoints);
    bool GetAabb2(int bodyId, b3Vector3* aabbMin, b3Vector3* aabbMax);
    
    // Debug methods
    void DumpWorld2();
    void DumpObject2(int bodyId);
    int GetNumCollisionObjects2();
    int GetNumManifolds2();
    int GetNumContactPoints2();
    
    bool isGpuAvailable() const { return m_gpuAvailable; }
    GpuPhysicsEngine* m_gpuEngine;
    void registerGpuBody(uint32_t id, uint32_t gpuId);
    void registerCpuBody(uint32_t id);

    void SetMaxSubSteps(int steps) { m_maxSubSteps = steps; }
    void SetContactImpulseThreshold(float threshold) { m_contactImpulseThreshold = threshold; }
    float getContactImpulseThreshold() const { return m_contactImpulseThreshold; }
    
	// Keep existing method for collision objects
    btCollisionObject* findBodyById(uint32_t id);
	
    void synchronizeGpuCpuData();
    
    virtual ~BulletSim()
	{
		exitPhysics2();
	}

    void initPhysics2(ParamBlock* parms, int maxCollisions, CollisionDesc* collisionArray, int maxUpdates, EntityProperties* updateArray);
    void exitPhysics2();

    // Add PhysicsStep2 declaration
	int PhysicsStep2(btScalar timeStep, int maxSubSteps, btScalar fixedTimeStep, int* updatedEntityCount, int* collidersCount);


    // GPU shape creation methods
    int CreateGpuMeshShape(int indicesCount, int* indices, int verticesCount, float* vertices);
    int CreateGpuHullShape(int hullCount, float* hulls);
    int CreateGpuBoxShape(b3Vector3 halfExtents);
    int CreateGpuSphereShape(float radius);
    int CreateGpuCylinderShape(b3Vector3 halfExtents);
    int CreateGpuCapsuleShape(float radius, float height);
    int CreateGpuConeShape(float radius, float height);
    int CreateGpuMultiSphereShape(int sphereCount, b3Vector3* positions, float* radii);
    int CreateGpuPlaneShape(b3Vector3 planeNormal, float planeConstant);
    int CreateGpuHeightfieldShape(int width, int length, float* heightData, float minHeight, float maxHeight);
    int CreateGpuCompoundShape();
    void AddChildShapeToGpuCompound(int compoundShapeId, int childShapeId, b3Vector3 position, b3Quaternion rotation);

    int registerPhysicsInstance2(float mass, const b3Vector3& position, const b3Quaternion& orientation, int collisionShapeIndex, int userData);

    int maxCollisionsPerFrame;
    int collisionsThisFrame;
    void RecordCollision(int objA, int objB, const b3Vector3& contact, const b3Vector3& norm, float penetration);

    WorldData* getWorldData() { return &m_worldData; }
    btDynamicsWorld* getDynamicsWorld() { return m_worldData.dynamicsWorld; };

    bool UpdateParameter2(IDTYPE localID, const char* parm, float value);
    void DumpPhysicsStats();
	
	RaycastHit RayTest(btVector3& from, btVector3& to, short filterGroup, short filterMask);

protected:
    void CreateGroundPlane();
    void CreateTerrain();
};

#endif //BULLET_SIM_H