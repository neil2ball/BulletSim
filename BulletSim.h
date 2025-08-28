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

#define CL_TARGET_OPENCL_VERSION 120
#include <CL/cl.h>

#include "DebugLogic.h"

#include "ArchStuff.h"
#include "APIData.h"
#include "WorldData.h"

#include "Bullet3Common/b3Vector3.h"
#include "Bullet3Common/b3Quaternion.h"
#include "Bullet3Common/b3Transform.h"



// Add GPU acceleration includes
#include "Bullet3OpenCL/Initialize/b3OpenCLUtils.h"
#include "Bullet3OpenCL/BroadphaseCollision/b3GpuBroadphaseInterface.h"
#include "Bullet3OpenCL/BroadphaseCollision/b3GpuSapBroadphase.h"
#include "Bullet3OpenCL/RigidBody/b3GpuNarrowPhase.h"
#include "Bullet3OpenCL/RigidBody/b3GpuRigidBodyPipeline.h"

#include "BulletCollision/CollisionDispatch/btGhostObject.h"
#include "LinearMath/btAlignedObjectArray.h"
#include "LinearMath/btMotionState.h"
#include "btBulletDynamicsCommon.h"

// Add missing include for btDynamicsWorld methods
#include "BulletDynamics/Dynamics/btDiscreteDynamicsWorld.h"

#include <set>
#include <map>
#include <vector>
#include <cstdint> // For uint32_t

class b3GpuRigidBodyPipeline;
class b3GpuBroadphaseInterface;
class b3GpuNarrowPhase;
struct b3RigidBodyData;

inline btVector3 b3ToBt(const b3Vector3& v) {
    return btVector3(v.x, v.y, v.z);
}

inline b3Vector3 btToB3(const btVector3& v) {
    b3Vector3 result;
    result.setValue(v.x(), v.y(), v.z());
    return result;
}

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
    
    // Constructor with theoretical OpenSim default values for 8GB GPU
    BulletGPUSettings() :
        MaxConvexBodies(50000),				// Up to 50,000 physics objects (vs default 131,072) [Memory Resource Reallocation]
        MaxConvexShapes(15000),				// 15,000 unique shapes (vs default 131,072) [Memory Resource Reallocation]
        MaxBroadphasePairs(2000000),		// 2 million potential collision pairs (vs default 2,097,152) [Memory Resource Reallocation]
        MaxContactCapacity(500000),			// 500,000 simultaneous contacts (vs default 2,097,152) [Memory Resource Reallocation]
        CompoundPairCapacity(100000),		// 100,000 compound object pairs (vs default 1,048,576) [Memory Resource Reallocation]
        MaxVerticesPerFace(64),				// same as OpenCL default - Maximum number of vertices allowed in a single face/polygon
        MaxFacesPerShape(12),				// same as OpenCL default - Maximum number of faces/polygons a convex shape can have
        MaxConvexVertices(8192),			// same as OpenCL default - Maximum total vertices for all convex shapes in the simulation
        MaxConvexIndices(81920),			// same as OpenCL default - Maximum total indices (vertex connections) for convex shapes
        MaxConvexUniqueEdges(8192),			// same as OpenCL default - Maximum unique edges for convex shape collision detection
        MaxCompoundChildShapes(8192),		// same as OpenCL default - Maximum child shapes allowed in compound objects
        MaxTriConvexPairCapacity(262144) {} // same as OpenCL default - Maximum triangle-convex collision pairs for mesh vs convex collisions
};

// Vertex array that manages the memory for copied mesh data to prevent leaks
class ManagedTriangleIndexVertexArray : public btTriangleIndexVertexArray
{
public:
    btAlignedObjectArray<std::pair<void*, void*>> m_allocatedData; // pairs of [indexData, vertexData]

    virtual ~ManagedTriangleIndexVertexArray()
    {
        for (int i = 0; i < m_allocatedData.size(); ++i)
        {
            delete[] (int*)m_allocatedData[i].first;    // delete index array
            delete[] (float*)m_allocatedData[i].second; // delete vertex array
        }
        m_allocatedData.clear();
    }

    // Helper function to add a mesh and track its allocated data
    void addManagedIndexedMesh(const btIndexedMesh& mesh, void* indexData, void* vertexData, PHY_ScalarType indexType = PHY_INTEGER)
    {
        addIndexedMesh(mesh, indexType);
        m_allocatedData.push_back(std::make_pair(indexData, vertexData));
    }
};

// #define TOLERANCE 0.00001
// these values match the ones in SceneObjectPart.SendScheduledUpdates()
#define POSITION_TOLERANCE 0.05f
#define VELOCITY_TOLERANCE 0.001f
#define ROTATION_TOLERANCE 0.01f
#define ANGULARVELOCITY_TOLERANCE 0.01f

// If defined, use the HACD included with the Bullet distribution
#define USEBULLETHACD 1

// If defined, use the external VHACD library
// #define USEVHACD 1

// Build version string of format BULLETSIMVERSION,BULLETENGINEVERSION ("1.3,3.25")
// This expects the version information to be passed in as defined variables (-D BULLETVERSION=...)
//   If the version variables are not defined, the version string is just a comma.
//   This uses the "#" pre-processor operator to stringify the variable and that adjacent strings are concatinated.
#define MACRO_AS_STRING1(X) #X
#define MACRO_AS_STRING(X) MACRO_AS_STRING1(X)

//increase sub-step resolution and filter by contact impulse
static const int    DEFAULT_MAX_SUBSTEPS = 12;
static const float  DEFAULT_CONTACT_IMPULSE_THRESHOLD = 0.2f;


static std::string BulletSimVersionString = MACRO_AS_STRING(BULLETSIMVERSION) "," MACRO_AS_STRING(BULLETVERSION);

// Helper method to determine if an object is phantom or not
static bool IsPhantom(const btCollisionObject* obj)
{
	// Characters are never phantom, but everything else with CF_NO_CONTACT_RESPONSE is
	// TODO: figure out of this assumption for phantom sensing is still true
	//    This is used in the raycast code an should be rethought for the real implementation.
	return obj->getCollisionShape()->getShapeType() != CAPSULE_SHAPE_PROXYTYPE &&
		(obj->getCollisionFlags() & btCollisionObject::CF_NO_CONTACT_RESPONSE) != 0;
};


// ============================================================================================
// Motion state for rigid bodies in the scene. Updates the map of changed 
// entities whenever the setWorldTransform callback is fired
class SimMotionState : public btMotionState
{
public:
	btRigidBody* RigidBody;
	Vector3 ZeroVect;

    SimMotionState(IDTYPE id, const btTransform& startTransform, std::map<IDTYPE, EntityProperties*>* updatesThisFrame)
		: m_properties(id, startTransform), m_lastProperties(id, startTransform)
	{
        m_xform = startTransform;
		m_updatesThisFrame = updatesThisFrame;
    }

    virtual ~SimMotionState()
	{
		m_updatesThisFrame->erase(m_properties.ID);
    }

    virtual void getWorldTransform(btTransform& worldTrans) const
	{
        worldTrans = m_xform;
    }

    virtual void setWorldTransform(const btTransform& worldTrans)
	{
	    setWorldTransform(worldTrans, false);
	}

    virtual void setWorldTransform(const btTransform& worldTrans, bool force)
	{
		m_xform = worldTrans;

		// Put the new transform into m_properties
		if (((RigidBody->getCollisionFlags() & BS_RETURN_ROOT_COMPOUND_SHAPE) != 0) 
								&& RigidBody->getCollisionShape()->isCompound())
		{
			// If this is a compound shape, return the position of the zero shape
			//     (the root of the linkset).
			btCompoundShape* cShape = (btCompoundShape*)RigidBody->getCollisionShape();
			btTransform rootChildTransformL = cShape->getChildTransform(0);
			btTransform rootChildTransformW = worldTrans * rootChildTransformL;
			m_properties.Position = rootChildTransformW.getOrigin();
			m_properties.Rotation = rootChildTransformW.getRotation();
			m_properties.AngularVelocity = RigidBody->getAngularVelocity();
		}
		else
		{
			m_properties.Position = m_xform.getOrigin();
			m_properties.Rotation = m_xform.getRotation();
			m_properties.AngularVelocity = RigidBody->getAngularVelocity();
		}
		// A problem with stock Bullet is that we don't get an event when an object is deactivated.
		// This means that the last non-zero values for linear and angular velocity
		// are left in the viewer who does dead reconning and the objects look like
		// they float off.
		// BulletSim ships with a patch to Bullet which creates such an event.
		m_properties.Velocity = RigidBody->getLinearVelocity();

		// Is this transform any different from the previous one?
		// TODO: decide of this 'if' statement is needed. Since the updates are kept by ID,
		//     couldn't we just always put any update into the map? The only down side would
		//     be sending updates every tick for very small jiggles which happen over a long period of time.
		if (force
			|| !m_properties.Position.AlmostEqual(m_lastProperties.Position, POSITION_TOLERANCE)
			|| !m_properties.Rotation.AlmostEqual(m_lastProperties.Rotation, ROTATION_TOLERANCE)
			// If the Velocity and AngularVelocity are zero, most likely the object has
			//    been deactivated. If they both are zero and they have become zero recently,
			//    make sure a property update is sent so the zeros make it to the viewer.
			|| ((m_properties.Velocity == ZeroVect && m_properties.AngularVelocity == ZeroVect)
				&& (m_properties.Velocity != m_lastProperties.Velocity || m_properties.AngularVelocity != m_lastProperties.AngularVelocity))
			//	If Velocity and AngularVelocity are non-zero but have changed, send an update.
			|| !m_properties.Velocity.AlmostEqual(m_lastProperties.Velocity, VELOCITY_TOLERANCE)
			|| !m_properties.AngularVelocity.AlmostEqual(m_lastProperties.AngularVelocity, ANGULARVELOCITY_TOLERANCE)
			)
		{
			// Add this update to the list of updates for this frame.
			m_lastProperties = m_properties;
			(*m_updatesThisFrame)[m_properties.ID] = &m_properties;
		}
    }

private:
	std::map<IDTYPE, EntityProperties*>* m_updatesThisFrame;
    btTransform m_xform;
	EntityProperties m_properties;
	EntityProperties m_lastProperties;
};

// ============================================================================================
// Callback for convex sweeps that excludes the object being swept
class ClosestNotMeConvexResultCallback : public btCollisionWorld::ClosestConvexResultCallback
{
public:
	ClosestNotMeConvexResultCallback (btCollisionObject* me) : btCollisionWorld::ClosestConvexResultCallback(btVector3(0.0, 0.0, 0.0), btVector3(0.0, 0.0, 0.0))
	{
		m_me = me;
	}

	virtual btScalar addSingleResult(btCollisionWorld::LocalConvexResult& convexResult,bool normalInWorldSpace)
	{
		// Ignore collisions with ourself and phantom objects
		if (convexResult.m_hitCollisionObject == m_me || IsPhantom(convexResult.m_hitCollisionObject))
			return 1.0;

		return ClosestConvexResultCallback::addSingleResult (convexResult, normalInWorldSpace);
	}
protected:
	btCollisionObject* m_me;
};

// ============================================================================================
// Callback for raycasts that excludes the object doing the raycast
class ClosestNotMeRayResultCallback : public btCollisionWorld::ClosestRayResultCallback
{
public:
	ClosestNotMeRayResultCallback (btCollisionObject* me) : btCollisionWorld::ClosestRayResultCallback(btVector3(0.0, 0.0, 0.0), btVector3(0.0, 0.0, 0.0))
	{
		m_me = me;
	}

	virtual btScalar addSingleResult(btCollisionWorld::LocalRayResult& rayResult,bool normalInWorldSpace)
	{
		if (rayResult.m_collisionObject == m_me || IsPhantom(rayResult.m_collisionObject))
			return 1.0;

		return ClosestRayResultCallback::addSingleResult (rayResult, normalInWorldSpace);
	}
protected:
	btCollisionObject* m_me;
};

// ============================================================================================
// Callback for non-moving overlap tests
class ContactSensorCallback : public btCollisionWorld::ContactResultCallback
{
public:
	btVector3 mOffset;

	ContactSensorCallback(btCollisionObject* collider)
		: btCollisionWorld::ContactResultCallback(), m_me(collider), m_maxPenetration(0.0), mOffset(0.0, 0.0, 0.0)
	{
	}

	virtual	btScalar addSingleResult(btManifoldPoint& cp, const btCollisionObject* colObj0, int partId0, int index0, const btCollisionObject* colObj1, int partId1, int index1)
	{
		// Ignore terrain collisions
		if (colObj0->getCollisionShape()->getShapeType() == TRIANGLE_SHAPE_PROXYTYPE ||
			colObj1->getCollisionShape()->getShapeType() == TRIANGLE_SHAPE_PROXYTYPE)
		{
			return 0;
		}

		// Ignore collisions with phantom objects
		if (IsPhantom(colObj0) || IsPhantom(colObj1))
		{
			return 0;
		}

		btScalar distance = cp.getDistance();

		if (distance < m_maxPenetration)
		{
			m_maxPenetration = distance;

			// Figure out if we are the first or second body in the collision 
			// pair so we know which direction to point the collision normal
			btScalar directionSign = (colObj0 == m_me) ? btScalar(-1.0) : btScalar(1.0);

			// Push offset back using the collision normal and the depth of the collision
			btVector3 touchingNormal = cp.m_normalWorldOnB * directionSign;
			mOffset = touchingNormal * distance;
		}

		return 0;
	}

protected:
	btCollisionObject* m_me;
	btScalar m_maxPenetration;
};

// ============================================================================================
// The main physics simulation class.
class BulletSim
{
private:

	// In BulletSim.h, add these to the private section of the BulletSim class
	b3DynamicBvhBroadphase* m_broadphaseDbvt;
	b3Config m_config;
	BulletGPUSettings gpuSettings; // Add this near other GPU-related members
	
	//increase sub-step resolution and filter by contact impulse
	int m_maxSubSteps;
    float m_contactImpulseThreshold;
	
	
	// Bullet world objects
	btBroadphaseInterface* m_broadphase;
	btCollisionDispatcher* m_dispatcher;
	btConstraintSolver*	m_solver;
	btDefaultCollisionConfiguration* m_collisionConfiguration;

	bool m_gpuAvailable = false;
	
    b3GpuRigidBodyPipeline* m_gpuPipeline = nullptr;
    b3GpuBroadphaseInterface* m_gpuBroadphase = nullptr;
    b3GpuNarrowPhase* m_gpuNarrowphase = nullptr;
    cl_context m_openclContext = nullptr;
    cl_command_queue m_openclQueue = nullptr;
    cl_device_id m_openclDevice = nullptr;
	std::map<btCollisionShape*, int> m_gpuShapeMap; // Maps CPU shapes to GPU shape indices

    std::vector<uint32_t> m_gpuBodyIds;
    std::vector<uint32_t> m_cpuBodyIds;
	
	int m_dumpStatsCount;

	// Information about the world that is shared with all the objects
	WorldData m_worldData;

	// Where we process the tick's updates for passing back to managed code
	int m_maxUpdatesPerFrame;
	EntityProperties* m_updatesThisFrameArray;

	// Used to expose colliders from Bullet to the BulletSim API
	CollisionDesc* m_collidersThisFrameArray;
	std::set<COLLIDERKEYTYPE> m_collidersThisFrame;

public:
	
	int registerGpuShape(btCollisionShape* shape);
	// GPU acceleration objects
	bool isGpuAvailable() const { return m_gpuAvailable; }
    b3GpuRigidBodyPipeline* getGpuPipeline() { return m_gpuPipeline; }
    void registerGpuBody(uint32_t id, uint32_t gpuId);
    void registerCpuBody(uint32_t id);

	//increase sub-step resolution and filter by contact impulse
    void SetMaxSubSteps(int steps) { m_maxSubSteps = steps; }
    void SetContactImpulseThreshold(float threshold) { m_contactImpulseThreshold = threshold; }
    float getContactImpulseThreshold() const { return m_contactImpulseThreshold; }
	
	// Keep existing method for collision objects
    btCollisionObject* findBodyById(uint32_t id);
    
    // Add rigid body specific version if needed
    btRigidBody* findRigidBodyById(uint32_t id);
    
    // Declare the new synchronization method
    void synchronizeGpuCpuData();
	
	BulletSim(btScalar maxX, btScalar maxY, btScalar maxZ);

	virtual ~BulletSim()
	{
		exitPhysics2();
	}

	void initPhysics2(ParamBlock* parms, int maxCollisions, CollisionDesc* collisionArray, int maxUpdates, EntityProperties* updateArray);
	void exitPhysics2();

	int PhysicsStep2(btScalar timeStep, int maxSubSteps, btScalar fixedTimeStep, int* updatedEntityCount, int* collidersCount);

	btCollisionShape* CreateMeshShape2(int indicesCount, int* indices, int verticesCount, float* vertices);
	btCollisionShape* CreateGImpactShape2(int indicesCount, int* indices, int verticesCount, float* vertices);
	btCollisionShape* CreateHullShape2(int hullCount, float* hulls );
	btCollisionShape* BuildHullShapeFromMesh2(btCollisionShape* mesh, HACDParams* parms);
	btCollisionShape* BuildVHACDHullShapeFromMesh2(btCollisionShape* mesh, HACDParams* parms);
	btCollisionShape* BuildConvexHullShapeFromMesh2(btCollisionShape* mesh);
	btCollisionShape* CreateConvexHullShape2(int indicesCount, int* indices, int verticesCount, float* vertices);
	btCollisionShape* CreateBoxShape2(btVector3 halfExtents);
	btCollisionShape* CreateSphereShape2(btScalar radius);
	btCollisionShape* CreateCylinderShape2(btVector3 halfExtents);
	btCollisionShape* CreateCapsuleShape2(btScalar radius, btScalar height);
	btCollisionShape* CreateConeShape2(btScalar radius, btScalar height);
	btCollisionShape* CreateMultiSphereShape2(int sphereCount, btVector3* positions, btScalar* radii);
	btCollisionShape* CreatePlaneShape2(btVector3 planeNormal, btScalar planeConstant);
	btCollisionShape* CreateHeightfieldShape2(int width, int length, float* heightData, btScalar minHeight, btScalar maxHeight);
	btCollisionShape* CreateEmptyShape2();
	btCollisionShape* CreateCompoundShape2();
	void AddChildShapeToCompound2(btCompoundShape* compoundShape, btCollisionShape* childShape, btVector3 position, btQuaternion rotation);
	int registerPhysicsInstance2(float mass, const btVector3& position, const btQuaternion& orientation, int collisionShapeIndex, int userData);

	// Missing function declarations
	btCollisionShape* CreateHeightfieldTerrainShape2(int width, int length, float* heightfieldData, 
                                                   float minHeight, float maxHeight, float heightScale, 
                                                   int upAxis, int heightDataType);
	btCollisionShape* CreateConvexTriangleMeshShape2(int indicesCount, int* indices, 
                                                    int verticesCount, float* vertices);
	btCollisionShape* CreateStaticPlaneShape2(btVector3 planeNormal, btScalar planeConstant);
	btCollisionShape* CreateHACDShape2(int indicesCount, int* indices, int verticesCount, float* vertices);
	btCollisionShape* CreateVHACDShape2(int indicesCount, int* indices, int verticesCount, float* vertices);

	// Collisions: called to add a collision record to the collisions for a simulation step
	int maxCollisionsPerFrame;
	int collisionsThisFrame;
	void RecordCollision(const btCollisionObject* objA, const btCollisionObject* objB, 
							const btVector3& contact, const btVector3& norm, const float penetration);
	void RecordGhostCollisions(btPairCachingGhostObject* obj);

	SweepHit ConvexSweepTest(btCollisionShape* obj, btVector3& fromPos, btVector3& targetPos, btScalar extraMargin);
	RaycastHit RayTest(btVector3& from, btVector3& to, short filterGroup, short filterMask);
	const btVector3 RecoverFromPenetration(IDTYPE id);

	WorldData* getWorldData() { return &m_worldData; }
	btDynamicsWorld* getDynamicsWorld() { return m_worldData.dynamicsWorld; };

	bool UpdateParameter2(IDTYPE localID, const char* parm, float value);
	void DumpPhysicsStats();

	// Add missing function declarations
	btCollisionObject* CreateCollisionObject2(btCollisionShape* collisionShape, IDTYPE localID, int collisionType, int collisionFilterGroup, int collisionFilterMask);
	btRigidBody* CreateRigidBody2(btScalar mass, btMotionState* motionState, btCollisionShape* collisionShape, IDTYPE localID, int collisionType, int collisionFilterGroup, int collisionFilterMask);
	btPairCachingGhostObject* CreateGhostObject2(btCollisionShape* collisionShape, IDTYPE localID, int collisionFilterGroup, int collisionFilterMask);
	void DestroyCollisionObject2(btCollisionObject* collisionObject);
	void AddChildShapeToCompoundShape2(btCompoundShape* compoundShape, btCollisionShape* childShape, btTransform& localTransform);
	void RemoveChildShapeFromCompoundShape2(btCompoundShape* compoundShape, btCollisionShape* childShape);
	void UpdateChildTransformInCompoundShape2(btCompoundShape* compoundShape, btCollisionShape* childShape, btTransform& newLocalTransform);
	void SetCollisionShapeMargin2(btCollisionShape* collisionShape, btScalar margin);
	void SetCollisionObjectActivationState2(btCollisionObject* collisionObject, int activationState);

protected:
	void CreateGroundPlane();
	void CreateTerrain();
};

#endif //BULLET_SIM_H
