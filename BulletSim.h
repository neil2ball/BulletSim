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

#define CL_TARGET_OPENCL_VERSION 120
#include <clew/clew.h>
#include <CL/cl.h>

#include "DebugLogic.h"
#include "ArchStuff.h"
#include "APIData.h"
#include "WorldData.h"
#include "ShapeData.h"

#include "Bullet3OpenCL/RigidBody/b3GpuRigidBodyPipeline.h"
#include "Bullet3Common/b3Vector3.h"
#include "Bullet3Common/b3Quaternion.h"
#include "Bullet3Common/b3Transform.h"
#include "Bullet3OpenCL/Initialize/b3OpenCLUtils.h"
#include "Bullet3OpenCL/BroadphaseCollision/b3GpuBroadphaseInterface.h"
#include "Bullet3OpenCL/BroadphaseCollision/b3GpuSapBroadphase.h"
#include "Bullet3OpenCL/RigidBody/b3GpuNarrowPhase.h"
#include "Bullet3Collision/NarrowPhaseCollision/b3RaycastInfo.h"

#include <BulletCollision/CollisionDispatch/btGhostObject.h>

#include <BulletDynamics/Dynamics/btDynamicsWorld.h>
#include <BulletDynamics/Dynamics/btRigidBody.h>
#include <BulletCollision/CollisionShapes/btCollisionShape.h>
#include <LinearMath/btDefaultMotionState.h>


#include <LinearMath/btScalar.h>

#include <set>
#include <map>
#include <vector>
#include <cstdint>
#include <string>

#ifndef PHYS_LOG
    #include <cstdio>
    #define PHYS_LOG(fmt, ...) do { std::printf("[PHYS] " fmt "\n", ##__VA_ARGS__); } while (0)
#endif

extern btDynamicsWorld* gWorld;

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
    std::vector<uint32_t> m_gpuBodyIds;
    std::vector<uint32_t> m_cpuBodyIds;
    
    int m_maxSubSteps;
    float m_contactImpulseThreshold;
    
    bool m_gpuAvailable = false;

    int m_dumpStatsCount;

    WorldData m_worldData;

    int m_maxUpdatesPerFrame;
    EntityProperties* m_updatesThisFrameArray;

    CollisionDesc* m_collidersThisFrameArray;
    std::set<COLLIDERKEYTYPE> m_collidersThisFrame;
	
	// Bullet world objects
	btBroadphaseInterface* m_broadphase;
	btCollisionDispatcher* m_dispatcher;
	btConstraintSolver*	m_solver;
	btDefaultCollisionConfiguration* m_collisionConfiguration;


public:
    // Constructor declaration
    BulletSim(float maxX, float maxY, float maxZ);
	
	
	WorldData* getWorldData() { return &m_worldData; }
	const WorldData* getWorldData() const { return &m_worldData; }
	
	std::map<btCollisionObject*, int> m_objToGpuBodyId;
	
	void setGpuBodyLinearVelocity(uint32_t bodyId, const b3Vector3& velocity);
	void setGpuBodyAngularVelocity(uint32_t bodyId, const b3Vector3& velocity);


	bool gpuRayTest(const b3Vector3& from, const b3Vector3& to, RayResult* result);

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

    SweepHit ConvexSweepTest(btCollisionShape* obj, btVector3& fromPos, btVector3& targetPos, btScalar extraMargin);
    RayResult RayTest(b3Vector3& from, b3Vector3& to, short filterGroup, short filterMask);
    const btVector3 RecoverFromPenetration(IDTYPE id);
    
    // Query methods
    bool RayTest2(b3Vector3 from, b3Vector3 to, RayResult* result);
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
	
	bool initOpenCL(cl_context &context, cl_device_id &device, cl_command_queue &queue);
	
	std::map<btCollisionShape*, int> m_gpuShapeMap;
	int registerGpuShape(btCollisionShape* shape);
	
	void registerWithGpu(int collidableIndex, int id, const btTransform& startTransform);
	
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
	int BulletSim::PhysicsStep2(btScalar timeStep, int maxSubSteps, btScalar fixedTimeStep, int* updatedEntityCount, int* collidersCount);


	float pickDensityForShape(const btCollisionShape* shape);
	float computeMassFromShape(const btCollisionShape* shape);
	float computeShapeVolume(const btCollisionShape* shape);

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
	
	void RecordCollision(const btCollisionObject* objA, const btCollisionObject* objB, const btVector3& contact, const btVector3& norm, const float penetration);
	void RecordGhostCollisions(btPairCachingGhostObject* obj);

    
    btDynamicsWorld* getDynamicsWorld() { return m_worldData.dynamicsWorld; };

    bool UpdateParameter2(IDTYPE localID, const char* parm, float value);
    void DumpPhysicsStats();
	
	RaycastHit RayTest(btVector3& from, btVector3& to, short filterGroup, short filterMask);
	
	static bool SingleSidedMeshCheckCallback(btManifoldPoint& cp, 
							const btCollisionObjectWrapper* colObj0, int partId0, int index0,
							const btCollisionObjectWrapper* colObj1, int partId1, int index1);
							
	/**
	 * Maps the GPU body buffer and returns a pointer to the requested body data.
	 * 
	 *    The returned pointer is valid only until the buffer is unmapped.
	 *    The caller is responsible for calling:
	 *        clEnqueueUnmapMemObject(queue, buf, hostPtr, 0, nullptr, nullptr);
	 *    when finished with the data.
	 * 
	 * @param bodyId Index of the body to retrieve.
	 * @return Pointer to the mapped b3RigidBodyData for the given body.
	 */
	b3RigidBodyData* getGpuBodyData(uint32_t bodyId);
	
	btCollisionShape* CreateMeshShape2(int indicesCount, int* indices, int verticesCount, float* vertices);
	btCollisionShape* CreateGImpactShape2(int indicesCount, int* indices, int verticesCount, float* vertices);
	btCollisionShape* CreateHullShape2(int hullCount, float* hulls );
	btCollisionShape* BuildHullShapeFromMesh2(btCollisionShape* mesh, HACDParams* parms);
	btCollisionShape* BuildVHACDHullShapeFromMesh2(btCollisionShape* mesh, HACDParams* parms);
	btCollisionShape* BuildConvexHullShapeFromMesh2(btCollisionShape* mesh);
	btCollisionShape* CreateConvexHullShape2(int indicesCount, int* indices, int verticesCount, float* vertices);


protected:
    void CreateGroundPlane();
    void CreateTerrain();
};

// ============================================================================================
// Motion state for rigid bodies in the scene. Updates the map of changed 
// entities whenever the setWorldTransform callback is fired

class SimMotionState : public btMotionState
{
public:
    btRigidBody* RigidBody;
    btVector3 ZeroVect; // Stays as btVector3

    // Static tolerance constants - move these to BulletSim.cpp
    static const btScalar POSITION_TOLERANCE;
    static const btScalar ROTATION_TOLERANCE;
    static const btScalar VELOCITY_TOLERANCE;
    static const btScalar ANGULARVELOCITY_TOLERANCE;

	SimMotionState(
		IDTYPE id,
		const btTransform& startTransform,
		std::map<IDTYPE, EntityProperties*>* updatesThisFrame)
		: m_properties(id, btToB3Transform(startTransform)),
		  m_lastProperties(id, btToB3Transform(startTransform)),
		  m_xformCPU(startTransform),
		  m_xformGPU(btToB3Transform(startTransform)),
		  m_updatesThisFrame(updatesThisFrame),
		  RigidBody(nullptr),
		  ZeroVect(0.0f, 0.0f, 0.0f) // ← No comma here!
	{
		ZeroVect = btVector3(0, 0, 0); // Optional, since you already initialized it
	}
	

    virtual ~SimMotionState()
    {
        m_updatesThisFrame->erase(m_properties.ID);
    }
	
	virtual void getWorldTransformGPU(b3Transform& worldTrans) const
    {
        worldTrans = m_xformGPU;
    }

    virtual void getWorldTransformCPU(btTransform& worldTrans) const
    {
        worldTrans = m_xformCPU;
    }
	
	btRigidBody* getRigidBody() const { return RigidBody; }
    void setRigidBody(btRigidBody* rb) { RigidBody = rb; }
	
    void setWorldTransformCPU(const btTransform& worldTrans, bool force)
	{
		if (!RigidBody) return;

		btCollisionShape* shape = RigidBody->getCollisionShape();
		if (!shape) return;

		m_xformCPU = worldTrans; // pure btTransform, no b3 conversion

		if ((RigidBody->getCollisionFlags() & BS_RETURN_ROOT_COMPOUND_SHAPE) != 0 
			&& shape->isCompound())
		{
			btCompoundShape* cShape = static_cast<btCompoundShape*>(shape);
			if (cShape->getNumChildShapes() > 0) {
				btTransform rootChildTransformL = cShape->getChildTransform(0);
				btTransform rootChildTransformW = worldTrans * rootChildTransformL;

				m_properties.Position        = rootChildTransformW.getOrigin();      // btVector3
				m_properties.Rotation        = rootChildTransformW.getRotation();    // btQuaternion
				m_properties.AngularVelocity = RigidBody->getAngularVelocity();
			}
		}
		else
		{
			m_properties.Position        = worldTrans.getOrigin();
			m_properties.Rotation        = worldTrans.getRotation();
			m_properties.AngularVelocity = RigidBody->getAngularVelocity();
		}

		m_properties.Velocity = RigidBody->getLinearVelocity();

		if (m_updatesThisFrame) {
			m_lastProperties = m_properties;
			(*m_updatesThisFrame)[m_properties.ID] = &m_properties;
		}
	}
	///setWorldTransformGPU NEEDS SERIOUS WORK!
	/*btCollisionShape* btShape = RigidBody->getCollisionShape();

		// Example: if it's a box shape
		if (btBoxShape* box = dynamic_cast<btBoxShape*>(btShape)) {
			btVector3 halfExtents = box->getHalfExtentsWithMargin();

			b3CollisionShapeData gpuShape;
			gpuShape.m_shapeType = BOX_SHAPE_PROXYTYPE;
			gpuShape.m_dimensions[0] = halfExtents.getX();
			gpuShape.m_dimensions[1] = halfExtents.getY();
			gpuShape.m_dimensions[2] = halfExtents.getZ();

			// Now gpuShape can be used to initialize a b3CollisionShape
		}
		
		Prepare b3CollisionShapeData: You already have this part—populate the fields like m_shapeType, m_dimensions, m_localScaling, etc.

		Allocate GPU memory for shapes: You’ll need to create a buffer on the GPU to hold your shape data. This is typically done using Bullet’s GPU utilities.

		cpp
		b3CollisionShapeData* shapeDataHost = new b3CollisionShapeData[numShapes];
		// Fill shapeDataHost[i] with your shape info

		cl_mem shapeBuffer = clCreateBuffer(context, CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR,
											sizeof(b3CollisionShapeData) * numShapes,
											shapeDataHost, &err);
		Register the shape buffer with the GPU pipeline: You’ll pass this buffer to the GPU pipeline, often via b3GpuNarrowPhase::setShapeData() or similar.

		cpp
		gpuNarrowPhase->setShapeData(shapeBuffer, numShapes);
		Associate the shape with a rigid body: When creating a rigid body in the GPU pipeline, you’ll reference the index of the shape in the buffer.

		cpp
		b3RigidBodyData body;
		body.m_collisionShapeIndex = shapeIndex; // Index into shape buffer

	*/
	void setWorldTransformGPU(const b3Transform& worldTrans, bool force)
	{
		/*// Bail if the rigid body pointer is invalid
		if (!RigidBody) return;

		btCollisionShape* shape = RigidBody->getCollisionShape();
		if (!shape) return;

		m_xformGPU = worldTrans;

		if ((RigidBody->getCollisionFlags() & BS_RETURN_ROOT_COMPOUND_SHAPE) != 0 
			&& shape->isCompound())
		{
			btCompoundShape* cShape = static_cast<btCompoundShape*>(shape);
			if (cShape->getNumChildShapes() > 0) {
				btTransform rootChildTransformL = cShape->getChildTransform(0);      //good luck making this work for gpu
				btTransform rootChildTransformW = worldTrans * rootChildTransformL;  //good luck making this work for gpu

				m_properties.Position        = rootChildTransformW.getOrigin();
				m_properties.Rotation        = rootChildTransformW.getRotation();
				m_properties.AngularVelocity = RigidBody->getAngularVelocity();
			}
		}
		else
		{
			m_properties.Position        = worldTrans.getOrigin();
			m_properties.Rotation        = worldTrans.getRotation();
			m_properties.AngularVelocity = RigidBody->getAngularVelocity();
		}

		m_properties.Velocity = RigidBody->getLinearVelocity();

		// Only push the update if the container exists
		if (m_updatesThisFrame) {
			m_lastProperties = m_properties;
			(*m_updatesThisFrame)[m_properties.ID] = &m_properties;
		}*/
	}


private:
    std::map<IDTYPE, EntityProperties*>* m_updatesThisFrame;
    btTransform m_xformCPU;
	b3Transform m_xformGPU;
    EntityProperties m_properties;
    EntityProperties m_lastProperties;
};



#endif //BULLET_SIM_H