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
// Include Bullet2 headers first
#include <btBulletCollisionCommon.h>
#include <btBulletDynamicsCommon.h>
#define CL_TARGET_OPENCL_VERSION 120
// Undef conflicting macros
#ifdef MAX_NUM_PARTS_IN_BITS
#undef MAX_NUM_PARTS_IN_BITS
#endif

// Then include Bullet3 headers
#include "Bullet3OpenCL/Initialize/b3OpenCLUtils.h"
#include "Bullet3OpenCL/BroadphaseCollision/b3GpuBroadphaseInterface.h"
#include "Bullet3OpenCL/RigidBody/b3GpuNarrowPhase.h"
#include "Bullet3OpenCL/RigidBody/b3GpuRigidBodyPipeline.h"

// Then include your headers
#include "WorldData.h"
#include "BulletSim.h"
#include "Util.h"

#include "Bullet3Common/b3Vector3.h"
#include "Bullet3Collision/NarrowPhaseCollision/b3RaycastInfo.h"

// Linkages to debugging dump routines
extern "C" void DumpPhysicsStatistics2(BulletSim* sim);
extern "C" void DumpActivationInfo2(BulletSim* sim);

BulletSim::BulletSim(float maxX, float maxY, float maxZ)
{
    // Initialize all members manually
    // Initialize the GPU engine
    m_gpuEngine = new GpuPhysicsEngine();
    m_gpuEngine->gpuPipeline = nullptr;
    m_gpuEngine->gpuBroadphase = nullptr;
    m_gpuEngine->gpuNarrowphase = nullptr;
    m_gpuEngine->openclContext = nullptr;
    m_gpuEngine->openclQueue = nullptr;
    m_gpuEngine->openclDevice = nullptr;
	
    m_dumpStatsCount = 0;
    m_maxUpdatesPerFrame = 0;
    m_updatesThisFrameArray = nullptr;
    m_collidersThisFrameArray = nullptr;
    maxCollisionsPerFrame = 0;
    collisionsThisFrame = 0;
    m_maxSubSteps = DEFAULT_MAX_SUBSTEPS;
    m_contactImpulseThreshold = DEFAULT_CONTACT_IMPULSE_THRESHOLD;
    m_gpuAvailable = false;

    bsDebug_Initialize();

    // GPU detection - simplified implementation
    cl_int ciErrNum = CL_SUCCESS;
    cl_uint numPlatforms = 0;
    ciErrNum = clGetPlatformIDs(0, NULL, &numPlatforms);
    if (ciErrNum == CL_SUCCESS && numPlatforms > 0) {
        cl_platform_id platform = NULL;
        ciErrNum = clGetPlatformIDs(1, &platform, NULL);
        
        cl_uint numDevices = 0;
        ciErrNum = clGetDeviceIDs(platform, CL_DEVICE_TYPE_GPU, 0, NULL, &numDevices);
        if (ciErrNum == CL_SUCCESS && numDevices > 0) {
            m_gpuAvailable = true;
            
            // Get device ID
            cl_device_id device;
            ciErrNum = clGetDeviceIDs(platform, CL_DEVICE_TYPE_GPU, 1, &device, NULL);
            m_gpuEngine->openclDevice = device;
            
            if (ciErrNum == CL_SUCCESS) {
                // Create context and command queue
                cl_context context = clCreateContext(NULL, 1, &device, NULL, NULL, &ciErrNum);
                m_gpuEngine->openclContext = context;
                
                cl_command_queue queue = clCreateCommandQueue(context, device, 0, &ciErrNum);
                m_gpuEngine->openclQueue = queue;
            }
        }
    }
}


// Step the simulation forward by one full step and potentially some number of substeps
// In BulletSim.cpp
int BulletSim::PhysicsStep2(btScalar timeStep, int maxSubSteps, btScalar fixedTimeStep, 
                           int* updatedEntityCount, int* collidersCount)
{
    int numSimSteps = 0;

    if (m_worldData.dynamicsWorld)
    {
        // Clear collision data
        m_collidersThisFrame.clear();
        collisionsThisFrame = 0;

        int actualMaxSubSteps = (maxSubSteps > 0) ? maxSubSteps : m_maxSubSteps;
        
        if (m_gpuAvailable && m_gpuEngine->gpuPipeline) {
            // Call stepSimulation on the GPU pipeline
            m_gpuEngine->gpuPipeline->stepSimulation(timeStep);
            
            // Handle substeps
            btScalar remainingTime = timeStep;
            int stepsTaken = 0;
            
            while (remainingTime > 0.0f && stepsTaken < actualMaxSubSteps) {
                btScalar dt = (remainingTime > fixedTimeStep) ? fixedTimeStep : remainingTime;
                m_gpuEngine->gpuPipeline->integrate(dt);
                remainingTime -= dt;
                stepsTaken++;
            }
            
            // Process CPU bodies
            numSimSteps = m_worldData.dynamicsWorld->stepSimulation(timeStep, actualMaxSubSteps, fixedTimeStep);
            
            // Synchronize data
            synchronizeGpuCpuData();
        } else {
            // CPU-only fallback
            numSimSteps = m_worldData.dynamicsWorld->stepSimulation(timeStep, actualMaxSubSteps, fixedTimeStep);
        }
        
        // Count rigid bodies for updatedEntityCount
        if (updatedEntityCount)
        {
            int rigidBodyCount = 0;
            for (int i = m_worldData.dynamicsWorld->getNumCollisionObjects() - 1; i >= 0; i--)
            {
                btCollisionObject* obj = m_worldData.dynamicsWorld->getCollisionObjectArray()[i];
                if (obj->getInternalType() & btCollisionObject::CO_RIGID_BODY)
                    rigidBodyCount++;
            }
            *updatedEntityCount = rigidBodyCount;
        }
        
        if (collidersCount)
            *collidersCount = collisionsThisFrame;
    }
    
    return numSimSteps;
}

void BulletSim::synchronizeGpuCpuData()
{
    if (!m_gpuAvailable || !m_gpuEngine->gpuPipeline) return;
    
    // Copy transforms from GPU to CPU
    for (uint32_t bodyId : m_gpuBodyIds) {
        if (btCollisionObject* obj = findBodyById(bodyId)) {
            btTransform trans;
            obj->setWorldTransform(trans);
        }
    }
    
    // Copy transforms from CPU to GPU
    for (uint32_t bodyId : m_cpuBodyIds) {
        if (btCollisionObject* obj = findBodyById(bodyId)) {
            if (obj->isActive()) {
                btTransform trans = obj->getWorldTransform();
            }
        }
    }
}

// In BulletSim.cpp, replace the gravity setting code:
void BulletSim::initPhysics2(ParamBlock* parms, 
                            int maxCollisions, CollisionDesc* collisionArray, 
                            int maxUpdates, EntityProperties* updateArray)
{
    // Remember the pointers to pinned memory for returning collisions and property updates
    maxCollisionsPerFrame = maxCollisions;
    m_collidersThisFrameArray = collisionArray;
    m_maxUpdatesPerFrame = maxUpdates;
    m_updatesThisFrameArray = updateArray;
    m_worldData.params = parms;

    // Initialize the dynamics world even if GPU is not available
    m_worldData.collisionConfiguration = new btDefaultCollisionConfiguration();
    m_worldData.dispatcher = new btCollisionDispatcher(m_worldData.collisionConfiguration);
    m_worldData.broadphase = new btDbvtBroadphase();
    m_worldData.solver = new btSequentialImpulseConstraintSolver();
    m_worldData.dynamicsWorld = new btDiscreteDynamicsWorld(
        m_worldData.dispatcher, 
        m_worldData.broadphase, 
        m_worldData.solver, 
        m_worldData.collisionConfiguration
    );

    // FIX: Use the correct gravity field from ParamBlock
    // The ParamBlock has a single 'gravity' field, not separate X/Y/Z components
    if (parms) {
        // Use the gravity value from parameters (assuming it's the Y component magnitude)
        // Default gravity is typically -9.8 m/s² on Y axis
        m_worldData.dynamicsWorld->setGravity(btVector3(0, -parms->gravity, 0));
    } else {
        // Default gravity if no parameters provided
        m_worldData.dynamicsWorld->setGravity(btVector3(0, -9.8f, 0));
    }

    if (m_gpuAvailable) {
        // GPU initialization code would go here
        // m_worldData.BSLog("GPU acceleration enabled with OpenCL");
    } else {
        // m_worldData.BSLog("GPU acceleration not available");
    }
}

void BulletSim::exitPhysics2()
{
    // Clean up GPU resources
    if (m_gpuEngine->gpuPipeline) {
        delete m_gpuEngine->gpuPipeline;
        m_gpuEngine->gpuPipeline = nullptr;
    }
    
    // Clean up CPU resources
    if (m_worldData.dynamicsWorld) {
        delete m_worldData.dynamicsWorld;
        m_worldData.dynamicsWorld = nullptr;
    }
    
    if (m_worldData.solver) {
        delete m_worldData.solver;
        m_worldData.solver = nullptr;
    }
    
    if (m_worldData.broadphase) {
        delete m_worldData.broadphase;
        m_worldData.broadphase = nullptr;
    }
    
    if (m_worldData.dispatcher) {
        delete m_worldData.dispatcher;
        m_worldData.dispatcher = nullptr;
    }
    
    if (m_worldData.collisionConfiguration) {
        delete m_worldData.collisionConfiguration;
        m_worldData.collisionConfiguration = nullptr;
    }
    
    // Clean up OpenCL resources
    if (m_gpuEngine->openclQueue) {
        clReleaseCommandQueue(m_gpuEngine->openclQueue);
        m_gpuEngine->openclQueue = nullptr;
    }
    
    if (m_gpuEngine->openclContext) {
        clReleaseContext(m_gpuEngine->openclContext);
        m_gpuEngine->openclContext = nullptr;
    }
}

// Placeholder implementations for GPU methods
void b3GpuRigidBodyPipeline::stepSimulation(float timeStep)
{
    // Placeholder - implement actual GPU simulation step
}

void b3GpuRigidBodyPipeline::integrate(float timeStep)
{
    // Placeholder - implement actual GPU integration
}

btCollisionObject* BulletSim::findBodyById(uint32_t id)
{
    for (int i = 0; i < m_worldData.dynamicsWorld->getNumCollisionObjects(); i++) {
        btCollisionObject* obj = m_worldData.dynamicsWorld->getCollisionObjectArray()[i];
        if (CONVLOCALID(obj->getUserPointer()) == id) {
            return obj;
        }
    }
    return nullptr;
}

void BulletSim::registerGpuBody(uint32_t id, uint32_t gpuId) {
    m_gpuBodyIds.push_back(id);
}

void BulletSim::registerCpuBody(uint32_t id) {
    m_cpuBodyIds.push_back(id);
}

// GPU shape creation implementations - all commented out due to dependency issues
int BulletSim::registerGpuBoxShape(const b3Vector3& halfExtents) {
    if (!m_gpuAvailable || !m_gpuEngine->gpuPipeline) return -1;
    return -1;
}

int BulletSim::registerGpuSphereShape(float radius) {
    if (!m_gpuAvailable || !m_gpuEngine->gpuPipeline) return -1;
    return -1;
}

int BulletSim::registerGpuCapsuleShape(float radius, float height) {
    if (!m_gpuAvailable || !m_gpuEngine->gpuPipeline) return -1;
    return -1;
}

int BulletSim::registerGpuCylinderShape(const b3Vector3& halfExtents) {
    if (!m_gpuAvailable || !m_gpuEngine->gpuPipeline) return -1;
    return -1;
}

int BulletSim::registerGpuConvexHullShape(int numPoints, float* points) {
    if (!m_gpuAvailable || !m_gpuEngine->gpuPipeline) return -1;
    return -1;
}

int BulletSim::registerGpuCompoundShape() {
    if (!m_gpuAvailable || !m_gpuEngine->gpuPipeline) return -1;
    return -1;
}

void BulletSim::addChildShapeToGpuCompound(int compoundShapeId, int childShapeId, const b3Transform& transform) {
    if (!m_gpuAvailable || !m_gpuEngine->gpuPipeline) return;
}

int BulletSim::registerGpuMeshShape(int indicesCount, int* indices, int verticesCount, float* vertices) {
    if (!m_gpuAvailable || !m_gpuEngine->gpuPipeline) return -1;
    return -1;
}

int BulletSim::registerGpuTerrainShape(int width, int length, float* heightData, 
                                     float minHeight, float maxHeight, float scale) {
    if (!m_gpuAvailable || !m_gpuEngine->gpuPipeline) return -1;
    return -1;
}

// GPU rigid body management - all commented out due to dependency issues
int BulletSim::registerGpuRigidBody(int shapeId, float mass, const b3Vector3& position, 
                                   const b3Quaternion& rotation, const b3Vector3& linearVelocity, 
                                   const b3Vector3& angularVelocity) {
    if (!m_gpuAvailable || !m_gpuEngine->gpuPipeline) return -1;
    return -1;
}

void BulletSim::setGpuBodyPosition(int bodyId, const b3Vector3& position) {
    if (!m_gpuAvailable || !m_gpuEngine->gpuPipeline) return;
}

void BulletSim::setGpuBodyRotation(int bodyId, const b3Quaternion& rotation) {
    if (!m_gpuAvailable || !m_gpuEngine->gpuPipeline) return;
}

void BulletSim::setGpuBodyLinearVelocity(int bodyId, const b3Vector3& velocity) {
    if (!m_gpuAvailable || !m_gpuEngine->gpuPipeline) return;
}

void BulletSim::setGpuBodyAngularVelocity(int bodyId, const b3Vector3& velocity) {
    if (!m_gpuAvailable || !m_gpuEngine->gpuPipeline) return;
}

void BulletSim::applyGpuBodyCentralForce(int bodyId, const b3Vector3& force) {
    if (!m_gpuAvailable || !m_gpuEngine->gpuPipeline) return;
}

void BulletSim::applyGpuBodyCentralImpulse(int bodyId, const b3Vector3& impulse) {
    if (!m_gpuAvailable || !m_gpuEngine->gpuPipeline) return;
}

void BulletSim::applyGpuBodyTorque(int bodyId, const b3Vector3& torque) {
    if (!m_gpuAvailable || !m_gpuEngine->gpuPipeline) return;
}

// GPU queries
bool BulletSim::gpuRayTest(const b3Vector3& from, const b3Vector3& to, RayResult* result) {
    if (!m_gpuAvailable || !m_gpuEngine->gpuPipeline) return false;
    return false;
}

bool BulletSim::gpuConvexSweepTest(int shapeId, const b3Vector3& fromPos, const b3Quaternion& fromRot,
                                 const b3Vector3& toPos, const b3Quaternion& toRot, SweepResult* result) {
    if (!m_gpuAvailable || !m_gpuEngine->gpuPipeline) return false;
    return false;
}

int BulletSim::gpuGetContactPoints(int bodyIdA, int bodyIdB, ContactPoint* contacts, int maxContacts) {
    if (!m_gpuAvailable || !m_gpuEngine->gpuPipeline) return 0;
    return 0;
}

bool BulletSim::gpuGetAabb(int bodyId, b3Vector3& aabbMin, b3Vector3& aabbMax) {
    if (!m_gpuAvailable || !m_gpuEngine->gpuPipeline) return false;
    return false;
}

// GPU info and debug
int BulletSim::gpuGetNumRigidBodies() {
    if (!m_gpuAvailable || !m_gpuEngine->gpuPipeline) return 0;
    return 0;
}

int BulletSim::gpuGetNumCollisionObjects() {
    if (!m_gpuAvailable || !m_gpuEngine->gpuPipeline) return 0;
    return 0;
}

void BulletSim::gpuDumpWorldState() {
    if (!m_gpuAvailable || !m_gpuEngine->gpuPipeline) return;
}

// GPU constraints
int BulletSim::createGpuPoint2PointConstraint(int bodyIdA, int bodyIdB, 
                                            const b3Vector3& pivotInA, const b3Vector3& pivotInB) {
    if (!m_gpuAvailable || !m_gpuEngine->gpuPipeline) return -1;
    return -1;
}

void BulletSim::removeGpuConstraint(int constraintId) {
    if (!m_gpuAvailable || !m_gpuEngine->gpuPipeline) return;
}

bool BulletSim::UpdateParameter2(IDTYPE localID, const char* parm, float val) {
    if (strcmp(parm, "gravity") == 0) {
        if (m_gpuAvailable && m_gpuEngine->gpuPipeline) {
            // b3Vector3 gravity(0, -val, 0);  // Commented out due to constructor issues
            // m_gpuEngine->gpuPipeline->setGravity(gravity);
            return true;
        }
    } else if (strcmp(parm, "max_substeps") == 0) {
        m_maxSubSteps = (int)val;
        return true;
    } else if (strcmp(parm, "contact_impulse_threshold") == 0) {
        m_contactImpulseThreshold = val;
        return true;
    }
    return false;
}

void BulletSim::DumpPhysicsStats() {
    // GPU-specific physics stats dumping
}

// Other necessary implementations
void BulletSim::RecordCollision(int objA, int objB, 
                               const b3Vector3& contact, const b3Vector3& norm, float penetration) {
    // GPU collision recording implementation
}

// WorldData logging functions - commented out due to initialization issues
/*
void WorldData::BSLog(const char* msg, ...) {
    va_list args;
    va_start(args, msg);
    BSLog2(msg, args);
    va_end(args);
}

void WorldData::BSLog2(const char* msg, va_list argp) {
    char buffer[4096];
    vsnprintf(buffer, sizeof(buffer), msg, argp);

    printf("%s\n", buffer);
    fflush(stdout);

    if (debugLogCallback != nullptr) {
        debugLogCallback(buffer);
    }
}
*/

RaycastHit BulletSim::RayTest(btVector3& fromBT, btVector3& toBT, short filterGroup, short filterMask)
{
    RaycastHit hit{};
    hit.Fraction = 1.0f;

    if (m_gpuAvailable && m_gpuEngine->gpuPipeline)
    {
        // Convert bt → b3 for GPU
        b3Vector3 from = btToB3Vector3(fromBT);
        b3Vector3 to   = btToB3Vector3(toBT);
        
        // Use GPU ray test
        RayResult result;
        if (gpuRayTest(from, to, &result))
        {
            hit.ID       = result.collisionObjectId;
            hit.Fraction = result.hitFraction;
            hit.Point    = result.hitPointWorld;   // b3Vector3 → Vector3 (ok)
            hit.Normal   = result.hitNormalWorld;  // b3Vector3 → Vector3 (ok)
        }
    }
    else
    {
        // CPU ray test (Bullet2)
        btCollisionWorld::ClosestRayResultCallback rayCallback(fromBT, toBT);
        rayCallback.m_collisionFilterGroup = filterGroup;
        rayCallback.m_collisionFilterMask  = filterMask;

        m_worldData.dynamicsWorld->rayTest(fromBT, toBT, rayCallback);

        if (rayCallback.hasHit())
        {
            hit.ID       = CONVLOCALID(rayCallback.m_collisionObject->getUserPointer());
            hit.Fraction = rayCallback.m_closestHitFraction;
            hit.Point    = btToB3Vector3(rayCallback.m_hitPointWorld); // converted
            hit.Normal   = btToB3Vector3(rayCallback.m_hitNormalWorld); // converted
        }
    }

    return hit;
}