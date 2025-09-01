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

#ifndef API_DATA_H
#define API_DATA_H
#include "Bullet3OpenCL/RigidBody/b3GpuRigidBodyPipeline.h"
#include "Bullet3Common/b3Vector3.h"
#include "Bullet3Common/b3Quaternion.h"
#include "Bullet3Common/b3Transform.h"
#include "Bullet3Common/b3Matrix3x3.h"

#include "LinearMath/btTransform.h"
#include "LinearMath/btVector3.h"
#include "LinearMath/btQuaternion.h"
#include "LinearMath/btMatrix3x3.h"

#include "ArchStuff.h"

// Fixed object ID codes used by OpenSimulator
#define ID_TERRAIN 0	// OpenSimulator identifies collisions with terrain by localID of zero
#define ID_GROUND_PLANE 1
#define ID_INVALID_HIT 0xFFFFFFFF

//hollow and cut primitives
#define HOLLOW_THRESHOLD 0.05f
#define CUT_THRESHOLD 0.05f
#define INNER_SHAPE_SCALE 0.9f

// API-exposed structure for a 3D vector
struct Vector3
{
	float X;
	float Y;
	float Z;

	Vector3()
	{
		X = 0.0;
		Y = 0.0;
		Z = 0.0;
	}

	Vector3(float x, float y, float z)
	{
		X = x;
		Y = y;
		Z = z;
	}

	Vector3(const b3Vector3& v)
	{
		X = v.getX();
		Y = v.getY();
		Z = v.getZ();
	}
	
	Vector3(const btVector3& v)
	{
		X = v.x();
		Y = v.y();
		Z = v.z();
	}


	bool AlmostEqual(const Vector3& v, const float nEpsilon)
	{
		return
			(((v.X - nEpsilon) < X) && (X < (v.X + nEpsilon))) &&
			(((v.Y - nEpsilon) < Y) && (Y < (v.Y + nEpsilon))) &&
			(((v.Z - nEpsilon) < Z) && (Z < (v.Z + nEpsilon)));
	}
	
	b3Vector3 GetB3Vector3()
	{
		b3Vector3 result;
		result.setValue(X, Y, Z);
		return result;
	}
	
	btVector3 GetBtVector3()
	{
		return btVector3(X, Y, Z);
	}

	void operator= (const b3Vector3& v)
	{
		X = v.getX();
		Y = v.getY();
		Z = v.getZ();
	}

	bool operator!= (const Vector3& o)
	{
		return (
			   X != o.X
			|| Y != o.Y
			|| Z != o.Z
		);
	}

	bool operator==(const Vector3& b)
	{
		return (X == b.X && Y == b.Y && Z == b.Z);
	}
	
	Vector3& operator=(const btVector3& v)
	{
		X = v.x();
		Y = v.y();
		Z = v.z();
		return *this;
	}

};

// API-exposed structure for a rotation
struct Quaternion
{
	float X;
	float Y;
	float Z;
	float W;

	Quaternion()
	{
		X = 0.0;
		Y = 0.0;
		Z = 0.0;
		W = 1.0;
	}

	Quaternion(float xx, float yy, float zz, float ww)
	{
		X = xx;
		Y = yy;
		Z = zz;
		W = ww;
	}

	Quaternion(const b3Quaternion& btq)
	{
		X = btq.getX();
		Y = btq.getY();
		Z = btq.getZ();
		W = btq.getW();
	}

	bool AlmostEqual(const Quaternion& q, float nEpsilon)
	{
		return
			(((q.X - nEpsilon) < X) && (X < (q.X + nEpsilon))) &&
			(((q.Y - nEpsilon) < Y) && (Y < (q.Y + nEpsilon))) &&
			(((q.Z - nEpsilon) < Z) && (Z < (q.Z + nEpsilon))) &&
			(((q.W - nEpsilon) < W) && (W < (q.W + nEpsilon)));
	}

	b3Quaternion GetB3Quaternion()
	{
		return b3Quaternion(X, Y, Z, W);
	}
	
	btQuaternion GetBtQuaternion()
	{
		return btQuaternion(X, Y, Z, W);
	}

	void operator= (const b3Quaternion& q)
	{
		X = q.getX();
		Y = q.getY();
		Z = q.getZ();
		W = q.getW();
	}
	
	Quaternion(const btQuaternion& btq)
	{
		X = btq.x();
		Y = btq.y();
		Z = btq.z();
		W = btq.w();
	}

	Quaternion& operator=(const btQuaternion& q)
	{
		X = q.x();
		Y = q.y();
		Z = q.z();
		W = q.w();
		return *this;
	}

};

struct Matrix3x3
{
	Vector3 m_el[3];
public:
	Matrix3x3()
	{
		m_el[0] = Vector3(1.0, 0.0, 0.0);
		m_el[1] = Vector3(0.0, 1.0, 0.0);
		m_el[2] = Vector3(0.0, 0.0, 1.0);
	}
	
	Matrix3x3(const Vector3& row0, const Vector3& row1, const Vector3& row2)
	{
		m_el[0] = row0;
		m_el[1] = row1;
		m_el[2] = row2;
	}

	
	void operator= (const b3Matrix3x3& o)
	{
		m_el[0] = o.getRow(0);
		m_el[1] = o.getRow(1);
		m_el[2] = o.getRow(2);
	}
	void operator=(const btMatrix3x3& o)
	{
		m_el[0] = Vector3(o[0].x(), o[0].y(), o[0].z());
		m_el[1] = Vector3(o[1].x(), o[1].y(), o[1].z());
		m_el[2] = Vector3(o[2].x(), o[2].y(), o[2].z());
	}

	b3Matrix3x3 GetB3Matrix3x3()
	{
		return b3Matrix3x3(
			m_el[0].X, m_el[0].Y, m_el[0].Z,
			m_el[1].X, m_el[1].Y, m_el[1].Z,
			m_el[2].X, m_el[2].Y, m_el[2].Z );
	}
	
	btMatrix3x3 GetBtMatrix3x3()
	{
		return btMatrix3x3(
			m_el[0].X, m_el[0].Y, m_el[0].Z,
			m_el[1].X, m_el[1].Y, m_el[1].Z,
			m_el[2].X, m_el[2].Y, m_el[2].Z );
	}
};

struct Transform
{
	// A b3Transform is made of a 3x3 array plus a b3Vector3 copy of the origin.
	// Note that a b3Vector3 is defined as 4 floats (probably for alignment)
	//    which is why the assignment is done explicitly to get type conversion.
	Matrix3x3 m_basis;
	Vector3 m_origin;
public:
	Transform() { 
		m_basis = Matrix3x3();
		m_origin = Vector3();
	}
	Transform(const b3Transform& t)
	{
		m_basis = t.getBasis();
		m_origin = t.getOrigin();
	}
	b3Transform GetB3Transform()
	{
		return b3Transform(m_basis.GetB3Matrix3x3(), m_origin.GetB3Vector3());
	}
	
	btTransform GetBtTransform()
	{
		return btTransform(m_basis.GetBtMatrix3x3(), m_origin.GetBtVector3());
	}
	
	Transform(const btTransform& t)
	{
		btMatrix3x3 btBasis = t.getBasis();
		m_basis = Matrix3x3(
			Vector3(btBasis[0].x(), btBasis[0].y(), btBasis[0].z()),
			Vector3(btBasis[1].x(), btBasis[1].y(), btBasis[1].z()),
			Vector3(btBasis[2].x(), btBasis[2].y(), btBasis[2].z())
		);
		btVector3 btOrigin = t.getOrigin();
		m_origin = Vector3(btOrigin.x(), btOrigin.y(), btOrigin.z());
	}

};

// API-exposed structure defining an object
struct ShapeData
{
	enum PhysicsShapeType
	{
		SHAPE_UNKNOWN	= 0,
		SHAPE_AVATAR	= 1,
		SHAPE_BOX		= 2,
		SHAPE_CONE		= 3,
		SHAPE_CYLINDER	= 4,
		SHAPE_SPHERE	= 5,
		SHAPE_MESH		= 6,
		SHAPE_HULL		= 7
	};

	// note that bool's are passed as floats's since bool size changes by language
	IDTYPE ID;
	PhysicsShapeType Type;
	Vector3 Position;
	Quaternion Rotation;
	Vector3 Velocity;
	Vector3 Scale;
	float Mass;
	float Buoyancy;		// gravity effect on the object
	MESHKEYTYPE HullKey;
	MESHKEYTYPE MeshKey;
	float Friction;
	float Restitution;
	float Collidable;	// things can collide with this object
	float Static;	// object is non-moving. Otherwise gravity, etc
	
	//fields for hollow and cut primitives
    float Hollow;
    float ProfileBegin;
    float ProfileEnd;
};

// API-exposed structure for reporting a collision
struct CollisionDesc
{
	IDTYPE aID;
	IDTYPE bID;
	Vector3 point;
	Vector3 normal;
	float penetration;
};

// BulletSim extends the definition of the collision flags
//   so we can control when collisions are desired.
#define BS_SUBSCRIBE_COLLISION_EVENTS    (0x0400)
#define BS_FLOATS_ON_WATER               (0x0800)
#define BS_VEHICLE_COLLISIONS            (0x1000)
#define BS_RETURN_ROOT_COMPOUND_SHAPE    (0x2000)

// Combination of above bits for all settings that want collisions reported
#define BS_WANTS_COLLISIONS              (0x1400)

// API-exposed structure to input a convex hull
struct ConvexHull
{
	Vector3 Offset;
	uint32_t VertexCount;
	Vector3* Vertices;
};

// API-exposed structured to return a raycast result
struct RaycastHit
{
	IDTYPE ID;
	float Fraction;
	Vector3 Normal;
	Vector3 Point;
};

// API-exposed structure to return a convex sweep result
struct SweepHit
{
	IDTYPE ID;
	float Fraction;
	Vector3 Normal;
	Vector3 Point;
};

// API-exposed structure to return physics updates from Bullet
struct EntityProperties
{
	IDTYPE ID;
	Vector3 Position;
	Quaternion Rotation;
	Vector3 Velocity;
	Vector3 Acceleration;
	Vector3 AngularVelocity;

	EntityProperties()
	{
		ID = 0;
		Position = b3Vector3();
		Rotation = b3Quaternion();
	}

	EntityProperties(IDTYPE id, const b3Transform& startTransform)
	{
		ID = id;
		Position = startTransform.getOrigin();
		Rotation = startTransform.getRotation();
	}

	void operator= (const EntityProperties& e)
	{
		ID = e.ID;
		Position = e.Position;
		Rotation = e.Rotation;
		Velocity = e.Velocity;
		Acceleration = e.Acceleration;
		AngularVelocity = e.AngularVelocity;
	}
};

// added values for collision CFM and ERP setting so we can set all axis at once
#define COLLISION_AXIS_LINEAR_ALL (20)
#define COLLISION_AXIS_ANGULAR_ALL (21)
#define COLLISION_AXIS_ALL (22)

// Block of parameters passed from the managed code.
// The memory layout MUST MATCH the layout in the managed code.
// Rely on the fact that 'float' is always 32 bits in both C# and C++
#define ParamTrue (1.0)
#define ParamFalse (0.0)
struct ParamBlock
{
    float defaultFriction;
    float defaultDensity;
	float defaultRestitution;
    float collisionMargin;
    float gravity;

	float maxPersistantManifoldPoolSize;
	float maxCollisionAlgorithmPoolSize;
	float shouldDisableContactPoolDynamicAllocation;
	float shouldForceUpdateAllAabbs;
	float shouldRandomizeSolverOrder;
	float shouldSplitSimulationIslands;
	float shouldEnableFrictionCaching;
	float numberOfSolverIterations;
    float useSingleSidedMeshes;
	float globalContactBreakingThreshold;

	float physicsLoggingFrames;
};

// Shape types for ShapeDesc
enum ShapeType
{
    BOX_SHAPE,
    SPHERE_SHAPE,
    CYLINDER_SHAPE,
    CAPSULE_SHAPE,
    CONE_SHAPE,
    MULTISPHERE_SHAPE,
    CONVEX_HULL_SHAPE,
    TRIANGLE_MESH_SHAPE,
    GIMPACT_SHAPE,
    COMPOUND_SHAPE,
    EMPTY_SHAPE,
    PLANE_SHAPE,
    HEIGHTFIELD_SHAPE,
    HACD_SHAPE,
    VHACD_SHAPE
};

// Block of parameters for HACD algorithm
struct HACDParams
{
	float maxVerticesPerHull;		// 100
	float minClusters;				// 2
	float compacityWeight;			// 0.1
	float volumeWeight;				// 0.0
	float concavity;				// 100
	float addExtraDistPoints;		// false
	float addNeighboursDistPoints;	// false
	float addFacesPoints;			// false
	float shouldAdjustCollisionMargin;	// false
	// VHACD
	float whichHACD;				// zero if Bullet HACD, non-zero says VHACD
	// http://kmamou.blogspot.ca/2014/12/v-hacd-20-parameters-description.html
	float vHACDresolution;			// max number of voxels generated during voxelization stage
	float vHACDdepth;				// max number of clipping stages
	float vHACDconcavity;			// maximum concavity
	float vHACDplaneDownsampling;	// granularity of search for best clipping plane
	float vHACDconvexHullDownsampling;	// precision of hull gen process
	float vHACDalpha;				// bias toward clipping along symmetry planes
	float vHACDbeta;				// bias toward clipping along revolution axis
	float vHACDdelta;				// bias toward clipping within local convex shape
	float vHACDgamma;				// max concavity when merging
	float vHACDpca;					// on/off normalizing mesh before decomp
	float vHACDmode;				// 0:voxel based, 1: tetrahedron based
	float vHACDmaxNumVerticesPerCH;	// max triangles per convex hull
	float vHACDminVolumePerCH;		// sampling of generated convex hulls
	float vHACDconvexHullApprox;	// approximate hulls to accelerate computation
	float vHACDoclAcceleration;		// use OpenCL
};

// --- Vector ---
inline btVector3 b3ToBtVector3(const b3Vector3& v) {
    return btVector3(v.x, v.y, v.z);
}

inline b3Vector3 btToB3Vector3(const btVector3& v) {
    b3Vector3 result;
    result.setValue(v.x(), v.y(), v.z());
    return result;
}

// --- Quaternion ---

inline btQuaternion b3ToBtQuaternion(const b3Quaternion& b3Quat) {
    return btQuaternion(b3Quat.x, b3Quat.y, b3Quat.z, b3Quat.w);
}

inline b3Quaternion btToB3Quaternion(const btQuaternion& btQuat) {
    b3Quaternion result;
    result.setValue(btQuat.x(), btQuat.y(), btQuat.z(), btQuat.w());
    return result;
}

// --- Transform ---

inline btTransform b3ToBtTransform(const b3Transform& t) {
    // btTransform expects (rotation quaternion, origin vector)
    return btTransform(
        b3ToBtQuaternion(t.getRotation()),
        b3ToBtVector3(t.getOrigin())
    );
}

inline b3Transform btToB3Transform(const btTransform& t) {
    // Use setters to avoid relying on constructor availability/signature differences
    b3Transform result;
    result.setRotation(btToB3Quaternion(t.getRotation()));
    result.setOrigin(btToB3Vector3(t.getOrigin()));
    return result;
}

// --- Matrix3x3 ---

inline btMatrix3x3 b3ToBtMatrix3x3(const b3Matrix3x3& m) {
    btMatrix3x3 result;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            result[i][j] = m[i][j];
    return result;
}

inline b3Matrix3x3 btToB3Matrix3x3(const btMatrix3x3& m) {
    b3Matrix3x3 result;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            result[i][j] = m[i][j];
    return result;
}

#define CONSTRAINT_NOT_SPECIFIED (-1)
#define CONSTRAINT_NOT_SPECIFIEDF (-1.0)


#endif // API_DATA_H