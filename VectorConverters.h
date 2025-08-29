// VectorConverters.h
#pragma once

#include "Bullet3Common/b3Vector3.h"
#include "Bullet3Common/b3Quaternion.h"
#include "LinearMath/btVector3.h"
#include "LinearMath/btQuaternion.h"

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