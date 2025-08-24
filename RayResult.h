#ifndef RAY_RESULT_H
#define RAY_RESULT_H

#include "btBulletDynamicsCommon.h"
#include "ArchStuff.h"

struct RayResult
{
    bool HasHit;
    IDTYPE HitObjectID;
    btVector3 HitNormal;
    btVector3 HitPoint;
    float HitFraction;
};

struct SweepResult
{
    bool HasHit;
    IDTYPE HitObjectID;
    btVector3 HitNormal;
    btVector3 HitPoint;
    float HitFraction;
};

struct ContactTestResult
{
    bool HasContacts;
    int ContactCount;
    // Add more fields as needed for contact information
};

#endif // RAY_RESULT_H