#ifndef RAY_RESULT_H
#define RAY_RESULT_H

#include "ArchStuff.h"

struct RayResult
{
    bool HasHit;
    IDTYPE HitObjectID;
    float HitFraction;
    // GPU-compatible vector types would be used here
    // btVector3 is retained for GPU compatibility
};

struct SweepResult
{
    bool HasHit;
    IDTYPE HitObjectID;
    float HitFraction;
    // GPU-compatible vector types would be used here
    // btVector3 is retained for GPU compatibility
};

struct ContactTestResult
{
    bool HasContacts;
    int ContactCount;
    // Add more fields as needed for contact information
};

#endif // RAY_RESULT_H