#pragma once

#include "LinearMath/btQuaternion.h"
#include "LinearMath/btVector3.h"

class MathUtil {
public:
    inline static btScalar deg2rad(btScalar deg) { return deg * SIMD_PI / 180.0; }
    
    inline static btVector3 scale_up(btScalar ratio, const btVector3& base, const btVector3& src) {
        return ratio * (src - base) + base;
    }
    
    static void rot_towards(const btVector3& v0, btVector3& v1, btScalar angle);
    
    static btVector3 rand_perpendicular(const btVector3& src);

    static btQuaternion rot_between(const btVector3& src, const btVector3& dest);
    
    static btVector3 find_head_axis(const btVector3& face, const btVector3& up,  const btScalar& tilt);
};
