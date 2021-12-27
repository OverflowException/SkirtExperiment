#include <iostream>
#include "math_util.h"

btVector3 MathUtil::rand_perpendicular(const btVector3& src) {
    btVector3 x_axis = btVector3(1.0, 0.0, 0.0);
    btVector3 y_axis = btVector3(0.0, 1.0, 0.0);
    btVector3 z_axis = btVector3(0.0, 0.0, 1.0);
    
    btVector3 perp = src.cross(x_axis);
    if (perp.length2() != btScalar(0.0)) {
        return perp;
    }
    
    perp = src.cross(y_axis);
    if (perp.length2() != btScalar(0.0)) {
        return perp;
    }
    
    perp = src.cross(z_axis);
    if (perp.length2() != btScalar(0.0)) {
        return perp;
    }
    
    assert(false);
    return btVector3();
}

void MathUtil::rot_towards(const btVector3& v0, btVector3& v1, btScalar angle) {
    btQuaternion full_rot = rot_between(v1, v0);
    btVector3 axis = full_rot.getAxis();

    v1 = v1.rotate(axis, angle);
}

btQuaternion MathUtil::rot_between(const btVector3& src, const btVector3& dest) {
    assert(src.length2() != btScalar(0.0));
    assert(dest.length2() != btScalar(0.0));
    
    btScalar angle;
    btVector3 axis = src.cross(dest);
    if (axis.length2() == 0) {
        // parallel
        btScalar proj = src.dot(dest);
        assert(proj != btScalar(0.0));
        
        axis = rand_perpendicular(src);
        if (proj < btScalar(0.0)) {
            // opposite direction
            angle = SIMD_PI;
        } else {
            angle = 0.0;
        }
    } else {
        // non-parallel
        angle = src.angle(dest);
    }

    return btQuaternion(axis, angle);
}

btVector3 MathUtil::find_head_axis(const btVector3& face, const btVector3& up,  const btScalar& tilt) {
    if (face == up) {
        return btVector3(1.0, 0.0, 0.0);
    }
    
    btScalar factor = face.dot(up) / face.dot(face);
    btVector3 no_tilt_axis =  (up - factor * face).normalize();
    
    // add tilt
    btVector3 tilt_axis =  no_tilt_axis.rotate(face, tilt);
    
    // point up
    return tilt_axis.dot(up) > 0 ? tilt_axis : -tilt_axis;
}
