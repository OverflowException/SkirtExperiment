#pragma once

#include <iostream>

#include "LinearMath/btTransform.h"
#include "glm/gtc/matrix_transform.hpp"

#define GLM_ENABLE_EXPERIMENTAL
#include "glm/gtx/quaternion.hpp"

template<typename T>
std::ostream& operator<<(std::ostream& os, const std::vector<T>& vec) {
    os << "[";
    for (const T& ele : vec) {
        os << ele << ", ";
    }
    os << "]";
    return os;
}

std::ostream& operator<<(std::ostream& os, const btVector3& vec) {
    os << "[" << vec.x() << ", " << vec.y() << ", " << vec.z() << "]";
    return os;
}

std::ostream& operator<<(std::ostream& os, const btQuaternion& quat) {
    os << "[axis = " << quat.getAxis() << ", angle = " << quat.getAngle()
       << ", degree = " << quat.getAngle()  / 180 * SIMD_PI << "]";
    return os;
}

std::ostream& operator<<(std::ostream& os, const btTransform& trans) {
    os << "[translation = " << trans.getOrigin()
       << ", rotation = " << trans.getRotation() << "]";
    return os;
}

std::ostream& operator<<(std::ostream& os, const glm::mat4& m) {
    os << m[0][0] << " " << m[1][0] << " " << m[2][0] << " " << m[3][0] << "\n";
    os << m[0][1] << " " << m[1][1] << " " << m[2][1] << " " << m[3][1] << "\n";
    os << m[0][2] << " " << m[1][2] << " " << m[2][2] << " " << m[3][2] << "\n";
    os << m[0][3] << " " << m[1][3] << " " << m[2][3] << " " << m[3][3];
    return os;
}

std::ostream& operator<<(std::ostream& os, const glm::vec4& v) {
    os << v[0] << " " << v[1] << " " << v[2] << " " << v[3] << std::endl;
    return os;
}

std::ostream& operator<<(std::ostream& os, const glm::quat& q) {
    float angle = 2 * acos(q.w);
    float sin_half_squared = 1 - q.w * q.w;
    
    float x, y, z;
    // angle = 0 or 2PI
    if (sin_half_squared <= 0.0f) {
        x = 1.0f;
        y = 0.0f;
        z = 0.0f;
    } else {
        float sin_half = sqrt(sin_half_squared);
        x = q.x / sin_half;
        y = q.y / sin_half;
        z = q.z / sin_half;
    }
    os << "angle = " << angle << ", axis = [" << x << ", " << y << ", " << z << "]";
    return os;
}
