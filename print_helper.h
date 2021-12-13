#pragma once

#include <iostream>

#include "LinearMath/btTransform.h"

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

