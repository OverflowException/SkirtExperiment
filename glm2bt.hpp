#pragma once

#include "LinearMath/btVector3.h"
#include "LinearMath/btTransform.h"

#define GLM_ENABLE_EXPERIMENTAL
#include "glm/gtx/quaternion.hpp"
#include "glm/gtx/matrix_decompose.hpp"

inline glm::vec3 bt2glm_vec3(const btVector3& src) {
    glm::vec3 result = glm::vec3(src.getX(), src.getY(), src.getZ());
    return result;
}

inline btVector3 glm2bt_vec3(const glm::vec3 src) {
    btVector3 result = btVector3(src.x, src.y, src.z);
    return result;
}

inline glm::vec4 bt2glm_vec4(const btVector4& src) {
    glm::vec4 result = glm::vec4(src.getX(), src.getY(), src.getZ(), src.getW());
    return result;
}

inline btVector4 glm2bt_vec4(const glm::vec4& src) {
    btVector4 result = btVector4(src.x, src.y, src.z, src.w);
    return result;
}

inline glm::quat bt2glm_quat(const btQuaternion& src) {
    glm::quat result = glm::quat(src.getW(), src.getX(), src.getY(), src.getZ());
    return result;
}

inline btQuaternion glm2bt_quat(const glm::quat& src) {
    btQuaternion result = btQuaternion(src.x, src.y, src.z, src.w);
    return result;
}

inline void decompose_matrix(const glm::mat4& m, glm::vec3& translation, glm::quat& rotation, glm::vec3& scale) {
    glm::vec3 skew;
    glm::vec4 pers;
    glm::decompose(m, scale, rotation, translation, skew, pers);
    rotation = glm::conjugate(rotation);
}

// btTransform does not support scaling and shearing
// So only translation and rotation are preserved
inline btTransform glm2bt_transform(const glm::mat4& mat) {
    glm::vec3 glm_t;
    glm::quat glm_r;
    glm::vec3 glm_s;
    decompose_matrix(mat, glm_t, glm_r, glm_s);
    
    btVector3 bt_t = glm2bt_vec3(glm_t);
    btQuaternion bt_q = glm2bt_quat(glm_r);
    
    return btTransform(bt_q, bt_t);
}

inline glm::mat4 bt2glm_transform(const btVector3& t, const btQuaternion& r) {
    glm::mat4 m = glm::mat4_cast(bt2glm_quat(r));
    m[3][0] = t[0];
    m[3][1] = t[1];
    m[3][2] = t[2];
    return m;
    // Warning: do not use glm::translate. Translation gets left multiplied by this method
    // return glm::translate(m, bt2glm_vec3(t));
}

inline btQuaternion bt_rot_between(const btVector3& orig, const btVector3& dest) {
    return glm2bt_quat(
                       glm::rotation(
                                     glm::normalize(bt2glm_vec3(orig)),
                                     glm::normalize(bt2glm_vec3(dest))));
}
