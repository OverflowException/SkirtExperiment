#include "glm/gtc/matrix_transform.hpp"

#define GLM_ENABLE_EXPERIMENTAL
#include "glm/gtx/quaternion.hpp"

#include <iostream>
#include "dynamic_bone.h"
#include "glm2bt.hpp"

#include "print_helper.h"

void DynamicBone::init(const Configs& configs, const Skeleton& skel) {
    append_particle(skel, configs.root_id, -1, configs);
    
    assert(_particles.size() >= 1);
    _object_transform = skel[configs.object_id].world_transform;
    _object_prev_pos = (*_object_transform)[3];
    _update_rate = configs.update_rate;
    _gravity = configs.gravity;
    _local_gravity = glm::inverse(*_particles[0].transform) * glm::vec4(_gravity, 0.0f);
    _force = configs.force;
}

void DynamicBone::append_particle(const Skeleton& skel, 
                                  size_t skel_id, 
                                  size_t parent_particle_id,
                                  const Configs& configs) {
    _particles.emplace_back();
    Particle& p = _particles.back();
    const Joint& j = skel[skel_id];

    p.transform = j.world_transform;
    p.init_local_transform = j.local_transform;
    p.parent_id = parent_particle_id;
    p.child_count = j.children.size();
    p.damping = configs.damping;
    p.elasticity = configs.elasticity;
    p.stiffness = configs.stiffness;
    p.inertia = configs.inertia;
    p.cur_pos = p.prev_pos = (*p.transform)[3];
    
//    // init rotation
//    glm::vec3 t;
//    glm::vec3 s;
//    decompose_matrix(j.local_transform, t, p.local_rot, s);
//
//    glm::quat parent_rot(1.0f, 0.0f, 0.0f, 0.0f);
//    if (parent_particle_id != -1) {
//        parent_rot = _particles[parent_particle_id].world_rot;
//    }
//    p.world_rot = parent_rot * p.local_rot;

    for (size_t c : j.children) {
        append_particle(skel, c, _particles.size() - 1, configs);
    }
}

void DynamicBone::update(float dt) {
    _object_move = glm::vec3((*_object_transform)[3]) - _object_prev_pos;
    _object_prev_pos = glm::vec3((*_object_transform)[3]);

    float steps = dt * _update_rate;

    inertia_integration(steps);
    retain_geometry(steps);

    sync();
}

void DynamicBone::inertia_integration(float steps) {
    assert(_particles.size() > 0);
    std::shared_ptr<glm::mat4> root = _particles[0].transform;
    assert(root);

    glm::vec3 residual_gravity = glm::vec3(0.0f);
    if (_gravity != glm::vec3(0.0f)) {
        glm::vec3 cur_gravity = *root * glm::vec4(_local_gravity, 0.0);
        float proj_gravity = std::max(glm::dot(cur_gravity, _gravity), 0.0f) / glm::length(_gravity);
        residual_gravity = _gravity - proj_gravity;
    }
    glm::vec3 force = (residual_gravity + _force) * steps;

    // root particle's position gets updated directly, no integration
    _particles[0].prev_pos = _particles[0].cur_pos;
    _particles[0].cur_pos = (*_particles[0].transform)[3];

    // perform integration on child particles
    for (size_t i = 1; i < _particles.size(); ++i) {
        Particle& p = _particles[i];
        glm::vec3 v = p.cur_pos - p.prev_pos;
        glm::vec3 o_move = _object_move * p.inertia;
        p.prev_pos = p.cur_pos + o_move;
        // TODO: add collision friction to damping factor
        p.cur_pos += v * (1 - p.damping) + force + o_move;
    }
}   

void DynamicBone::retain_geometry(float steps) {
    for (size_t i = 1; i < _particles.size(); ++i) {
        Particle& p1 = _particles[i];
        Particle& p0 = _particles[p1.parent_id];
        glm::mat4 m1_local = glm::inverse(*p0.transform) * (*p1.transform);

        // Set m0 as p0's current transform matrix
        glm::mat4 m0 = *p0.transform;
        m0[3] = glm::vec4(p0.cur_pos, 1.0);

        // elasticity
        glm::vec3 rest_pos = (m0 * m1_local)[3];

        glm::vec3 bias = rest_pos - p1.cur_pos;
        // p1 trying to spring back into place under the influence of elasticity
        // potential overshoot
        p1.cur_pos += bias * (p1.elasticity * steps);

        // stiffness
        float rest_len = glm::length(glm::vec3((*p1.transform)[3] - (*p0.transform)[3]));
        bias = rest_pos - p1.cur_pos;
        float bias_len = glm::length(bias);
        // stiffness here to resolve potential overshoot introduced by elasticity
        float len_threshold = rest_len * (1 - p1.stiffness) * 2;
        if (bias_len > len_threshold) {
            p1.cur_pos += bias * ((bias_len - len_threshold) / bias_len);
        }

        // retain bone length
        glm::vec3 bone = p0.cur_pos - p1.cur_pos;
        float bone_len = glm::length(bone);
        p1.cur_pos += bone * ((bone_len - rest_len) / bone_len);
    }
}

// TODO: child_count > 1?
void DynamicBone::sync() {
    for (size_t i = 1; i < _particles.size(); ++i) {
        Particle& p1 = _particles[i];
        Particle& p0 = _particles[p1.parent_id];

        if (p0.child_count <= 1) {
            glm::vec3 prev_bone = (*p1.transform)[3] - (*p0.transform)[3];
            glm::vec3 cur_bone = p1.cur_pos - p0.cur_pos;
            glm::quat rot = glm::rotation(prev_bone, cur_bone);

            // apply rot
            *p0.transform = apply_rotation(*p0.transform, rot);
        }

        (*p1.transform)[3] = glm::vec4(p1.cur_pos, 1.0);
    }
}

glm::mat4 DynamicBone::apply_rotation(const glm::mat4& m, const glm::quat& q) {
    glm::vec3 t;
    glm::quat r;
    glm::vec3 s;
    decompose_matrix(m, t, r, s);
    
    r = q * r;
    return glm::translate(glm::mat4(1.0), t) *
           glm::mat4_cast(r) *
           glm::scale(glm::mat4(1.0), s);
}
