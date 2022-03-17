#pragma once

#include <memory>
#include <vector>

#include "glm/glm.hpp"
#include "skeleton.h"

class DynamicBone {
public:
    struct Particle {
        // World transform
        std::shared_ptr<glm::mat4>  transform   = nullptr;
        glm::mat4   init_local_transform        = glm::mat4(1.0);
        
        int         parent_id   = -1; // id of _particles
        size_t      child_count = 0;
        float       damping     = 0.0f;
        float       elasticity  = 0.0f;
        float       stiffness   = 0.0f;
        float       inertia     = 0.0f;

        // data for solver
        glm::vec3 cur_pos;
        glm::vec3 prev_pos;
    };

    struct Configs {
        size_t              object_id;
        size_t              root_id;
        float               update_rate;
        glm::vec3           gravity         = glm::vec3(0.0f);
        glm::vec3           force           = glm::vec3(0.0f);

        // TODO: add parameter distribution function
        // constant parameter at this point
        float               damping         = 0.1f;
        float               elasticity      = 0.1f;
        float               stiffness       = 0.1f;
        float               inertia         = 0.1f;
    };

    void init(const Configs& configs, const Skeleton& skel);

    // output. dynamic bone -> skeleton
    void sync();

    void update(float dt);

    // modified verlet integration
    void inertia_integration(float stps);

    void retain_geometry(float step);

private:
    void append_particle(const Skeleton& skel, 
                         size_t skel_id, 
                         size_t parent_particle_id,
                         const Configs& configs);
    
    glm::mat4 apply_rotation(const glm::mat4& m, const glm::quat& q);

    std::vector<Particle>   _particles;

    // Object movement related
    std::shared_ptr<glm::mat4>  _object_transform   = nullptr;
    glm::vec3   _object_prev_pos    = glm::vec3(0.0);
    glm::vec3   _object_move        = glm::vec3(0.0);

    // Global parameters
    float       _update_rate    = 60.0f;
    glm::vec3   _gravity        = glm::vec3(0.0f);
    glm::vec3   _local_gravity  = glm::vec3(0.0f);
    glm::vec3   _force          = glm::vec3(0.0f);
};
