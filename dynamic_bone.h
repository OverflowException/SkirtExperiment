#pragma once

#include <memory>
#include <vector>

#include "glm/glm.hpp"

class ARDynamicBone {
public:
    struct Joint {
        glm::mat4                   local_transform = glm::mat4(1.0);
        std::shared_ptr<glm::mat4>  world_transform = nullptr;
        size_t                      parent          = -1;
        std::vector<size_t>         children;
    };

    typedef std::vector<Joint> Skeleton;
    
    enum GravityType {
        NONE,
        PSEUDO,
        DISTRIBUTED,
        DROOPY
    };
    
    struct Configs {
        size_t              object_id;
        size_t              root_id;
        float               update_rate;
        GravityType         gravity_type    = GravityType::NONE;
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
    
    void pre_apply_gravity();
    
    // modified verlet integration
    void inertia_integration(float stps);

    void retain_geometry(float step);

private:
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
    
    void append_particle(const Skeleton& skel, 
                         size_t skel_id, 
                         size_t parent_particle_id,
                         const Configs& configs);
    
    void apply_rotation(glm::mat4& m, const glm::quat& q);
    
    void get_angle_and_axis(const glm::quat& r, float& a, glm::vec3& axis);

    std::vector<Particle>   _particles;

    // Object movement related
    std::shared_ptr<glm::mat4>  _object_transform   = nullptr;
    glm::vec3   _object_prev_pos    = glm::vec3(0.0);
    glm::vec3   _object_move        = glm::vec3(0.0);

    // Global parameters
    float       _update_rate    = 60.0f;
    GravityType _gravity_type   = GravityType::NONE;
    glm::vec3   _gravity        = glm::vec3(0.0f);
    glm::vec3   _gravity_datum  = glm::vec3(0.0f);
    glm::vec3   _local_gravity  = glm::vec3(0.0f);
    glm::vec3   _force          = glm::vec3(0.0f);
};
