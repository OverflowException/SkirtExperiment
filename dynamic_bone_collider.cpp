#include "dynamic_bone_collider.h"

ARDynamicBoneCollider::ARDynamicBoneCollider(const Configs& configs) {
    _ori = configs.ori;
    _radius = configs.radius;
    _height = configs.height;
    _anchor = configs.anchor;
    _center = configs.center;
    _handlers = configs.handlers;
    
    if (!_handlers.empty()) {
        assert((_height == 0.0f && _handlers.size() == 1) ||
               (_height != 0.0f && _handlers.size() == 2));
    }
}


bool ARDynamicBoneCollider::collide(glm::vec3& c, float r) {
    if (!_anchor) {
        return false;
    }
    
    if (_height == 0.0f) {
        glm::vec3 cc = *_anchor * glm::vec4(_center, 1.0f);
        
        // debug handler
        if (!_handlers.empty()) {
            *_handlers[0] = cc;
        }
        
        return sphere_check_and_push_out(c, r, cc, _radius);
    } else {
        float h = _height * 0.5 - _radius;
        glm::vec3 cc0(_center);
        glm::vec3 cc1(_center);
        switch (_ori) {
            case Orientation::X:
                cc0.x -= h;
                cc1.x += h;
                break;
        
            case Orientation::Y:
                cc0.y -= h;
                cc1.y += h;
                break;
        
            case Orientation::Z:
                cc0.z -= h;
                cc1.z += h;
                break;
        
            default:
                assert(false);
                break;
        }
        
        cc0 = *_anchor * glm::vec4(cc0, 1.0f);
        cc1 = *_anchor * glm::vec4(cc1, 1.0f);
        
        // debug handler
        if (!_handlers.empty()) {
            *_handlers[0] = cc0;
            *_handlers[1] = cc1;
        }
        
        return capsule_check_and_push_out(c, r, cc0, cc1, _radius);
    }
    
}

bool ARDynamicBoneCollider::sphere_check_and_push_out(glm::vec3& pc, float pr,
                                                      const glm::vec3& cc, float cr) {
    glm::vec3 d = pc - cc;
    float d_len2 = glm::dot(d, d);
    float r_sum = pr + cr;
    float r_sum2 = r_sum * r_sum;
    
    if (r_sum2 < d_len2) {
        // No contact
        return false;
    }
    
    // Contact. Push out
    pc = cc + d * (r_sum / glm::sqrt(d_len2));
    return true;
}

bool ARDynamicBoneCollider::capsule_check_and_push_out(glm::vec3& pc, float pr,
                                                       const glm::vec3& cc0, const glm::vec3& cc1,
                                                       float cr) {
    // Baricentric coordinates. proj = u * cc0 + v * cc1, u + v = 1
    glm::vec3 s = cc1 - cc0;
    glm::vec3 d0 = pc - cc0;
    float s2 = glm::dot(s, s);
    float v = glm::dot(d0, s) / s2;
    float u = 1 - v;
    
    if (u <= 0) {
        // try collide c1
        return sphere_check_and_push_out(pc, pr, cc1, cr);
    } else if (v <= 0) {
        // try collide c0
        return sphere_check_and_push_out(pc, pr, cc0, cr);
    } else {
        // try collide shaft
        glm::vec3 proj = cc0 * u + cc1 * v;
        glm::vec3 d = pc - proj;
        float d_len2 = glm::dot(d, d);
        float r_sum = pr + cr;
        float r_sum2 = r_sum * r_sum;
        
        if (r_sum2 < d_len2) {
            // No contact
            return false;
        }
        
        // Contact. Push out
        pc = proj + d * (r_sum / glm::sqrt(d_len2));
        return true;
    }
}
