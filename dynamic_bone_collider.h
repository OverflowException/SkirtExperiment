#include <memory>
#include <vector>

#include "glm/glm.hpp"

class ARDynamicBoneCollider {
public:
    
    enum Orientation {
        X,
        Y,
        Z
    };
    
    struct Configs {
        Orientation ori     = Orientation::Y;
        glm::vec3   center  = glm::vec3(0.0f);
        float       radius  = 0.1f;
        float       height  = 0.0f;
        
        std::shared_ptr<glm::mat4> anchor = nullptr;
        
        // for visual debug only
        std::vector<std::shared_ptr<glm::vec3>> handlers;
    };
    
    ARDynamicBoneCollider(const Configs& configs);
    
    bool collide(glm::vec3& c, float r);
    
private:
    
    bool sphere_check_and_push_out(glm::vec3& pc, float pr,
                                   const glm::vec3& cc, float cr);
    
    bool capsule_check_and_push_out(glm::vec3& pc, float pr,
                                    const glm::vec3& cc0, const glm::vec3& cc1,
                                    float cr);
    
    Orientation                 _ori;
    float                       _radius = 0.0f;
    float                       _height = 0.0f;
    std::shared_ptr<glm::mat4>  _anchor = nullptr;
    glm::vec3                   _center = glm::vec3(0.0f);
    
    // For visual debug only
    std::vector<std::shared_ptr<glm::vec3>> _handlers;
};
