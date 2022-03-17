#include <vector>
#include "glm/glm.hpp"

struct Joint {
    glm::mat4                   local_transform = glm::mat4(1.0);
    std::shared_ptr<glm::mat4>  world_transform = nullptr;
    size_t                      parent          = -1;
    std::vector<size_t>         children;
};

typedef std::vector<Joint> Skeleton;