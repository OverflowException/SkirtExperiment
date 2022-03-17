#include <iostream>

#include "glm/gtc/matrix_transform.hpp"

#include "dynamic_bone.h"
#include "gui.h"
#include "timer.hpp"

std::vector<glm::mat4> local_transforms = {
    glm::translate(glm::mat4(1.0), glm::vec3(0.0f, 0.0f, 6.0f)),
    glm::translate(glm::mat4(1.0), glm::vec3(0.0f, -1.0f, 1.0f)),
    glm::translate(glm::mat4(1.0), glm::vec3(0.0f, -1.0f, -1.0f)),
    glm::translate(glm::mat4(1.0), glm::vec3(0.0f, -0.5f, -1.0f)),
    glm::translate(glm::mat4(1.0), glm::vec3(0.0f, 0.0f, -1.0f))
};

int main (int argc, char** argv) {
    // Construct joint hierarchy
    Skeleton skel(5);

    for (size_t i = 0; i < skel.size(); ++i) {
        Joint& j = skel[i];
        j.parent = i - 1;
        if (i != skel.size() - 1) {
            // Not the last
            j.children.push_back(i + 1);
        }
        j.local_transform = local_transforms[i];

        glm::mat4 parent_transfom = glm::mat4(1.0f);
        if (i != 0){
            // Not the first
            parent_transfom = *skel[j.parent].world_transform;
        }
        j.world_transform.reset(new glm::mat4(parent_transfom * local_transforms[i]));
    }

    std::shared_ptr<Gui> gui = std::make_shared<Gui>();

    // joints
    for (auto& j : skel) {
    float radius = 0.5f;
        gui->add_sphere(j.world_transform, radius);
    }

    // ground cube
    std::shared_ptr<glm::mat4> cube_pos = std::make_shared<glm::mat4>(1.0);
    *cube_pos = glm::translate(*cube_pos, glm::vec3(0.0f, 0.0f, -0.5f));
    gui->add_cube(cube_pos, glm::vec3(50.0f, 50.0f, 1.0f));

    // dynamic bones
    std::shared_ptr<DynamicBone> db = std::make_shared<DynamicBone>();
    DynamicBone::Configs db_cfg;
    db_cfg.object_id = 0;
    db_cfg.root_id = 0; // Try change this to 1. Should this be the same as object_id? 
    db_cfg.update_rate = 60.0f;
    db_cfg.inertia = 0.7f;
    db_cfg.elasticity = 0.7f;
    db->init(db_cfg, skel);

    Timer timer;
    float t_stop = 1.25f;
    auto callback = [&]() {
        float dt = timer.interval();
        if (timer.current() < t_stop) {
            float xy = glm::sin(2 * M_PI * timer.current());
            *skel[0].world_transform = glm::translate(*skel[0].world_transform, 
                                                      glm::vec3(xy, xy, 0.0f) * glm::vec3(0.3f));
        }
        db->update(dt);
    };
    gui->prerender_callback(callback);

    timer.start();
    gui->run();

    return 0;
}