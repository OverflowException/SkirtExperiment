#include <iostream>
#include <map>

#include "skirt_experiment.h"
#include "data.h"

#include "btBulletDynamicsCommon.h"
#include "LinearMath/btVector3.h"
#include "LinearMath/btAlignedObjectArray.h"

#include "../CommonInterfaces/CommonRigidBodyBase.h"

#include "glm/gtc/matrix_transform.hpp"

#include "timer.hpp"
#include "glm2bt.hpp"
#include "dynamic_bone.h"

const std::string base_filename = "./data/hip";
const float       base_radius = 0.1;


// for visual debug only
std::vector<std::shared_ptr<glm::vec3>> empty_handlers;

const std::vector<ARDynamicBoneCollider::Configs> collider_cfgs = {
    {
        .ori = ARDynamicBoneCollider::Orientation::X,
        .center = glm::vec3(-1.9, -0.5, 0.9),
        .radius = 0.6f,
        .height = 1.0f
    },
    {
        .ori = ARDynamicBoneCollider::Orientation::X,
        .center = glm::vec3(-1.9, -0.5, -0.9),
        .radius = 0.6f,
        .height = 1.0f
    },
    {
        .ori = ARDynamicBoneCollider::Orientation::Y,
        .center = glm::vec3(-1.5, -0.5, 0.0),
        .radius = 0.4f,
        .height = 1.7f
    },
    {
        .ori = ARDynamicBoneCollider::Orientation::Y,
        .center = glm::vec3(-1.5, -0.8, 0.0),
        .radius = 0.75f,
        .height = 0.0f
    }
};

const  int                     strand_count = 6;
const std::string              strand_prefix = "./data/skirt";
const std::vector<std::string> strand_suffixes = {"1", "2", "3", "4", "5", "6"};
const float                    bone_radius = 0.07;
const float                    joint_radius = 0.1;
const float                    bone_length_ratio = 0.8;

const btVector3                x_handle_pos = btVector3(-1.0, 1.0, -2.5);
const btVector3                y_handle_pos = btVector3(0.0, 1.0, -2.5);
const btVector3                z_handle_pos = btVector3(1.0, 1.0, -2.5);

const btVector3                yaw_handle_pos = btVector3(-2.0, 0.5, 2.0);
const btVector3                pitch_handle_pos = btVector3(-2.0, 0.5, -1.5);
const btVector3                roll_handle_pos = btVector3(-3.0, 0.5, 0);

const std::vector<glm::mat4> local_transforms = {
    glm::translate(glm::mat4(1.0), glm::vec3(1.0f, 1.0f, 0.0f)),
    glm::translate(glm::mat4(1.0), glm::vec3(-1.0f, 1.0f, 0.0f)),
    glm::translate(glm::mat4(1.0), glm::vec3(-0.5f, -0.5f, 0.0f)),
    glm::translate(glm::mat4(1.0), glm::vec3(-0.5f, -1.0f, 0.0f)),
    glm::translate(glm::mat4(1.0), glm::vec3(0.0f, -1.0f, 0.0f))
};

const int root_id = 1;


struct SkirtExperiment : public CommonRigidBodyBase
{
    // All rigid body pointers in this class will get its ownership transfered to
    // btDiscreteDynamicsWold soon after construnction.
    // Object life cycle lies with btDiscreteDynamicsWold.
    // See CommonRigidBodyBase::exitPhysics()
    btRigidBody*  m_base;
    btTransform   m_base_trans;
    btRigidBody*  m_x_handle;
    btRigidBody*  m_y_handle;
    btRigidBody*  m_z_handle;
    
    btRigidBody*  m_yaw_handle;
    btRigidBody*  m_pitch_handle;
    btRigidBody*  m_roll_handle;
    
    std::vector<btRigidBody*>  m_colliders;
    
    ARDynamicBone::Skeleton skel;
    std::map<size_t, btRigidBody*> j2rb_map;
    std::map<std::pair<btRigidBody*, btRigidBody*>, btRigidBody*> s2c_map;
    
    std::vector<std::vector<std::shared_ptr<glm::vec3>>> c_handlers;
    
    Timer timer;
    ARDynamicBone db;
    
	SkirtExperiment(struct GUIHelperInterface* helper) : CommonRigidBodyBase(helper) {}
    
	virtual ~SkirtExperiment() {}
	virtual void initPhysics();
	virtual void renderScene();
	void resetCamera() {
		float dist = 4;
		float pitch = -35;
		float yaw = 52;
		float targetPos[3] = {0, 0, 0};
		m_guiHelper->resetCamera(dist, yaw, pitch, targetPos[0], targetPos[1], targetPos[2]);
	}

    // creation functions & add functions here
    void create_dynamic_bones();
    
    void create_position_handle();
    
    void create_rotation_handle();
    
    void create_cylinder_between_spheres();
    
    void update_cylinder_between_spheres();
    
    void update_colliders();
    
    btPoint2PointConstraint* create_p2p_constraint(btRigidBody& rb_a,
                                                   btRigidBody& rb_b,
                                                   btVector3& pivot);
    
    btTypedConstraint* create_line_fix_constraint(btRigidBody& body, const btVector3& normal);
    
    btTypedConstraint* create_plane_fix_constraint(btRigidBody& body, const btVector3& normal);
    
    btTypedConstraint* create_spin_only_constraint(btRigidBody& body, const btVector3& axis);
};

// pre-tick callback for kinematic bodies
void kinematicPreTickCallback(btDynamicsWorld* world, btScalar delta_time) {
    // base body manipulation
    SkirtExperiment* exp = reinterpret_cast<SkirtExperiment*>(world->getWorldUserInfo());

    btVector3 x = exp->m_x_handle->getWorldTransform().getOrigin() - x_handle_pos;
    btVector3 y = exp->m_y_handle->getWorldTransform().getOrigin() - y_handle_pos;
    btVector3 z = exp->m_z_handle->getWorldTransform().getOrigin() - z_handle_pos;
    
    btQuaternion yaw = exp->m_yaw_handle->getWorldTransform().getRotation();
    btQuaternion pitch = exp->m_pitch_handle->getWorldTransform().getRotation();
    btQuaternion roll = exp->m_roll_handle->getWorldTransform().getRotation();
    
    exp->m_base->getWorldTransform().setRotation(yaw * pitch * roll);
    exp->m_base->getWorldTransform().getOrigin() = exp->m_base_trans.getOrigin() + x + y + z;
    
    // sync manual control to object
    *exp->skel[0].world_transform = bt2glm_transform(exp->m_base->getWorldTransform().getOrigin(),
                                                     exp->m_base->getWorldTransform().getRotation());
    
    // simulate animation update
    for (size_t i = 1; i < exp->skel.size(); ++i) {
        ARDynamicBone::Joint& j = exp->skel[i];
        
        glm::mat4 parent_transfom = *exp->skel[j.parent].world_transform;
        *j.world_transform = glm::mat4(parent_transfom * j.local_transform);
    }
    
    // dynamic bone step
    float dt = exp->timer.interval();
    exp->db.update(dt);

    //sync back to bt rigid body
    for (size_t i = root_id; i < exp->skel.size(); ++i) {
        btTransform bt_trans = glm2bt_transform(*exp->skel[i].world_transform);
        exp->j2rb_map[i]->setWorldTransform(bt_trans);
    }
    
    // sync bone cylinder
    exp->update_cylinder_between_spheres();
    
    // sync collider
    exp->update_colliders();
    
    // sync to skeleton
////    btTransform trans;
////    btVector3 linear_vel(0, 0, 0);
////    btVector3 ang_vel(0, 3, 0);
////    btTransformUtil::integrateTransform(head->getWorldTransform(), linear_vel, ang_vel, delta_time, trans);
////    head->getMotionState()->setWorldTransform(trans);
}


void SkirtExperiment::create_cylinder_between_spheres() {
    for (size_t i = 1; i < skel.size(); ++i) {
        size_t j = skel[i].parent;
        glm::vec3 child_world_pos = (*skel[i].world_transform)[3];
        glm::vec3 parent_world_pos = (*skel[j].world_transform)[3];
        float half_length = glm::distance(child_world_pos, parent_world_pos) * 0.5;
        
        btCylinderShapeX* cylinder_shape = new btCylinderShapeX(btVector3(half_length * bone_length_ratio, bone_radius, bone_radius));
        m_collisionShapes.push_back(cylinder_shape);
        btRigidBody* bone = createRigidBody(0.0f, btTransform(), cylinder_shape);
        
        s2c_map[std::make_pair(j2rb_map[j], j2rb_map[i])] = bone;
    }
}

void SkirtExperiment::update_cylinder_between_spheres() {
    for (auto& ele : s2c_map) {
        btRigidBody* s0 = ele.first.first;  // parent joint
        btRigidBody* s1 = ele.first.second; // child joint
        btRigidBody* b = ele.second;        // bone

        btVector3 bone_vec = s1->getWorldTransform().getOrigin() - s0->getWorldTransform().getOrigin();
        btQuaternion bone_rot = bt_rot_between(btVector3(1.0, 0.0, 0.0), bone_vec);
        btVector3 bone_pos = (s0->getWorldTransform().getOrigin() + s1->getWorldTransform().getOrigin()) * 0.5;
        
        b->setWorldTransform(btTransform(bone_rot, bone_pos));
    }
}

void SkirtExperiment::update_colliders() {
    for (int i = 0; i < m_colliders.size(); ++i) {
        auto& h = c_handlers[i];
        auto c = m_colliders[i];
        if (h.size() == 1) {
            // Collider is a sphere
            c->getWorldTransform().setOrigin(glm2bt_vec3(*h[0]));
        } else if (h.size() == 2) {
            // Collider is a capsule
            glm::vec3 c0 = *h[0];
            glm::vec3 c1 = *h[1];
            glm::vec3 m = (c0 + c1) * 0.5f;
            
            btQuaternion cap_rot = bt_rot_between(btVector3(1.0f, 0.0f, 0.0f), glm2bt_vec3(c1 - c0));
            // m_collider->getWorldTransform().setOrigin(glm2bt_vec3(m));
            // m_collider->getWorldTransform().setBasis(btMatrix3x3(cap_rot));
            c->setWorldTransform(btTransform(cap_rot, glm2bt_vec3(m)));
        } else {
            assert(false);
        }
    }
}

void SkirtExperiment::create_rotation_handle() {
    btCylinderShapeX* cylinder_shape = new btCylinderShapeX(btVector3(0.4, 0.1, 0.1));
    // btSphereShape* cylinder_shape = new btSphereShape(0.2);
    m_collisionShapes.push_back(cylinder_shape);
    
    btCompoundShape* comp_shape = new btCompoundShape();
    m_collisionShapes.push_back(comp_shape);
    
    btVector3 x_axis(1.0, 0.0, 0.0);
    btTransform local_trans(bt_rot_between(x_axis, btVector3(1.0, 0.0, 0.0)));
    comp_shape->addChildShape(local_trans, cylinder_shape);
    
    local_trans = btTransform(bt_rot_between(x_axis, btVector3(0.0, 1.0, 0.0)));
    comp_shape->addChildShape(local_trans, cylinder_shape);

    local_trans = btTransform(bt_rot_between(x_axis, btVector3(0.0, 0.0, 1.0)));
    comp_shape->addChildShape(local_trans, cylinder_shape);
    
    m_yaw_handle = createRigidBody(1.0, btTransform(btQuaternion(0.0, 0.0, 0.0, 1.0), yaw_handle_pos), comp_shape);
    m_dynamicsWorld->addConstraint(create_spin_only_constraint(*m_yaw_handle, btVector3(0.0, 1.0, 0.0)));
    
    m_pitch_handle = createRigidBody(1.0, btTransform(btQuaternion(0.0, 0.0, 0.0, 1.0), pitch_handle_pos), comp_shape);
    m_dynamicsWorld->addConstraint(create_spin_only_constraint(*m_pitch_handle, btVector3(0.0, 0.0, 1.0)));
    
    m_roll_handle = createRigidBody(1.0, btTransform(btQuaternion(0.0, 0.0, 0.0, 1.0), roll_handle_pos), comp_shape);
    m_dynamicsWorld->addConstraint(create_spin_only_constraint(*m_roll_handle, btVector3(1.0, 0.0, 0.0)));
}

void SkirtExperiment::create_position_handle() {
    btBoxShape* box_shape = new btBoxShape(btVector3(0.1, 0.1, 0.1));
    m_collisionShapes.push_back(box_shape);
    
    m_x_handle = createRigidBody(1.0, btTransform(btQuaternion(0.0, 0.0, 0.0, 1.0), x_handle_pos), box_shape);
    m_dynamicsWorld->addConstraint(create_line_fix_constraint(*m_x_handle, btVector3(1.0, 0.0, 0.0)));
    
    m_y_handle = createRigidBody(1.0, btTransform(btQuaternion(0.0, 0.0, 0.0, 1.0), y_handle_pos), box_shape);
    m_dynamicsWorld->addConstraint(create_line_fix_constraint(*m_y_handle, btVector3(0.0, 1.0, 0.0)));
    
    m_z_handle = createRigidBody(1.0, btTransform(btQuaternion(0.0, 0.0, 0.0, 1.0), z_handle_pos), box_shape);
    m_dynamicsWorld->addConstraint(create_line_fix_constraint(*m_z_handle, btVector3(0.0, 0.0, 1.0)));
}

void SkirtExperiment::create_dynamic_bones() {
    std::vector<btTransform> base_trans;
    if (!get_transforms_from_file(base_filename, base_trans)) {
        std::cout << "cannot parse base file" << std::endl;
        return;
    }
    
    // Create skeleton hierarchy
    skel.resize(local_transforms.size());
    for (size_t i = 0; i < skel.size(); ++i) {
        ARDynamicBone::Joint& j = skel[i];
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
        
        // create bt body
        float radius = i == 0 ? base_radius : joint_radius;
        btSphereShape* sphere_shape = new btSphereShape(radius);
        m_collisionShapes.push_back(sphere_shape);
        btTransform j_trans = glm2bt_transform(*j.world_transform);
        btRigidBody* rb = createRigidBody(0.0, j_trans, sphere_shape);
        j2rb_map[i] = rb;
    }
    
    create_cylinder_between_spheres();
    update_cylinder_between_spheres();
    
    m_base = j2rb_map[0];
    m_base_trans = m_base->getWorldTransform();
    
//    // Create collision box
//    btCapsuleShapeZ* cap_shape = new btCapsuleShapeZ(capsule_radius, capsule_height);
//    m_collisionShapes.push_back(cap_shape);
//    btRigidBody*
    
    // init dynamic bones
    ARDynamicBone::Configs db_cfg;
    db_cfg.object_id = 0;
    db_cfg.root_id = root_id;
    db_cfg.update_rate = 60.0f;
    db_cfg.inertia = 0.1f;
    db_cfg.elasticity = 0.03f;
    db_cfg.gravity_type = ARDynamicBone::GravityType::DISTRIBUTED;
    db_cfg.gravity = glm::vec3(0.0, -1.0, 0.0);
    db.init(db_cfg, skel);
    
    // init colliders's debug handlers
    c_handlers.resize(collider_cfgs.size());
    for (int i = 0; i < collider_cfgs.size(); ++i) {
        auto c_cfg = collider_cfgs[i];
        if (c_cfg.height == 0.0f) {
            c_handlers[i].resize(1);
            c_handlers[i][0].reset(new glm::vec3(0.0f));
            
            btSphereShape* c_sphere = new btSphereShape(c_cfg.radius);
            m_collisionShapes.push_back(c_sphere);
        } else {
            c_handlers[i].resize(2);
            c_handlers[i][0].reset(new glm::vec3(0.0f));
            c_handlers[i][1].reset(new glm::vec3(0.0f));
            
            btCapsuleShape* c_capsule = new btCapsuleShapeX(c_cfg.radius, c_cfg.height);
            m_collisionShapes.push_back(c_capsule);
            // Will cause a visible 'jump' of the initial position of collider
            m_colliders.push_back(createRigidBody(0.0f, btTransform(btQuaternion(0.0, 0.0, 0.0, 1.0)), c_capsule));
        }
    }
    
    // init colliders
    std::vector<std::shared_ptr<ARDynamicBoneCollider>> colliders(collider_cfgs.size());
    for (int i = 0; i < collider_cfgs.size(); ++i) {
        auto c_cfg = collider_cfgs[i];
        c_cfg.anchor = skel[0].world_transform;
        c_cfg.handlers = c_handlers[i];
        
        colliders[i].reset(new ARDynamicBoneCollider(c_cfg));
    }
    
    // add to DynamicBone
    for (int i = 0; i < collider_cfgs.size(); ++i) {
        db.add_collider(colliders[i]);
    }
    
    timer.start();
    
    create_position_handle();
    create_rotation_handle();
}

btPoint2PointConstraint* SkirtExperiment::create_p2p_constraint(btRigidBody& rb_a,
                                                                btRigidBody& rb_b,
                                                                btVector3& pivot) {
    btVector3 pivot_a = rb_a.getCenterOfMassTransform().inverse() * pivot;
    btVector3 pivot_b = rb_b.getCenterOfMassTransform().inverse() * pivot;

    return new btPoint2PointConstraint(rb_a, rb_b, pivot_a, pivot_b);
}

btTypedConstraint* SkirtExperiment::create_line_fix_constraint(btRigidBody& body, const btVector3& dir) {
    // rotation between x-axis and plane normal
    btVector3 x_axis = btVector3(1.0, 0.0, 0.0);
    btVector3 x2n_axis = x_axis.cross(dir);
    btScalar x2n_angle = x_axis.angle(dir);
    
    if (x2n_angle == btScalar(0.0) || x2n_axis.length() == btScalar(0.0)) {
        x2n_axis = x_axis;
        x2n_angle = btScalar(0.0);
    }
    
    btQuaternion x2n(x2n_axis, x2n_angle);
    
    // world transform of pivot
    btTransform pivot_trans_world = btTransform(x2n, body.getWorldTransform().getOrigin());
    // b's local transform of pivot
    btTransform pivot_trans_b = body.getCenterOfMassTransform().inverse() * pivot_trans_world;
    btGeneric6DofSpring2Constraint* constraint = new btGeneric6DofSpring2Constraint(body, pivot_trans_b);
    
    // lock all directions other than dir, which is x in joint coordinate
    constraint->setLinearLowerLimit(btVector3(1.0, 0.0, 0.0));
    constraint->setLinearUpperLimit(btVector3(-1.0, 0.0, 0.0));
    constraint->enableSpring(0, true);
    constraint->setStiffness(0, 10);
    constraint->setDamping(0, 5);
    
    constraint->setAngularLowerLimit(btVector3(0.0, 0.0, 0.0));
    constraint->setAngularUpperLimit(btVector3(0.0, 0.0, 0.0));
    
    return constraint;
}

btTypedConstraint* SkirtExperiment::create_plane_fix_constraint(btRigidBody& body, const btVector3& normal) {
    // rotation between x-axis and plane normal
    btVector3 x_axis = btVector3(1.0, 0.0, 0.0);
    btVector3 x2n_axis = x_axis.cross(normal);
    btScalar x2n_angle = x_axis.angle(normal);
    
    if (x2n_angle == btScalar(0.0) || x2n_axis.length() == btScalar(0.0)) {
        x2n_axis = x_axis;
        x2n_angle = btScalar(0.0);
    }
    
    btQuaternion x2n(x2n_axis, x2n_angle);
    
    // world transform of pivot
    btTransform pivot_trans_world = btTransform(x2n, body.getWorldTransform().getOrigin());
    // b's local transform of pivot
    btTransform pivot_trans_b = body.getCenterOfMassTransform().inverse() * pivot_trans_world;
    btGeneric6DofSpring2Constraint* constraint = new btGeneric6DofSpring2Constraint(body, pivot_trans_b);
    
    // lock normal direction, which is x in joint coordinate
    constraint->setLinearLowerLimit(btVector3(0.0, 1.0, 1.0));
    constraint->setLinearUpperLimit(btVector3(0.0, -1.0, -1.0));
    constraint->enableSpring(1, true);
    constraint->setStiffness(1, 10);
    constraint->setDamping(1, 5);
    constraint->enableSpring(2, true);
    constraint->setStiffness(2, 10);
    constraint->setDamping(2, 5);
    
    constraint->setAngularLowerLimit(btVector3(0.0, 0.0, 0.0));
    constraint->setAngularUpperLimit(btVector3(0.0, 0.0, 0.0));
    
    return constraint;
}

btTypedConstraint* SkirtExperiment::create_spin_only_constraint(btRigidBody& body, const btVector3& normal) {
    // rotation between x-axis and plane normal
    btVector3 x_axis = btVector3(1.0, 0.0, 0.0);
    btVector3 x2n_axis = x_axis.cross(normal);
    btScalar x2n_angle = x_axis.angle(normal);
    
    if (x2n_angle == btScalar(0.0) || x2n_axis.length() == btScalar(0.0)) {
        x2n_axis = x_axis;
        x2n_angle = btScalar(0.0);
    }
    
    btQuaternion x2n(x2n_axis, x2n_angle);
    
    // world transform of pivot
    btTransform pivot_trans_world = btTransform(x2n, body.getWorldTransform().getOrigin());
    // b's local transform of pivot
    btTransform pivot_trans_b = body.getCenterOfMassTransform().inverse() * pivot_trans_world;
    btGeneric6DofSpring2Constraint* constraint = new btGeneric6DofSpring2Constraint(body, pivot_trans_b);
    
    // lock normal direction, which is x in joint coordinate
    constraint->setLinearLowerLimit(btVector3(0.0, 0.0, 0.0));
    constraint->setLinearUpperLimit(btVector3(0.0, 0.0, 0.0));
    
    constraint->setAngularLowerLimit(btVector3(1.0, 0.0, 0.0));
    constraint->setAngularUpperLimit(btVector3(-1.0, 0.0, 0.0));
    constraint->enableSpring(3, true);
    constraint->setStiffness(3, 10);
    constraint->setDamping(3, 5);
    
    return constraint;
}

void SkirtExperiment::initPhysics()
{
	m_guiHelper->setUpAxis(1);

	createEmptyDynamicsWorld();
    
	m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);

	if (m_dynamicsWorld->getDebugDrawer()) {
		m_dynamicsWorld->getDebugDrawer()->setDebugMode(btIDebugDraw::DBG_DrawWireframe + btIDebugDraw::DBG_DrawContactPoints);
    }
    
    // creations here
    // create_ground();
    
//    for (int i = 1; i <= 6; ++i) {
//        create_skirt_strand(i, hip->getWorldTransform());
//    }
    
    // create_test();
    create_dynamic_bones();

    m_dynamicsWorld->setInternalTickCallback(kinematicPreTickCallback, this, true);
    m_dynamicsWorld->setGravity(btVector3(0.0, 0.0, 0.0));
	m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
}

void SkirtExperiment::renderScene()
{
	CommonRigidBodyBase::renderScene();
}

CommonExampleInterface* SkirtExperimentCreateFunc(CommonExampleOptions& options)
{
	return new SkirtExperiment(options.m_guiHelper);
}

B3_STANDALONE_EXAMPLE(SkirtExperimentCreateFunc)
