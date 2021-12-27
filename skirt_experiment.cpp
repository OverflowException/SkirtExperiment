#include <iostream>
#include <queue>

#include "skirt_experiment.h"
#include "data.h"

#include "btBulletDynamicsCommon.h"
#include "LinearMath/btVector3.h"
#include "LinearMath/btAlignedObjectArray.h"

#include "../CommonInterfaces/CommonRigidBodyBase.h"

#include "math_util.h"
#include "print_helper.h"

const std::string base_filename = "./data/hip";
const float       base_radius = 0.5;

const  int                     strand_count = 6;
const std::string              strand_prefix = "./data/skirt";
const std::vector<std::string> strand_suffixes = {"1", "2", "3", "4", "5", "6"};
const float                    bone_mass = 1;
const float                    bone_radius = 0.1;
const float                    bone_length_ratio = 0.8;

const btVector3                p_handle_pos = btVector3(-1.0, 1.0, -2.5);
const btVector3                r_handle_pos = btVector3(-2.0, 1.0, 2.0);

const float                    scaling = 10;

struct SkirtExperiment : public CommonRigidBodyBase
{
    // All rigid body pointers in this class will get its ownership transfered to
    // btDiscreteDynamicsWold soon after construnction.
    // Object life cycle lies with btDiscreteDynamicsWold.
    // See CommonRigidBodyBase::exitPhysics()
    btRigidBody*  m_base;
    btTransform   m_base_trans;
    btRigidBody*  m_p_handle;
    btRigidBody*  m_r_handle;
    
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
    void create_ground();
    
    void create_test();
    
    void create_dynamic_bones();
    
    bool create_strand(int i, btRigidBody* base);
    
    void create_position_handle();
    
    void create_rotation_handle();
    
    btPoint2PointConstraint* create_p2p_constraint(btRigidBody& rb_a,
                                                   btRigidBody& rb_b,
                                                   btVector3& pivot);
    
    btTypedConstraint* create_plane_fix_constraint(btRigidBody& body, const btVector3& normal);
    
    btTypedConstraint* create_spin_only_constraint(btRigidBody& body, const btVector3& axis);
};

// pre-tick callback for kinematic bodies
void kinematicPreTickCallback(btDynamicsWorld* world, btScalar delta_time) {
    SkirtExperiment* exp = reinterpret_cast<SkirtExperiment*>(world->getWorldUserInfo());

    btVector3 translation = exp->m_p_handle->getWorldTransform().getOrigin() - p_handle_pos;
    btQuaternion rotation = exp->m_r_handle->getWorldTransform().getRotation();
    
    exp->m_base->getWorldTransform().setRotation(rotation);
    exp->m_base->getWorldTransform().getOrigin() = exp->m_base_trans.getOrigin() + translation;
    
////    btTransform trans;
////    btVector3 linear_vel(0, 0, 0);
////    btVector3 ang_vel(0, 3, 0);
////    btTransformUtil::integrateTransform(head->getWorldTransform(), linear_vel, ang_vel, delta_time, trans);
////    head->getMotionState()->setWorldTransform(trans);
}

void SkirtExperiment::create_rotation_handle() {
    btCylinderShapeX* cylinder_shape = new btCylinderShapeX(btVector3(0.4, 0.1, 0.0));
    // btSphereShape* cylinder_shape = new btSphereShape(0.2);
    m_collisionShapes.push_back(cylinder_shape);
    
    btCompoundShape* comp_shape = new btCompoundShape();
    m_collisionShapes.push_back(comp_shape);
    
    btVector3 x_axis(1.0, 0.0, 0.0);
    btTransform local_trans(MathUtil::rot_between(x_axis, btVector3(1.0, 0.0, 0.0)));
    comp_shape->addChildShape(local_trans, cylinder_shape);
    
    local_trans = btTransform(MathUtil::rot_between(x_axis, btVector3(0.0, 1.0, 0.0)));
    comp_shape->addChildShape(local_trans, cylinder_shape);

    local_trans = btTransform(MathUtil::rot_between(x_axis, btVector3(0.0, 0.0, 1.0)));
    comp_shape->addChildShape(local_trans, cylinder_shape);
    
    m_r_handle = createRigidBody(1.0, btTransform(btQuaternion(0.0, 0.0, 0.0, 1.0), r_handle_pos), comp_shape);
    m_dynamicsWorld->addConstraint(create_spin_only_constraint(*m_r_handle, btVector3(0.0, 1.0, 0.0)));
}

void SkirtExperiment::create_position_handle() {
    btBoxShape* box_shape = new btBoxShape(btVector3(0.1, 0.1, 0.1));
    m_collisionShapes.push_back(box_shape);
    m_p_handle = createRigidBody(1.0, btTransform(btQuaternion(0.0, 0.0, 0.0, 1.0), p_handle_pos), box_shape);
    m_dynamicsWorld->addConstraint(create_plane_fix_constraint(*m_p_handle, btVector3(0.0, 1.0, 0.0)));
}

void SkirtExperiment::create_ground() {
    btBoxShape* ground_shape = createBoxShape(btVector3(50., 50., 50.));
    m_collisionShapes.push_back(ground_shape);
    
    btTransform ground_transform;
    ground_transform.setIdentity();
    ground_transform.setOrigin(btVector3(0, -50, 0));
    
    btScalar ground_mass(0.);
    createRigidBody(ground_mass, ground_transform, ground_shape, btVector4(0, 0, 1, 1));
}

bool SkirtExperiment::create_strand(int i, btRigidBody* base) {
    std::vector<btTransform> joint_trans;
    if (!get_transforms_from_file(strand_prefix + strand_suffixes[i], joint_trans)) {
        return false;
    }
    assert(!joint_trans.empty());
    
    // scale up joints
    for (auto& trans : joint_trans) {
        btVector3 base_translate = base->getWorldTransform().getOrigin();
        btVector3 joint_translate = trans.getOrigin();
        trans.setOrigin(MathUtil::scale_up(scaling, base_translate, joint_translate));
    }
    
    joint_trans.insert(joint_trans.begin(), base->getWorldTransform());
    
    // curl this strand up a bit
    for (int i = 1; i < joint_trans.size() - 1; ++i) {
        btVector3 bone_vec0 = joint_trans[i - 1].getOrigin() - joint_trans[i].getOrigin();
        btVector3 bone_vec1 = joint_trans[i + 1].getOrigin() - joint_trans[i].getOrigin();
        std::cout << bone_vec1 << std::endl;
        MathUtil::rot_towards(bone_vec0, bone_vec1, SIMD_PI * (1.0 / 12.0));
        std::cout << bone_vec1 << std::endl;

        joint_trans[i + 1].getOrigin() = joint_trans[i].getOrigin() + bone_vec1;
    }
    
    btVector3 z_axis = btVector3(0.0, 0.0, 1.0);
    std::vector<btRigidBody*> strand;
    
    // child bones
    for (int i = 1; i < joint_trans.size(); ++i) {
        btVector3 bone_vec = joint_trans[i].getOrigin() - joint_trans[i - 1].getOrigin();
        btVector3 bone_mid = (joint_trans[i].getOrigin() + joint_trans[i - 1].getOrigin()) * 0.5;
        btTransform bone_trans(MathUtil::rot_between(z_axis, bone_vec), bone_mid);
        btCylinderShapeZ* bone_shape = new btCylinderShapeZ(btVector3(bone_radius, 0.0, bone_vec.length() * bone_length_ratio / 2));
        m_collisionShapes.push_back(bone_shape);
        
        btRigidBody* bone = createRigidBody(bone_mass, bone_trans, bone_shape);
        bone->setSleepingThresholds(0.1, 0.1);
        bone->setActivationState(DISABLE_DEACTIVATION);
        // bone->setDamping(1.0 / i, 0.0);
        strand.push_back(bone);
    }
    assert(!strand.empty());
    strand.insert(strand.begin(), base);
    
    // create constraints
    for (int i = 1; i < strand.size(); ++i) {
        btRigidBody* bone0 = strand[i - 1];
        btRigidBody* bone1 = strand[i];
        btVector3 joint0 = joint_trans[i - 1].getOrigin();
        btVector3 joint1 = joint_trans[i].getOrigin();

        btTransform pivot_frame;
        pivot_frame.setIdentity();
        pivot_frame.setRotation(MathUtil::rot_between(z_axis, joint1 - joint0));
        pivot_frame.setOrigin(joint0);

        btTransform frame0 = bone0->getWorldTransform().inverse() * pivot_frame;
        btTransform frame1 = bone1->getWorldTransform().inverse() * pivot_frame;

        auto constraint = new btGeneric6DofSpring2Constraint(*bone0, *bone1, frame0, frame1);
        constraint->setLinearLowerLimit(btVector3(0.0, 0.0, 0.0));
        constraint->setLinearUpperLimit(btVector3(0.0, 0.0, 0.0));

        constraint->setAngularLowerLimit(btVector3(-1.0, -1.0, 0.0));
        constraint->setAngularUpperLimit(btVector3(1.0, 1.0, 0.0));
        constraint->enableSpring(3, true);
        constraint->setStiffness(3, 100);
        constraint->setDamping(3, 10);
        constraint->enableSpring(4, true);
        constraint->setStiffness(4, 100);
        constraint->setDamping(4, 10);

        m_dynamicsWorld->addConstraint(constraint, true);
    }
    
    return true;
}

void SkirtExperiment::create_test() {
    const float angle  = SIMD_PI * (2.0 / 3.0);
    const float length = 1.5;
    const float radius = 0.2;
    
    std::vector<btVector3> joints;
    joints.emplace_back(0.0, 0.0, length);
    joints.emplace_back(0.0, 0.0, 0.0);
    joints.emplace_back(0.0, length * sin(angle), length * cos(angle));
    
    btCylinderShapeZ* shape = new btCylinderShapeZ(btVector3(radius, 0.0, length / 2));
    m_collisionShapes.push_back(shape);
    
    btVector3 z_axis(0.0, 0.0, 1.0);
    std::vector<btRigidBody*> bones;
    std::vector<btQuaternion> rots;
    // bones
    for (int i = 1; i < joints.size(); ++i) {
        btVector3& joint0 = joints[i - 1];
        btVector3& joint1 = joints[i];
        btVector3 bone_vec = joint1 - joint0;
        btQuaternion rot_z2bone = MathUtil::rot_between(z_axis, bone_vec);
        btTransform bone_trans(rot_z2bone, (joint0 + joint1) / 2);
        
        rots.push_back(rot_z2bone);
        if (i == 1) {
            bones.push_back(createRigidBody(0.0, bone_trans, shape));
        } else {
            bones.push_back(createRigidBody(1.0, bone_trans, shape));
        }
    }
    
    // Fix the first bone
    // m_dynamicsWorld->addConstraint(create_plane_fix_constraint(*bones[0], btVector3(0.0, 1.0, 0.0)));
    
    // joints
    for (int i = 1; i < joints.size() - 1; ++i) {
        btRigidBody* bone0 = bones[i - 1];
        btRigidBody* bone1 = bones[i];
        
        btVector3& joint = joints[i];
        btVector3 bone1_vec = joint - bone1->getWorldTransform().getOrigin();
        // pivot world frame
        btTransform pivot_frame(MathUtil::rot_between(z_axis, bone1_vec), joint);
        // local frames
        btTransform bone0_frame = bone0->getWorldTransform().inverse() * pivot_frame;
        btTransform bone1_frame = bone1->getWorldTransform().inverse() * pivot_frame;
        auto constraint = new btGeneric6DofSpring2Constraint(*bone0, *bone1, bone0_frame, bone1_frame);
        
        constraint->setLinearLowerLimit(btVector3(0.0, 0.0, 0.0));
        constraint->setLinearUpperLimit(btVector3(0.0, 0.0, 0.0));
        
        constraint->setAngularLowerLimit(btVector3(1.0, 1.0, 0.0));
        constraint->setAngularUpperLimit(btVector3(-1.0, -1.0, 0.0));
        constraint->enableSpring(3, true);
        constraint->setStiffness(3, 50);
        constraint->setDamping(3, 10);
        constraint->enableSpring(4, true);
        constraint->setStiffness(4, 50);
        constraint->setDamping(4, 10);
        // constraint->setDbgDrawSize(5.0f);
        
        m_dynamicsWorld->addConstraint(constraint, true);
        
    }
}

void SkirtExperiment::create_dynamic_bones() {
    std::vector<btTransform> base_trans;
    if (!get_transforms_from_file(base_filename, base_trans)) {
        std::cout << "cannot parse base file" << std::endl;
        return;
    }

    btSphereShape* sphere_shape = new btSphereShape(base_radius);
    m_collisionShapes.push_back(sphere_shape);
    m_base_trans = base_trans[0];
    m_base = createRigidBody(0.0, m_base_trans, sphere_shape);
    m_dynamicsWorld->addConstraint(create_spin_only_constraint(*m_base, btVector3(0.0, 1.0, 0.0)));

//    for (int i = 0; i < strand_count; ++i) {
//        create_strand(i, m_base);
//    }
    create_strand(5, m_base);
    
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

btTypedConstraint* SkirtExperiment::create_plane_fix_constraint(btRigidBody& body, const btVector3& normal) {
    // rotation between x-axis and plane normal
    btVector3 x_axis = btVector3(1.0, 0.0, 0.0);
    btQuaternion x_rot2_n = btQuaternion(x_axis.cross(normal), x_axis.angle(normal));
    // world transform of pivot
    btTransform pivot_trans_world = btTransform(x_rot2_n, body.getWorldTransform().getOrigin());
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
    btQuaternion x_rot2_n = btQuaternion(x_axis.cross(normal), x_axis.angle(normal));
    // world transform of pivot
    btTransform pivot_trans_world = btTransform(x_rot2_n, body.getWorldTransform().getOrigin());
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
