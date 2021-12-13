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

const std::string hip_filename = "./data/hip";
const float hip_cone_radius = 0.08;
const float hip_cone_height = 0.6;
const btTransform hip_handle_offset = btTransform(btQuaternion(0.0, 0.0, 0.0), btVector3(-1.0, 0.0, -2.0));

const std::string skirt_prefix = "./data/skirt";
const float skirt_node_mass = 0.01;
const float skirt_node_radius = 0.02;

struct SkirtExperiment : public CommonRigidBodyBase
{
    // All rigid body pointers in this class will get its ownership transfered to
    // btDiscreteDynamicsWold soon after construnction.
    // Object life cycle lies with btDiscreteDynamicsWold.
    // See CommonRigidBodyBase::exitPhysics()
    btRigidBody*                           ground;
    btRigidBody*                           hip;        // A kinematic object
    btRigidBody*                           hip_handle; // An handle object, its movement will be passed onto actual kinematic hip
    std::vector<std::vector<btTransform>>  skirt_trans;
    std::vector<std::vector<btRigidBody*>> skirt_strands;
    std::vector<btTransform>               strand_base_trans;
    
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
    
    void create_hip_cone();
    
    void create_skirt_strand(int i, btTransform anchor_frame);
    
    btPoint2PointConstraint* create_p2p_constraint(btRigidBody& rb_a,
                                                   btRigidBody& rb_b,
                                                   btVector3& pivot);
    
    btTypedConstraint* create_plane_fix_constraint(btRigidBody& body, const btVector3& normal);
};

// pre-tick callback for kinematic bodies
void kinematicPreTickCallback(btDynamicsWorld* world, btScalar deltaTime);

void SkirtExperiment::create_ground() {
    btBoxShape* ground_shape = createBoxShape(btVector3(50., 50., 50.));
    m_collisionShapes.push_back(ground_shape);
    
    btTransform ground_transform;
    ground_transform.setIdentity();
    ground_transform.setOrigin(btVector3(0, -50, 0));
    
    btScalar ground_mass(0.);
    createRigidBody(ground_mass, ground_transform, ground_shape, btVector4(0, 0, 1, 1));
}

void SkirtExperiment::create_hip_cone() {
    std::vector<btTransform> transforms;
    if (!get_transforms_from_file(hip_filename, transforms)) {
        return;
    }
    
    btConeShape* cone_shape = new btConeShape(hip_cone_radius, hip_cone_height);
    m_collisionShapes.push_back(cone_shape);
    
    hip = createRigidBody(0.0, transforms[0], cone_shape);
    hip_handle = createRigidBody(1.0, hip_handle_offset * hip->getWorldTransform(), cone_shape);
    
    m_dynamicsWorld->addConstraint(create_plane_fix_constraint(*hip_handle, btVector3(0.0, 1.0, 0.0)));
}

void SkirtExperiment::create_skirt_strand(int i, btTransform anchor_base) {
    std::vector<btTransform> transforms;
    if (!get_transforms_from_file(skirt_prefix + char('0' + i), transforms)) {
        return;
    }
    assert(!transforms.empty());

    btSphereShape* sphere = new btSphereShape(skirt_node_radius);
    m_collisionShapes.push_back(sphere);
    skirt_strands.emplace_back();
    for (int i = 0; i < transforms.size(); ++i) {
        std::cout << "i = " << i << std::endl;
        float mass = 0.0;
        if (i != 0) {
            mass = skirt_node_mass;
        }

        skirt_strands.back().push_back(createRigidBody(mass, transforms[i], sphere));
    }
    
    strand_base_trans.push_back(anchor_base.inverse() * transforms[0]);
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
    
    constraint->setAngularLowerLimit(btVector3(1.0, 0.0, 0.0));
    constraint->setAngularUpperLimit(btVector3(-1.0, 0.0, 0.0));
    constraint->enableSpring(3, true);
    constraint->setStiffness(3, 10);
    constraint->setDamping(3, 1);
    
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
    
    create_hip_cone();
    
    for (int i = 1; i <= 6; ++i) {
        create_skirt_strand(i, hip->getWorldTransform());
    }

    m_dynamicsWorld->setInternalTickCallback(kinematicPreTickCallback, this, true);
    m_dynamicsWorld->setGravity(btVector3(0.0, 0.0, 0.0));
	m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
}

void kinematicPreTickCallback(btDynamicsWorld* world, btScalar delta_time) {
    SkirtExperiment* exp = reinterpret_cast<SkirtExperiment*>(world->getWorldUserInfo());
    
    // pass on handles's movement to hip
    exp->hip->setWorldTransform(hip_handle_offset.inverse() * exp->hip_handle->getWorldTransform());
    
    // sync strands
    for (int i = 0; i < exp->skirt_strands.size(); ++i) {
        exp->skirt_strands[i][0]->setWorldTransform(exp->hip->getWorldTransform() * exp->strand_base_trans[i]);
    }
    
//    btTransform trans;
//	btVector3 linear_vel(0, 0, 0);
//	btVector3 ang_vel(0, 3, 0);
//    btTransformUtil::integrateTransform(head->getWorldTransform(), linear_vel, ang_vel, delta_time, trans);
//    head->getMotionState()->setWorldTransform(trans);
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
