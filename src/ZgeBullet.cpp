 /*
ZgeBullet Library
Copyright (c) 2012-2018 Radovan Cervenka

This software is provided 'as-is', without any express or implied
warranty. In no event will the authors be held liable for any damages
arising from the use of this software.

Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it
freely, subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not
   claim that you wrote the original software. If you use this software
   in a product, an acknowledgment in the product documentation would be
   appreciated but is not required.

2. Altered source versions must be plainly marked as such, and must not be
   misrepresented as being the original software.

3. This notice may not be removed or altered from any source distribution.
*/

/// The main file used to compile Windows DLL and Android shared library

#pragma unmanaged


// Includes

#include "btBulletDynamicsCommon.h"
#include "BulletCollision/CollisionShapes/btHeightfieldTerrainShape.h"
#include "BulletCollision/Gimpact/btGImpactShape.h"
#include "BulletCollision/CollisionDispatch/btGhostObject.h"
#include "BulletDynamics/Character/btKinematicCharacterController.h"
#include "BulletCollision/CollisionShapes/btShapeHull.h"


// Definitions

#ifdef _WIN32
#define EXPORT extern "C" __declspec(dllexport)
#undef NULL
#define NULL nullptr
#else
#define EXPORT extern "C"
#endif


// Declarations of functions
struct simulationWorld;
EXPORT void zbtSetCurrentWorld(simulationWorld* world);
EXPORT void zbtDeleteRigidBody(btRigidBody*);
EXPORT void zbtDeleteAllShapes();
EXPORT void zbtDeleteGhostObject(btGhostObject*);


// Types

// Used for passing 3D vectors to public functions
struct v3{
	float x;
	float y;
	float z;

	inline void set(btVector3 const & src) {
		x = src.x();
		y = src.y();
		z = src.z();
	}
};

struct v4 : v3{
	float w;

	inline void set(btQuaternion const & src) {
		x = src.getX();
		y = src.getY();
		z = src.getZ();
		w = src.getW();
	}
};

// A single physical simulation world
ATTRIBUTE_ALIGNED16(struct) simulationWorld {

	BT_DECLARE_ALIGNED_ALLOCATOR();

	simulationWorld() :
		rayTestHitPoint(btVector3(0, 0, 0)),
		rayTestHitNormal(btVector3(0, 0, 0)),
		createGhostPairCallback(true),
		vehicleRaycaster(NULL),
		manifoldIndex(-1),
		manifold(NULL),
		manifoldPointIndex(-1)
	{
		broadphase = new btDbvtBroadphase();
		collisionConfiguration = new btDefaultCollisionConfiguration();
		dispatcher = new btCollisionDispatcher(collisionConfiguration);
		solver = new btSequentialImpulseConstraintSolver();
		world = new btDiscreteDynamicsWorld(dispatcher, broadphase, solver, collisionConfiguration);
	}

	~simulationWorld() {

		zbtSetCurrentWorld(this);

		// delete collision objects
		btCollisionObjectArray colArray = world->getCollisionObjectArray();
		for (int i = world->getNumCollisionObjects() - 1; i >= 0; --i){
			btCollisionObject* obj = colArray[i];
			if (obj->getInternalType() == btCollisionObject::CO_RIGID_BODY)
				zbtDeleteRigidBody(btRigidBody::upcast(obj));
			else
				//btCollisionObject::CO_GHOST_OBJECT
				zbtDeleteGhostObject(btGhostObject::upcast(obj));
		}

		// delete remaining constraints
		for (int i = world->getNumConstraints() - 1; i >= 0; --i){
			btTypedConstraint* co = world->getConstraint(i);
			world->removeConstraint(co);
			delete co;
		}

		// destroy vehicle raycaster, if any
		if (vehicleRaycaster) delete vehicleRaycaster;

		// delete collision shapes
		deleteAllShapes();

		// delete related Bullet objects
		delete world;
		delete solver;
		delete dispatcher;
		delete collisionConfiguration;
		delete broadphase;
	}

	void deleteAllShapes() {
		for (int i = collisionShapeList.size() - 1; i >= 0; --i)
			delete collisionShapeList[i];

		collisionShapeList.clear();
	}

	btBroadphaseInterface* broadphase;
	btDefaultCollisionConfiguration* collisionConfiguration;
	btCollisionDispatcher* dispatcher;
	btSequentialImpulseConstraintSolver* solver;
	btDiscreteDynamicsWorld* world;

	btVector3 rayTestHitPoint;
	btVector3 rayTestHitNormal;

	bool createGhostPairCallback;
	btVehicleRaycaster* vehicleRaycaster;
	btRaycastVehicle::btVehicleTuning tuning;

	int manifoldIndex;
	btPersistentManifold* manifold;
	int manifoldPointIndex;

	btAlignedObjectArray<btCollisionShape*> collisionShapeList;
};


// Macros

#define ADD_COLLISION_SHAPE(shape) \
	gCurrentWorld->collisionShapeList.push_back(shape); \
	return shape

#define ITERATE_MANIFOLDS_FOR_OBJECT(obj, body) \
	for (int i = gCurrentWorld->world->getDispatcher()->getNumManifolds() - 1; i >= 0; --i) { \
		btPersistentManifold* pm = gCurrentWorld->world->getDispatcher()->getManifoldByIndexInternal(i); \
		if (const_cast<btCollisionObject*>(pm->getBody0()) == obj || const_cast<btCollisionObject*>(pm->getBody1()) == obj){ body; } }\


// Globals

// Constants

const btVector3 FORWARD = btVector3(0.0, 0.0, 1.0);
const btVector3 BACK = btVector3(0.0, 0.0, -1.0);
const btVector3 LEFT = btVector3(1.0, 0.0, 0.0);

// Global variables

simulationWorld* gCurrentWorld = NULL;

// Utilities

// Returns quaternion from Euler angles
inline btQuaternion rot(float rx, float ry, float rz) {
	btQuaternion q = btQuaternion();

	// fixing gimbal lock
	if (ry == 0.25 || ry == -0.25) ry += 0.0001f;
	q.setEulerZYX(rz*SIMD_2_PI, ry*SIMD_2_PI, rx*SIMD_2_PI);
	return q;
}

// Creates btTransform from position and rotation given by Euler angles
inline btTransform transform(float x, float y, float z, float rx, float ry, float rz) {
	return btTransform(rot(rx, ry, rz), btVector3(x, y, z));
}

/*inline void checkWheel(btRaycastVehicle* vehicle, int wheelId) {
	btAssert(wheelId >= 0 && wheelId < vehicle->getNumWheels());
}*/

// Returns a sum of all applied impulses to persistent manifold
inline float sumAppliedImpulses(btPersistentManifold* pm) {
	float imp = 0;
	for (int j = pm->getNumContacts() - 1; j >= 0; --j)
		imp += pm->getContactPoint(j).getAppliedImpulse();
		
	return imp;
}


// Public API

// World

EXPORT simulationWorld* zbtCreateWorld() {
	return new simulationWorld();
}

EXPORT void zbtDestroyWorld(simulationWorld* world) {
	delete world;
}

EXPORT void zbtSetCurrentWorld(simulationWorld* world) {
	gCurrentWorld = world;
}

EXPORT void zbtSetWorldGravity(float x, float y, float z) {
	gCurrentWorld->world->setGravity(btVector3(x, y, z));
}

EXPORT void zbtStepSimulation(float timeStep, int maxSubSteps, float fixedTimeStep) {
	gCurrentWorld->world->stepSimulation(timeStep, maxSubSteps, fixedTimeStep);
}


// Collision shapes

EXPORT btCollisionShape* zbtCreateStaticPlaneShape(float normalX, float normalY, float normalZ, float planeConstant) {
	ADD_COLLISION_SHAPE(new btStaticPlaneShape(btVector3(normalX, normalY, normalZ), planeConstant));
}

EXPORT btCollisionShape* zbtCreateBoxShape(float x, float y, float z) {
	ADD_COLLISION_SHAPE(new btBoxShape((btVector3(x, y, z))));
}

EXPORT btCollisionShape* zbtCreateSphereShape(float radius) {
	ADD_COLLISION_SHAPE(new btSphereShape(radius));
}

EXPORT btCollisionShape* zbtCreateScalableSphereShape(float radius) {
#ifdef _WIN32
	ADD_COLLISION_SHAPE(new btMultiSphereShape(&btVector3(0,0,0), &radius, 1));
#else
	static const btVector3 positions[1] = {btVector3(0,0,0)};
	static btScalar radi[1] = {radius};

	ADD_COLLISION_SHAPE(new btMultiSphereShape(positions, radi, 1));
#endif
}

EXPORT btCollisionShape* zbtCreateConeShape(float radius, float height) {
	ADD_COLLISION_SHAPE(new btConeShape(radius, height));
}

EXPORT btCollisionShape* zbtCreateCylinderShape(float radius, float height) {
	ADD_COLLISION_SHAPE(new btCylinderShape(btVector3(radius, height, radius)));
}

EXPORT btCollisionShape* zbtCreateCapsuleShape(float radius, float height) {
	ADD_COLLISION_SHAPE(new btCapsuleShape(radius, height));
}

EXPORT btCollisionShape* zbtCreateCompoundShape() {
	ADD_COLLISION_SHAPE(new btCompoundShape());
}

EXPORT btCollisionShape* zbtAddChildShape(btCompoundShape* compoundShape, btCollisionShape* childShape,
	float x, float y, float z, float rx, float ry, float rz) {

	compoundShape->addChildShape(transform(x, y, z, rx, ry, rz), childShape);
	return compoundShape;
}

EXPORT btCollisionShape* zbtRemoveChildShape(btCompoundShape* compoundShape, btCollisionShape* childShape) {

	compoundShape->removeChildShape(childShape);
	return compoundShape;
}

EXPORT btCollisionShape* zbtCreateHeightfieldTerrainShape(void* heightfieldData, int width, int length,
	float minHeight, float maxHeight, int upAxis, bool bFlipQuadEdges, bool bDiamondSubdivision) {

	try {
		btHeightfieldTerrainShape* hf = new btHeightfieldTerrainShape(width, length, heightfieldData,
			1, minHeight, maxHeight, upAxis, PHY_FLOAT, bFlipQuadEdges);
		hf->setUseDiamondSubdivision(bDiamondSubdivision);
		ADD_COLLISION_SHAPE(hf);
	} catch (...) { return NULL; }
}

EXPORT btCollisionShape* zbtCreateConvexHullShape(float* points, int numPoints) {

	try {
		btConvexHullShape* ch = new btConvexHullShape(points, numPoints, sizeof(float) * 3);

		//create a hull approximation - optimization of hull points
		btShapeHull* hull = new btShapeHull(ch);
		hull->buildHull(ch->getMargin());
		delete ch;

		ADD_COLLISION_SHAPE(new btConvexHullShape((const float*)hull->getVertexPointer(), hull->numVertices()));
	} catch (...) { return NULL; }
}

EXPORT btCollisionShape* zbtCreateMultiSphereShape(float* positions, float* radii, int numSpheres) {

	try {
		// copy float array to btVector3 array
		btAlignedObjectArray<btVector3> pos;
		for (int i = 0; i < numSpheres; ++i)
			pos.push_back(btVector3(positions[i*3], positions[i*3+1], positions[i*3+2]));

		ADD_COLLISION_SHAPE(new btMultiSphereShape((btVector3*)positions, radii, numSpheres));
	} catch (...) { return NULL; }
}

EXPORT btCollisionShape* zbtCreateTriangleMeshShape(float* triangles, int numTriangles, int meshType) {

	// create triangle mesh
	btTriangleMesh* triangleMesh = new btTriangleMesh(false, false);

	// add triangles
	try {
		for (int i = (numTriangles-1) * 9; i >= 0; --i)
			triangleMesh->addTriangle(
				btVector3(triangles[i], triangles[i+1], triangles[i+2]),
				btVector3(triangles[i+3], triangles[i+4], triangles[i+5]),
				btVector3(triangles[i+6], triangles[i+7], triangles[i+8]), true);
	} catch (...) { return NULL; }

	// create triangle mesh collision shape

	btCollisionShape* tm;

	switch (meshType) {
	case 1: // convex hull
		tm = new btConvexTriangleMeshShape(triangleMesh);
		break;
	case 2: // concave static-triangle mesh shape
		tm = new btBvhTriangleMeshShape(triangleMesh, true, true);
		break;
	case 3: // concave deformable mesh
		tm = new btGImpactMeshShape(triangleMesh);
		static_cast<btGImpactMeshShape*>(tm)->updateBound();
	}

	ADD_COLLISION_SHAPE(tm);
}

EXPORT void zbtUpdateDeformableTriangleMesh(btGImpactMeshShape* triangleMeshShape) {
	triangleMeshShape->postUpdate();
}

EXPORT void zbtSetShapeLocalScaling(btCollisionShape* shape, float x, float y, float z) {
	shape->setLocalScaling(btVector3(x, y, z));

	if (shape->getShapeType() == GIMPACT_SHAPE_PROXYTYPE)
		static_cast<btGImpactMeshShape*>(shape)->updateBound();
}

EXPORT void zbtSetShapeMargin(btCollisionShape* shape, float margin) {
	shape->setMargin(margin);
}

EXPORT void zbtDeleteShape(btCollisionShape* shape) {
	delete shape;
}

EXPORT void zbtDeleteAllShapes() {
	gCurrentWorld->deleteAllShapes();
}


// Rigid bodies

EXPORT btRigidBody* zbtCreateRigidBodyXYZ(float mass, btCollisionShape* shape, float x, float y, float z, float rx, float ry, float rz) {

	btDefaultMotionState* motionState = new btDefaultMotionState(transform(x, y, z, rx, ry, rz));
	btVector3 inertia(0, 0, 0);

	if (mass > 0.f) // mass == 0 - fixed rigid body
		shape->calculateLocalInertia(mass, inertia);

	btRigidBody::btRigidBodyConstructionInfo rigidBodyCI(mass, motionState, shape, inertia);
	btRigidBody* rigidBody = new btRigidBody(rigidBodyCI);
	gCurrentWorld->world->addRigidBody(rigidBody);

	return rigidBody;
}

EXPORT btRigidBody* zbtCreateRigidBody(float mass, btCollisionShape* shape, v3 &position, v3 &rotation) {
	return zbtCreateRigidBodyXYZ(mass, shape, position.x, position.y, position.z,
		rotation.x, rotation.y, rotation.z);
}

EXPORT void zbtDeleteRigidBody(btRigidBody* rigidBody) {

	// remove related constraints
	while (rigidBody->getNumConstraintRefs()) {
		btTypedConstraint* tc = rigidBody->getConstraintRef(0);

		gCurrentWorld->world->removeConstraint(tc);
		delete tc;
	}

	// remove motion state
	delete rigidBody->getMotionState();

	// remove body
	gCurrentWorld->world->removeRigidBody(rigidBody);
	delete rigidBody;
}

EXPORT void zbtSetMass(btRigidBody* rigidBody, float mass) {
	btVector3 inertia;
	rigidBody->getCollisionShape()->calculateLocalInertia(mass, inertia);
	rigidBody->setMassProps(mass, inertia);
}

EXPORT void zbtSetDamping(btRigidBody* rigidBody, float linearDamping, float angularDamping) {
	rigidBody->setDamping(linearDamping, angularDamping);
}

EXPORT void zbtSetLinearFactor(btRigidBody* rigidBody, float x, float y, float z) {
	rigidBody->setLinearFactor(btVector3(x, y, z));
}

EXPORT void zbtSetAngularFactor(btRigidBody* rigidBody, float x, float y, float z) {
	rigidBody->setAngularFactor(btVector3(x, y, z));
}

EXPORT void zbtSetGravity(btRigidBody* rigidBody, float x, float y, float z) {
	rigidBody->setGravity(btVector3(x, y, z));
}

EXPORT void zbtSetLinearVelocity(btRigidBody* rigidBody, float x, float y, float z) {
	rigidBody->setLinearVelocity(btVector3(x, y, z));
	rigidBody->activate(true);
}

EXPORT void zbtGetLinearVelocity(btRigidBody* rigidBody, float &outX, float &outY, float &outZ) {
	btVector3 linearVelocity = rigidBody->getLinearVelocity();
	outX = linearVelocity.getX();
	outY = linearVelocity.getY();
	outZ = linearVelocity.getZ();
}

EXPORT void zbtSetAngularVelocity(btRigidBody* rigidBody, float x, float y, float z) {
	rigidBody->setAngularVelocity(btVector3(x, y, z));
	rigidBody->activate(true);
}

EXPORT void zbtGetAngularVelocity(btRigidBody* rigidBody, float &outX, float &outY, float &outZ) {
	btVector3 angularVelocity = rigidBody->getAngularVelocity();
	outX = angularVelocity.getX();
	outY = angularVelocity.getY();
	outZ = angularVelocity.getZ();
}

EXPORT void zbtApplyCentralImpulse(btRigidBody* rigidBody, float x, float y, float z) {
	rigidBody->applyCentralImpulse(btVector3(x, y, z));
	rigidBody->activate(true);
}

EXPORT void zbtApplyCentralImpulseLocal(btRigidBody* rigidBody, float x, float y, float z) {
	rigidBody->applyCentralImpulse(rigidBody->getWorldTransform().getBasis() * btVector3(x, y, z));
	rigidBody->activate(true);
}

EXPORT void zbtApplyImpulse(btRigidBody* rigidBody, float x, float y, float z,
	float relX, float relY, float relZ) {

	rigidBody->applyImpulse(btVector3(x, y, z), btVector3(relX, relY, relZ));
	rigidBody->activate(true);
}

EXPORT void zbtApplyTorque(btRigidBody* rigidBody, float x, float y, float z) {
	rigidBody->applyTorque(btVector3(x, y, z));
	rigidBody->activate(true);
}

EXPORT void zbtApplyTorqueImpulse(btRigidBody* rigidBody, float x, float y, float z) {
	rigidBody->applyTorqueImpulse(btVector3(x, y, z));
	rigidBody->activate(true);
}

EXPORT void zbtApplyTorqueLocal(btRigidBody* rigidBody, float x, float y, float z) {
	rigidBody->applyTorque(rigidBody->getWorldTransform().getBasis() * btVector3(x, y, z));
	rigidBody->activate(true);
}

EXPORT void zbtApplyTorqueImpulseLocal(btRigidBody* rigidBody, float x, float y, float z) {
	rigidBody->applyTorqueImpulse(rigidBody->getWorldTransform().getBasis() * btVector3(x, y, z));
	rigidBody->activate(true);
}

EXPORT void zbtSetSleepingThresholds(btRigidBody* rigidBody, float linear, float angular) {
	rigidBody->setSleepingThresholds(linear, angular*SIMD_2_PI);
}


// Constraints and limits

EXPORT int zbtAreConnected(btRigidBody* rigidBodyA, btRigidBody* rigidBodyB) {
	return !rigidBodyA->checkCollideWithOverride(rigidBodyB);
}

EXPORT btTypedConstraint* zbtAddFixedConstraint(btRigidBody* rigidBodyA, btRigidBody* rigidBodyB,
	float pivotAx, float pivotAy, float pivotAz,
	float pivotBx, float pivotBy, float pivotBz,
	float rotAx, float rotAy, float rotAz,
	float rotBx, float rotBy, float rotBz, bool bDisableCollision) {

	btTypedConstraint* tc = new btFixedConstraint(*rigidBodyA, *rigidBodyB,
		transform(pivotAx, pivotAy, pivotAz, rotAx, rotAy, rotAz),
		transform(pivotBx, pivotBy, pivotBz, rotBx, rotBy, rotBz));
	gCurrentWorld->world->addConstraint(tc, bDisableCollision);

	return tc;
}

EXPORT btTypedConstraint* zbtAddPoint2PointConstraint1(btRigidBody* rigidBody,
	float pivotX, float pivotY, float pivotZ) {

	btTypedConstraint* tc = new btPoint2PointConstraint(*rigidBody, btVector3(pivotX, pivotY, pivotZ));
	gCurrentWorld->world->addConstraint(tc, true);

	return tc;
}

EXPORT btTypedConstraint* zbtAddPoint2PointConstraint(btRigidBody* rigidBodyA, btRigidBody* rigidBodyB,
	float pivotAx, float pivotAy, float pivotAz,
	float pivotBx, float pivotBy, float pivotBz, bool bDisableCollision) {

	btTypedConstraint* tc = new btPoint2PointConstraint(*rigidBodyA, *rigidBodyB,
		btVector3(pivotAx, pivotAy, pivotAz), btVector3(pivotBx, pivotBy, pivotBz));
	gCurrentWorld->world->addConstraint(tc, bDisableCollision);

	return tc;
}

EXPORT btTypedConstraint* zbtAddHingeConstraint1(btRigidBody* rigidBody,
	float pivotX, float pivotY, float pivotZ,
	float axisX, float axisY, float axisZ) {

	btTypedConstraint* tc = new btHingeConstraint(*rigidBody,
		btVector3(pivotX, pivotY, pivotZ), btVector3(axisX, axisY, axisZ));
	gCurrentWorld->world->addConstraint(tc, true);

	return tc;
}

EXPORT btTypedConstraint* zbtAddHingeConstraint(btRigidBody* rigidBodyA, btRigidBody* rigidBodyB,
	float pivotAx, float pivotAy, float pivotAz,
	float pivotBx, float pivotBy, float pivotBz,
	float axisAx, float axisAy, float axisAz,
	float axisBx, float axisBy, float axisBz, bool bDisableCollision) {

	btTypedConstraint* tc = new btHingeConstraint(*rigidBodyA, *rigidBodyB,
		btVector3(pivotAx, pivotAy, pivotAz), btVector3(pivotBx, pivotBy, pivotBz),
		btVector3(axisAx, axisAy, axisAz), btVector3(axisBx, axisBy, axisBz));
	gCurrentWorld->world->addConstraint(tc, bDisableCollision);

	return tc;
}

EXPORT void zbtSetHingeLimits(btHingeConstraint* hinge, float low, float high,
	float softness, float biasFactor, float relaxationFactor) {

	hinge->setLimit(low*SIMD_2_PI , high*SIMD_2_PI, softness, biasFactor, relaxationFactor);
}

EXPORT void zbtEnableHingeAngularMotor(btHingeConstraint* hinge, bool bEnableMotor, float targetVelocity, float maxMotorImpulse) {
	hinge->enableAngularMotor(bEnableMotor, targetVelocity, maxMotorImpulse);
}

EXPORT btTypedConstraint* zbtAddConeTwistConstraint1(btRigidBody* rigidBody,
	float pivotX, float pivotY, float pivotZ,
	float rotX, float rotY, float rotZ) {

	btTypedConstraint* tc = new btConeTwistConstraint(*rigidBody,
		transform(pivotX, pivotY, pivotZ, rotX, rotY, rotZ));
	gCurrentWorld->world->addConstraint(tc, true);

	return tc;
}

EXPORT btTypedConstraint* zbtAddConeTwistConstraint(btRigidBody* rigidBodyA, btRigidBody* rigidBodyB,
	float pivotAx, float pivotAy, float pivotAz,
	float pivotBx, float pivotBy, float pivotBz,
	float rotAx, float rotAy, float rotAz,
	float rotBx, float rotBy, float rotBz, bool bDisableCollision ) {

	btTypedConstraint* tc = new btConeTwistConstraint(*rigidBodyA, *rigidBodyB,
		transform(pivotAx, pivotAy, pivotAz, rotAx, rotAy, rotAz),
		transform(pivotBx, pivotBy, pivotBz, rotBx, rotBy, rotBz));
	gCurrentWorld->world->addConstraint(tc, bDisableCollision);

	return tc;
}

EXPORT void zbtSetConeTwistLimits(btConeTwistConstraint* twist,
	float swingSpanA, float swingSpanB, float twistSpan,
	float damping, float softness, float biasFactor, float relaxationFactor) {

	twist->setLimit(swingSpanA*SIMD_2_PI, swingSpanB*SIMD_2_PI, twistSpan*SIMD_2_PI,
		softness, biasFactor, relaxationFactor);
	twist->setDamping(damping);
}

EXPORT void zbtEnableConeTwistMotor(btConeTwistConstraint* twist,
	bool bEnableMotor, float maxMotorImpulse,
	float targetX, float targetY, float targetZ) {

	twist->enableMotor(bEnableMotor);
	twist->setMaxMotorImpulse(maxMotorImpulse);
	twist->setMotorTarget(btQuaternion(targetY*SIMD_2_PI, targetX*SIMD_2_PI, targetZ*SIMD_2_PI));
}

EXPORT btTypedConstraint* zbtAddSliderConstraint1(btRigidBody* rigidBody,
	float pivotX, float pivotY, float pivotZ,
	float rotX, float rotY, float rotZ,
	bool bUseLinearReferenceWorldFrame) {

	btTypedConstraint* tc = new btSliderConstraint(*rigidBody,
		transform(pivotX, pivotY, pivotZ, rotX, rotY, rotZ),
		bUseLinearReferenceWorldFrame);
	gCurrentWorld->world->addConstraint(tc, true);

	return tc;
}

EXPORT btTypedConstraint* zbtAddSliderConstraint(btRigidBody* rigidBodyA, btRigidBody* rigidBodyB,
	float pivotAx, float pivotAy, float pivotAz,
	float pivotBx, float pivotBy, float pivotBz,
	float rotAx, float rotAy, float rotAz,
	float rotBx, float rotBy, float rotBz, bool bUseLinearReferenceFrameA, bool bDisableCollision ) {

	btTypedConstraint* tc = new btSliderConstraint(*rigidBodyA, *rigidBodyB,
		transform(pivotAx, pivotAy, pivotAz, rotAx, rotAy, rotAz),
		transform(pivotBx, pivotBy, pivotBz, rotBx, rotBy, rotBz),
		bUseLinearReferenceFrameA);
	gCurrentWorld->world->addConstraint(tc, bDisableCollision);

	return tc;
}

EXPORT void zbtSetSliderLimits(btSliderConstraint* slider,
	float linLower, float linUpper, float angLower, float angUpper) {

	slider->setLowerLinLimit(linLower);
	slider->setUpperLinLimit(linUpper);
	slider->setLowerAngLimit(angLower*SIMD_2_PI);
	slider->setUpperAngLimit(angUpper*SIMD_2_PI);
}

EXPORT void zbtSetSliderSoftness(btSliderConstraint* slider,
	float dirLin, float dirAng, float limLin, float limAng,
	float orthoLin, float orthoAng) {

	slider->setSoftnessDirLin(dirLin);
	slider->setSoftnessDirAng(dirAng*SIMD_2_PI);
	slider->setSoftnessLimLin(limLin);
	slider->setSoftnessLimAng(limAng*SIMD_2_PI);
	slider->setSoftnessOrthoLin(orthoLin);
	slider->setSoftnessOrthoAng(orthoAng*SIMD_2_PI);
}

EXPORT void zbtSetSliderRestitution(btSliderConstraint* slider,
	float dirLin, float dirAng, float limLin, float limAng,
	float orthoLin, float orthoAng) {

	slider->setRestitutionDirLin(dirLin);
	slider->setRestitutionDirAng(dirAng*SIMD_2_PI);
	slider->setRestitutionLimLin(limLin);
	slider->setRestitutionLimAng(limAng*SIMD_2_PI);
	slider->setRestitutionOrthoLin(orthoLin);
	slider->setRestitutionOrthoAng(orthoAng*SIMD_2_PI);
}

EXPORT void zbtSetSliderDamping(btSliderConstraint* slider,
	float dirLin, float dirAng, float limLin, float limAng,
	float orthoLin, float orthoAng) {

	slider->setDampingDirLin(dirLin);
	slider->setDampingDirAng(dirAng);
	slider->setDampingLimLin(limLin);
	slider->setDampingLimAng(limAng);
	slider->setDampingOrthoLin(orthoLin);
	slider->setDampingOrthoAng(orthoAng);
}

EXPORT void zbtEnableSliderLinearMotor(btSliderConstraint* slider,
	bool bEnableMotor, float targetVelocity, float maxForce) {

	slider->setPoweredLinMotor(bEnableMotor);
	if (bEnableMotor) {
		slider->setTargetLinMotorVelocity(targetVelocity);
		slider->setMaxLinMotorForce(maxForce);
	}
}

EXPORT void zbtEnableSliderAngularMotor(btSliderConstraint* slider,
	bool bEnableMotor, float targetVelocity, float maxForce) {

	slider->setPoweredAngMotor(bEnableMotor);
	if (bEnableMotor) {
		slider->setTargetAngMotorVelocity(targetVelocity);
		slider->setMaxAngMotorForce(maxForce);
	}
}

EXPORT btTypedConstraint* zbtAddGearConstraint(btRigidBody* rigidBodyA, btRigidBody* rigidBodyB,
	float axisAx, float axisAy, float axisAz, float axisBx, float axisBy, float axisBz,
	float ratio) {

	btTypedConstraint* tc = new btGearConstraint(*rigidBodyA, *rigidBodyB,
		btVector3(axisAx, axisAy, axisAz), btVector3(axisBx, axisBy, axisBz),
		ratio);
	gCurrentWorld->world->addConstraint(tc, true);

	return tc;
}

EXPORT void zbtSetGearConstraint(btGearConstraint* gear,
	float axisAx, float axisAy, float axisAz, float axisBx, float axisBy, float axisBz,
	float ratio) {

	btVector3 axisA = btVector3(axisAx, axisAy, axisAz);
	btVector3 axisB = btVector3(axisBx, axisBy, axisBz);

	gear->setAxisA(static_cast<btVector3&>(axisA));
	gear->setAxisB(static_cast<btVector3&>(axisB));
	gear->setRatio(ratio);
}

EXPORT btTypedConstraint* zbtAddGeneric6DofConstraint1(btRigidBody* rigidBody,
	float pivotX, float pivotY, float pivotZ,
	float rotX, float rotY, float rotZ,
	bool bUseLinearReferenceWorldFrame) {

	btTypedConstraint* tc = new btGeneric6DofConstraint(*rigidBody,
		transform(pivotX, pivotY, pivotZ, rotX, rotY, rotZ),
		bUseLinearReferenceWorldFrame);
	gCurrentWorld->world->addConstraint(tc, false);

	return tc;
}

EXPORT btTypedConstraint* zbtAddGeneric6DofConstraint(btRigidBody* rigidBodyA, btRigidBody* rigidBodyB,
	float pivotAx, float pivotAy, float pivotAz,
	float pivotBx, float pivotBy, float pivotBz,
	float rotAx, float rotAy, float rotAz,
	float rotBx, float rotBy, float rotBz, bool bUseLinearReferenceFrameA, bool bDisableCollision ) {

	btTypedConstraint* tc = new btGeneric6DofConstraint(*rigidBodyA, *rigidBodyB,
		transform(pivotAx, pivotAy, pivotAz, rotAx, rotAy, rotAz),
		transform(pivotBx, pivotBy, pivotBz, rotBx, rotBy, rotBz),
		bUseLinearReferenceFrameA);
	gCurrentWorld->world->addConstraint(tc, bDisableCollision );

	return tc;
}

EXPORT void zbtSetGeneric6DofLimits(btGeneric6DofConstraint* dof,
	int axis, float lower, float upper) {

	if(axis < 3)
		dof->setLimit(axis, lower, upper); // linear
	else
		dof->setLimit(axis, lower*SIMD_2_PI, upper*SIMD_2_PI); // angular
}

EXPORT void zbtSetGeneric6DofLinearLimits(btGeneric6DofConstraint* dof,
	float lowerX, float lowerY, float lowerZ,
	float upperX, float upperY, float upperZ) {

	dof->setLinearLowerLimit(btVector3(lowerX, lowerY, lowerZ));
	dof->setLinearUpperLimit(btVector3(upperX, upperY, upperZ));
}

EXPORT void zbtSetGeneric6DofAngularLimits(btGeneric6DofConstraint* dof,
	float lowerX, float lowerY, float lowerZ,
	float upperX, float upperY, float upperZ) {

	dof->setAngularLowerLimit(btVector3(lowerX, lowerY, lowerZ) * SIMD_2_PI);
	dof->setAngularUpperLimit(btVector3(upperX, upperY, upperZ) * SIMD_2_PI);
}

EXPORT btTypedConstraint* zbtAddGeneric6DofSpringConstraint1(btRigidBody* rigidBody,
	float pivotX, float pivotY, float pivotZ,
	float rotX, float rotY, float rotZ,
	bool bUseLinearReferenceWorldFrame) {

	btTypedConstraint* tc = new btGeneric6DofSpringConstraint(*rigidBody,
		transform(pivotX, pivotY, pivotZ, rotX, rotY, rotZ),
		bUseLinearReferenceWorldFrame);
	gCurrentWorld->world->addConstraint(tc, false);

	return tc;
}

EXPORT btTypedConstraint* zbtAddGeneric6DofSpringConstraint(btRigidBody* rigidBodyA, btRigidBody* rigidBodyB,
	float pivotAx, float pivotAy, float pivotAz,
	float pivotBx, float pivotBy, float pivotBz,
	float rotAx, float rotAy, float rotAz,
	float rotBx, float rotBy, float rotBz, bool bUseLinearReferenceFrameA, bool bDisableCollision) {

	btTypedConstraint* tc = new btGeneric6DofSpringConstraint(*rigidBodyA, *rigidBodyB,
		transform(pivotAx, pivotAy, pivotAz, rotAx, rotAy, rotAz),
		transform(pivotBx, pivotBy, pivotBz, rotBx, rotBy, rotBz),
		bUseLinearReferenceFrameA);
	gCurrentWorld->world->addConstraint(tc, bDisableCollision);

	return tc;
}

EXPORT void zbtSetGeneric6DofSpring(btGeneric6DofSpringConstraint* spring,
	int axis, bool bEnableSpring, float stiffness, float damping,
	float equilibriumPoint) {

	spring->enableSpring(axis, bEnableSpring);
	if (bEnableSpring){
		spring->setStiffness(axis, stiffness);
		spring->setDamping(axis, damping);

		if (axis < 3)
			spring->setEquilibriumPoint(axis, equilibriumPoint); // linear
		else
			spring->setEquilibriumPoint(axis, equilibriumPoint*SIMD_2_PI); // angular
	}
}

EXPORT void zbtSetEnabled(btTypedConstraint* constraint, bool bEnabled) {
	constraint->setEnabled(bEnabled);
}

EXPORT void zbtDeleteConstraint(btTypedConstraint* constraint) {
	gCurrentWorld->world->removeConstraint(constraint);
	delete constraint;
}


// Raycast vehicle

EXPORT void zbtSetVehicleTunning(float suspStiffness, float suspCompression,
	float suspDamping, float maxSuspTravelCm, float maxSuspForce,
	float frictionSlip) {

	gCurrentWorld->tuning.m_suspensionStiffness = suspStiffness;
	gCurrentWorld->tuning.m_suspensionCompression = suspCompression;
	gCurrentWorld->tuning.m_suspensionDamping = suspDamping;
	gCurrentWorld->tuning.m_maxSuspensionTravelCm = maxSuspTravelCm;
	gCurrentWorld->tuning.m_frictionSlip = frictionSlip;
	gCurrentWorld->tuning.m_maxSuspensionForce = maxSuspForce;
}

EXPORT btRaycastVehicle* zbtCreateRaycastVehicle(btRigidBody* carChassis,
	int rightAxis, int upAxis, int forwardAxis) {

	if(!gCurrentWorld->vehicleRaycaster) gCurrentWorld->vehicleRaycaster = new btDefaultVehicleRaycaster(gCurrentWorld->world);
	btRaycastVehicle* rv = new btRaycastVehicle(gCurrentWorld->tuning, carChassis, gCurrentWorld->vehicleRaycaster);
	rv->setCoordinateSystem(rightAxis, upAxis, forwardAxis);
	gCurrentWorld->world->addVehicle(rv);
	carChassis->setActivationState(DISABLE_DEACTIVATION); //necessary

	return rv;
}

EXPORT int zbtAddWheel(btRaycastVehicle* vehicle,
	float connectionPointX, float connectionPointY, float connectionPointZ,
	float directionX, float directionY, float directionZ,
	float wheelAxleX, float wheelAxleY, float wheelAxleZ,
	float wheelRadius, float suspRestLength, bool bIsFrontWheel) {

	vehicle->addWheel(btVector3(connectionPointX, connectionPointY, connectionPointZ),
		btVector3(directionX, directionY, directionZ), btVector3(wheelAxleX, wheelAxleY, wheelAxleZ),
		suspRestLength, wheelRadius, gCurrentWorld->tuning, bIsFrontWheel);

	return vehicle->getNumWheels() - 1;
}

EXPORT void zbtSetWheelIsFront(btRaycastVehicle* vehicle, int wheelId, bool bIsFront) {
	//checkWheel(vehicle, wheelId);
	vehicle->getWheelInfo(wheelId).m_bIsFrontWheel = bIsFront;
}

EXPORT void zbtSetWheelRadius(btRaycastVehicle* vehicle, int wheelId, float radius) {
	//checkWheel(vehicle, wheelId);
	vehicle->getWheelInfo(wheelId).m_wheelsRadius = radius;
}

EXPORT void zbtSetWheelRollInfluence(btRaycastVehicle* vehicle, int wheelId, float rollInfluence) {
	//checkWheel(vehicle, wheelId);
	vehicle->getWheelInfo(wheelId).m_rollInfluence = rollInfluence;
}

EXPORT void zbtSetWheelFrictionSlip(btRaycastVehicle* vehicle, int wheelId, float frictionSlip) {
	//checkWheel(vehicle, wheelId);
	vehicle->getWheelInfo(wheelId).m_frictionSlip = frictionSlip;
}

EXPORT void zbtSetWheelSuspRestLength(btRaycastVehicle* vehicle, int wheelId, float suspRestLength) {
	//checkWheel(vehicle, wheelId);
	vehicle->getWheelInfo(wheelId).m_suspensionRestLength1 = suspRestLength;
}

EXPORT void zbtSetWheelMaxSuspTravel(btRaycastVehicle* vehicle, int wheelId, float maxSuspTravel) {
	//checkWheel(vehicle, wheelId);
	vehicle->getWheelInfo(wheelId).m_maxSuspensionTravelCm = maxSuspTravel;
}

EXPORT void zbtSetWheelSuspStiffness(btRaycastVehicle* vehicle, int wheelId, float suspStiffness) {
	//checkWheel(vehicle, wheelId);
	vehicle->getWheelInfo(wheelId).m_suspensionStiffness = suspStiffness;
}

EXPORT void zbtSetWheelDampingCompression(btRaycastVehicle* vehicle, int wheelId, float dampingCompression) {
	//checkWheel(vehicle, wheelId);
	vehicle->getWheelInfo(wheelId).m_wheelsDampingCompression = dampingCompression;
}

EXPORT void zbtSetWheelDampingRelaxation(btRaycastVehicle* vehicle, int wheelId, float dampingRelaxation) {
	//checkWheel(vehicle, wheelId);
	vehicle->getWheelInfo(wheelId).m_wheelsDampingRelaxation = dampingRelaxation;
}

EXPORT void zbtSetWheelSteering(btRaycastVehicle* vehicle, int wheelId, float steering) {
	vehicle->setSteeringValue(steering, wheelId);
}

EXPORT void zbtSetWheelEngineForce(btRaycastVehicle* vehicle, int wheelId, float force) {
	vehicle->applyEngineForce(force, wheelId);
}

EXPORT void zbtSetWheelBrake(btRaycastVehicle* vehicle, int wheelId, float brake) {
	vehicle->setBrake(brake, wheelId);
}

/* Bullet ignores setting pitch control
EXPORT void zbtSetVehiclePitchControl(btRaycastVehicle* vehicle, float pitch) {
	vehicle->setPitchControl(pitch);
}*/

EXPORT void zbtResetVehicleSusp(btRaycastVehicle* vehicle) {
	vehicle->resetSuspension();
}

EXPORT float zbtGetVehicleCurrentSpeed(btRaycastVehicle* vehicle) {
	return vehicle->getCurrentSpeedKmHour();
}

EXPORT void zbtGetWheelPositionXYZ(btRaycastVehicle* vehicle, int wheelId,
	float &outX, float &outY, float &outZ) {

	btVector3 pos = vehicle->getWheelTransformWS(wheelId).getOrigin();
	outX = pos.getX();
	outY = pos.getY();
	outZ = pos.getZ();
}

EXPORT void zbtGetWheelPosition(btRaycastVehicle* vehicle, int wheelId, v3 &outPosition) {
	zbtGetWheelPositionXYZ(vehicle, wheelId, outPosition.x, outPosition.y, outPosition.z);
}

EXPORT void zbtGetWheelRotationXYZ(btRaycastVehicle* vehicle, int wheelId,
	float &outRx, float &outRy, float &outRz) {

	vehicle->getWheelTransformWS(wheelId).getBasis().getEulerZYX(outRz, outRy, outRx);
	outRx /= SIMD_2_PI;
	outRy /= SIMD_2_PI;
	outRz /= SIMD_2_PI;
}

EXPORT void zbtGetWheelRotation(btRaycastVehicle* vehicle, int wheelId, v3 &outRotation) {
	zbtGetWheelRotationXYZ(vehicle, wheelId, outRotation.x, outRotation.y, outRotation.z);
}

EXPORT void zbtGetWheelPosRotXYZ(btRaycastVehicle* vehicle, int wheelId,
	float &outX, float &outY, float &outZ, float &outRx, float &outRy, float &outRz) {

	zbtGetWheelPositionXYZ(vehicle, wheelId, outX, outY, outZ);
	zbtGetWheelRotationXYZ(vehicle, wheelId, outRx, outRy, outRz);
}

EXPORT void zbtGetWheelPosRot(btRaycastVehicle* vehicle, int wheelId,
	v3 &outPosition, v3 &outRotation) {

	zbtGetWheelPositionXYZ(vehicle, wheelId, outPosition.x, outPosition.y, outPosition.z);
	zbtGetWheelRotationXYZ(vehicle, wheelId, outRotation.x, outRotation.y, outRotation.z);
}

// must be called before zbtDestroyWorld explicitly
EXPORT void zbtDeleteRaycastVehicle(btRaycastVehicle* vehicle) {
	gCurrentWorld->world->removeVehicle(vehicle);
	delete vehicle;
}


// Ghost object

EXPORT btCollisionObject* zbtCreateGhostObject(btCollisionShape* shape,
	float x, float y, float z, float rx, float ry, float rz) {

	btGhostObject* ghostObject = new btPairCachingGhostObject();
	ghostObject->setCollisionShape(shape);
	ghostObject->setWorldTransform(transform(x, y, z, rx, ry, rz));
	if (gCurrentWorld->createGhostPairCallback){
		//gCurrentWorld->world->getPairCache()->setInternalGhostPairCallback(new btGhostPairCallback());
		gCurrentWorld->broadphase->getOverlappingPairCache()->setInternalGhostPairCallback(new btGhostPairCallback());
		gCurrentWorld->createGhostPairCallback = false;
	}
	gCurrentWorld->world->addCollisionObject(ghostObject);

	return ghostObject;
}

EXPORT void zbtDeleteGhostObject(btGhostObject* ghostObject) {
	gCurrentWorld->world->removeCollisionObject(ghostObject);
	delete ghostObject;
}

EXPORT int zbtGetNumOverlappingObjects(btGhostObject* ghostObject) {
	return ghostObject->getNumOverlappingObjects();
}

EXPORT btCollisionObject* zbtGetOverlappingObject(btGhostObject* ghostObject, int index) {
	return ghostObject->getOverlappingObject(index);
}


// Kinematic character controller

EXPORT btKinematicCharacterController* zbtCreateKinematicCharacterController(
	btPairCachingGhostObject* ghostObject, float stepHeight){

	ghostObject->setCollisionFlags(btCollisionObject::CF_CHARACTER_OBJECT);
	btKinematicCharacterController* controller = new btKinematicCharacterController(ghostObject,
		static_cast<btConvexShape*>(ghostObject->getCollisionShape()), stepHeight);
	gCurrentWorld->world->addAction(controller);

	return controller;
}

EXPORT void zbtDeleteKinematicCharacterController(btKinematicCharacterController* controller) {
	gCurrentWorld->world->removeAction(controller);
	delete controller;
}

EXPORT void zbtSetCharacterUp(btKinematicCharacterController* controller,
	float x, float y, float z) {
	
	controller->setUp(btVector3(x, y, z));
}

EXPORT void zbtSetCharacterWalkDirection(btKinematicCharacterController* controller,
	float x, float y, float z) {

	controller->setWalkDirection(btVector3(x, y, z));
}

EXPORT void zbtSetCharacterVelocityForTimeInterval(btKinematicCharacterController* controller,
	float x, float y, float z, float timeInterval) {

	controller->setVelocityForTimeInterval(btVector3(x, y, z), timeInterval);
}

EXPORT void zbtCharacterWarp(btKinematicCharacterController* controller,
	float x, float y, float z) {

	controller->warp(btVector3(x, y, z));
}

EXPORT void zbtSetCharacterFallSpeed(btKinematicCharacterController* controller,
	float fallSpeed) {

	controller->setFallSpeed(fallSpeed);
}

EXPORT void zbtSetCharacterJumpSpeed(btKinematicCharacterController* controller,
	float jumpSpeed) {

	controller->setJumpSpeed(jumpSpeed);
}

EXPORT void zbtSetCharacterMaxJumpHeight(btKinematicCharacterController* controller,
	float maxJumpHeight) {

	controller->setMaxJumpHeight(maxJumpHeight);
}

EXPORT int zbtCharacterCanJump(btKinematicCharacterController* controller) {

	return controller->canJump();
}

EXPORT void zbtCharacterJump(btKinematicCharacterController* controller) {

	controller->jump();
}

EXPORT void zbtSetCharacterGravity(btKinematicCharacterController* controller,
	float x, float y, float z) {

	controller->setGravity(btVector3(x, y, z));
}

EXPORT void zbtSetCharacterMaxSlope(btKinematicCharacterController* controller,
	float slope) {

	controller->setMaxSlope(slope * SIMD_2_PI);
}

EXPORT void zbtSetCharacterUseGhostSweepTest(btKinematicCharacterController* controller,
	bool bUseGhostObjectSweepTest) {

	controller->setUseGhostSweepTest(bUseGhostObjectSweepTest);
}

EXPORT int zbtCharacterOnGround(btKinematicCharacterController* controller) {

	return controller->onGround();
}

EXPORT void zbtCharacterReset(btKinematicCharacterController* controller) {

	controller->reset(gCurrentWorld->world);
}

EXPORT void zbtSetCharacterUpInterpolate(btKinematicCharacterController* controller,
	bool bInterpolate) {

	controller->setUpInterpolate(bInterpolate);
}


// Collision objects (in general)

EXPORT void zbtSetFriction(btCollisionObject* obj, float friction) {
	obj->setFriction(friction);
}

EXPORT void zbtSetRollingFriction(btCollisionObject* obj, float friction) {
	obj->setRollingFriction(friction);
}

EXPORT void zbtSetRestitution(btCollisionObject* obj, float restitution) {
	obj->setRestitution(restitution);
}

EXPORT void zbtGetPositionXYZ(btCollisionObject* obj,
	float &outX, float &outY, float &outZ) {

	btVector3 pos = obj->getWorldTransform().getOrigin();
	outX = pos.getX();
	outY = pos.getY();
	outZ = pos.getZ();
}

EXPORT void zbtGetPosition(btCollisionObject* obj, v3 &outPosition) {
	zbtGetPositionXYZ(obj, outPosition.x, outPosition.y, outPosition.z);
}

EXPORT void zbtSetPositionXYZ(btCollisionObject* obj, float x, float y, float z) {
	obj->getWorldTransform().setOrigin(btVector3(x, y, z));
}

EXPORT void zbtSetPosition(btCollisionObject* obj, v3 &position) {
	zbtSetPositionXYZ(obj, position.x, position.y, position.z);
}

EXPORT void zbtGetRotationXYZ(btCollisionObject* obj,
	float &outRx, float &outRy, float &outRz) {

	obj->getWorldTransform().getBasis().getEulerZYX(outRz, outRy, outRx);
	outRx /= SIMD_2_PI;
	outRy /= SIMD_2_PI;
	outRz /= SIMD_2_PI;
}

EXPORT void zbtGetRotation(btCollisionObject* obj, v3 &outRotation) {
	zbtGetRotationXYZ(obj, outRotation.x, outRotation.y, outRotation.z);
}

EXPORT void zbtSetRotationXYZ(btCollisionObject* obj, float rx, float ry, float rz) {
	obj->getWorldTransform().setRotation(rot(rx, ry, rz));
}

EXPORT void zbtSetRotation(btCollisionObject* obj, v3 &rotation) {
	zbtSetRotationXYZ(obj, rotation.x, rotation.y, rotation.z);
}

EXPORT void zbtGetRotationQuat(btCollisionObject* obj, v4 &outOrientation) {
	outOrientation.set(obj->getWorldTransform().getRotation());
}

EXPORT void zbtGetRotationDirection(btCollisionObject* obj, v3 &outDirection) {
	outDirection.set(obj->getWorldTransform().getBasis() * FORWARD);
}

EXPORT void zbtSetRotationDirectionXYZ(btCollisionObject* obj, float x, float y, float z) {
	btVector3 v = btVector3(x, y, z);

	// skip if almost zero - would throw exception
	if (v.fuzzyZero()) return;

	v = v.normalize();

	if (v == BACK)
		obj->getWorldTransform().setRotation(btQuaternion(LEFT, SIMD_PI));
	else if (v != FORWARD)
		obj->getWorldTransform().setRotation(btQuaternion(FORWARD.cross(v), btAcos(FORWARD.dot(v))));
}

EXPORT void zbtSetRotationDirection(btCollisionObject* obj, v3 &direction) {
	zbtSetRotationDirectionXYZ(obj, direction.x, direction.y, direction.z);
}

EXPORT void zbtGetPosRotXYZ(btCollisionObject* obj,
	float &outX, float &outY, float &outZ,
	float &outRx, float &outRy, float &outRz) {

	zbtGetPositionXYZ(obj, outX, outY, outZ);
	zbtGetRotationXYZ(obj, outRx, outRy, outRz);
}

EXPORT void zbtGetPosRot(btCollisionObject* obj, v3 &outPosition, v3 &outRotation) {

	zbtGetPositionXYZ(obj, outPosition.x, outPosition.y, outPosition.z);
	zbtGetRotationXYZ(obj, outRotation.x, outRotation.y, outRotation.z);
}

EXPORT void zbtSetPosRotXYZ(btCollisionObject* obj,
	float x, float y, float z, float rx, float ry, float rz) {

	obj->setWorldTransform(transform(x, y, z, rx, ry, rz));
}

EXPORT void zbtSetPosRot(btCollisionObject* obj,
	v3 &position, v3 &rotation) {

	zbtSetPosRotXYZ(obj, position.x, position.y, position.z,
		rotation.x, rotation.y, rotation.z);
}

EXPORT void zbtGetModelMatrix(btCollisionObject* obj, float* matrix) {
	obj->getWorldTransform().getOpenGLMatrix(matrix);
	matrix[15] = 1.0;
}

EXPORT void zbtGetModelMatrixInv(btCollisionObject* obj, float *matrix) {
	obj->getWorldTransform().inverse().getOpenGLMatrix(matrix);
	matrix[15] = 1.0;
}

EXPORT void zbtSetCollisionFlags(btCollisionObject* obj, int flags) {
	obj->setCollisionFlags(flags);
}

EXPORT int zbtIsActive(btCollisionObject* obj) {
	return obj->isActive();
}

EXPORT void zbtActivate(btCollisionObject* obj, bool bForceActivation) {
	obj->activate(bForceActivation);
}

EXPORT void zbtSetActivationState(btCollisionObject* obj, int newState) {
	obj->setActivationState(newState);
}

EXPORT void zbtForceActivationState(btCollisionObject* obj, int newState) {
	obj->forceActivationState(newState);
}

EXPORT void zbtSetDeactivationTime(btCollisionObject* obj, float time) {
	obj->setDeactivationTime(time);
}

EXPORT void zbtSetUserIndex(btCollisionObject* obj, int index) {
	obj->setUserIndex(index);
}

EXPORT int zbtGetUserIndex(btCollisionObject* obj) {
	return obj->getUserIndex();
}

EXPORT void zbtSetUserModel(btCollisionObject* obj, void* userModel) {
	obj->setUserPointer(userModel);
}

EXPORT void* zbtGetUserModel(btCollisionObject* obj) {
	return obj->getUserPointer();
}


// Collision detection

EXPORT void zbtSetIgnoreCollisionCheck(btCollisionObject* objA, btCollisionObject* objB,
	bool bIgnoreCollisionCheck) {

	objA->setIgnoreCollisionCheck(objB, bIgnoreCollisionCheck);
}

EXPORT void zbtSetCollisionFilterGroupAndMask(btCollisionObject* obj, int group, int mask) {
	obj->getBroadphaseHandle()->m_collisionFilterGroup = group;
	obj->getBroadphaseHandle()->m_collisionFilterMask = mask;
}

EXPORT int zbtStartCollisionDetection() {
	gCurrentWorld->manifoldIndex = gCurrentWorld->dispatcher->getNumManifolds();
	gCurrentWorld->manifoldPointIndex = -1;

	return gCurrentWorld->manifoldIndex;
}

EXPORT int zbtGetNextContact(btCollisionObject* &outObjA, btCollisionObject* &outObjB,
	v3 &outPosA, v3 &outPosB, v3 &outNormal) {

	if (gCurrentWorld->manifoldPointIndex < 0){

		if (gCurrentWorld->manifoldIndex == 0) return false;
		gCurrentWorld->manifoldIndex--;
		gCurrentWorld->manifold = gCurrentWorld->dispatcher->getManifoldByIndexInternal(gCurrentWorld->manifoldIndex);

		gCurrentWorld->manifoldPointIndex = gCurrentWorld->manifold->getNumContacts() - 1;
	}

	outObjA = const_cast<btCollisionObject*>(gCurrentWorld->manifold->getBody0());
	outObjB = const_cast<btCollisionObject*>(gCurrentWorld->manifold->getBody1());

	btManifoldPoint& pt = gCurrentWorld->manifold->getContactPoint(gCurrentWorld->manifoldPointIndex);

	outPosA.set(pt.getPositionWorldOnA());
	outPosB.set(pt.getPositionWorldOnB());
	outNormal.set(pt.m_normalWorldOnB);

	gCurrentWorld->manifoldPointIndex--;

	return true;
}

EXPORT void zbtGetCollidedObjects(int contactIndex,
	btCollisionObject* &outObjA, btCollisionObject* &outObjB, float &outAppliedImpulse) {

	btPersistentManifold* pm = gCurrentWorld->dispatcher->getManifoldByIndexInternal(contactIndex);

	outObjA = const_cast<btCollisionObject*>(pm->getBody0());
	outObjB = const_cast<btCollisionObject*>(pm->getBody1());
	outAppliedImpulse = sumAppliedImpulses(pm);
}

EXPORT int zbtIsColliding(btCollisionObject* obj) {

	ITERATE_MANIFOLDS_FOR_OBJECT(obj, return true)
	return false;
}

EXPORT int zbtGetNumberOfCollisions(btCollisionObject* obj) {

	int ret = 0;
	ITERATE_MANIFOLDS_FOR_OBJECT(obj, ret++)

	return ret;
}

EXPORT int zbtIsCollidedWith(btCollisionObject* objA, btCollisionObject* objB) {

	for (int i = gCurrentWorld->dispatcher->getNumManifolds() - 1; i >= 0; --i) {
		btPersistentManifold* pm = gCurrentWorld->dispatcher->getManifoldByIndexInternal(i);
		btCollisionObject* coA = const_cast<btCollisionObject*>(pm->getBody0());
		btCollisionObject* coB = const_cast<btCollisionObject*>(pm->getBody1());

		if ((coA == objA && coB == objB) || (coA == objB && coB == objA)) {
			return pm->getNumContacts();

			/* tests shown that this is not necessary
			int j = pm->getNumContacts() - 1;
			for (; j >= 0; --j) 
				if (pm->validContactDistance(pm->getContactPoint(j))) break;
			if (j >= 0) return true;
			return false;*/
		}
	}

	return 0;
}

EXPORT float zbtGetCollisionImpulse(btCollisionObject* obj) {

	float imp = 0;
	ITERATE_MANIFOLDS_FOR_OBJECT(obj, imp += sumAppliedImpulses(pm))

	return imp;
}


// Raycasting

EXPORT btCollisionObject* zbtRayTestFiltered(float fromX, float fromY, float fromZ, float toX, float toY, float toZ,
	int filterGroup, int filterMask) {

	btCollisionWorld::ClosestRayResultCallback crrc(btVector3(fromX, fromY, fromZ), btVector3(toX, toY, toZ));
	crrc.m_collisionFilterGroup = filterGroup;
	crrc.m_collisionFilterMask = filterMask;
	gCurrentWorld->world->rayTest(btVector3(fromX, fromY, fromZ), btVector3(toX, toY, toZ), crrc);
	if (crrc.hasHit()) {
		gCurrentWorld->rayTestHitPoint = crrc.m_hitPointWorld;
		gCurrentWorld->rayTestHitNormal = crrc.m_hitNormalWorld;

		return const_cast<btCollisionObject*>(crrc.m_collisionObject);
	}
	else
		return NULL;
}

EXPORT btCollisionObject* zbtRayTest(float fromX, float fromY, float fromZ, float toX, float toY, float toZ) {

	return zbtRayTestFiltered(fromX, fromY, fromZ, toX, toY, toZ,
				btBroadphaseProxy::DefaultFilter, btBroadphaseProxy::AllFilter);
}

EXPORT void zbtGetRayTestHitPointXYZ(float &outX, float &outY, float &outZ) {
	outX = gCurrentWorld->rayTestHitPoint.getX();
	outY = gCurrentWorld->rayTestHitPoint.getY();
	outZ = gCurrentWorld->rayTestHitPoint.getZ();
}

EXPORT void zbtGetRayTestHitPoint(v3 &outPosition) {
	outPosition.set(gCurrentWorld->rayTestHitPoint);
}

EXPORT void zbtGetRayTestHitNormalXYZ(float &outX, float &outY, float &outZ) {
	outX = gCurrentWorld->rayTestHitNormal.getX();
	outY = gCurrentWorld->rayTestHitNormal.getY();
	outZ = gCurrentWorld->rayTestHitNormal.getZ();
}

EXPORT void zbtGetRayTestHitNormal(v3 &outNormal) {
	outNormal.set(gCurrentWorld->rayTestHitNormal);
}