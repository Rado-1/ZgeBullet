 /*
ZGEBullet Library
Copyright (c) 2012 Radovan Cervenka

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

// Definitions

#ifdef _WIN32
#define export extern "C" __declspec(dllexport)
#else
#define export extern "C"
#endif

#define BT_EULER_DEFAULT_ZYX

#define DONE 0
#define ERROR -1
#define TRUE 1
#define FALSE 0

// Includes

#include "btBulletDynamicsCommon.h"
#include "BulletCollision/CollisionShapes/btHeightfieldTerrainShape.h"
#include "BulletCollision/Gimpact/btGImpactShape.h"
//#include "BulletCollision/CollisionDispatch/btGhostObject.h"
#include <vector>
#include <typeinfo>

// Macros for list operations

#define INT_TO_BOOL(int_value) int_value!=0

#define GET_ITEM_FROM_LIST(list, type, var, id) \
	if(id<0 || id>=list.size() || !list[id]) return ERROR; \
	type var =(type)list[id]; \
	if(!var) return ERROR;

#define GET_ITEM_FROM_MIXED_LIST(list, type, var, id) \
	if(id<0 || id>=list.size() || !list[id]) return ERROR; \
	type var; \
	try { var=(type)list[id];} catch (...) {return ERROR;} \
	if(!var) return ERROR;

#define ADD_TO_LIST(list, item) \
	int _i = list.findLinearSearch(NULL); \
	if(_i == list.size()) { \
		list.push_back(item); \
		return list.size()-1; \
	} else { \
		list[_i] = item; \
		return _i; \
	}

#define CHECK_INDEX_IN_VECTOR(vector, index) \
	if(index<0 || (unsigned)index>=vector.size()) return ERROR;

#define IS_INITIALIZED if(!gIsInitialized) return ERROR;

// Globals

// Variables
bool gIsInitialized = false;
btBroadphaseInterface* gBroadphase;
btDefaultCollisionConfiguration* gCollisionConfiguration;
btCollisionDispatcher* gDispatcher;
btSequentialImpulseConstraintSolver* gSolver;
btDiscreteDynamicsWorld* gWorld;
btTriangleMesh* gTmpTriangleMesh = NULL;
btVector3 gRayTestHitPoint;
btVector3 gRayTestHitNormal;
//btVehicleRaycaster* gVehicleRaycaster = NULL;

// Global lists
btAlignedObjectArray<btCollisionShape*> gCollisionShapeList;
btAlignedObjectArray<btCollisionObject*> gCollisionObjectList;
btAlignedObjectArray<btTypedConstraint*> gConstraintList;

// Declarations of functions

export int zbtDeleteRigidBody(int id);
//export int zbtDeleteGhostObject(int ghostObjectId);
export int zbtDeleteConstraint(int id);

// Utilities

float getRotationXFromMatrix(const btMatrix3x3* matrix)
{
	if(matrix->getRow(2).x()< +0.999)
		if(matrix->getRow(2).x() > -0.999)
			return -atan2(-matrix->getRow(2).y(),matrix->getRow(2).z())/SIMD_2_PI;
		else
			return atan2(matrix->getRow(0).y(),matrix->getRow(1).y())/SIMD_2_PI;
	else
		return -atan2(matrix->getRow(0).y(),matrix->getRow(1).y())/SIMD_2_PI;
}

float getRotationYFromMatrix(const btMatrix3x3* matrix)
{
	if(matrix->getRow(2).x()< +0.999)
		if(matrix->getRow(2).x() > -0.999)
			return -asin(matrix->getRow(2).x())/SIMD_2_PI;
		else
			return 0.25;
	else
		return -0.25;
}

float getRotationZFromMatrix(const btMatrix3x3* matrix)
{
	if(matrix->getRow(2).x()< 0.999 && matrix->getRow(2).x() > -0.999)
		return -atan2(-matrix->getRow(1).x(), matrix->getRow(0).x())/SIMD_2_PI;
	return 0;
}

// World

export int zbtCreateWorld()
{
	if(gIsInitialized)
		return ERROR;

	gBroadphase = new btDbvtBroadphase();
	gCollisionConfiguration = new btDefaultCollisionConfiguration();
	gDispatcher = new btCollisionDispatcher(gCollisionConfiguration);
	gSolver = new btSequentialImpulseConstraintSolver();
	gWorld = new btDiscreteDynamicsWorld(gDispatcher, gBroadphase, gSolver, gCollisionConfiguration);

	gRayTestHitPoint = btVector3(0, 0, 0);
	gRayTestHitNormal = btVector3(0, 0, 0);

	gIsInitialized = true;
	return DONE;
}

export int zbtDestroyWorld()
{
	IS_INITIALIZED

	int i;

	// delete rigid bodies
	for(i = gCollisionObjectList.size()-1; i >= 0; i--)
		if(typeid(gCollisionObjectList[i]) == typeid(btRigidBody))
			zbtDeleteRigidBody(i);
		//else
		//	zbtDeleteGhostObject(i);

	gCollisionObjectList.clear();

	// clear list of constraints
	gConstraintList.clear();

	// destroy vehicle raycaster, if any
	//if(gVehicleRaycaster) delete gVehicleRaycaster;

	delete gWorld;
    delete gSolver;
    delete gDispatcher;
    delete gCollisionConfiguration;
    delete gBroadphase;

	gIsInitialized = false;
	return DONE;
}

export int zbtSetWorldGravity(float x, float y, float z)
{
	IS_INITIALIZED
	gWorld->setGravity(btVector3(x, y, z));
	return DONE;
}

export int zbtStepSimulation(float timeStep, int maxSubSteps, float fixedTimeStep)
{
	IS_INITIALIZED
	gWorld->stepSimulation(timeStep, maxSubSteps, fixedTimeStep);
	return DONE;
}

// Collision shapes

export int zbtCreateStaticPlaneShape(float normalX, float normalY, float normalZ, float planeConstant)
{
	ADD_TO_LIST(gCollisionShapeList, new btStaticPlaneShape(btVector3(normalX, normalY, normalZ), planeConstant))
}

export int zbtCreateBoxShape(float x, float y, float z)
{
	ADD_TO_LIST(gCollisionShapeList, new btBoxShape(btVector3(x, y, z)))
}

export int zbtCreateSphereShape(float radius)
{
	ADD_TO_LIST(gCollisionShapeList, new btSphereShape(radius))
}

export int zbtCreateScalableSphereShape(float radius)
{
#ifdef _WIN32
	ADD_TO_LIST(gCollisionShapeList, new btMultiSphereShape(&btVector3(0,0,0), &radius, 1))
#else
	btVector3 positions[1] = {btVector3(0,0,0)};
	btScalar radii[1] = {radius};

	ADD_TO_LIST(gCollisionShapeList, new btMultiSphereShape(positions, radii, 1))
#endif
}

export int zbtCreateConeShape(float radius, float height)
{
	ADD_TO_LIST(gCollisionShapeList, new btConeShape(radius, height))
}

export int zbtCreateCylinderShape(float radius, float height)
{
	ADD_TO_LIST(gCollisionShapeList, new btCylinderShape(btVector3(radius, height, radius)))
}

export int zbtCreateCapsuleShape(float radius, float height)
{
	ADD_TO_LIST(gCollisionShapeList, new btCapsuleShape(radius, height))
}

export int zbtCreateCompoundShape()
{
	ADD_TO_LIST(gCollisionShapeList, new btCompoundShape())
}

export int zbtAddChildShape(int compoundId, int childId, float x, float y, float z, float rx, float ry, float rz)
{
	GET_ITEM_FROM_LIST(gCollisionShapeList, btCollisionShape*, compShape, compoundId)
	GET_ITEM_FROM_LIST(gCollisionShapeList, btCollisionShape*, childShape, childId)

	try
	{
		((btCompoundShape*)compShape)->addChildShape(
			btTransform(btQuaternion(rz*SIMD_2_PI, ry*SIMD_2_PI, rx*SIMD_2_PI), btVector3(x, y, z)), childShape);
	} catch (...) {return ERROR;}

	return DONE;
}

export int zbtRemoveChildShape(int compoundId, int childId)
{
	GET_ITEM_FROM_LIST(gCollisionShapeList, btCollisionShape*, compShape, compoundId)
	GET_ITEM_FROM_LIST(gCollisionShapeList, btCollisionShape*, childShape, childId)

	try
	{
		((btCompoundShape*)compShape)->removeChildShape(childShape);
	} catch (...) {return ERROR;}

	return DONE;
}

export int zbtCreateHeightfieldTerrainShape(void *heightfieldData, int dataType, int width, int length,
	float heightScale, float minHeight, float maxHeight, int upAxis, int bFlipQuadEdges, int bDiamondSubdivision)
{
	try
	{
		btHeightfieldTerrainShape* hf = new btHeightfieldTerrainShape(width, length, heightfieldData,
			heightScale, minHeight, maxHeight, upAxis, (dataType == 0 ? PHY_FLOAT : PHY_INTEGER), INT_TO_BOOL(bFlipQuadEdges));
		hf->setUseDiamondSubdivision(INT_TO_BOOL(bDiamondSubdivision));
		ADD_TO_LIST(gCollisionShapeList, hf)
	} catch (...) {return ERROR;}
}

export int zbtCreateConvexHullShape(float *points, int numPoints)
{
	try
	{
		ADD_TO_LIST(gCollisionShapeList, new btConvexHullShape(points, numPoints, 3 * sizeof(float)))
	} catch (...) {return ERROR;}

}

export int zbtCreateMultiSphereShape(float *positions, float *radiuses, int num)
{
	try
	{
		ADD_TO_LIST(gCollisionShapeList, new btMultiSphereShape((btVector3*)positions, radiuses, num)) 
	} catch (...) {return ERROR;}

}

export int zbtStartTriangleMeshShape()
{
    if(gTmpTriangleMesh) return ERROR;
    gTmpTriangleMesh = new btTriangleMesh(false,false);
    return DONE;
}

export int zbtAddTriangle(float x1,float y1,float z1,
	float x2,float y2,float z2,float x3,float y3,float z3)
{
    if(!gTmpTriangleMesh) return ERROR;
    gTmpTriangleMesh->addTriangle(btVector3(x1,y1,z1), btVector3(x2,y2,z2), btVector3(x3,y3,z3));
    return DONE;
}

export int zbtFinishTriangleMeshShape(int meshType)
{
    if(!gTmpTriangleMesh) return ERROR;
	btCollisionShape* tm;

	switch(meshType){
	case 1: // convex hull
		tm = new btConvexTriangleMeshShape(gTmpTriangleMesh);
		break;
	case 2: // concave static-triangle mesh shape
		tm = new btBvhTriangleMeshShape(gTmpTriangleMesh, true, true);
		break;
	case 3: // concave deformable mesh
		tm = new btGImpactMeshShape(gTmpTriangleMesh);
		((btGImpactMeshShape*)tm)->updateBound();
	}

	gTmpTriangleMesh=NULL;
    ADD_TO_LIST(gCollisionShapeList, tm)
}

export int zbtSetShapeLocalScaling(int shapeId, float x, float y, float z)
{
	GET_ITEM_FROM_LIST(gCollisionShapeList, btCollisionShape*, cs, shapeId)
		cs->setLocalScaling(btVector3(x, y, z));
	return DONE;
}

export int zbtSetShapeMargin(int shapeId, float margin)
{
	GET_ITEM_FROM_LIST(gCollisionShapeList, btCollisionShape*, cs, shapeId)
	cs->setMargin(margin);
	return DONE;
}

export int zbtDeleteShape(int shapeId)
{
	GET_ITEM_FROM_LIST(gCollisionShapeList, btCollisionShape*, cs, shapeId)
	gCollisionShapeList[shapeId] = NULL;
	delete cs;

	return DONE;
}

export int zbtDeleteAllShapes()
{
	for(int i = gCollisionShapeList.size()-1; i >= 0; i--)
		zbtDeleteShape(i);
	gCollisionShapeList.clear();

	return DONE;
}

// Rigid bodies

export int zbtAddRigidBody(float mass, int shapeId, float x, float y, float z, float rx, float ry, float rz)
{
	IS_INITIALIZED

	btDefaultMotionState* motionState = new btDefaultMotionState(btTransform(btQuaternion(rz*SIMD_2_PI, ry*SIMD_2_PI, rx*SIMD_2_PI), btVector3(x, y, z)));
    btVector3 inertia(0, 0, 0);
	GET_ITEM_FROM_LIST(gCollisionShapeList, btCollisionShape*, shape, shapeId)
	if(mass < 0.f) mass = NULL; // mass < 0 - kinematic rigid body
	else if(mass > 0.f) // mass == 0 - static rigid body
	    shape->calculateLocalInertia(mass, inertia);
    btRigidBody::btRigidBodyConstructionInfo rigidBodyCI(mass, motionState, shape, inertia);
    btRigidBody* rigidBody = new btRigidBody(rigidBodyCI);
    gWorld->addRigidBody(rigidBody);

	ADD_TO_LIST(gCollisionObjectList, rigidBody)
}

export int zbtDeleteRigidBody(int rigidBodyId)
{
	GET_ITEM_FROM_MIXED_LIST(gCollisionObjectList, btRigidBody*, rb, rigidBodyId)

	// remove related constraints
	while (rb->getNumConstraintRefs())
	{
		btTypedConstraint* tc = rb->getConstraintRef(0);

		int i = gConstraintList.findLinearSearch(tc);
		if(i < gConstraintList.size()) gConstraintList[i] = NULL;

		gWorld->removeConstraint(tc);
		delete tc;
	}

	gWorld->removeRigidBody(rb);
	gCollisionObjectList[rigidBodyId] = NULL;
	delete rb->getMotionState();
	delete rb;

	return DONE;
}

export int zbtSetMass(int rigidBodyId, float mass)
{
	GET_ITEM_FROM_MIXED_LIST(gCollisionObjectList, btRigidBody*, rb, rigidBodyId)
	btVector3 inertia;
	rb->getCollisionShape()->calculateLocalInertia(mass, inertia);
	rb->setMassProps(mass, inertia);
	return DONE;
}

export int zbtSetDamping(int rigidBodyId, float linDamping, float angDamping)
{
	GET_ITEM_FROM_MIXED_LIST(gCollisionObjectList, btRigidBody*, rb, rigidBodyId)
	rb->setDamping(linDamping, angDamping*SIMD_2_PI);
	return DONE;
}

export int zbtSetLinearFactor(int rigidBodyId, float x, float y, float z)
{
	GET_ITEM_FROM_MIXED_LIST(gCollisionObjectList, btRigidBody*, rb, rigidBodyId)
	rb->setLinearFactor(btVector3(x, y, z));
	return DONE;
}

export int zbtSetAngularFactor(int rigidBodyId, float x, float y, float z)
{
	GET_ITEM_FROM_MIXED_LIST(gCollisionObjectList, btRigidBody*, rb, rigidBodyId)
	rb->setAngularFactor(btVector3(x, y, z));
	return DONE;
}

export int zbtSetGravity(int rigidBodyId, float x, float y, float z)
{
	GET_ITEM_FROM_MIXED_LIST(gCollisionObjectList, btRigidBody*, rb, rigidBodyId)
	rb->setGravity(btVector3(x, y, z));
	return DONE;
}

export int zbtGetLinearVelocity(int rigidBodyId, float &x, float &y, float &z)
{
	GET_ITEM_FROM_MIXED_LIST(gCollisionObjectList, btRigidBody*, rb, rigidBodyId)
	x = rb->getLinearVelocity().getX();
	y = rb->getLinearVelocity().getY();
	z = rb->getLinearVelocity().getZ();
	return DONE;
}

export int zbtSetLinearVelocity(int rigidBodyId, float x, float y, float z)
{
	GET_ITEM_FROM_MIXED_LIST(gCollisionObjectList, btRigidBody*, rb, rigidBodyId)
	rb->setLinearVelocity(btVector3(x, y, z));
	rb->activate(true);
	return DONE;
}

export int zbtGetAngularVelocity(int rigidBodyId, float &x, float &y, float &z)
{
	GET_ITEM_FROM_MIXED_LIST(gCollisionObjectList, btRigidBody*, rb, rigidBodyId)
	x = rb->getAngularVelocity().getX() / SIMD_2_PI;
	y = rb->getAngularVelocity().getY() / SIMD_2_PI;
	z = rb->getAngularVelocity().getZ() / SIMD_2_PI;
	return DONE;
}

export int zbtSetAngularVelocity(int rigidBodyId, float x, float y, float z)
{
	GET_ITEM_FROM_MIXED_LIST(gCollisionObjectList, btRigidBody*, rb, rigidBodyId)
	rb->setAngularVelocity(btVector3(x*SIMD_2_PI, y*SIMD_2_PI, z*SIMD_2_PI));
	rb->activate(true);
	return DONE;
}

export int zbtApplyCentralImpulse(int rigidBodyId, float x, float y, float z)
{
	GET_ITEM_FROM_MIXED_LIST(gCollisionObjectList, btRigidBody*, rb, rigidBodyId)
	rb->applyCentralImpulse(btVector3(x, y, z));
	rb->activate(true);
	return DONE;
}

export int zbtApplyTorqueImpulse(int rigidBodyId, float rx, float ry, float rz)
{
	GET_ITEM_FROM_MIXED_LIST(gCollisionObjectList, btRigidBody*, rb, rigidBodyId)
	rb->applyTorqueImpulse(btVector3(rx*SIMD_2_PI, ry*SIMD_2_PI, rz*SIMD_2_PI));
	rb->activate(true);
	return DONE;
}

export int zbtApplyImpulse(int rigidBodyId, float x, float y, float z, float relX, float relY, float relZ)
{
	GET_ITEM_FROM_MIXED_LIST(gCollisionObjectList, btRigidBody*, rb, rigidBodyId)
	rb->applyImpulse(btVector3(x, y, z), btVector3(relX, relY, relZ));
	rb->activate(true);
	return DONE;
}

// Constraints and limits

export int zbtAreConnected(int rigidBodyAId, int rigidBodyBId)
{
	GET_ITEM_FROM_LIST(gCollisionObjectList, btRigidBody*, rb1, rigidBodyAId)
	GET_ITEM_FROM_LIST(gCollisionObjectList, btRigidBody*, rb2, rigidBodyBId)

	return int(!rb1->checkCollideWithOverride(rb2));
}

export int zbtAddPoint2PointConstraint(int rigidBodyAId, int rigidBodyBId,
	float pivotAx, float pivotAy, float pivotAz,
	float pivotBx, float pivotBy, float pivotBz, int bDisableCollision)
{
	GET_ITEM_FROM_LIST(gCollisionObjectList, btRigidBody*, rbA, rigidBodyAId)
	GET_ITEM_FROM_LIST(gCollisionObjectList, btRigidBody*, rbB, rigidBodyBId)

	btTypedConstraint* tc = new btPoint2PointConstraint(*rbA, *rbB,
		btVector3(pivotAx, pivotAy, pivotAz), btVector3(pivotBx, pivotBy, pivotBz));
	gWorld->addConstraint(tc, INT_TO_BOOL(bDisableCollision));

	ADD_TO_LIST(gConstraintList, tc)
}

export int zbtAddPoint2PointConstraint1(int rigidBodyId, float pivotX, float pivotY, float pivotZ,
	int bDisableCollision)
{
	GET_ITEM_FROM_LIST(gCollisionObjectList, btRigidBody*, rb, rigidBodyId)

	btTypedConstraint* tc = new btPoint2PointConstraint(*rb, btVector3(pivotX, pivotY, pivotZ));
	gWorld->addConstraint(tc, INT_TO_BOOL(bDisableCollision));

	ADD_TO_LIST(gConstraintList, tc)
}

export int zbtAddHingeConstraint(int rigidBodyAId, int rigidBodyBId,
	float pivotAx, float pivotAy, float pivotAz,
	float pivotBx, float pivotBy, float pivotBz,
	float axisAx, float axisAy, float axisAz,
	float axisBx, float axisBy, float axisBz, int bDisableCollision)
{
	GET_ITEM_FROM_LIST(gCollisionObjectList, btRigidBody*, rbA, rigidBodyAId)
	GET_ITEM_FROM_LIST(gCollisionObjectList, btRigidBody*, rbB, rigidBodyBId)

	btTypedConstraint* tc = new btHingeConstraint(*rbA, *rbB,
		btVector3(pivotAx, pivotAy, pivotAz), btVector3(pivotBx, pivotBy, pivotBz),
		btVector3(axisAx, axisAy, axisAz), btVector3(axisBx, axisBy, axisBz));
	gWorld->addConstraint(tc, INT_TO_BOOL(bDisableCollision));

	ADD_TO_LIST(gConstraintList, tc)
}

export int zbtAddHingeConstraint1(int rigidBodyId,
	float pivotX, float pivotY, float pivotZ,
	float axisX, float axisY, float axisZ, int bDisableCollision)
{
	GET_ITEM_FROM_LIST(gCollisionObjectList, btRigidBody*, rb, rigidBodyId)

	btTypedConstraint* tc = new btHingeConstraint(*rb,
		btVector3(pivotX, pivotY, pivotZ), btVector3(axisX, axisY, axisZ));
	gWorld->addConstraint(tc, INT_TO_BOOL(bDisableCollision));

	ADD_TO_LIST(gConstraintList, tc)
}

export int zbtSetHingeLimits(int constraintId, float low, float high,
	float softness, float biasFactor, float relaxationFactor)
{
	GET_ITEM_FROM_MIXED_LIST(gConstraintList, btHingeConstraint*, hc, constraintId)

	hc->setLimit(low*SIMD_2_PI , high*SIMD_2_PI, softness, biasFactor, relaxationFactor);

	return DONE;
}

export int zbtEnableHingeAngularMotor(int constraintId, int bEnableMotor, float targetVelocity, float maxMotorImpulse)
{
	GET_ITEM_FROM_MIXED_LIST(gConstraintList, btHingeConstraint*, hc, constraintId)

	hc->enableAngularMotor(INT_TO_BOOL(bEnableMotor), targetVelocity, maxMotorImpulse);

	return DONE;
}

export int zbtAddConeTwistConstraint(int rigidBodyAId, int rigidBodyBId,
	float pivotAx, float pivotAy, float pivotAz,
	float pivotBx, float pivotBy, float pivotBz,
	float rotAx, float rotAy, float rotAz,
	float rotBx, float rotBy, float rotBz, int bDisableCollision)
{
	GET_ITEM_FROM_LIST(gCollisionObjectList, btRigidBody*, rbA, rigidBodyAId)
	GET_ITEM_FROM_LIST(gCollisionObjectList, btRigidBody*, rbB, rigidBodyBId)

	btTypedConstraint* tc = new btConeTwistConstraint(*rbA, *rbB,
		btTransform(btQuaternion(rotAz*SIMD_2_PI, rotAy*SIMD_2_PI, rotAx*SIMD_2_PI), btVector3(pivotAx, pivotAy, pivotAz)),
		btTransform(btQuaternion(rotBz*SIMD_2_PI, rotBy*SIMD_2_PI, rotBx*SIMD_2_PI), btVector3(pivotBx, pivotBy, pivotBz)));
	gWorld->addConstraint(tc, INT_TO_BOOL(bDisableCollision));

	ADD_TO_LIST(gConstraintList, tc)
}

export int zbtAddConeTwistConstraint1(int rigidBodyId,
	float pivotX, float pivotY, float pivotZ,
	float rotX, float rotY, float rotZ, int bDisableCollision)
{
	GET_ITEM_FROM_LIST(gCollisionObjectList, btRigidBody*, rb, rigidBodyId)

	btTypedConstraint* tc = new btConeTwistConstraint(*rb,
		btTransform(btQuaternion(rotZ*SIMD_2_PI, rotY*SIMD_2_PI, rotX*SIMD_2_PI), btVector3(pivotX, pivotY, pivotZ)));
	gWorld->addConstraint(tc, INT_TO_BOOL(bDisableCollision));

	ADD_TO_LIST(gConstraintList, tc)
}

export int zbtSetConeTwistLimits(int constraintId, float swingX, float swingY, float swingZ,
	float softness, float biasFactor, float relaxationFactor)
{
	GET_ITEM_FROM_MIXED_LIST(gConstraintList, btConeTwistConstraint*, cc, constraintId)

	cc->setLimit(swingZ*SIMD_2_PI, swingY*SIMD_2_PI, swingX*SIMD_2_PI,
		softness, biasFactor, relaxationFactor);

	return DONE;
}

export int zbtAddSliderConstraint(int rigidBodyAId, int rigidBodyBId,
	float pivotAx, float pivotAy, float pivotAz,
	float pivotBx, float pivotBy, float pivotBz,
	float rotAx, float rotAy, float rotAz,
	float rotBx, float rotBy, float rotBz, int bUseLinearReferenceFrameA, int bDisableCollision)
{
	GET_ITEM_FROM_LIST(gCollisionObjectList, btRigidBody*, rbA, rigidBodyAId)
	GET_ITEM_FROM_LIST(gCollisionObjectList, btRigidBody*, rbB, rigidBodyBId)

	btTypedConstraint* tc = new btSliderConstraint(*rbA, *rbB,
		btTransform(btQuaternion(rotAz*SIMD_2_PI, rotAy*SIMD_2_PI, rotAx*SIMD_2_PI), btVector3(pivotAx, pivotAy, pivotAz)),
		btTransform(btQuaternion(rotBz*SIMD_2_PI, rotBy*SIMD_2_PI, rotBx*SIMD_2_PI), btVector3(pivotBx, pivotBy, pivotBz)),
		INT_TO_BOOL(bUseLinearReferenceFrameA));
	gWorld->addConstraint(tc, INT_TO_BOOL(bDisableCollision));

	ADD_TO_LIST(gConstraintList, tc)
}

export int zbtSetSliderLimits(int constraintId, float linLower, float linUpper, float angLower, float angUpper)
{
	GET_ITEM_FROM_MIXED_LIST(gConstraintList, btSliderConstraint*, sc, constraintId)

	sc->setLowerLinLimit(linLower);
	sc->setUpperLinLimit(linUpper);
	sc->setLowerAngLimit(angLower*SIMD_2_PI);
	sc->setUpperAngLimit(angUpper*SIMD_2_PI);

	return DONE;
}

export int zbtSetSliderSoftness(int constraintId, float dirLin, float dirAng, float limLin, float limAng,
	float orthoLin, float orthoAng)
{
	GET_ITEM_FROM_MIXED_LIST(gConstraintList, btSliderConstraint*, sc, constraintId)

	sc->setSoftnessDirLin(dirLin);
	sc->setSoftnessDirAng(dirAng*SIMD_2_PI);
	sc->setSoftnessLimLin(limLin);
	sc->setSoftnessLimAng(limAng*SIMD_2_PI);
	sc->setSoftnessOrthoLin(orthoLin);
	sc->setSoftnessOrthoAng(orthoAng*SIMD_2_PI);

	return DONE;
}

export int zbtSetSliderRestitution(int constraintId, float dirLin, float dirAng, float limLin, float limAng,
	float orthoLin, float orthoAng)
{
	GET_ITEM_FROM_MIXED_LIST(gConstraintList, btSliderConstraint*, sc, constraintId)

	sc->setRestitutionDirLin(dirLin);
	sc->setRestitutionDirAng(dirAng*SIMD_2_PI);
	sc->setRestitutionLimLin(limLin);
	sc->setRestitutionLimAng(limAng*SIMD_2_PI);
	sc->setRestitutionOrthoLin(orthoLin);
	sc->setRestitutionOrthoAng(orthoAng*SIMD_2_PI);

	return DONE;
}

export int zbtSetSliderDamping(int constraintId, float dirLin, float dirAng, float limLin, float limAng,
	float orthoLin, float orthoAng)
{
	GET_ITEM_FROM_MIXED_LIST(gConstraintList, btSliderConstraint*, sc, constraintId)

	sc->setDampingDirLin(dirLin);
	sc->setDampingDirAng(dirAng*SIMD_2_PI);
	sc->setDampingLimLin(limLin);
	sc->setDampingLimAng(limAng*SIMD_2_PI);
	sc->setDampingOrthoLin(orthoLin);
	sc->setDampingOrthoAng(orthoAng*SIMD_2_PI);

	return DONE;
}

export int zbtEnableSliderMotor(int constraintId, int bEnableLinMotor, float targetLinVelocity, float maxLinForce,
	int bEnableAngMotor, float targetAngVelocity, float maxAngForce)
{
	GET_ITEM_FROM_MIXED_LIST(gConstraintList, btSliderConstraint*, sc, constraintId)

	sc->setPoweredLinMotor(INT_TO_BOOL(bEnableLinMotor));
	if(INT_TO_BOOL(bEnableLinMotor))
	{
		sc->setTargetLinMotorVelocity(targetLinVelocity);
		sc->setMaxLinMotorForce(maxLinForce);
	}

	sc->setPoweredAngMotor(INT_TO_BOOL(bEnableAngMotor));
	if(INT_TO_BOOL(bEnableAngMotor))
	{
		sc->setTargetAngMotorVelocity(targetAngVelocity*SIMD_2_PI);
		sc->setMaxAngMotorForce(maxAngForce);
	}

	return DONE;
}

export int zbtAddGeneric6DofConstraint(int rigidBodyAId, int rigidBodyBId,
	float pivotAx, float pivotAy, float pivotAz,
	float pivotBx, float pivotBy, float pivotBz,
	float rotAx, float rotAy, float rotAz,
	float rotBx, float rotBy, float rotBz, int bUseLinearReferenceFrameA, int bDisableCollision)
{
	GET_ITEM_FROM_LIST(gCollisionObjectList, btRigidBody*, rbA, rigidBodyAId)
	GET_ITEM_FROM_LIST(gCollisionObjectList, btRigidBody*, rbB, rigidBodyBId)

	btTypedConstraint* tc = new btGeneric6DofConstraint(*rbA, *rbB,
		btTransform(btQuaternion(rotAz*SIMD_2_PI, rotAy*SIMD_2_PI, rotAx*SIMD_2_PI), btVector3(pivotAx, pivotAy, pivotAz)),
		btTransform(btQuaternion(rotBz*SIMD_2_PI, rotBy*SIMD_2_PI, rotBx*SIMD_2_PI), btVector3(pivotBx, pivotBy, pivotBz)),
		INT_TO_BOOL(bUseLinearReferenceFrameA));
	gWorld->addConstraint(tc, INT_TO_BOOL(bDisableCollision));

	ADD_TO_LIST(gConstraintList, tc)
}

export int zbtSetGeneric6DofLimits(int constraintId, int axis, float lo, float hi)
{
	GET_ITEM_FROM_MIXED_LIST(gConstraintList, btGeneric6DofConstraint*, dc, constraintId)

	if(axis < 3)
		dc->setLimit(axis, lo, hi); // linear
	else
		dc->setLimit(axis, lo*SIMD_2_PI, hi*SIMD_2_PI); // angular

	return DONE;
}

export int zbtSetGeneric6DofLinearLimits(int constraintId, float lowerX, float lowerY, float lowerZ,
	float upperX, float upperY, float upperZ)
{
	GET_ITEM_FROM_MIXED_LIST(gConstraintList, btGeneric6DofConstraint*, dc, constraintId)

	dc->setLinearLowerLimit(btVector3(lowerX, lowerY, lowerZ));
	dc->setLinearUpperLimit(btVector3(upperX, upperY, upperZ));

	return DONE;
}

export int zbtSetGeneric6DofAngularLimits(int constraintId, float lowerX, float lowerY, float lowerZ,
	float upperX, float upperY, float upperZ)
{
	GET_ITEM_FROM_MIXED_LIST(gConstraintList, btGeneric6DofConstraint*, dc, constraintId)

	dc->setAngularLowerLimit(btVector3(lowerX*SIMD_2_PI, lowerY*SIMD_2_PI, lowerZ*SIMD_2_PI));
	dc->setAngularUpperLimit(btVector3(upperX*SIMD_2_PI, upperY*SIMD_2_PI, upperZ*SIMD_2_PI));

	return DONE;
}

export int zbtDeleteConstraint(int constraintId)
{
	GET_ITEM_FROM_LIST(gConstraintList, btTypedConstraint*, tc, constraintId)
	gWorld->removeConstraint(tc);
	gConstraintList[constraintId] = NULL;
	delete tc;

	return DONE;
}


/* ***************************************************************************

// Raycast vehicle

export int zbtCreateRaycastVehicle(int rigidBodyId, int rightIndex, int upIndex, int forwardIndex)
{
	GET_ITEM_FROM_LIST(gCollisionObjectList, btRigidBody*, carChassis, rigidBodyId)

	btRaycastVehicle::btVehicleTuning tuning; //unused in Bullet 2.79
	if(!gVehicleRaycaster) gVehicleRaycaster = new btDefaultVehicleRaycaster(gWorld);
	btRaycastVehicle* rv = new btRaycastVehicle(tuning, carChassis, gVehicleRaycaster);
	rv->setCoordinateSystem(rightIndex, upIndex, forwardIndex);
	gWorld->addVehicle(rv);
	carChassis->setActivationState(DISABLE_DEACTIVATION); //necessary?
	ADD_TO_LIST(gConstraintList, (btTypedConstraint*) rv)
}

export int zbtAddWheel(int vehicleId, float connectionPointX, float connectionPointY, float connectionPointZ,
	float directionX, float directionY, float directionZ, float wheelAxleX, float wheelAxleY, float wheelAxleZ,
	float wheelRadius, float suspensionRestLength, float suspensionStiffness,
	float suspensionCompression, float suspensionDamping, float maxSuspensionTravelCm, float frictionSlip,
	float maxSuspensionForce, float rollInfluence, int bIsFrontWheel)
{
	GET_ITEM_FROM_MIXED_LIST(gConstraintList, btRaycastVehicle*, rv, vehicleId)

	btRaycastVehicle::btVehicleTuning tuning;
	tuning.m_suspensionStiffness = suspensionStiffness;
	tuning.m_suspensionCompression = suspensionCompression;
	tuning.m_suspensionDamping = suspensionDamping;
	tuning.m_maxSuspensionTravelCm = maxSuspensionTravelCm;
	tuning.m_frictionSlip = frictionSlip;
	tuning.m_maxSuspensionForce = maxSuspensionForce;

	btWheelInfo wi = rv->addWheel(btVector3(connectionPointX, connectionPointY, connectionPointZ),
		btVector3(directionX, directionY, directionZ), btVector3(wheelAxleX, wheelAxleY, wheelAxleZ),
		suspensionRestLength, wheelRadius, tuning, INT_TO_BOOL(bIsFrontWheel));

	wi.m_rollInfluence = rollInfluence;

	return rv->getNumWheels()-1;
}

export int zbtSetSteeringValue(int vehicleId, int wheelId, float steering)
{
	GET_ITEM_FROM_MIXED_LIST(gConstraintList, btRaycastVehicle*, rv, vehicleId)
	if(wheelId<0 || wheelId>=rv->getNumWheels()) return ERROR;
	rv->setSteeringValue(steering, wheelId);
	return DONE;
}

export int zbtApplyEngineForce(int vehicleId, int wheelId, float force)
{
	GET_ITEM_FROM_MIXED_LIST(gConstraintList, btRaycastVehicle*, rv, vehicleId)
	if(wheelId<0 || wheelId>=rv->getNumWheels()) return ERROR;
	rv->applyEngineForce(force, wheelId);
	return DONE;
}

export int zbtSetBrake(int vehicleId, int wheelId, float brake)
{
	GET_ITEM_FROM_MIXED_LIST(gConstraintList, btRaycastVehicle*, rv, vehicleId)
	if(wheelId<0 || wheelId>=rv->getNumWheels()) return ERROR;
	rv->setBrake(brake, wheelId);
	return DONE;
}

export int zbtSetPitchControl(int vehicleId, float pitch)
{
	GET_ITEM_FROM_MIXED_LIST(gConstraintList, btRaycastVehicle*, rv, vehicleId)
	rv->setPitchControl(pitch);
	return DONE;
}

export int zbtUpdateWheelTransform(int vehicleId, int wheelId, int bInterpolatedTransform)
{
	GET_ITEM_FROM_MIXED_LIST(gConstraintList, btRaycastVehicle*, rv, vehicleId)
	if(wheelId<0 || wheelId>=rv->getNumWheels()) return ERROR;
	rv->updateWheelTransform(wheelId, INT_TO_BOOL(bInterpolatedTransform));
	return DONE;
}

export float zbtGetCurrentSpeed(int vehicleId)
{
	GET_ITEM_FROM_MIXED_LIST(gConstraintList, btRaycastVehicle*, rv, vehicleId)
	return rv->getCurrentSpeedKmHour();
}

export int zbtGetWheelPosition(int vehicleId, int wheelId, float &x, float &y, float &z)
{
	GET_ITEM_FROM_MIXED_LIST(gConstraintList, btRaycastVehicle*, rv, vehicleId)
	if(wheelId<0 || wheelId>=rv->getNumWheels()) return ERROR;
	x = rv->getWheelTransformWS(wheelId).getOrigin().getX();
	y = rv->getWheelTransformWS(wheelId).getOrigin().getY();
	z = rv->getWheelTransformWS(wheelId).getOrigin().getZ();
	return DONE;
}

export int zbtGetWheelRotation(int vehicleId, int wheelId, float &x, float &y, float &z)
{
	// UNDER CONSTRUCTION
	GET_ITEM_FROM_MIXED_LIST(gConstraintList, btRaycastVehicle*, rv, vehicleId)
	if(wheelId<0 || wheelId>=rv->getNumWheels()) return ERROR;

	rv->getWheelTransformWS(wheelId).getBasis().getEulerZYX(z, y, x); // or rv->getWheelInfo(wheelId).m_worldTransform.getBasis() ?
	x /= SIMD_2_PI;
	y /= SIMD_2_PI;
	z /= SIMD_2_PI;
	return DONE;
}

export int zbtDeleteRaycastVehicle(int vehicleId)
{
	GET_ITEM_FROM_MIXED_LIST(gConstraintList, btRaycastVehicle*, rv, vehicleId)
	gWorld->removeVehicle(rv);
	gConstraintList[vehicleId] = NULL;
	delete rv;

	return DONE;
}

// Ghost object

// TODO !!! WORKING HERE !!!
// TODO ghost objects - find more examples and check the existing code

export int zbtAddGhostObject(int shapeId, float x, float y, float z, float rx, float ry, float rz)
{
	IS_INITIALIZED

	GET_ITEM_FROM_LIST(gCollisionShapeList, btCollisionShape*, shape, shapeId)
	btPairCachingGhostObject* ghostObject = new btPairCachingGhostObject();
	ghostObject->setCollisionShape(shape);
	ghostObject->setWorldTransform(btTransform(btQuaternion(rz*SIMD_2_PI, ry*SIMD_2_PI, rx*SIMD_2_PI), btVector3(x, y, z)));
	ghostObject->setCollisionFlags (btCollisionObject::CF_CHARACTER_OBJECT); // ???
	gWorld->addCollisionObject(ghostObject);
	ADD_TO_LIST(gCollisionObjectList, ghostObject)
}

export int zbtDeleteGhostObject(int ghostObjectId)
{
	GET_ITEM_FROM_MIXED_LIST(gCollisionObjectList, btGhostObject*, go, ghostObjectId)
	gWorld->removeCollisionObject(go);
	gCollisionObjectList[ghostObjectId] = NULL;
	delete go;

	return DONE;
}

// TODO Kinematic character controller - is it really necessary in ZGE???, is not ghost object sufficient?

// ***************************************************************************
*/

// Collision objects (in general)

export int zbtSetFriction(int collisionObjectId, float friction)
{
	GET_ITEM_FROM_LIST(gCollisionObjectList, btCollisionObject*, co, collisionObjectId)
	co->setFriction(friction);
	return DONE;
}

export int zbtSetRestitution(int collisionObjectId, float restitution)
{
	GET_ITEM_FROM_LIST(gCollisionObjectList, btCollisionObject*, co, collisionObjectId)
	co->setRestitution(restitution);
	return DONE;
}

export int zbtSetHitFraction(int collisionObjectId, float hitFraction )
{
	GET_ITEM_FROM_LIST(gCollisionObjectList, btCollisionObject*, co, collisionObjectId)
	co->setHitFraction(hitFraction);
	return DONE;
}

export int zbtGetPosition(int collisionObjectId, float &x, float &y, float &z)
{
	GET_ITEM_FROM_LIST(gCollisionObjectList, btCollisionObject*, co, collisionObjectId)
	x = co->getWorldTransform().getOrigin().getX();
	y = co->getWorldTransform().getOrigin().getY();
	z = co->getWorldTransform().getOrigin().getZ();
	return DONE;
}

export int zbtSetPosition(int collisionObjectId, float x, float y, float z)
{
	GET_ITEM_FROM_LIST(gCollisionObjectList, btCollisionObject*, co, collisionObjectId)
	btTransform trans = co->getWorldTransform();
	trans.setOrigin(btVector3(x, y, z));
	co->setWorldTransform(trans);
	return DONE;
}

export int zbtGetRotation(int collisionObjectId, float &rx, float &ry, float &rz)
{
	GET_ITEM_FROM_LIST(gCollisionObjectList, btCollisionObject*, co, collisionObjectId)
	/*co->getWorldTransform().getBasis().getEulerZYX(rz, ry, rx);
	rx /= SIMD_2_PI;
	ry /= SIMD_2_PI;
	rz /= SIMD_2_PI;*/

	btMatrix3x3 ma = co->getWorldTransform().getBasis();
	rx = getRotationXFromMatrix(&ma);
	ry = getRotationYFromMatrix(&ma);
	rz = getRotationZFromMatrix(&ma);
	return DONE;
}

export int zbtSetRotation(int collisionObjectId, float rx, float ry, float rz)
{
	GET_ITEM_FROM_LIST(gCollisionObjectList, btCollisionObject*, co, collisionObjectId)
	btTransform trans = co->getWorldTransform();
	trans.setRotation(btQuaternion(rz*SIMD_2_PI, ry*SIMD_2_PI, rx*SIMD_2_PI));
	co->setWorldTransform(trans);
	return DONE;
}

// Activation and deactivation

export int zbtIsActive(int collisionObjectId)
{
	GET_ITEM_FROM_LIST(gCollisionObjectList, btCollisionObject*, co, collisionObjectId)
	return int(co->isActive());
}

export int zbtActivate(int collisionObjectId, int bForceActivation)
{
	GET_ITEM_FROM_LIST(gCollisionObjectList, btCollisionObject*, co, collisionObjectId)
	co->activate(INT_TO_BOOL(bForceActivation));
	return DONE;
}

export int zbtSetActivationState(int collisionObjectId, int newState)
{
	GET_ITEM_FROM_LIST(gCollisionObjectList, btCollisionObject*, co, collisionObjectId)
	co->setActivationState(newState);
	return DONE;
}

export int zbtSetDeactivationTime(int collisionObjectId, float time)
{
	GET_ITEM_FROM_LIST(gCollisionObjectList, btCollisionObject*, co, collisionObjectId)
	co->setDeactivationTime(time);
	return DONE;
}

export int zbtSetDeactivationThresholds(int rigidBodyId, float linear, float angular)
{
	GET_ITEM_FROM_LIST(gCollisionObjectList, btRigidBody*, rb, rigidBodyId)
	rb->setSleepingThresholds(linear, angular*SIMD_2_PI);
	return DONE;
}

// Collision detection

export int zbtGetCollisionNum()
{
	IS_INITIALIZED
	return gWorld->getDispatcher()->getNumManifolds();
}

export int zbtGetCollisionObjects(int index, int &collisionObject1, int &collisionObject2)
{
	IS_INITIALIZED
	try
	{
		collisionObject1 = gCollisionObjectList.findLinearSearch((btCollisionObject*)gWorld->getDispatcher()->
			getManifoldByIndexInternal(index)->getBody0());

		collisionObject2 = gCollisionObjectList.findLinearSearch((btCollisionObject*)gWorld->getDispatcher()->
			getManifoldByIndexInternal(index)->getBody0());

		if(collisionObject1 < gCollisionObjectList.size() && collisionObject2 < gCollisionObjectList.size()) return DONE;
		else return ERROR;
	} catch (...) {return ERROR;}
}

// NEEDED? !!!
/*export int zbtGetCollisionObject1(int index)
{
	IS_INITIALIZED
	try
	{
		int i = gCollisionObjectList.findLinearSearch((btCollisionObject*)gWorld->getDispatcher()->
			getManifoldByIndexInternal(index)->getBody0());

		if(i < gCollisionObjectList.size()) return i;
		else return ERROR;
	} catch (...) {return ERROR;}
}*/

// NEEDED? !!!
/*export int zbtGetCollisionObject2(int index)
{
	IS_INITIALIZED
	try
	{
		int i = gCollisionObjectList.findLinearSearch((btCollisionObject*)gWorld->getDispatcher()->
			getManifoldByIndexInternal(index)->getBody1());

		if(i < gCollisionObjectList.size()) return i;
		else return ERROR;
	} catch (...) {return ERROR;}
}*/

export int zbtGetCollisionOfNum(int collisionObjectId)
{
	GET_ITEM_FROM_LIST(gCollisionObjectList, btCollisionObject*, co, collisionObjectId)

	int ret = 0;
	for(int i = gWorld->getDispatcher()->getNumManifolds()-1; i >= 0; i--)
	{
		btPersistentManifold* pm = gWorld->getDispatcher()->getManifoldByIndexInternal(i);

		if((btCollisionObject*)pm->getBody0() == co || (btCollisionObject*)pm->getBody1() == co)
		{
			int j = pm->getNumContacts()-1;
			for(;j>=0;j--) if(pm->validContactDistance(pm->getContactPoint(j))) break;
			if(j>=0) ret++;
		}
	}
	return ret;
}

// REWORK !!!
/*export int zbtGetCollisionOf(int collisionObjectId, int index)
{
	if(!gComputeCollisions || collisionObjectId < 0 || collisionObjectId >= int(gCollisionList.size()) ||
		index < 0 || index >= int(gCollisionList[collisionObjectId].size())) return ERROR;

	return gCollisionList[collisionObjectId][index];
}
*/

export int zbtIsCollidedWith(int collisionObjectId1, int collisionObjectId2)
{
	GET_ITEM_FROM_LIST(gCollisionObjectList, btCollisionObject*, co1, collisionObjectId1)
	GET_ITEM_FROM_LIST(gCollisionObjectList, btCollisionObject*, co2, collisionObjectId2)

	for(int i = gWorld->getDispatcher()->getNumManifolds()-1; i >= 0; i--)
	{
		btPersistentManifold* pm = gWorld->getDispatcher()->getManifoldByIndexInternal(i);
		btCollisionObject* bo1 = (btCollisionObject*)pm->getBody0();
		btCollisionObject* bo2 = (btCollisionObject*)pm->getBody1();

		if((bo1 == co1 && bo2 == co2) || (bo2 == co1 && bo1 == co2))
		{
			int j = pm->getNumContacts()-1;
			for(;j>=0;j--) if(pm->validContactDistance(pm->getContactPoint(j))) break;
			if(j>=0) return TRUE;
			else return FALSE;
		}
	}
	return FALSE;
}

// Raycasting

// TODO Check code of raycasting with btDefaultVehicleRaycaster::castRay

export int zbtRayTest(float fromX, float fromY, float fromZ, float toX, float toY, float toZ)
{
	IS_INITIALIZED

	btCollisionWorld::ClosestRayResultCallback crrc(btVector3(fromX, fromY, fromZ), btVector3(toX, toY, toZ)); 
	gWorld->rayTest(btVector3(fromX, fromY, fromZ), btVector3(toX, toY, toZ), crrc);
	if(crrc.hasHit())
		try
		{
			int i = gCollisionObjectList.findLinearSearch((btRigidBody*)crrc.m_collisionObject);

			if(i < gCollisionObjectList.size())
			{
				gRayTestHitPoint = crrc.m_hitPointWorld;
				gRayTestHitNormal = crrc.m_hitNormalWorld;
				return i;
			}
			else return ERROR;

		} catch (...) {return ERROR;}
	else
		return ERROR;
}

export int zbtGetRayTestHitPoint(float &x, float &y, float &z)
{
	IS_INITIALIZED
	x = gRayTestHitPoint.getX();
	y = gRayTestHitPoint.getY();
	z = gRayTestHitPoint.getZ();
	return DONE;
}

export int zbtGetRayTestHitNormal(float &x, float &y, float &z)
{
	IS_INITIALIZED
	x = gRayTestHitNormal.getX();
	y = gRayTestHitNormal.getY();
	z = gRayTestHitNormal.getZ();
	return DONE;
}
