LOCAL_PATH := $(call my-dir)/../..

include $(CLEAR_VARS)

LOCAL_MODULE := ZGEBullet

MY_BULLET_PATH := bullet-2.80-rev2531/src
LOCAL_C_INCLUDES := $(LOCAL_PATH)/$(MY_BULLET_PATH)
LOCAL_CPPFLAGS += -fexceptions -frtti
LOCAL_LDLIBS := -L$(SYSROOT)/usr/lib -ldl -lm -lstdc++

# uncomment for ARMv7-A
#LOCAL_CFLAGS := -march=armv7-a -mfloat-abi=softfp

TARGET_PLATFORM := android-8

LOCAL_SRC_FILES := \
    src/ZGEBullet.cpp\
    $(MY_BULLET_PATH)/BulletCollision/BroadphaseCollision/btAxisSweep3.cpp\
    $(MY_BULLET_PATH)/BulletCollision/BroadphaseCollision/btBroadphaseProxy.cpp\
    $(MY_BULLET_PATH)/BulletCollision/BroadphaseCollision/btCollisionAlgorithm.cpp\
    $(MY_BULLET_PATH)/BulletCollision/BroadphaseCollision/btDbvt.cpp\
    $(MY_BULLET_PATH)/BulletCollision/BroadphaseCollision/btDbvtBroadphase.cpp\
    $(MY_BULLET_PATH)/BulletCollision/BroadphaseCollision/btDispatcher.cpp\
    $(MY_BULLET_PATH)/BulletCollision/BroadphaseCollision/btMultiSapBroadphase.cpp\
    $(MY_BULLET_PATH)/BulletCollision/BroadphaseCollision/btOverlappingPairCache.cpp\
    $(MY_BULLET_PATH)/BulletCollision/BroadphaseCollision/btQuantizedBvh.cpp\
    $(MY_BULLET_PATH)/BulletCollision/BroadphaseCollision/btSimpleBroadphase.cpp\
    $(MY_BULLET_PATH)/BulletCollision/CollisionDispatch/btActivatingCollisionAlgorithm.cpp\
    $(MY_BULLET_PATH)/BulletCollision/CollisionDispatch/btBox2dBox2dCollisionAlgorithm.cpp\
    $(MY_BULLET_PATH)/BulletCollision/CollisionDispatch/btBoxBoxCollisionAlgorithm.cpp\
    $(MY_BULLET_PATH)/BulletCollision/CollisionDispatch/btBoxBoxDetector.cpp\
    $(MY_BULLET_PATH)/BulletCollision/CollisionDispatch/btCollisionDispatcher.cpp\
    $(MY_BULLET_PATH)/BulletCollision/CollisionDispatch/btCollisionObject.cpp\
    $(MY_BULLET_PATH)/BulletCollision/CollisionDispatch/btCollisionWorld.cpp\
    $(MY_BULLET_PATH)/BulletCollision/CollisionDispatch/btCompoundCollisionAlgorithm.cpp\
    $(MY_BULLET_PATH)/BulletCollision/CollisionDispatch/btConvex2dConvex2dAlgorithm.cpp\
    $(MY_BULLET_PATH)/BulletCollision/CollisionDispatch/btConvexConcaveCollisionAlgorithm.cpp\
    $(MY_BULLET_PATH)/BulletCollision/CollisionDispatch/btConvexConvexAlgorithm.cpp\
    $(MY_BULLET_PATH)/BulletCollision/CollisionDispatch/btConvexPlaneCollisionAlgorithm.cpp\
    $(MY_BULLET_PATH)/BulletCollision/CollisionDispatch/btDefaultCollisionConfiguration.cpp\
    $(MY_BULLET_PATH)/BulletCollision/CollisionDispatch/btEmptyCollisionAlgorithm.cpp\
    $(MY_BULLET_PATH)/BulletCollision/CollisionDispatch/btGhostObject.cpp\
    $(MY_BULLET_PATH)/BulletCollision/CollisionDispatch/btInternalEdgeUtility.cpp\
    $(MY_BULLET_PATH)/BulletCollision/CollisionDispatch/btManifoldResult.cpp\
    $(MY_BULLET_PATH)/BulletCollision/CollisionDispatch/btSimulationIslandManager.cpp\
    $(MY_BULLET_PATH)/BulletCollision/CollisionDispatch/btSphereBoxCollisionAlgorithm.cpp\
    $(MY_BULLET_PATH)/BulletCollision/CollisionDispatch/btSphereSphereCollisionAlgorithm.cpp\
    $(MY_BULLET_PATH)/BulletCollision/CollisionDispatch/btSphereTriangleCollisionAlgorithm.cpp\
    $(MY_BULLET_PATH)/BulletCollision/CollisionDispatch/btUnionFind.cpp\
    $(MY_BULLET_PATH)/BulletCollision/CollisionDispatch/SphereTriangleDetector.cpp\
    $(MY_BULLET_PATH)/BulletCollision/CollisionShapes/btBox2dShape.cpp\
    $(MY_BULLET_PATH)/BulletCollision/CollisionShapes/btBoxShape.cpp\
    $(MY_BULLET_PATH)/BulletCollision/CollisionShapes/btBvhTriangleMeshShape.cpp\
    $(MY_BULLET_PATH)/BulletCollision/CollisionShapes/btCapsuleShape.cpp\
    $(MY_BULLET_PATH)/BulletCollision/CollisionShapes/btCollisionShape.cpp\
    $(MY_BULLET_PATH)/BulletCollision/CollisionShapes/btCompoundShape.cpp\
    $(MY_BULLET_PATH)/BulletCollision/CollisionShapes/btConcaveShape.cpp\
    $(MY_BULLET_PATH)/BulletCollision/CollisionShapes/btConeShape.cpp\
    $(MY_BULLET_PATH)/BulletCollision/CollisionShapes/btConvex2dShape.cpp\
    $(MY_BULLET_PATH)/BulletCollision/CollisionShapes/btConvexHullShape.cpp\
    $(MY_BULLET_PATH)/BulletCollision/CollisionShapes/btConvexInternalShape.cpp\
    $(MY_BULLET_PATH)/BulletCollision/CollisionShapes/btConvexPointCloudShape.cpp\
    $(MY_BULLET_PATH)/BulletCollision/CollisionShapes/btConvexPolyhedron.cpp\
    $(MY_BULLET_PATH)/BulletCollision/CollisionShapes/btConvexShape.cpp\
    $(MY_BULLET_PATH)/BulletCollision/CollisionShapes/btConvexTriangleMeshShape.cpp\
    $(MY_BULLET_PATH)/BulletCollision/CollisionShapes/btCylinderShape.cpp\
    $(MY_BULLET_PATH)/BulletCollision/CollisionShapes/btEmptyShape.cpp\
    $(MY_BULLET_PATH)/BulletCollision/CollisionShapes/btHeightfieldTerrainShape.cpp\
    $(MY_BULLET_PATH)/BulletCollision/CollisionShapes/btMinkowskiSumShape.cpp\
    $(MY_BULLET_PATH)/BulletCollision/CollisionShapes/btMultimaterialTriangleMeshShape.cpp\
    $(MY_BULLET_PATH)/BulletCollision/CollisionShapes/btMultiSphereShape.cpp\
    $(MY_BULLET_PATH)/BulletCollision/CollisionShapes/btOptimizedBvh.cpp\
    $(MY_BULLET_PATH)/BulletCollision/CollisionShapes/btPolyhedralConvexShape.cpp\
    $(MY_BULLET_PATH)/BulletCollision/CollisionShapes/btScaledBvhTriangleMeshShape.cpp\
    $(MY_BULLET_PATH)/BulletCollision/CollisionShapes/btShapeHull.cpp\
    $(MY_BULLET_PATH)/BulletCollision/CollisionShapes/btSphereShape.cpp\
    $(MY_BULLET_PATH)/BulletCollision/CollisionShapes/btStaticPlaneShape.cpp\
    $(MY_BULLET_PATH)/BulletCollision/CollisionShapes/btStridingMeshInterface.cpp\
    $(MY_BULLET_PATH)/BulletCollision/CollisionShapes/btTetrahedronShape.cpp\
    $(MY_BULLET_PATH)/BulletCollision/CollisionShapes/btTriangleBuffer.cpp\
    $(MY_BULLET_PATH)/BulletCollision/CollisionShapes/btTriangleCallback.cpp\
    $(MY_BULLET_PATH)/BulletCollision/CollisionShapes/btTriangleIndexVertexArray.cpp\
    $(MY_BULLET_PATH)/BulletCollision/CollisionShapes/btTriangleIndexVertexMaterialArray.cpp\
    $(MY_BULLET_PATH)/BulletCollision/CollisionShapes/btTriangleMesh.cpp\
    $(MY_BULLET_PATH)/BulletCollision/CollisionShapes/btTriangleMeshShape.cpp\
    $(MY_BULLET_PATH)/BulletCollision/CollisionShapes/btUniformScalingShape.cpp\
    $(MY_BULLET_PATH)/BulletCollision/Gimpact/btContactProcessing.cpp\
    $(MY_BULLET_PATH)/BulletCollision/Gimpact/btGenericPoolAllocator.cpp\
    $(MY_BULLET_PATH)/BulletCollision/Gimpact/btGImpactBvh.cpp\
    $(MY_BULLET_PATH)/BulletCollision/Gimpact/btGImpactCollisionAlgorithm.cpp\
    $(MY_BULLET_PATH)/BulletCollision/Gimpact/btGImpactQuantizedBvh.cpp\
    $(MY_BULLET_PATH)/BulletCollision/Gimpact/btGImpactShape.cpp\
    $(MY_BULLET_PATH)/BulletCollision/Gimpact/btTriangleShapeEx.cpp\
    $(MY_BULLET_PATH)/BulletCollision/Gimpact/gim_box_set.cpp\
    $(MY_BULLET_PATH)/BulletCollision/Gimpact/gim_contact.cpp\
    $(MY_BULLET_PATH)/BulletCollision/Gimpact/gim_memory.cpp\
    $(MY_BULLET_PATH)/BulletCollision/Gimpact/gim_tri_collision.cpp\
    $(MY_BULLET_PATH)/BulletCollision/NarrowPhaseCollision/btContinuousConvexCollision.cpp\
    $(MY_BULLET_PATH)/BulletCollision/NarrowPhaseCollision/btConvexCast.cpp\
    $(MY_BULLET_PATH)/BulletCollision/NarrowPhaseCollision/btGjkConvexCast.cpp\
    $(MY_BULLET_PATH)/BulletCollision/NarrowPhaseCollision/btGjkEpa2.cpp\
    $(MY_BULLET_PATH)/BulletCollision/NarrowPhaseCollision/btGjkEpaPenetrationDepthSolver.cpp\
    $(MY_BULLET_PATH)/BulletCollision/NarrowPhaseCollision/btGjkPairDetector.cpp\
    $(MY_BULLET_PATH)/BulletCollision/NarrowPhaseCollision/btMinkowskiPenetrationDepthSolver.cpp\
    $(MY_BULLET_PATH)/BulletCollision/NarrowPhaseCollision/btPersistentManifold.cpp\
    $(MY_BULLET_PATH)/BulletCollision/NarrowPhaseCollision/btPolyhedralContactClipping.cpp\
    $(MY_BULLET_PATH)/BulletCollision/NarrowPhaseCollision/btRaycastCallback.cpp\
    $(MY_BULLET_PATH)/BulletCollision/NarrowPhaseCollision/btSubSimplexConvexCast.cpp\
    $(MY_BULLET_PATH)/BulletCollision/NarrowPhaseCollision/btVoronoiSimplexSolver.cpp\
    $(MY_BULLET_PATH)/BulletDynamics/ConstraintSolver/btConeTwistConstraint.cpp\
    $(MY_BULLET_PATH)/BulletDynamics/ConstraintSolver/btContactConstraint.cpp\
    $(MY_BULLET_PATH)/BulletDynamics/ConstraintSolver/btGeneric6DofConstraint.cpp\
    $(MY_BULLET_PATH)/BulletDynamics/ConstraintSolver/btGeneric6DofSpringConstraint.cpp\
    $(MY_BULLET_PATH)/BulletDynamics/ConstraintSolver/btHinge2Constraint.cpp\
    $(MY_BULLET_PATH)/BulletDynamics/ConstraintSolver/btHingeConstraint.cpp\
    $(MY_BULLET_PATH)/BulletDynamics/ConstraintSolver/btPoint2PointConstraint.cpp\
    $(MY_BULLET_PATH)/BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolver.cpp\
    $(MY_BULLET_PATH)/BulletDynamics/ConstraintSolver/btSliderConstraint.cpp\
    $(MY_BULLET_PATH)/BulletDynamics/ConstraintSolver/btSolve2LinearConstraint.cpp\
    $(MY_BULLET_PATH)/BulletDynamics/ConstraintSolver/btTypedConstraint.cpp\
    $(MY_BULLET_PATH)/BulletDynamics/ConstraintSolver/btUniversalConstraint.cpp\
    $(MY_BULLET_PATH)/BulletDynamics/Dynamics/btDiscreteDynamicsWorld.cpp\
    $(MY_BULLET_PATH)/BulletDynamics/Dynamics/btRigidBody.cpp\
    $(MY_BULLET_PATH)/BulletDynamics/Dynamics/btSimpleDynamicsWorld.cpp\
    $(MY_BULLET_PATH)/BulletDynamics/Dynamics/Bullet-C-API.cpp\
    $(MY_BULLET_PATH)/LinearMath/btAlignedAllocator.cpp\
    $(MY_BULLET_PATH)/LinearMath/btConvexHull.cpp\
    $(MY_BULLET_PATH)/LinearMath/btConvexHullComputer.cpp\
    $(MY_BULLET_PATH)/LinearMath/btGeometryUtil.cpp\
    $(MY_BULLET_PATH)/LinearMath/btQuickprof.cpp

include $(BUILD_SHARED_LIBRARY)
