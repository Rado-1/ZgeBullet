mkdir lib
gcc -Wall -O2 -fPIC -I ./bullet3-2.85.1/src -c ./src/ZgeBullet.cpp -o ./lib/ZgeBullet.o
C="./bullet3-2.85.1/build_cmake/src/BulletCollision/CMakeFiles/BulletCollision.dir"
D="./bullet3-2.85.1/build_cmake/src/BulletDynamics/CMakeFiles/BulletDynamics.dir"
M="./bullet3-2.85.1/build_cmake/src/LinearMath/CMakeFiles/LinearMath.dir"
gcc -shared -O2 -o ./lib/ZgeBullet.so ./lib/ZgeBullet.o \
	$C/BroadphaseCollision/btAxisSweep3.o \
	$C/BroadphaseCollision/btCollisionAlgorithm.o \
	$C/BroadphaseCollision/btDbvt.o \
	$C/BroadphaseCollision/btDbvtBroadphase.o \
	$C/BroadphaseCollision/btDispatcher.o \
	$C/BroadphaseCollision/btMultiSapBroadphase.o \
	$C/BroadphaseCollision/btOverlappingPairCache.o \
	$C/BroadphaseCollision/btQuantizedBvh.o \
	$C/BroadphaseCollision/btSimpleBroadphase.o \
	$C/CollisionDispatch/btActivatingCollisionAlgorithm.o \
	$C/CollisionDispatch/btBox2dBox2dCollisionAlgorithm.o \
	$C/CollisionDispatch/btBoxBoxCollisionAlgorithm.o \
	$C/CollisionDispatch/btBoxBoxDetector.o \
	$C/CollisionDispatch/btCollisionDispatcher.o \
	$C/CollisionDispatch/btCollisionObject.o \
	$C/CollisionDispatch/btCollisionWorld.o \
	$C/CollisionDispatch/btCollisionWorldImporter.o \
	$C/CollisionDispatch/btCompoundCollisionAlgorithm.o \
	$C/CollisionDispatch/btCompoundCompoundCollisionAlgorithm.o \
	$C/CollisionDispatch/btConvex2dConvex2dAlgorithm.o \
	$C/CollisionDispatch/btConvexConcaveCollisionAlgorithm.o \
	$C/CollisionDispatch/btConvexConvexAlgorithm.o \
	$C/CollisionDispatch/btConvexPlaneCollisionAlgorithm.o \
	$C/CollisionDispatch/btDefaultCollisionConfiguration.o \
	$C/CollisionDispatch/btEmptyCollisionAlgorithm.o \
	$C/CollisionDispatch/btGhostObject.o \
	$C/CollisionDispatch/btHashedSimplePairCache.o \
	$C/CollisionDispatch/btInternalEdgeUtility.o \
	$C/CollisionDispatch/btManifoldResult.o \
	$C/CollisionDispatch/btSimulationIslandManager.o \
	$C/CollisionDispatch/btSphereBoxCollisionAlgorithm.o \
	$C/CollisionDispatch/btSphereSphereCollisionAlgorithm.o \
	$C/CollisionDispatch/btSphereTriangleCollisionAlgorithm.o \
	$C/CollisionDispatch/btUnionFind.o \
	$C/CollisionDispatch/SphereTriangleDetector.o \
	$C/CollisionShapes/btBoxShape.o \
	$C/CollisionShapes/btBvhTriangleMeshShape.o \
	$C/CollisionShapes/btCapsuleShape.o \
	$C/CollisionShapes/btCollisionShape.o \
	$C/CollisionShapes/btCompoundShape.o \
	$C/CollisionShapes/btConcaveShape.o \
	$C/CollisionShapes/btConeShape.o \
	$C/CollisionShapes/btConvexHullShape.o \
	$C/CollisionShapes/btConvexInternalShape.o \
	$C/CollisionShapes/btConvexPolyhedron.o \
	$C/CollisionShapes/btConvexShape.o \
	$C/CollisionShapes/btConvexTriangleMeshShape.o \
	$C/CollisionShapes/btCylinderShape.o \
	$C/CollisionShapes/btHeightfieldTerrainShape.o \
	$C/CollisionShapes/btMultiSphereShape.o \
	$C/CollisionShapes/btOptimizedBvh.o \
	$C/CollisionShapes/btPolyhedralConvexShape.o \
	$C/CollisionShapes/btScaledBvhTriangleMeshShape.o \
	$C/CollisionShapes/btShapeHull.o \
	$C/CollisionShapes/btSphereShape.o \
	$C/CollisionShapes/btStaticPlaneShape.o \
	$C/CollisionShapes/btStridingMeshInterface.o \
	$C/CollisionShapes/btTetrahedronShape.o \
	$C/CollisionShapes/btTriangleBuffer.o \
	$C/CollisionShapes/btTriangleCallback.o \
	$C/CollisionShapes/btTriangleIndexVertexArray.o \
	$C/CollisionShapes/btTriangleIndexVertexMaterialArray.o \
	$C/CollisionShapes/btTriangleMesh.o \
	$C/CollisionShapes/btTriangleMeshShape.o \
	$C/CollisionShapes/btUniformScalingShape.o \
	$C/Gimpact/btContactProcessing.o \
	$C/Gimpact/btGenericPoolAllocator.o \
	$C/Gimpact/btGImpactBvh.o \
	$C/Gimpact/btGImpactCollisionAlgorithm.o \
	$C/Gimpact/btGImpactQuantizedBvh.o \
	$C/Gimpact/btGImpactShape.o \
	$C/Gimpact/btTriangleShapeEx.o \
	$C/Gimpact/gim_box_set.o \
	$C/Gimpact/gim_contact.o \
	$C/Gimpact/gim_memory.o \
	$C/Gimpact/gim_tri_collision.o \
	$C/NarrowPhaseCollision/btContinuousConvexCollision.o \
	$C/NarrowPhaseCollision/btConvexCast.o \
	$C/NarrowPhaseCollision/btGjkConvexCast.o \
	$C/NarrowPhaseCollision/btGjkEpa2.o \
	$C/NarrowPhaseCollision/btGjkEpaPenetrationDepthSolver.o \
	$C/NarrowPhaseCollision/btGjkPairDetector.o \
	$C/NarrowPhaseCollision/btMinkowskiPenetrationDepthSolver.o \
	$C/NarrowPhaseCollision/btPersistentManifold.o \
	$C/NarrowPhaseCollision/btPolyhedralContactClipping.o \
	$C/NarrowPhaseCollision/btRaycastCallback.o \
	$C/NarrowPhaseCollision/btSubSimplexConvexCast.o \
	$C/NarrowPhaseCollision/btVoronoiSimplexSolver.o \
	$D/Character/btKinematicCharacterController.o \
	$D/ConstraintSolver/btConeTwistConstraint.o \
	$D/ConstraintSolver/btContactConstraint.o \
	$D/ConstraintSolver/btFixedConstraint.o \
	$D/ConstraintSolver/btGearConstraint.o \
	$D/ConstraintSolver/btGeneric6DofConstraint.o \
	$D/ConstraintSolver/btGeneric6DofSpring2Constraint.o \
	$D/ConstraintSolver/btGeneric6DofSpringConstraint.o \
	$D/ConstraintSolver/btHinge2Constraint.o \
	$D/ConstraintSolver/btHingeConstraint.o \
	$D/ConstraintSolver/btNNCGConstraintSolver.o \
	$D/ConstraintSolver/btPoint2PointConstraint.o \
	$D/ConstraintSolver/btSequentialImpulseConstraintSolver.o \
	$D/ConstraintSolver/btSliderConstraint.o \
	$D/ConstraintSolver/btSolve2LinearConstraint.o \
	$D/ConstraintSolver/btTypedConstraint.o \
	$D/ConstraintSolver/btUniversalConstraint.o \
	$D/Dynamics/btDiscreteDynamicsWorld.o \
	$D/Dynamics/btRigidBody.o \
	$D/Dynamics/btSimpleDynamicsWorld.o \
	$D/MLCPSolvers/btDantzigLCP.o \
	$D/MLCPSolvers/btMLCPSolver.o \
	$D/Vehicle/btRaycastVehicle.o \
	$D/Vehicle/btWheelInfo.o \
	$M/btAlignedAllocator.o \
	$M/btConvexHull.o \
	$M/btConvexHullComputer.o \
	$M/btGeometryUtil.o \
	$M/btPolarDecomposition.o \
	$M/btQuickprof.o \
	$M/btSerializer.o \
	$M/btVector3.o