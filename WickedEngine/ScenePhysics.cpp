#include "wiScene.h"
#include "wiProfiler.h"
#include "wiMath.h"
#include "wiBackLog.h"
#include "wiJobSystem.h"

#include "btBulletDynamicsCommon.h"
#include "BulletSoftBody/btSoftBodyHelpers.h"
#include "BulletSoftBody/btDefaultSoftBodySolver.h"
#include "BulletSoftBody/btSoftRigidDynamicsWorld.h"
#include "BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.h"

#include <mutex>
#include <memory>
#include <algorithm>

using namespace std;
using namespace wiECS;
using namespace wiScene;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#ifndef M_PI_2
#define M_PI_2 1.57079632679489661923
#endif

#ifndef M_PI_4
#define M_PI_4 0.785398163397448309616
#endif

struct wiScene::Scene::PhysicsEngine
{
	std::mutex physicsLock;

	btGeneric6DofSpring2Constraint* servoMotorConstraint = nullptr; // DEBUG

	btVector3 gravity = btVector3(0, /*-10*/0, 0);
	int softbodyIterationCount = 5;
	std::unique_ptr<btCollisionConfiguration> collisionConfiguration;
	std::unique_ptr<btCollisionDispatcher> dispatcher;
	std::unique_ptr<btBroadphaseInterface> overlappingPairCache;
	std::unique_ptr<btSequentialImpulseConstraintSolver> solver;
	std::unique_ptr<btDynamicsWorld> dynamicsWorld;

	void AddRigidBody(Entity entity, wiScene::RigidBodyPhysicsComponent& physicscomponent, const wiScene::MeshComponent& mesh, const wiScene::TransformComponent& transform);
	void AddSoftBody(Entity entity, wiScene::SoftBodyPhysicsComponent& physicscomponent, const wiScene::MeshComponent& mesh);
};

class BulletDebugDrawer : public btIDebugDraw
{
public:
	virtual void drawLine(const btVector3& from, const btVector3& to, const btVector3& color) override
	{
		wiRenderer::RenderableLine line;
		line.start = XMFLOAT3(from.x(), from.y(), from.z());
		line.end = XMFLOAT3(to.x(), to.y(), to.z());
		line.color_start = XMFLOAT4(color.x(), color.y(), color.z(), 1.0f);
		line.color_end = XMFLOAT4(color.x(), color.y(), color.z(), 1.0f);
		wiRenderer::DrawLine(line);
	}

	virtual void reportErrorWarning(const char* warningString) override {}

	virtual void draw3dText(const btVector3& location, const char* textString) override {}

	virtual void setDebugMode(int debugMode) override
	{
		debugDrawMode = debugMode;
	}

	virtual int getDebugMode() const override
	{
		return debugDrawMode;
	}

	virtual void drawContactPoint(const btVector3& PointOnB, const btVector3& normalOnB,
								  btScalar distance, int lifeTime, const btVector3& color) override
	{}

private:
	int debugDrawMode = btIDebugDraw::DBG_DrawWireframe | DBG_DrawConstraints | DBG_DrawConstraintLimits;
};

void wiScene::Scene::RunSpringUpdateSystem(wiJobSystem::context& ctx, float dt)
{
	static float time = 0;
	time += dt;
	const XMVECTOR windDir = XMLoadFloat3(&weather->windDirection);
	const XMVECTOR gravity = XMVectorSet(0, -9.8f, 0, 0);

	for (size_t i = 0; i < springs.GetCount(); ++i)
	{
		SpringComponent& spring = springs[i];
		if (spring.IsDisabled())
		{
			continue;
		}
		Entity entity = springs.GetEntity(i);
		TransformComponent* transform = transforms.GetComponent(entity);
		if (transform == nullptr)
		{
			assert(0);
			continue;
		}

		if (spring.IsResetting())
		{
			spring.Reset(false);
			spring.center_of_mass = transform->GetPosition();
			spring.velocity = XMFLOAT3(0, 0, 0);
		}

		const HierarchyComponent* hier = hierarchy.GetComponent(entity);
		TransformComponent* parent_transform = hier == nullptr ? nullptr : transforms.GetComponent(hier->parentID);
		if (parent_transform != nullptr)
		{
			// Spring hierarchy resolve depends on spring component order!
			//	It works best when parent spring is located before child spring!
			//	It will work the other way, but results will be less convincing
			transform->UpdateTransform_Parented(*parent_transform);
		}

		const XMVECTOR position_current = transform->GetPositionV();
		XMVECTOR position_prev = XMLoadFloat3(&spring.center_of_mass);
		XMVECTOR force = XMVectorSubtract(position_current, position_prev) * spring.stiffness;

		if (spring.wind_affection > 0)
		{
			force += std::sin(time * weather->windSpeed + XMVectorGetX(XMVector3Dot(position_current, windDir))) * windDir * spring.wind_affection;
		}
		if (spring.IsGravityEnabled())
		{
			force += gravity;
		}

		XMVECTOR velocity = XMLoadFloat3(&spring.velocity);
		velocity += force * dt;
		XMVECTOR position_target = XMVectorAdd(position_prev, velocity * dt);

		if (parent_transform != nullptr)
		{
			const XMVECTOR position_parent = parent_transform->GetPositionV();
			const XMVECTOR parent_to_child = XMVectorSubtract(position_current, position_parent);
			const XMVECTOR parent_to_target = XMVectorSubtract(position_target, position_parent);

			if (!spring.IsStretchEnabled())
			{
				// Limit offset to keep distance from parent:
				const XMVECTOR len = XMVector3Length(parent_to_child);
				position_target = XMVectorAdd(position_parent, XMVectorMultiply(XMVector3Normalize(parent_to_target), len));
			}

			// Parent rotation to point to new child position:
			const XMVECTOR dir_parent_to_child = XMVector3Normalize(parent_to_child);
			const XMVECTOR dir_parent_to_target = XMVector3Normalize(parent_to_target);
			const XMVECTOR axis = XMVector3Normalize(XMVector3Cross(dir_parent_to_child, dir_parent_to_target));
			const float angle = XMScalarACos(XMVectorGetX(XMVector3Dot(dir_parent_to_child, dir_parent_to_target))); // don't use std::acos!
			const XMVECTOR Q = XMQuaternionNormalize(XMQuaternionRotationNormal(axis, angle));
			TransformComponent saved_parent = *parent_transform;
			saved_parent.ApplyTransform();
			saved_parent.Rotate(Q);
			saved_parent.UpdateTransform();
			std::swap(saved_parent.world, parent_transform->world); // only store temporary result, not modifying actual local space!
		}

		XMStoreFloat3(&spring.center_of_mass, position_target);
		velocity *= spring.damping;
		XMStoreFloat3(&spring.velocity, velocity);
		*((XMFLOAT3*)&transform->world._41) = spring.center_of_mass;
	}
}

void wiScene::Scene::RunInverseKinematicsUpdateSystem(wiJobSystem::context& ctx)
{
	bool recompute_hierarchy = false;
	for (size_t i = 0; i < inverse_kinematics.GetCount(); ++i)
	{
		const InverseKinematicsComponent& ik = inverse_kinematics[i];
		if (ik.IsDisabled())
		{
			continue;
		}
		Entity entity = inverse_kinematics.GetEntity(i);
		TransformComponent* transform = transforms.GetComponent(entity);
		TransformComponent* target = transforms.GetComponent(ik.target);
		const HierarchyComponent* hier = hierarchy.GetComponent(entity);
		if (transform == nullptr || target == nullptr || hier == nullptr)
		{
			continue;
		}

		const XMVECTOR target_pos = target->GetPositionV();
		for (uint32_t iteration = 0; iteration < ik.iteration_count; ++iteration)
		{
			TransformComponent* stack[32] = {};
			Entity parent_entity = hier->parentID;
			TransformComponent* child_transform = transform;
			for (uint32_t chain = 0; chain < std::min(ik.chain_length, (uint32_t)arraysize(stack)); ++chain)
			{
				recompute_hierarchy = true; // any IK will trigger a full transform hierarchy recompute step at the end(**)

				// stack stores all traversed chain links so far:
				stack[chain] = child_transform;

				// Compute required parent rotation that moves ik transform closer to target transform:
				TransformComponent* parent_transform = transforms.GetComponent(parent_entity);
				const XMVECTOR parent_pos = parent_transform->GetPositionV();
				const XMVECTOR dir_parent_to_ik = XMVector3Normalize(XMVectorSubtract(transform->GetPositionV(), parent_pos));
				const XMVECTOR dir_parent_to_target = XMVector3Normalize(XMVectorSubtract(target_pos, parent_pos));
				const XMVECTOR axis = XMVector3Normalize(XMVector3Cross(dir_parent_to_ik, dir_parent_to_target));
				const float angle = XMScalarACos(XMVectorGetX(XMVector3Dot(dir_parent_to_ik, dir_parent_to_target)));
				const XMVECTOR Q = XMQuaternionNormalize(XMQuaternionRotationNormal(axis, angle));

				// parent to world space:
				parent_transform->ApplyTransform();
				// rotate parent:
				parent_transform->Rotate(Q);
				parent_transform->UpdateTransform();
				// parent back to local space (if parent has parent):
				const HierarchyComponent* hier_parent = hierarchy.GetComponent(parent_entity);
				if (hier_parent != nullptr)
				{
					Entity parent_of_parent_entity = hier_parent->parentID;
					const TransformComponent* transform_parent_of_parent = transforms.GetComponent(parent_of_parent_entity);
					XMMATRIX parent_of_parent_inverse = XMMatrixInverse(nullptr, XMLoadFloat4x4(&transform_parent_of_parent->world));
					parent_transform->MatrixTransform(parent_of_parent_inverse);
					// Do not call UpdateTransform() here, to keep parent world matrix in world space!
				}

				// update chain from parent to children:
				const TransformComponent* recurse_parent = parent_transform;
				for (int recurse_chain = (int)chain; recurse_chain >= 0; --recurse_chain)
				{
					stack[recurse_chain]->UpdateTransform_Parented(*recurse_parent);
					recurse_parent = stack[recurse_chain];
				}

				if (hier_parent == nullptr)
				{
					// chain root reached, exit
					break;
				}

				// move up in the chain by one:
				child_transform = parent_transform;
				parent_entity = hier_parent->parentID;
				assert(chain < (uint32_t)arraysize(stack) - 1); // if this is encountered, just extend stack array size

			}
		}
	}

	if (recompute_hierarchy)
	{
		// (**)If there was IK, we need to recompute transform hierarchy. This is only necessary for transforms that have parent
		//	transforms that are IK. Because the IK chain is computed from child to parent upwards, IK that have child would not update
		//	its transform properly in some cases (such as if animation writes to that child)
		for (size_t i = 0; i < hierarchy.GetCount(); ++i)
		{
			const HierarchyComponent& parentcomponent = hierarchy[i];
			Entity entity = hierarchy.GetEntity(i);

			TransformComponent* transform_child = transforms.GetComponent(entity);
			TransformComponent* transform_parent = transforms.GetComponent(parentcomponent.parentID);
			if (transform_child != nullptr && transform_parent != nullptr)
			{
				transform_child->UpdateTransform_Parented(*transform_parent);
			}
		}
	}
}

void wiScene::SoftBodyPhysicsComponent::CreateFromMesh(const MeshComponent& mesh)
{
	vertex_positions_simulation.resize(mesh.vertex_positions.size());

	XMMATRIX W = XMLoadFloat4x4(&worldMatrix);
	XMFLOAT3 _min = XMFLOAT3(FLT_MAX, FLT_MAX, FLT_MAX);
	XMFLOAT3 _max = XMFLOAT3(-FLT_MAX, -FLT_MAX, -FLT_MAX);
	for (size_t i = 0; i < vertex_positions_simulation.size(); ++i)
	{
		XMFLOAT3 pos = mesh.vertex_positions[i];
		XMStoreFloat3(&pos, XMVector3Transform(XMLoadFloat3(&pos), W));
		XMFLOAT3 nor = mesh.vertex_normals.empty() ? XMFLOAT3(1, 1, 1) : mesh.vertex_normals[i];
		XMStoreFloat3(&nor, XMVector3Normalize(XMVector3TransformNormal(XMLoadFloat3(&nor), W)));
		const uint8_t wind = mesh.vertex_windweights.empty() ? 0xFF : mesh.vertex_windweights[i];
		vertex_positions_simulation[i].FromFULL(pos, nor, wind);
		_min = wiMath::Min(_min, pos);
		_max = wiMath::Max(_max, pos);
	}
	aabb = AABB(_min, _max);

	if (physicsToGraphicsVertexMapping.empty())
	{
		// Create a mapping that maps unique vertex positions to all vertex indices that share that. Unique vertex positions will make up the physics mesh:
		std::unordered_map<size_t, uint32_t> uniquePositions;
		graphicsToPhysicsVertexMapping.resize(mesh.vertex_positions.size());
		physicsToGraphicsVertexMapping.clear();
		weights.clear();

		for (size_t i = 0; i < mesh.vertex_positions.size(); ++i)
		{
			const XMFLOAT3& position = mesh.vertex_positions[i];

			size_t hashes[] = {
				std::hash<float>{}(position.x),
				std::hash<float>{}(position.y),
				std::hash<float>{}(position.z),
			};
			size_t vertexHash = (((hashes[0] ^ (hashes[1] << 1) >> 1) ^ (hashes[2] << 1)) >> 1);

			if (uniquePositions.count(vertexHash) == 0)
			{
				uniquePositions[vertexHash] = (uint32_t)physicsToGraphicsVertexMapping.size();
				physicsToGraphicsVertexMapping.push_back((uint32_t)i);
			}
			graphicsToPhysicsVertexMapping[i] = uniquePositions[vertexHash];
		}

		weights.resize(physicsToGraphicsVertexMapping.size());
		std::fill(weights.begin(), weights.end(), 1.0f);
	}
}

void wiScene::Scene::DebugDrawPhysicsWorld()
{
	physicsEngine->dynamicsWorld->debugDrawWorld();
}

void wiScene::Scene::InitPhysicsEngine()
{
	physicsEngine = new PhysicsEngine;
	// collision configuration contains default setup for memory, collision setup. Advanced users can create their own configuration.
	physicsEngine->collisionConfiguration = std::make_unique<btSoftBodyRigidBodyCollisionConfiguration>();

	// use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
	physicsEngine->dispatcher = std::make_unique<btCollisionDispatcher>(physicsEngine->collisionConfiguration.get());

	// btDbvtBroadphase is a good general purpose broadphase. You can also try out btAxis3Sweep.
	physicsEngine->overlappingPairCache = std::make_unique<btDbvtBroadphase>();

	// the default constraint solver. For parallel processing you can use a different solver (see Extras/BulletMultiThreaded)
	physicsEngine->solver = std::make_unique<btSequentialImpulseConstraintSolver>();

	physicsEngine->dynamicsWorld = std::make_unique<btSoftRigidDynamicsWorld>(physicsEngine->dispatcher.get(), physicsEngine->overlappingPairCache.get(), physicsEngine->solver.get(), physicsEngine->collisionConfiguration.get());

	physicsEngine->dynamicsWorld->getSolverInfo().m_solverMode |= SOLVER_RANDMIZE_ORDER;
	physicsEngine->dynamicsWorld->getDispatchInfo().m_enableSatConvex = true;
	physicsEngine->dynamicsWorld->getSolverInfo().m_splitImpulse = true;

	physicsEngine->dynamicsWorld->setGravity(physicsEngine->gravity);

	physicsEngine->dynamicsWorld->setDebugDrawer(new BulletDebugDrawer());

	btSoftRigidDynamicsWorld* softRigidWorld = (btSoftRigidDynamicsWorld*)physicsEngine->dynamicsWorld.get();
	btSoftBodyWorldInfo& softWorldInfo = softRigidWorld->getWorldInfo();
	softWorldInfo.air_density = btScalar(1.2f);
	softWorldInfo.water_density = 0;
	softWorldInfo.water_offset = 0;
	softWorldInfo.water_normal = btVector3(0, 0, 0);
	softWorldInfo.m_gravity.setValue(physicsEngine->gravity.x(), physicsEngine->gravity.y(), physicsEngine->gravity.z());
	softWorldInfo.m_sparsesdf.Initialize();

	wiBackLog::post("wiPhysicsEngine_Bullet Initialized");
}

void wiScene::Scene::DestroyPhysicsEngine()
{
	if (physicsEngine)
	{
		if (physicsEngine->servoMotorConstraint)
		{
			physicsEngine->dynamicsWorld->removeConstraint(physicsEngine->servoMotorConstraint);
			physicsEngine->servoMotorConstraint = nullptr;
		}
		delete physicsEngine;
	}

	physicsEngine = nullptr;
}

void wiScene::Scene::PhysicsEngine::AddRigidBody(Entity entity, wiScene::RigidBodyPhysicsComponent& physicscomponent, const wiScene::MeshComponent& mesh, const wiScene::TransformComponent& transform)
{
	btVector3 S(transform.scale_local.x, transform.scale_local.y, transform.scale_local.z);

	btCollisionShape* shape = nullptr;

	switch (physicscomponent.shape)
	{
		case RigidBodyPhysicsComponent::CollisionShape::BOX:
			shape = new btBoxShape(S);
			break;

		case RigidBodyPhysicsComponent::CollisionShape::SPHERE:
			shape = new btSphereShape(btScalar(S.x()));
			break;

		case RigidBodyPhysicsComponent::CollisionShape::CAPSULE:
			shape = new btCapsuleShape(btScalar(S.x()), btScalar(S.y()));
			break;

		case RigidBodyPhysicsComponent::CollisionShape::CONVEX_HULL:
		{
			shape = new btConvexHullShape();
			for (auto& pos : mesh.vertex_positions)
			{
				((btConvexHullShape*)shape)->addPoint(btVector3(pos.x, pos.y, pos.z));
			}
			shape->setLocalScaling(S);
		}
		break;

		case RigidBodyPhysicsComponent::CollisionShape::TRIANGLE_MESH:
		{
			int totalVerts = (int)mesh.vertex_positions.size();
			int totalTriangles = (int)mesh.indices.size() / 3;

			btVector3* btVerts = new btVector3[totalVerts];
			size_t i = 0;
			for (auto& pos : mesh.vertex_positions)
			{
				btVerts[i++] = btVector3(pos.x, pos.y, pos.z);
			}

			int* btInd = new int[mesh.indices.size()];
			i = 0;
			for (auto& ind : mesh.indices)
			{
				btInd[i++] = ind;
			}

			int vertStride = sizeof(btVector3);
			int indexStride = 3 * sizeof(int);

			btTriangleIndexVertexArray* indexVertexArrays = new btTriangleIndexVertexArray(
				totalTriangles,
				btInd,
				indexStride,
				totalVerts,
				(btScalar*)&btVerts[0].x(),
				vertStride
			);

			bool useQuantizedAabbCompression = true;
			shape = new btBvhTriangleMeshShape(indexVertexArrays, useQuantizedAabbCompression);
			shape->setLocalScaling(S);
		}
		break;
	}

	if (shape != nullptr)
	{
		// Use default margin for now
		//shape->setMargin(btScalar(0.01));

		btScalar mass = physicscomponent.mass;

		bool isDynamic = (mass != 0.f && !physicscomponent.IsKinematic());

		btVector3 localInertia(0, 0, 0);
		if (isDynamic)
		{
			shape->calculateLocalInertia(mass, localInertia);
		}
		else
		{
			mass = 0;
		}

		//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
		btTransform shapeTransform;
		shapeTransform.setIdentity();
		shapeTransform.setOrigin(btVector3(transform.translation_local.x, transform.translation_local.y, transform.translation_local.z));
		shapeTransform.setRotation(btQuaternion(transform.rotation_local.x, transform.rotation_local.y, transform.rotation_local.z, transform.rotation_local.w));
		btDefaultMotionState* myMotionState = new btDefaultMotionState(shapeTransform);

		btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, shape, localInertia);
		//rbInfo.m_friction = physicscomponent.friction;
		//rbInfo.m_restitution = physicscomponent.restitution;
		//rbInfo.m_linearDamping = physicscomponent.damping;
		//rbInfo.m_angularDamping = physicscomponent.damping;

		btRigidBody* rigidbody = new btRigidBody(rbInfo);
		rigidbody->setUserIndex(entity);

		if (physicscomponent.IsKinematic())
		{
			rigidbody->setCollisionFlags(rigidbody->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);
		}

		if (physicscomponent.IsDisableDeactivation())
		{
			rigidbody->setActivationState(DISABLE_DEACTIVATION);
		}

		dynamicsWorld->addRigidBody(rigidbody);
		physicscomponent.physicsobject = rigidbody;
	}
}

void wiScene::Scene::PhysicsEngine::AddSoftBody(Entity entity, wiScene::SoftBodyPhysicsComponent& physicscomponent, const wiScene::MeshComponent& mesh)
{
	physicscomponent.CreateFromMesh(mesh);

	XMMATRIX worldMatrix = XMLoadFloat4x4(&physicscomponent.worldMatrix);

	const int vCount = (int)physicscomponent.physicsToGraphicsVertexMapping.size();
	btScalar* btVerts = new btScalar[vCount * 3];
	for (int i = 0; i < vCount; ++i)
	{
		uint32_t graphicsInd = physicscomponent.physicsToGraphicsVertexMapping[i];

		XMFLOAT3 position = mesh.vertex_positions[graphicsInd];
		XMVECTOR P = XMLoadFloat3(&position);
		P = XMVector3Transform(P, worldMatrix);
		XMStoreFloat3(&position, P);

		btVerts[i * 3 + 0] = btScalar(position.x);
		btVerts[i * 3 + 1] = btScalar(position.y);
		btVerts[i * 3 + 2] = btScalar(position.z);
	}

	const int iCount = (int)mesh.indices.size();
	const int tCount = iCount / 3;
	int* btInd = new int[iCount];
	for (int i = 0; i < iCount; ++i)
	{
		uint32_t ind = mesh.indices[i];
		uint32_t mappedIndex = physicscomponent.graphicsToPhysicsVertexMapping[ind];
		btInd[i] = (int)mappedIndex;
	}

	btSoftBody* softbody = btSoftBodyHelpers::CreateFromTriMesh(
		((btSoftRigidDynamicsWorld*)dynamicsWorld.get())->getWorldInfo()
		, btVerts
		, btInd
		, tCount
		, false
	);
	delete[] btVerts;
	delete[] btInd;

	if (softbody)
	{
		softbody->setUserIndex(entity);

		//btSoftBody::Material* pm = softbody->appendMaterial();
		btSoftBody::Material* pm = softbody->m_materials[0];
		pm->m_kLST = btScalar(0.9f);
		pm->m_kVST = btScalar(0.9f);
		pm->m_kAST = btScalar(0.9f);
		pm->m_flags = 0;
		softbody->generateBendingConstraints(2, pm);
		softbody->randomizeConstraints();

		softbody->m_cfg.piterations = softbodyIterationCount;
		softbody->m_cfg.aeromodel = btSoftBody::eAeroModel::F_TwoSidedLiftDrag;

		softbody->m_cfg.kAHR = btScalar(.69); //0.69		Anchor hardness  [0,1]
		softbody->m_cfg.kCHR = btScalar(1.0); //1			Rigid contact hardness  [0,1]
		softbody->m_cfg.kDF = btScalar(0.2); //0.2			Dynamic friction coefficient  [0,1]
		softbody->m_cfg.kDG = btScalar(0.01); //0			Drag coefficient  [0,+inf]
		softbody->m_cfg.kDP = btScalar(0.0); //0			Damping coefficient  [0,1]
		softbody->m_cfg.kKHR = btScalar(0.1); //0.1			Kinetic contact hardness  [0,1]
		softbody->m_cfg.kLF = btScalar(0.1); //0			Lift coefficient  [0,+inf]
		softbody->m_cfg.kMT = btScalar(0.0); //0			Pose matching coefficient  [0,1]
		softbody->m_cfg.kPR = btScalar(0.0); //0			Pressure coefficient  [-1,1]
		softbody->m_cfg.kSHR = btScalar(1.0); //1			Soft contacts hardness  [0,1]
		softbody->m_cfg.kVC = btScalar(0.0); //0			Volume conseration coefficient  [0,+inf]
		softbody->m_cfg.kVCF = btScalar(1.0); //1			Velocities correction factor (Baumgarte)

		softbody->m_cfg.kSKHR_CL = btScalar(1.0); //1			Soft vs. kinetic hardness   [0,1]
		softbody->m_cfg.kSK_SPLT_CL = btScalar(0.5); //0.5		Soft vs. rigid impulse split  [0,1]
		softbody->m_cfg.kSRHR_CL = btScalar(0.1); //0.1			Soft vs. rigid hardness  [0,1]
		softbody->m_cfg.kSR_SPLT_CL = btScalar(0.5); //0.5		Soft vs. rigid impulse split  [0,1]
		softbody->m_cfg.kSSHR_CL = btScalar(0.5); //0.5			Soft vs. soft hardness  [0,1]
		softbody->m_cfg.kSS_SPLT_CL = btScalar(0.5); //0.5		Soft vs. rigid impulse split  [0,1]

		for (size_t i = 0; i < physicscomponent.physicsToGraphicsVertexMapping.size(); ++i)
		{
			float weight = physicscomponent.weights[i];
			softbody->setMass((int)i, weight);
		}
		softbody->setTotalMass(physicscomponent.mass); // this must be AFTER softbody->setMass(), so that weights will be averaged

		if (physicscomponent.IsDisableDeactivation())
		{
			softbody->setActivationState(DISABLE_DEACTIVATION);
		}

		softbody->setPose(true, true);

		((btSoftRigidDynamicsWorld*)dynamicsWorld.get())->addSoftBody(softbody);
		physicscomponent.physicsobject = softbody;
	}
}

void wiScene::Scene::RunPhysicsUpdateSystem(wiJobSystem::context& ctx, float dt)
{
	auto range = wiProfiler::BeginRangeCPU("Physics");

	btVector3 wind = btVector3(weather->windDirection.x, weather->windDirection.y, weather->windDirection.z);

	// System will register rigidbodies to objects, and update physics engine state for kinematics:
	wiJobSystem::Dispatch(ctx, (uint32_t)rigidbodies.GetCount(), 256, [&](wiJobArgs args)
	{
		RigidBodyPhysicsComponent& physicscomponent = rigidbodies[args.jobIndex];
		Entity entity = rigidbodies.GetEntity(args.jobIndex);

		if (physicscomponent.physicsobject == nullptr)
		{
			TransformComponent& transform = *transforms.GetComponent(entity);
			const ObjectComponent& object = *objects.GetComponent(entity);
			const MeshComponent& mesh = *meshes.GetComponent(object.meshID);
			physicsEngine->physicsLock.lock();
			physicsEngine->AddRigidBody(entity, physicscomponent, mesh, transform);
			physicsEngine->physicsLock.unlock();
		}

		if (physicscomponent.physicsobject != nullptr)
		{
			btRigidBody* rigidbody = (btRigidBody*)physicscomponent.physicsobject;

			int activationState = rigidbody->getActivationState();
			if (physicscomponent.IsDisableDeactivation())
			{
				activationState |= DISABLE_DEACTIVATION;
			}
			else
			{
				activationState &= ~DISABLE_DEACTIVATION;
			}
			rigidbody->setActivationState(activationState);

			// For kinematic object, system updates physics state, else the physics updates system state:
			if (physicscomponent.IsKinematic())
			{
				TransformComponent& transform = *transforms.GetComponent(entity);

				btMotionState* motionState = rigidbody->getMotionState();
				btTransform physicsTransform;

				XMFLOAT3 position = transform.GetPosition();
				XMFLOAT4 rotation = transform.GetRotation();
				btVector3 T(position.x, position.y, position.z);
				btQuaternion R(rotation.x, rotation.y, rotation.z, rotation.w);
				physicsTransform.setOrigin(T);
				physicsTransform.setRotation(R);
				motionState->setWorldTransform(physicsTransform);
			}
		}
	});

	// System will register softbodies to meshes and update physics engine state:
	wiJobSystem::Dispatch(ctx, (uint32_t)softbodies.GetCount(), 1, [&](wiJobArgs args)
	{
		SoftBodyPhysicsComponent& physicscomponent = softbodies[args.jobIndex];
		Entity entity = softbodies.GetEntity(args.jobIndex);
		MeshComponent& mesh = *meshes.GetComponent(entity);
		const ArmatureComponent* armature = mesh.IsSkinned() ? armatures.GetComponent(mesh.armatureID) : nullptr;
		mesh.SetDynamic(true);

		if (physicscomponent._flags & SoftBodyPhysicsComponent::FORCE_RESET)
		{
			physicscomponent._flags &= ~SoftBodyPhysicsComponent::FORCE_RESET;
			if (physicscomponent.physicsobject != nullptr)
			{
				((btSoftRigidDynamicsWorld*)physicsEngine->dynamicsWorld.get())->removeSoftBody((btSoftBody*)physicscomponent.physicsobject);
				physicscomponent.physicsobject = nullptr;
			}
		}
		if (physicscomponent._flags & SoftBodyPhysicsComponent::SAFE_TO_REGISTER && physicscomponent.physicsobject == nullptr)
		{
			physicsEngine->physicsLock.lock();
			physicsEngine->AddSoftBody(entity, physicscomponent, mesh);
			physicsEngine->physicsLock.unlock();
		}

		if (physicscomponent.physicsobject != nullptr)
		{
			btSoftBody* softbody = (btSoftBody*)physicscomponent.physicsobject;
			softbody->m_cfg.kDF = physicscomponent.friction;
			softbody->setWindVelocity(wind);

			// This is different from rigid bodies, because soft body is a per mesh component (no TransformComponent). World matrix is propagated down from single mesh instance (ObjectUpdateSystem).
			XMMATRIX worldMatrix = XMLoadFloat4x4(&physicscomponent.worldMatrix);

			// System controls zero weight soft body nodes:
			for (size_t ind = 0; ind < physicscomponent.weights.size(); ++ind)
			{
				float weight = physicscomponent.weights[ind];

				if (weight == 0)
				{
					btSoftBody::Node& node = softbody->m_nodes[(uint32_t)ind];
					uint32_t graphicsInd = physicscomponent.physicsToGraphicsVertexMapping[ind];
					XMFLOAT3 position = mesh.vertex_positions[graphicsInd];
					XMVECTOR P = armature == nullptr ? XMLoadFloat3(&position) : wiScene::SkinVertex(mesh, *armature, graphicsInd);
					P = XMVector3Transform(P, worldMatrix);
					XMStoreFloat3(&position, P);
					node.m_x = btVector3(position.x, position.y, position.z);
				}
			}
		}
	});

	// System will register flexi bone chains
	wiJobSystem::Dispatch(ctx, (uint32_t)flexiChains.GetCount(), 1, [&](wiJobArgs args)
	{
		FlexiBoneChainComponent& boneChain = flexiChains[args.jobIndex];
		Entity entity = flexiChains.GetEntity(args.jobIndex);

		if (boneChain.bones.size() != boneChain.physicsBodies.size())
		{
			boneChain.physicsBodies.resize(boneChain.bones.size(), nullptr);
			boneChain.boneLengths.resize(boneChain.bones.size(), 0.0f);

			for (int i = 0; i < (int)boneChain.bones.size() - 1; ++i)
			{
				wiECS::Entity parent = boneChain.bones[i];
				wiECS::Entity child = boneChain.bones[i + 1];

				const wiScene::TransformComponent* tmParent = transforms.GetComponent(parent);
				const wiScene::TransformComponent* tmChild = transforms.GetComponent(child);
				assert(tmParent && tmChild);

				XMVECTOR childBonePos = tmChild->GetPositionV();
				XMVECTOR parentBonePos = tmParent->GetPositionV();
				XMVECTOR boneVec = XMVectorSubtract(childBonePos, parentBonePos);

				const float boneVecLen = XMVectorGetX(XMVector3Length(boneVec));
				boneChain.boneLengths[i] = boneVecLen;

				const float capsuleHeight = boneVecLen - 2.0f * boneChain.capsuleRadius;
				assert(capsuleHeight > 0.0f);

				XMVECTOR capsulePosV = XMVectorAdd(parentBonePos, XMVectorScale(boneVec, 0.5f));
				XMFLOAT3 capsulePos;
				XMStoreFloat3(&capsulePos, capsulePosV);

				XMFLOAT3 position = tmParent->GetPosition();
				XMFLOAT4 rotation = tmParent->GetRotation();
				btVector3 T(capsulePos.x, capsulePos.y, capsulePos.z);
				btQuaternion R(rotation.x, rotation.y, rotation.z, rotation.w);

				btCapsuleShape* capsuleShape = new btCapsuleShape(boneChain.capsuleRadius, capsuleHeight);

				const float mass = 1;

				btVector3 localInertia(0, 0, 0);
				capsuleShape->calculateLocalInertia(mass, localInertia);

				btTransform capsuleTransform;
				capsuleTransform.setIdentity();
				capsuleTransform.setRotation(R);
				capsuleTransform.setOrigin(T);

				// create the right body
				btDefaultMotionState* myMotionState = new btDefaultMotionState(capsuleTransform);

				btRigidBody::btRigidBodyConstructionInfo rbInfo(1, myMotionState, capsuleShape, localInertia);
				btRigidBody* rigidbody = new btRigidBody(rbInfo);

				rigidbody->setActivationState(DISABLE_DEACTIVATION);
				physicsEngine->dynamicsWorld->addRigidBody(rigidbody);

				boneChain.physicsBodies[i] = rigidbody;
			}

			// Setup constraints
			for (int i = 0; i < (int)boneChain.constraints.size(); ++i)
			{
				const float boneLen = boneChain.boneLengths[i];

				btTransform localA, localB;
				localA.setIdentity();
				localB.setIdentity();
				localA.setOrigin(btVector3(0.0f, btScalar(boneLen * 0.5f), 0.0f));
				localB.setOrigin(btVector3(0.0f, btScalar(-boneLen * 0.5f), 0.0f));

				const wiScene::FlexiBoneChainComponent::ConstraintParams& p = boneChain.constraints[i];

				btRigidBody* bodyA = reinterpret_cast<btRigidBody*>(boneChain.physicsBodies[i + 0]);
				btRigidBody* bodyB = reinterpret_cast<btRigidBody*>(boneChain.physicsBodies[i + 1]);

				btTypedConstraint* constr = nullptr;
#if false
				auto constrCone = new btConeTwistConstraint(*bodyA, *bodyB, localA, localB);

				constrCone->setLimit(0.1f,
								 0.1f,
								 0,
								 p.softness,
								 p.biasFactor,
								 p.relaxationFactor);
#else
				auto constr6Dof = new btGeneric6DofSpring2Constraint(*bodyA, *bodyB, localA, localB);
				constr6Dof->setLimit(0, 0, 0);
				constr6Dof->setLimit(1, 0, 0);
				constr6Dof->setLimit(2, 0, 0);
				constr6Dof->setLimit(3, -1, 1);
				constr6Dof->setLimit(4, 0, 0);
				constr6Dof->setLimit(5, 0, 0);
//				constr6Dof->enableSpring(3, true);
//				void setStiffness(int index, btScalar stiffness, bool limitIfNeeded = true);  // if limitIfNeeded is true the system will automatically limit the stiffness in necessary situations where otherwise the spring would move unrealistically too widely
//				void setDamping(int index, btScalar damping, bool limitIfNeeded = true);      // if limitIfNeeded is true the system will automatically limit the damping in necessary situations where otherwise the spring would blow up

				constr = constr6Dof;
#endif
				physicsEngine->dynamicsWorld->addConstraint(constr, true);

				if (i == 0)
				{
					{
						auto servoMotorBody = reinterpret_cast<btRigidBody*>(boneChain.physicsBodies[0]);//new btRigidBody(mass, motionState, shape, localInertia);

						localA.setIdentity();

						btGeneric6DofSpring2Constraint* constraint = new btGeneric6DofSpring2Constraint(*servoMotorBody, localB);
						constraint->setLimit(0, 0, 0);
						constraint->setLimit(1, 0, 0);
						constraint->setLimit(2, 0, 0);
						constraint->setLimit(3, 1, -1);
						constraint->setLimit(4, 0, 0);
						constraint->setLimit(5, 0, 0);
						constraint->enableMotor(3, true);
						constraint->setTargetVelocity(3, 3.f);
						constraint->setMaxMotorForce(3, 600.f);
						constraint->setServo(3, true);
						constraint->setServoTarget(3, M_PI_2);
						constraint->setDbgDrawSize(btScalar(2.f));

						physicsEngine->dynamicsWorld->addConstraint(constraint, true);
						physicsEngine->servoMotorConstraint = constraint;
					}
				}
			}
		}
	});

	wiJobSystem::Wait(ctx);

	//////////////////////debug testing
	if (physicsEngine->servoMotorConstraint)
	{
		static float servoNextFrame = -1;
		if (servoNextFrame < 0)
		{
			physicsEngine->servoMotorConstraint->getRotationalLimitMotor(0)->m_servoTarget *= -1;
			servoNextFrame = 3.0;
		}
		servoNextFrame -= dt;
	}
	///////////////////////

	// Perform internal simulation step:
	physicsEngine->dynamicsWorld->stepSimulation(dt, 10);

	// Feedback physics engine state to system:
	for (int i = 0; i < physicsEngine->dynamicsWorld->getCollisionObjectArray().size(); ++i)
	{
		btCollisionObject* collisionobject = physicsEngine->dynamicsWorld->getCollisionObjectArray()[i];
		Entity entity = (Entity)collisionobject->getUserIndex();

		btRigidBody* rigidbody = btRigidBody::upcast(collisionobject);
		if (false)//rigidbody != nullptr) <---- this will fuck up our flexi bone chains
		{
			RigidBodyPhysicsComponent* physicscomponent = rigidbodies.GetComponent(entity);
			if (physicscomponent == nullptr)
			{
				physicsEngine->dynamicsWorld->removeRigidBody(rigidbody);
				i--;
				continue;
			}

			// Feedback non-kinematic objects to system:
			if (!physicscomponent->IsKinematic())
			{
				TransformComponent& transform = *transforms.GetComponent(entity);

				btMotionState* motionState = rigidbody->getMotionState();
				btTransform physicsTransform;

				motionState->getWorldTransform(physicsTransform);
				btVector3 T = physicsTransform.getOrigin();
				btQuaternion R = physicsTransform.getRotation();

				transform.translation_local = XMFLOAT3(T.x(), T.y(), T.z());
				transform.rotation_local = XMFLOAT4(R.x(), R.y(), R.z(), R.w());
				transform.SetDirty();
			}
		}
		else
		{
			btSoftBody* softbody = btSoftBody::upcast(collisionobject);

			if (softbody != nullptr)
			{
				SoftBodyPhysicsComponent* physicscomponent = softbodies.GetComponent(entity);
				if (physicscomponent == nullptr)
				{
					((btSoftRigidDynamicsWorld*)physicsEngine->dynamicsWorld.get())->removeSoftBody(softbody);
					i--;
					continue;
				}

				MeshComponent& mesh = *meshes.GetComponent(entity);

				// System mesh aabb will be queried from physics engine soft body:
				btVector3 aabb_min;
				btVector3 aabb_max;
				softbody->getAabb(aabb_min, aabb_max);
				physicscomponent->aabb = AABB(XMFLOAT3(aabb_min.x(), aabb_min.y(), aabb_min.z()), XMFLOAT3(aabb_max.x(), aabb_max.y(), aabb_max.z()));

				// Soft body simulation nodes will update graphics mesh:
				for (size_t ind = 0; ind < physicscomponent->vertex_positions_simulation.size(); ++ind)
				{
					uint32_t physicsInd = physicscomponent->graphicsToPhysicsVertexMapping[ind];
					float weight = physicscomponent->weights[physicsInd];

					btSoftBody::Node& node = softbody->m_nodes[physicsInd];

					MeshComponent::Vertex_POS& vertex = physicscomponent->vertex_positions_simulation[ind];
					vertex.pos.x = node.m_x.getX();
					vertex.pos.y = node.m_x.getY();
					vertex.pos.z = node.m_x.getZ();

					XMFLOAT3 normal;
					normal.x = -node.m_n.getX();
					normal.y = -node.m_n.getY();
					normal.z = -node.m_n.getZ();
					vertex.MakeFromParams(normal);
				}
			}
		}
	}

	wiProfiler::EndRange(range); // Physics
}

void wiScene::Scene::RunForceUpdateSystem(wiJobSystem::context& ctx)
{
	wiJobSystem::Dispatch(ctx, (uint32_t)forces.GetCount(), small_subtask_groupsize, [&](wiJobArgs args)
	{
		ForceFieldComponent& force = forces[args.jobIndex];
		Entity entity = forces.GetEntity(args.jobIndex);
		const TransformComponent& transform = *transforms.GetComponent(entity);

		XMMATRIX W = XMLoadFloat4x4(&transform.world);
		XMVECTOR S, R, T;
		XMMatrixDecompose(&S, &R, &T, W);

		XMStoreFloat3(&force.position, T);
		XMStoreFloat3(&force.direction, XMVector3Normalize(XMVector3TransformNormal(XMVectorSet(0, -1, 0, 0), W)));

		force.range_global = force.range_local * std::max(XMVectorGetX(S), std::max(XMVectorGetY(S), XMVectorGetZ(S)));
	});
}
