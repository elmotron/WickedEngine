#pragma once


//#include "wiEnums.h"

#include "SceneAnimation.h"
#include "SceneAudio.h"
#include "SceneHierarchy.h"
#include "SceneGraphics.h"
#include "SceneMisc.h"
#include "ScenePhysics.h"

#include "wiHairParticle.h"
#include "wiEmittedParticle.h"

//#include "wiEmittedParticle.h"
//#include "wiHairParticle.h"
//#include "ShaderInterop_Renderer.h"
//#include "wiJobSystem.h"
//#include "wiAudio.h"
//#include "wiRenderer.h"
//#include "wiResourceManager.h"
#include "wiECS.h"
#include "wiEnums.h"
#include "wiIntersect.h"
#include "wiGraphics.h"
#include "wiSpinLock.h"
#include "wiScene_Decl.h"
#include "CommonInclude.h"

//#include <string>
//#include <vector>
//#include <memory>

class wiArchive;

namespace wiJobSystem
{
	struct context;
}

namespace wiScene
{
	struct PickResult
	{
		wiECS::Entity entity = wiECS::INVALID_ENTITY;
		XMFLOAT3 position = XMFLOAT3(0, 0, 0);
		XMFLOAT3 normal = XMFLOAT3(0, 0, 0);
		float distance = FLT_MAX;
		int subsetIndex = -1;
		int vertexID0 = -1;
		int vertexID1 = -1;
		int vertexID2 = -1;
		XMFLOAT2 bary = XMFLOAT2(0, 0);
		XMFLOAT4X4 orientation = IDENTITYMATRIX;

		bool operator==(const PickResult& other)
		{
			return entity == other.entity;
		}
	};

	struct IntersectSphereResult
	{
		wiECS::Entity entity = wiECS::INVALID_ENTITY;
		XMFLOAT3 position = XMFLOAT3(0, 0, 0);
		XMFLOAT3 normal = XMFLOAT3(0, 0, 0);
		float depth = 0;
	};

	class Scene
	{
	public:
		wiECS::ComponentManager<NameComponent> names;
		wiECS::ComponentManager<LayerComponent> layers;
		wiECS::ComponentManager<TransformComponent> transforms;
		wiECS::ComponentManager<PreviousFrameTransformComponent> prev_transforms;
		wiECS::ComponentManager<HierarchyComponent> hierarchy;
		wiECS::ComponentManager<MaterialComponent> materials;
		wiECS::ComponentManager<MeshComponent> meshes;
		wiECS::ComponentManager<ImpostorComponent> impostors;
		wiECS::ComponentManager<ObjectComponent> objects;
		wiECS::ComponentManager<AABB> aabb_objects;
		wiECS::ComponentManager<RigidBodyPhysicsComponent> rigidbodies;
		wiECS::ComponentManager<SoftBodyPhysicsComponent> softbodies;
		wiECS::ComponentManager<ArmatureComponent> armatures;
		wiECS::ComponentManager<LightComponent> lights;
		wiECS::ComponentManager<AABB> aabb_lights;
		wiECS::ComponentManager<CameraComponent> cameras;
		wiECS::ComponentManager<EnvironmentProbeComponent> probes;
		wiECS::ComponentManager<AABB> aabb_probes;
		wiECS::ComponentManager<ForceFieldComponent> forces;
		wiECS::ComponentManager<DecalComponent> decals;
		wiECS::ComponentManager<AABB> aabb_decals;
		wiECS::ComponentManager<AnimationComponent> animations;
		wiECS::ComponentManager<AnimationDataComponent> animation_datas;
		wiECS::ComponentManager<wiEmittedParticle> emitters;
		wiECS::ComponentManager<wiHairParticle> hairs;
		wiECS::ComponentManager<WeatherComponent> weathers;
		wiECS::ComponentManager<SoundComponent> sounds;
		wiECS::ComponentManager<InverseKinematicsComponent> inverse_kinematics;
		wiECS::ComponentManager<SpringComponent> springs;
		wiECS::ComponentManager<FlexiBoneChainComponent> flexiChains;

		// Non-serialized attributes:
		wiSpinLock locker;
		AABB bounds;
		std::vector<AABB> parallel_bounds;
		WeatherComponent* weather = nullptr;
		wiGraphics::RaytracingAccelerationStructure TLAS;
		wiGraphics::DescriptorTable descriptorTable;

		enum DESCRIPTORTABLE_ENTRY
		{
			DESCRIPTORTABLE_ENTRY_SUBSETS_MATERIAL,
			DESCRIPTORTABLE_ENTRY_SUBSETS_TEXTURE_BASECOLOR,
			DESCRIPTORTABLE_ENTRY_SUBSETS_INDEXBUFFER,
			DESCRIPTORTABLE_ENTRY_SUBSETS_VERTEXBUFFER_UV0,
			DESCRIPTORTABLE_ENTRY_SUBSETS_VERTEXBUFFER_UV1,

			DESCRIPTORTABLE_ENTRY_COUNT
		};
		std::atomic<uint32_t> geometryOffset;

		Scene();
		~Scene();

		// Update all components by a given timestep (in seconds):
		void Update(float dt);
		// Remove everything from the scene that it owns:
		void Clear();
		// Merge with an other scene.
		void Merge(Scene& other);

		// Removes a specific entity from the scene (if it exists):
		void Entity_Remove(wiECS::Entity entity);
		// Finds the first entity by the name (if it exists, otherwise returns INVALID_ENTITY):
		wiECS::Entity Entity_FindByName(const std::string& name);
		// Duplicates all of an entity's components and creates a new entity with them:
		wiECS::Entity Entity_Duplicate(wiECS::Entity entity);
		// Serializes entity and all of its components to archive:
		//	You can specify entity = INVALID_ENTITY when the entity needs to be created from archive
		//	You can specify seed = INVALID_ENTITY when the archive is guaranteed to be storing persistent and unique entities
		//	propagateDeepSeed : request that entity references inside components should be seeded as well
		//	Returns either the new entity that was read, or the original entity that was written
		wiECS::Entity Entity_Serialize(wiArchive& archive, wiECS::Entity entity = wiECS::INVALID_ENTITY, wiECS::Entity seed = wiECS::INVALID_ENTITY, bool propagateSeedDeep = true);

		wiECS::Entity Entity_CreateMaterial(const std::string& name);
		wiECS::Entity Entity_CreateObject(const std::string& name);
		wiECS::Entity Entity_CreateMesh(const std::string& name);
		wiECS::Entity Entity_CreateLight(
			const std::string& name, 
			const XMFLOAT3& position = XMFLOAT3(0, 0, 0), 
			const XMFLOAT3& color = XMFLOAT3(1, 1, 1), 
			float energy = 1, 
			float range = 10
		);
		wiECS::Entity Entity_CreateForce(const std::string& name, const XMFLOAT3& position = XMFLOAT3(0, 0, 0));
		wiECS::Entity Entity_CreateEnvironmentProbe(const std::string& name, const XMFLOAT3& position = XMFLOAT3(0, 0, 0));
		wiECS::Entity Entity_CreateDecal(const std::string& name, const std::string& textureName, const std::string& normalMapName = "");
		wiECS::Entity Entity_CreateCamera(
			const std::string& name, float width, float height, 
			float nearPlane = 0.01f, float farPlane = 1000.0f, float fov = XM_PIDIV4);
		wiECS::Entity Entity_CreateEmitter(const std::string& name, const XMFLOAT3& position = XMFLOAT3(0, 0, 0));
		wiECS::Entity Entity_CreateHair(const std::string& name, const XMFLOAT3& position = XMFLOAT3(0, 0, 0));
		wiECS::Entity Entity_CreateSound(const std::string& name, const std::string& filename, const XMFLOAT3& position = XMFLOAT3(0, 0, 0));

		// Attaches an entity to a parent:
		//	child_already_in_local_space	:	child won't be transformed from world space to local space
		void Component_Attach(wiECS::Entity entity, wiECS::Entity parent, bool child_already_in_local_space = false);
		// Detaches the entity from its parent (if it is attached):
		void Component_Detach(wiECS::Entity entity);
		// Detaches all children from an entity (if there are any):
		void Component_DetachChildren(wiECS::Entity parent);

		void Serialize(wiArchive& archive);

		void DebugDrawPhysicsWorld();

		// Given a ray, finds the closest intersection point against all mesh instances
		//	ray				:	the incoming ray that will be traced
		//	renderTypeMask	:	filter based on render type
		//	layerMask		:	filter based on layer
		//	scene			:	the scene that will be traced against the ray
		PickResult PickObject(const RAY& ray, uint32_t renderTypeMask = RENDERTYPE_OPAQUE, uint32_t layerMask = ~0) const;

		IntersectSphereResult IntersectSphere(const SPHERE& sphere, uint32_t renderTypeMask = RENDERTYPE_OPAQUE, uint32_t layerMask = ~0) const;
		IntersectSphereResult IntersectCapsule(const CAPSULE& capsule, uint32_t renderTypeMask = RENDERTYPE_OPAQUE, uint32_t layerMask = ~0) const;

		void DestroyPhysicsEngine();//TODO remove from here

	private:
		struct PhysicsEngine;

		PhysicsEngine* physicsEngine = nullptr;

		const uint32_t small_subtask_groupsize = 64;

	private:
		void InitPhysicsEngine();
		
		void RunPreviousFrameTransformUpdateSystem(wiJobSystem::context& ctx);
		void RunAnimationUpdateSystem(wiJobSystem::context& ctx, float dt);
		void RunTransformUpdateSystem(wiJobSystem::context& ctx);
		void RunHierarchyUpdateSystem(wiJobSystem::context& ctx);
		void RunSpringUpdateSystem(wiJobSystem::context& ctx, float dt);
		void RunInverseKinematicsUpdateSystem(wiJobSystem::context& ctx);
		void RunArmatureUpdateSystem(wiJobSystem::context& ctx);
		void RunMeshUpdateSystem(wiJobSystem::context& ctx);
		void RunMaterialUpdateSystem(wiJobSystem::context& ctx, float dt);
		void RunImpostorUpdateSystem(wiJobSystem::context& ctx);
		void RunObjectUpdateSystem(wiJobSystem::context& ctx);
		void RunCameraUpdateSystem(wiJobSystem::context& ctx);
		void RunDecalUpdateSystem(wiJobSystem::context& ctx);
		void RunProbeUpdateSystem(wiJobSystem::context& ctx);
		void RunPhysicsUpdateSystem(wiJobSystem::context& ctx, float dt);
		void RunForceUpdateSystem(wiJobSystem::context& ctx);
		void RunLightUpdateSystem(wiJobSystem::context& ctx);
		void RunParticleUpdateSystem(wiJobSystem::context& ctx, float dt);
		void RunWeatherUpdateSystem(wiJobSystem::context& ctx);
		void RunSoundUpdateSystem(wiJobSystem::context& ctx);
	};

	// Helper that manages a global scene
	inline Scene& GetScene()
	{
		static Scene scene;
		return scene;
	}

	// Helper function to open a wiscene file and add the contents to the specified scene. This is thread safe as it doesn't modify global scene
	//	scene			:	the scene that will contain the model
	//	fileName		:	file path
	//	transformMatrix	:	everything will be transformed by this matrix (optional)
	//	attached		:	everything will be attached to a base entity
	//
	//	returns INVALID_ENTITY if attached argument was false, else it returns the base entity handle
	wiECS::Entity LoadModel(const std::string& fileName, wiScene::Scene& scene, const XMMATRIX& transformMatrix = XMMatrixIdentity(), bool attached = false);
}
