#include "wiScene.h"
#include "wiArchive.h"
#include "wiMath.h"
#include "wiTextureHelper.h"
#include "wiResourceManager.h"
#include "wiEmittedParticle.h"
#include "wiHairParticle.h"
#include "wiHelper.h"
#include "wiJobSystem.h"
#include "wiRenderer.h"
#include "ShaderInterop_Renderer.h"

#include <functional>
#include <unordered_map>

using namespace wiECS;
using namespace wiGraphics;

wiScene::Scene::Scene()
{
	InitPhysicsEngine();
	weather = new WeatherComponent;
}

wiScene::Scene::~Scene()
{
	delete weather;
	DestroyPhysicsEngine();
}

void wiScene::Scene::Update(float dt)
{
	GraphicsDevice* device = wiRenderer::GetDevice();
	if (device->CheckCapability(GRAPHICSDEVICE_CAPABILITY_DESCRIPTOR_MANAGEMENT) && !descriptorTable.IsValid())
	{
		descriptorTable.resources.resize(DESCRIPTORTABLE_ENTRY_COUNT);
		descriptorTable.resources[DESCRIPTORTABLE_ENTRY_SUBSETS_MATERIAL] = { CONSTANTBUFFER, 0, MAX_DESCRIPTOR_INDEXING };
		descriptorTable.resources[DESCRIPTORTABLE_ENTRY_SUBSETS_TEXTURE_BASECOLOR] = { TEXTURE2D, 0, MAX_DESCRIPTOR_INDEXING };
		descriptorTable.resources[DESCRIPTORTABLE_ENTRY_SUBSETS_INDEXBUFFER] = { TYPEDBUFFER, MAX_DESCRIPTOR_INDEXING, MAX_DESCRIPTOR_INDEXING };
		descriptorTable.resources[DESCRIPTORTABLE_ENTRY_SUBSETS_VERTEXBUFFER_UV0] = { TYPEDBUFFER, MAX_DESCRIPTOR_INDEXING * 3, MAX_DESCRIPTOR_INDEXING };
		descriptorTable.resources[DESCRIPTORTABLE_ENTRY_SUBSETS_VERTEXBUFFER_UV1] = { TYPEDBUFFER, MAX_DESCRIPTOR_INDEXING * 4, MAX_DESCRIPTOR_INDEXING };

		bool success = device->CreateDescriptorTable(&descriptorTable);
		assert(success);
	}

	wiJobSystem::context ctx;

	RunPreviousFrameTransformUpdateSystem(ctx);
	RunAnimationUpdateSystem(ctx, dt);
	RunTransformUpdateSystem(ctx);

	wiJobSystem::Wait(ctx); // dependencies

	RunHierarchyUpdateSystem(ctx);
	RunSpringUpdateSystem(ctx, dt);
	RunInverseKinematicsUpdateSystem(ctx);
	RunArmatureUpdateSystem(ctx);
	RunMeshUpdateSystem(ctx);
	RunMaterialUpdateSystem(ctx, dt);
	RunImpostorUpdateSystem(ctx);
	RunWeatherUpdateSystem(ctx);
	RunPhysicsUpdateSystem(ctx, dt); // this syncs dependencies internally
	RunObjectUpdateSystem(ctx);
	RunCameraUpdateSystem(ctx);
	RunDecalUpdateSystem(ctx);
	RunProbeUpdateSystem(ctx);
	RunForceUpdateSystem(ctx);
	RunLightUpdateSystem(ctx);
	RunParticleUpdateSystem(ctx, dt);
	RunSoundUpdateSystem(ctx);

	wiJobSystem::Wait(ctx); // dependencies

	// Merge parallel bounds computation (depends on object update system):
	bounds = AABB();
	for (auto& group_bound : parallel_bounds)
	{
		bounds = AABB::Merge(bounds, group_bound);
	}

	if (device->CheckCapability(GRAPHICSDEVICE_CAPABILITY_RAYTRACING))
	{
		// Recreate top level acceleration structure if the object count changed:
		if (dt > 0 && objects.GetCount() > 0 && objects.GetCount() != TLAS.desc.toplevel.count)
		{
			RaytracingAccelerationStructureDesc desc;
			desc._flags = RaytracingAccelerationStructureDesc::FLAG_PREFER_FAST_BUILD;
			desc.type = RaytracingAccelerationStructureDesc::TOPLEVEL;
			desc.toplevel.count = (uint32_t)objects.GetCount();
			GPUBufferDesc bufdesc;
			bufdesc.MiscFlags |= RESOURCE_MISC_RAY_TRACING;
			bufdesc.ByteWidth = desc.toplevel.count * (uint32_t)device->GetTopLevelAccelerationStructureInstanceSize();
			bool success = device->CreateBuffer(&bufdesc, nullptr, &desc.toplevel.instanceBuffer);
			assert(success);
			device->SetName(&desc.toplevel.instanceBuffer, "TLAS.instanceBuffer");
			success = device->CreateRaytracingAccelerationStructure(&desc, &TLAS);
			assert(success);
			device->SetName(&TLAS, "TLAS");
		}
	}

}
void wiScene::Scene::Clear()
{
	names.Clear();
	layers.Clear();
	transforms.Clear();
	prev_transforms.Clear();
	hierarchy.Clear();
	materials.Clear();
	meshes.Clear();
	impostors.Clear();
	objects.Clear();
	aabb_objects.Clear();
	rigidbodies.Clear();
	softbodies.Clear();
	armatures.Clear();
	lights.Clear();
	aabb_lights.Clear();
	cameras.Clear();
	probes.Clear();
	aabb_probes.Clear();
	forces.Clear();
	decals.Clear();
	aabb_decals.Clear();
	animations.Clear();
	animation_datas.Clear();
	emitters.Clear();
	hairs.Clear();
	weathers.Clear();
	sounds.Clear();
	inverse_kinematics.Clear();
	springs.Clear();
	flexiChains.Clear();

	TLAS = RaytracingAccelerationStructure();
}

void wiScene::Scene::Merge(Scene& other)
{
	names.Merge(other.names);
	layers.Merge(other.layers);
	transforms.Merge(other.transforms);
	prev_transforms.Merge(other.prev_transforms);
	hierarchy.Merge(other.hierarchy);
	materials.Merge(other.materials);
	meshes.Merge(other.meshes);
	impostors.Merge(other.impostors);
	objects.Merge(other.objects);
	aabb_objects.Merge(other.aabb_objects);
	rigidbodies.Merge(other.rigidbodies);
	softbodies.Merge(other.softbodies);
	armatures.Merge(other.armatures);
	lights.Merge(other.lights);
	aabb_lights.Merge(other.aabb_lights);
	cameras.Merge(other.cameras);
	probes.Merge(other.probes);
	aabb_probes.Merge(other.aabb_probes);
	forces.Merge(other.forces);
	decals.Merge(other.decals);
	aabb_decals.Merge(other.aabb_decals);
	animations.Merge(other.animations);
	animation_datas.Merge(other.animation_datas);
	emitters.Merge(other.emitters);
	hairs.Merge(other.hairs);
	weathers.Merge(other.weathers);
	sounds.Merge(other.sounds);
	inverse_kinematics.Merge(other.inverse_kinematics);
	springs.Merge(other.springs);
	flexiChains.Merge(other.flexiChains);

	bounds = AABB::Merge(bounds, other.bounds);
}

void wiScene::Scene::Entity_Remove(Entity entity)
{
	Component_Detach(entity); // special case, this will also remove entity from hierarchy but also do more!

	names.Remove(entity);
	layers.Remove(entity);
	transforms.Remove(entity);
	prev_transforms.Remove(entity);
	materials.Remove(entity);
	meshes.Remove(entity);
	impostors.Remove(entity);
	objects.Remove(entity);
	aabb_objects.Remove(entity);
	rigidbodies.Remove(entity);
	softbodies.Remove(entity);
	armatures.Remove(entity);
	lights.Remove(entity);
	aabb_lights.Remove(entity);
	cameras.Remove(entity);
	probes.Remove(entity);
	aabb_probes.Remove(entity);
	forces.Remove(entity);
	decals.Remove(entity);
	aabb_decals.Remove(entity);
	animations.Remove(entity);
	animation_datas.Remove(entity);
	emitters.Remove(entity);
	hairs.Remove(entity);
	weathers.Remove(entity);
	sounds.Remove(entity);
	inverse_kinematics.Remove(entity);
	springs.Remove(entity);
	flexiChains.Remove(entity);
}

Entity wiScene::Scene::Entity_FindByName(const std::string& name)
{
	for (size_t i = 0; i < names.GetCount(); ++i)
	{
		if (names[i] == name)
		{
			return names.GetEntity(i);
		}
	}
	return INVALID_ENTITY;
}

Entity wiScene::Scene::Entity_Duplicate(Entity entity)
{
	wiArchive archive;

	// First write the entity to staging area:
	archive.SetReadModeAndResetPos(false);
	Entity_Serialize(archive, entity, 0);

	// Then deserialize with a unique seed:
	archive.SetReadModeAndResetPos(true);
	return Entity_Serialize(archive, entity, CreateEntity(), false);
}

Entity wiScene::Scene::Entity_CreateMaterial(
	const std::string& name
)
{
	Entity entity = CreateEntity();

	names.Create(entity) = name;

	materials.Create(entity);

	return entity;
}

Entity wiScene::Scene::Entity_CreateObject(const std::string& name)
{
	Entity entity = CreateEntity();
	names.Create(entity) = name;
	layers.Create(entity);
	transforms.Create(entity);
	prev_transforms.Create(entity);
	aabb_objects.Create(entity);
	objects.Create(entity);
	return entity;
}

Entity wiScene::Scene::Entity_CreateMesh(const std::string& name)
{
	Entity entity = CreateEntity();
	names.Create(entity) = name;
	meshes.Create(entity);
	return entity;
}

Entity wiScene::Scene::Entity_CreateLight(
	const std::string& name,
	const XMFLOAT3& position,
	const XMFLOAT3& color,
	float energy,
	float range)
{
	Entity entity = CreateEntity();

	names.Create(entity) = name;

	layers.Create(entity);

	TransformComponent& transform = transforms.Create(entity);
	transform.Translate(position);
	transform.UpdateTransform();

	aabb_lights.Create(entity).createFromHalfWidth(position, XMFLOAT3(range, range, range));

	LightComponent& light = lights.Create(entity);
	light.energy = energy;
	light.range_local = range;
	light.fov = XM_PIDIV4;
	light.color = color;
	light.SetType(LightComponent::POINT);

	return entity;
}

Entity wiScene::Scene::Entity_CreateForce(
	const std::string& name,
	const XMFLOAT3& position
)
{
	Entity entity = CreateEntity();

	names.Create(entity) = name;

	layers.Create(entity);

	TransformComponent& transform = transforms.Create(entity);
	transform.Translate(position);
	transform.UpdateTransform();

	ForceFieldComponent& force = forces.Create(entity);
	force.gravity = 0;
	force.range_local = 0;
	force.type = ENTITY_TYPE_FORCEFIELD_POINT;

	return entity;
}

Entity wiScene::Scene::Entity_CreateEnvironmentProbe(
	const std::string& name,
	const XMFLOAT3& position
)
{
	Entity entity = CreateEntity();

	names.Create(entity) = name;

	layers.Create(entity);

	TransformComponent& transform = transforms.Create(entity);
	transform.Translate(position);
	transform.UpdateTransform();

	aabb_probes.Create(entity);

	probes.Create(entity);

	return entity;
}

Entity wiScene::Scene::Entity_CreateDecal(
	const std::string& name,
	const std::string& textureName,
	const std::string& normalMapName
)
{
	Entity entity = CreateEntity();

	names.Create(entity) = name;

	layers.Create(entity);

	transforms.Create(entity);

	aabb_decals.Create(entity);

	decals.Create(entity);

	MaterialComponent& material = materials.Create(entity);

	if (!textureName.empty())
	{
		material.baseColorMapName = textureName;
		material.baseColorMap = wiResourceManager::Load(material.baseColorMapName);
	}
	if (!normalMapName.empty())
	{
		material.normalMapName = normalMapName;
		material.normalMap = wiResourceManager::Load(material.normalMapName);
	}

	return entity;
}

Entity wiScene::Scene::Entity_CreateCamera(
	const std::string& name,
	float width, float height, float nearPlane, float farPlane, float fov
)
{
	Entity entity = CreateEntity();

	names.Create(entity) = name;

	layers.Create(entity);

	transforms.Create(entity);

	CameraComponent& camera = cameras.Create(entity);
	camera.CreatePerspective(width, height, nearPlane, farPlane, fov);

	return entity;
}

Entity wiScene::Scene::Entity_CreateEmitter(
	const std::string& name,
	const XMFLOAT3& position
)
{
	Entity entity = CreateEntity();

	names.Create(entity) = name;

	emitters.Create(entity).count = 10;

	TransformComponent& transform = transforms.Create(entity);
	transform.Translate(position);
	transform.UpdateTransform();

	materials.Create(entity).userBlendMode = BLENDMODE_ALPHA;

	return entity;
}

Entity wiScene::Scene::Entity_CreateHair(
	const std::string& name,
	const XMFLOAT3& position
)
{
	Entity entity = CreateEntity();

	names.Create(entity) = name;

	hairs.Create(entity);

	TransformComponent& transform = transforms.Create(entity);
	transform.Translate(position);
	transform.UpdateTransform();

	materials.Create(entity);

	return entity;
}

Entity wiScene::Scene::Entity_CreateSound(
	const std::string& name,
	const std::string& filename,
	const XMFLOAT3& position
)
{
	Entity entity = CreateEntity();

	names.Create(entity) = name;

	SoundComponent& sound = sounds.Create(entity);
	sound.filename = filename;
	sound.soundResource = wiResourceManager::Load(filename);
	wiAudio::CreateSoundInstance(sound.soundResource->sound, &sound.soundinstance);

	TransformComponent& transform = transforms.Create(entity);
	transform.Translate(position);
	transform.UpdateTransform();

	return entity;
}

void wiScene::Scene::Component_Attach(Entity entity, Entity parent, bool child_already_in_local_space)
{
	assert(entity != parent);

	if (hierarchy.Contains(entity))
	{
		Component_Detach(entity);
	}

	// Add a new hierarchy node to the end of container:
	hierarchy.Create(entity).parentID = parent;

	// Detect breaks in the tree and fix them:
	//	when children are before parents, we move the parents before the children while keeping ordering of other components intact
	if (hierarchy.GetCount() > 1)
	{
		for (size_t i = hierarchy.GetCount() - 1; i > 0; --i)
		{
			Entity parent_candidate_entity = hierarchy.GetEntity(i);
			for (size_t j = 0; j < i; ++j)
			{
				const HierarchyComponent& child_candidate = hierarchy[j];

				if (child_candidate.parentID == parent_candidate_entity)
				{
					hierarchy.MoveItem(i, j);
					++i; // next outer iteration will check the same index again as parent candidate, however things were moved upwards, so it will be a different entity!
					break;
				}
			}
		}
	}

	// Re-query parent after potential MoveItem(), because it invalidates references:
	HierarchyComponent& parentcomponent = *hierarchy.GetComponent(entity);

	TransformComponent* transform_parent = transforms.GetComponent(parent);
	if (transform_parent == nullptr)
	{
		transform_parent = &transforms.Create(parent);
	}

	TransformComponent* transform_child = transforms.GetComponent(entity);
	if (transform_child == nullptr)
	{
		transform_child = &transforms.Create(entity);
		transform_parent = transforms.GetComponent(parent); // after transforms.Create(), transform_parent pointer could have become invalidated!
	}
	if (!child_already_in_local_space)
	{
		XMMATRIX B = XMMatrixInverse(nullptr, XMLoadFloat4x4(&transform_parent->world));
		transform_child->MatrixTransform(B);
		transform_child->UpdateTransform();
	}
	transform_child->UpdateTransform_Parented(*transform_parent);

	LayerComponent* layer_parent = layers.GetComponent(parent);
	if (layer_parent == nullptr)
	{
		layer_parent = &layers.Create(parent);
	}
	LayerComponent* layer_child = layers.GetComponent(entity);
	if (layer_child == nullptr)
	{
		layer_child = &layers.Create(entity);
	}
	// Save the initial layermask of the child so that it can be restored if detached:
	parentcomponent.layerMask_bind = layer_child->GetLayerMask();
}

void wiScene::Scene::Component_Detach(Entity entity)
{
	const HierarchyComponent* parent = hierarchy.GetComponent(entity);

	if (parent != nullptr)
	{
		TransformComponent* transform = transforms.GetComponent(entity);
		if (transform != nullptr)
		{
			transform->ApplyTransform();
		}

		LayerComponent* layer = layers.GetComponent(entity);
		if (layer != nullptr)
		{
			layer->layerMask = parent->layerMask_bind;
		}

		hierarchy.Remove_KeepSorted(entity);
	}
}

void wiScene::Scene::Component_DetachChildren(Entity parent)
{
	for (size_t i = 0; i < hierarchy.GetCount(); )
	{
		if (hierarchy[i].parentID == parent)
		{
			Entity entity = hierarchy.GetEntity(i);
			Component_Detach(entity);
		}
		else
		{
			++i;
		}
	}
}

wiScene::IntersectSphereResult wiScene::Scene::IntersectSphere(const SPHERE& sphere, uint32_t renderTypeMask, uint32_t layerMask) const
{
	IntersectSphereResult result;
	XMVECTOR Center = XMLoadFloat3(&sphere.center);
	XMVECTOR Radius = XMVectorReplicate(sphere.radius);
	XMVECTOR RadiusSq = XMVectorMultiply(Radius, Radius);

	if (objects.GetCount() > 0)
	{
		for (size_t i = 0; i < aabb_objects.GetCount(); ++i)
		{
			const AABB& aabb = aabb_objects[i];
			if (!sphere.intersects(aabb))
			{
				continue;
			}

			const ObjectComponent& object = objects[i];
			if (object.meshID == INVALID_ENTITY)
			{
				continue;
			}
			if (!(renderTypeMask & object.GetRenderTypes()))
			{
				continue;
			}

			Entity entity = aabb_objects.GetEntity(i);
			const LayerComponent* layer = layers.GetComponent(entity);
			if (layer != nullptr && !(layer->GetLayerMask() & layerMask))
			{
				continue;
			}

			const MeshComponent& mesh = *meshes.GetComponent(object.meshID);
			const SoftBodyPhysicsComponent* softbody = softbodies.GetComponent(object.meshID);
			const bool softbody_active = softbody != nullptr && !softbody->vertex_positions_simulation.empty();

			const XMMATRIX objectMat = object.transform_index >= 0 ? XMLoadFloat4x4(&transforms[object.transform_index].world) : XMMatrixIdentity();

			const ArmatureComponent* armature = mesh.IsSkinned() ? armatures.GetComponent(mesh.armatureID) : nullptr;

			int subsetCounter = 0;
			for (auto& subset : mesh.subsets)
			{
				for (size_t i = 0; i < subset.indexCount; i += 3)
				{
					const uint32_t i0 = mesh.indices[subset.indexOffset + i + 0];
					const uint32_t i1 = mesh.indices[subset.indexOffset + i + 1];
					const uint32_t i2 = mesh.indices[subset.indexOffset + i + 2];

					XMVECTOR p0;
					XMVECTOR p1;
					XMVECTOR p2;

					if (softbody_active)
					{
						p0 = softbody->vertex_positions_simulation[i0].LoadPOS();
						p1 = softbody->vertex_positions_simulation[i1].LoadPOS();
						p2 = softbody->vertex_positions_simulation[i2].LoadPOS();
					}
					else
					{
						if (armature == nullptr)
						{
							p0 = XMLoadFloat3(&mesh.vertex_positions[i0]);
							p1 = XMLoadFloat3(&mesh.vertex_positions[i1]);
							p2 = XMLoadFloat3(&mesh.vertex_positions[i2]);
						}
						else
						{
							p0 = SkinVertex(mesh, *armature, i0);
							p1 = SkinVertex(mesh, *armature, i1);
							p2 = SkinVertex(mesh, *armature, i2);
						}
					}

					p0 = XMVector3Transform(p0, objectMat);
					p1 = XMVector3Transform(p1, objectMat);
					p2 = XMVector3Transform(p2, objectMat);

					XMFLOAT3 min, max;
					XMStoreFloat3(&min, XMVectorMin(p0, XMVectorMin(p1, p2)));
					XMStoreFloat3(&max, XMVectorMax(p0, XMVectorMax(p1, p2)));
					AABB aabb_triangle(min, max);
					if (sphere.intersects(aabb_triangle) == AABB::OUTSIDE)
					{
						continue;
					}

					// Compute the plane of the triangle (has to be normalized).
					XMVECTOR N = XMVector3Normalize(XMVector3Cross(XMVectorSubtract(p1, p0), XMVectorSubtract(p2, p0)));

					// Assert that the triangle is not degenerate.
					assert(!XMVector3Equal(N, XMVectorZero()));

					// Find the nearest feature on the triangle to the sphere.
					XMVECTOR Dist = XMVector3Dot(XMVectorSubtract(Center, p0), N);

					if (!mesh.IsDoubleSided() && XMVectorGetX(Dist) > 0)
					{
						continue; // pass through back faces
					}

					// If the center of the sphere is farther from the plane of the triangle than
					// the radius of the sphere, then there cannot be an intersection.
					XMVECTOR NoIntersection = XMVectorLess(Dist, XMVectorNegate(Radius));
					NoIntersection = XMVectorOrInt(NoIntersection, XMVectorGreater(Dist, Radius));

					// Project the center of the sphere onto the plane of the triangle.
					XMVECTOR Point0 = XMVectorNegativeMultiplySubtract(N, Dist, Center);

					// Is it inside all the edges? If so we intersect because the distance 
					// to the plane is less than the radius.
					//XMVECTOR Intersection = DirectX::Internal::PointOnPlaneInsideTriangle(Point0, p0, p1, p2);

					// Compute the cross products of the vector from the base of each edge to 
					// the point with each edge vector.
					XMVECTOR C0 = XMVector3Cross(XMVectorSubtract(Point0, p0), XMVectorSubtract(p1, p0));
					XMVECTOR C1 = XMVector3Cross(XMVectorSubtract(Point0, p1), XMVectorSubtract(p2, p1));
					XMVECTOR C2 = XMVector3Cross(XMVectorSubtract(Point0, p2), XMVectorSubtract(p0, p2));

					// If the cross product points in the same direction as the normal the the
					// point is inside the edge (it is zero if is on the edge).
					XMVECTOR Zero = XMVectorZero();
					XMVECTOR Inside0 = XMVectorLessOrEqual(XMVector3Dot(C0, N), Zero);
					XMVECTOR Inside1 = XMVectorLessOrEqual(XMVector3Dot(C1, N), Zero);
					XMVECTOR Inside2 = XMVectorLessOrEqual(XMVector3Dot(C2, N), Zero);

					// If the point inside all of the edges it is inside.
					XMVECTOR Intersection = XMVectorAndInt(XMVectorAndInt(Inside0, Inside1), Inside2);

					bool inside = XMVector4EqualInt(XMVectorAndCInt(Intersection, NoIntersection), XMVectorTrueInt());

					// Find the nearest point on each edge.

					// Edge 0,1
					XMVECTOR Point1 = DirectX::Internal::PointOnLineSegmentNearestPoint(p0, p1, Center);

					// If the distance to the center of the sphere to the point is less than 
					// the radius of the sphere then it must intersect.
					Intersection = XMVectorOrInt(Intersection, XMVectorLessOrEqual(XMVector3LengthSq(XMVectorSubtract(Center, Point1)), RadiusSq));

					// Edge 1,2
					XMVECTOR Point2 = DirectX::Internal::PointOnLineSegmentNearestPoint(p1, p2, Center);

					// If the distance to the center of the sphere to the point is less than 
					// the radius of the sphere then it must intersect.
					Intersection = XMVectorOrInt(Intersection, XMVectorLessOrEqual(XMVector3LengthSq(XMVectorSubtract(Center, Point2)), RadiusSq));

					// Edge 2,0
					XMVECTOR Point3 = DirectX::Internal::PointOnLineSegmentNearestPoint(p2, p0, Center);

					// If the distance to the center of the sphere to the point is less than 
					// the radius of the sphere then it must intersect.
					Intersection = XMVectorOrInt(Intersection, XMVectorLessOrEqual(XMVector3LengthSq(XMVectorSubtract(Center, Point3)), RadiusSq));

					bool intersects = XMVector4EqualInt(XMVectorAndCInt(Intersection, NoIntersection), XMVectorTrueInt());

					if (intersects)
					{
						XMVECTOR bestPoint = Point0;
						if (!inside)
						{
							// If the sphere center's projection on the triangle plane is not within the triangle,
							//	determine the closest point on triangle to the sphere center
							float bestDist = XMVectorGetX(XMVector3LengthSq(Point1 - Center));
							bestPoint = Point1;

							float d = XMVectorGetX(XMVector3LengthSq(Point2 - Center));
							if (d < bestDist)
							{
								bestDist = d;
								bestPoint = Point2;
							}
							d = XMVectorGetX(XMVector3LengthSq(Point3 - Center));
							if (d < bestDist)
							{
								bestDist = d;
								bestPoint = Point3;
							}
						}
						XMVECTOR intersectionVec = Center - bestPoint;
						XMVECTOR intersectionVecLen = XMVector3Length(intersectionVec);

						result.entity = entity;
						result.depth = sphere.radius - XMVectorGetX(intersectionVecLen);
						XMStoreFloat3(&result.position, bestPoint);
						XMStoreFloat3(&result.normal, intersectionVec / intersectionVecLen);
						return result;
					}
				}
				subsetCounter++;
			}
		}
	}

	return result;
}

wiScene::IntersectSphereResult wiScene::Scene::IntersectCapsule(const CAPSULE& capsule, uint32_t renderTypeMask, uint32_t layerMask) const
{
	IntersectSphereResult result;
	XMVECTOR Base = XMLoadFloat3(&capsule.base);
	XMVECTOR Tip = XMLoadFloat3(&capsule.tip);
	XMVECTOR Radius = XMVectorReplicate(capsule.radius);
	XMVECTOR LineEndOffset = XMVector3Normalize(Tip - Base) * Radius;
	XMVECTOR A = Base + LineEndOffset;
	XMVECTOR B = Tip - LineEndOffset;
	XMVECTOR RadiusSq = XMVectorMultiply(Radius, Radius);
	AABB capsule_aabb = capsule.getAABB();

	if (objects.GetCount() > 0)
	{

		for (size_t i = 0; i < aabb_objects.GetCount(); ++i)
		{
			const AABB& aabb = aabb_objects[i];
			if (capsule_aabb.intersects(aabb) == AABB::INTERSECTION_TYPE::OUTSIDE)
			{
				continue;
			}

			const ObjectComponent& object = objects[i];
			if (object.meshID == INVALID_ENTITY)
			{
				continue;
			}
			if (!(renderTypeMask & object.GetRenderTypes()))
			{
				continue;
			}

			Entity entity = aabb_objects.GetEntity(i);
			const LayerComponent* layer = layers.GetComponent(entity);
			if (layer != nullptr && !(layer->GetLayerMask() & layerMask))
			{
				continue;
			}

			const MeshComponent& mesh = *meshes.GetComponent(object.meshID);
			const SoftBodyPhysicsComponent* softbody = softbodies.GetComponent(object.meshID);
			const bool softbody_active = softbody != nullptr && !softbody->vertex_positions_simulation.empty();

			const XMMATRIX objectMat = object.transform_index >= 0 ? XMLoadFloat4x4(&transforms[object.transform_index].world) : XMMatrixIdentity();

			const ArmatureComponent* armature = mesh.IsSkinned() ? armatures.GetComponent(mesh.armatureID) : nullptr;

			int subsetCounter = 0;
			for (auto& subset : mesh.subsets)
			{
				for (size_t i = 0; i < subset.indexCount; i += 3)
				{
					const uint32_t i0 = mesh.indices[subset.indexOffset + i + 0];
					const uint32_t i1 = mesh.indices[subset.indexOffset + i + 1];
					const uint32_t i2 = mesh.indices[subset.indexOffset + i + 2];

					XMVECTOR p0;
					XMVECTOR p1;
					XMVECTOR p2;

					if (softbody_active)
					{
						p0 = softbody->vertex_positions_simulation[i0].LoadPOS();
						p1 = softbody->vertex_positions_simulation[i1].LoadPOS();
						p2 = softbody->vertex_positions_simulation[i2].LoadPOS();
					}
					else
					{
						if (armature == nullptr)
						{
							p0 = XMLoadFloat3(&mesh.vertex_positions[i0]);
							p1 = XMLoadFloat3(&mesh.vertex_positions[i1]);
							p2 = XMLoadFloat3(&mesh.vertex_positions[i2]);
						}
						else
						{
							p0 = SkinVertex(mesh, *armature, i0);
							p1 = SkinVertex(mesh, *armature, i1);
							p2 = SkinVertex(mesh, *armature, i2);
						}
					}

					p0 = XMVector3Transform(p0, objectMat);
					p1 = XMVector3Transform(p1, objectMat);
					p2 = XMVector3Transform(p2, objectMat);

					XMFLOAT3 min, max;
					XMStoreFloat3(&min, XMVectorMin(p0, XMVectorMin(p1, p2)));
					XMStoreFloat3(&max, XMVectorMax(p0, XMVectorMax(p1, p2)));
					AABB aabb_triangle(min, max);
					if (capsule_aabb.intersects(aabb_triangle) == AABB::OUTSIDE)
					{
						continue;
					}

					// Compute the plane of the triangle (has to be normalized).
					XMVECTOR N = XMVector3Normalize(XMVector3Cross(XMVectorSubtract(p1, p0), XMVectorSubtract(p2, p0)));

					XMVECTOR ReferencePoint;
					XMVECTOR d = XMVector3Normalize(B - A);
					if (abs(XMVectorGetX(XMVector3Dot(N, d))) < FLT_EPSILON)
					{
						// Capsule line cannot be intersected with triangle plane (they are parallel)
						//	In this case, just take a point from triangle
						ReferencePoint = p0;
					}
					else
					{
						// Intersect capsule line with triangle plane:
						XMVECTOR t = XMVector3Dot(N, (Base - p0) / XMVectorAbs(XMVector3Dot(N, d)));
						XMVECTOR LinePlaneIntersection = Base + d * t;

						// Compute the cross products of the vector from the base of each edge to 
						// the point with each edge vector.
						XMVECTOR C0 = XMVector3Cross(XMVectorSubtract(LinePlaneIntersection, p0), XMVectorSubtract(p1, p0));
						XMVECTOR C1 = XMVector3Cross(XMVectorSubtract(LinePlaneIntersection, p1), XMVectorSubtract(p2, p1));
						XMVECTOR C2 = XMVector3Cross(XMVectorSubtract(LinePlaneIntersection, p2), XMVectorSubtract(p0, p2));

						// If the cross product points in the same direction as the normal the the
						// point is inside the edge (it is zero if is on the edge).
						XMVECTOR Zero = XMVectorZero();
						XMVECTOR Inside0 = XMVectorLessOrEqual(XMVector3Dot(C0, N), Zero);
						XMVECTOR Inside1 = XMVectorLessOrEqual(XMVector3Dot(C1, N), Zero);
						XMVECTOR Inside2 = XMVectorLessOrEqual(XMVector3Dot(C2, N), Zero);

						// If the point inside all of the edges it is inside.
						XMVECTOR Intersection = XMVectorAndInt(XMVectorAndInt(Inside0, Inside1), Inside2);

						bool inside = XMVectorGetIntX(Intersection) != 0;

						if (inside)
						{
							ReferencePoint = LinePlaneIntersection;
						}
						else
						{
							// Find the nearest point on each edge.

							// Edge 0,1
							XMVECTOR Point1 = wiMath::ClosestPointOnLineSegment(p0, p1, LinePlaneIntersection);

							// Edge 1,2
							XMVECTOR Point2 = wiMath::ClosestPointOnLineSegment(p1, p2, LinePlaneIntersection);

							// Edge 2,0
							XMVECTOR Point3 = wiMath::ClosestPointOnLineSegment(p2, p0, LinePlaneIntersection);

							ReferencePoint = Point1;
							float bestDist = XMVectorGetX(XMVector3LengthSq(Point1 - LinePlaneIntersection));
							float d = abs(XMVectorGetX(XMVector3LengthSq(Point2 - LinePlaneIntersection)));
							if (d < bestDist)
							{
								bestDist = d;
								ReferencePoint = Point2;
							}
							d = abs(XMVectorGetX(XMVector3LengthSq(Point3 - LinePlaneIntersection)));
							if (d < bestDist)
							{
								bestDist = d;
								ReferencePoint = Point3;
							}
						}


					}

					// Place a sphere on closest point on line segment to intersection:
					XMVECTOR Center = wiMath::ClosestPointOnLineSegment(A, B, ReferencePoint);

					// Assert that the triangle is not degenerate.
					assert(!XMVector3Equal(N, XMVectorZero()));

					// Find the nearest feature on the triangle to the sphere.
					XMVECTOR Dist = XMVector3Dot(XMVectorSubtract(Center, p0), N);

					if (!mesh.IsDoubleSided() && XMVectorGetX(Dist) > 0)
					{
						continue; // pass through back faces
					}

					// If the center of the sphere is farther from the plane of the triangle than
					// the radius of the sphere, then there cannot be an intersection.
					XMVECTOR NoIntersection = XMVectorLess(Dist, XMVectorNegate(Radius));
					NoIntersection = XMVectorOrInt(NoIntersection, XMVectorGreater(Dist, Radius));

					// Project the center of the sphere onto the plane of the triangle.
					XMVECTOR Point0 = XMVectorNegativeMultiplySubtract(N, Dist, Center);

					// Is it inside all the edges? If so we intersect because the distance 
					// to the plane is less than the radius.
					//XMVECTOR Intersection = DirectX::Internal::PointOnPlaneInsideTriangle(Point0, p0, p1, p2);

					// Compute the cross products of the vector from the base of each edge to 
					// the point with each edge vector.
					XMVECTOR C0 = XMVector3Cross(XMVectorSubtract(Point0, p0), XMVectorSubtract(p1, p0));
					XMVECTOR C1 = XMVector3Cross(XMVectorSubtract(Point0, p1), XMVectorSubtract(p2, p1));
					XMVECTOR C2 = XMVector3Cross(XMVectorSubtract(Point0, p2), XMVectorSubtract(p0, p2));

					// If the cross product points in the same direction as the normal the the
					// point is inside the edge (it is zero if is on the edge).
					XMVECTOR Zero = XMVectorZero();
					XMVECTOR Inside0 = XMVectorLessOrEqual(XMVector3Dot(C0, N), Zero);
					XMVECTOR Inside1 = XMVectorLessOrEqual(XMVector3Dot(C1, N), Zero);
					XMVECTOR Inside2 = XMVectorLessOrEqual(XMVector3Dot(C2, N), Zero);

					// If the point inside all of the edges it is inside.
					XMVECTOR Intersection = XMVectorAndInt(XMVectorAndInt(Inside0, Inside1), Inside2);

					bool inside = XMVector4EqualInt(XMVectorAndCInt(Intersection, NoIntersection), XMVectorTrueInt());

					// Find the nearest point on each edge.

					// Edge 0,1
					XMVECTOR Point1 = wiMath::ClosestPointOnLineSegment(p0, p1, Center);

					// If the distance to the center of the sphere to the point is less than 
					// the radius of the sphere then it must intersect.
					Intersection = XMVectorOrInt(Intersection, XMVectorLessOrEqual(XMVector3LengthSq(XMVectorSubtract(Center, Point1)), RadiusSq));

					// Edge 1,2
					XMVECTOR Point2 = wiMath::ClosestPointOnLineSegment(p1, p2, Center);

					// If the distance to the center of the sphere to the point is less than 
					// the radius of the sphere then it must intersect.
					Intersection = XMVectorOrInt(Intersection, XMVectorLessOrEqual(XMVector3LengthSq(XMVectorSubtract(Center, Point2)), RadiusSq));

					// Edge 2,0
					XMVECTOR Point3 = wiMath::ClosestPointOnLineSegment(p2, p0, Center);

					// If the distance to the center of the sphere to the point is less than 
					// the radius of the sphere then it must intersect.
					Intersection = XMVectorOrInt(Intersection, XMVectorLessOrEqual(XMVector3LengthSq(XMVectorSubtract(Center, Point3)), RadiusSq));

					bool intersects = XMVector4EqualInt(XMVectorAndCInt(Intersection, NoIntersection), XMVectorTrueInt());

					if (intersects)
					{
						XMVECTOR bestPoint = Point0;
						if (!inside)
						{
							// If the sphere center's projection on the triangle plane is not within the triangle,
							//	determine the closest point on triangle to the sphere center
							float bestDist = XMVectorGetX(XMVector3LengthSq(Point1 - Center));
							bestPoint = Point1;

							float d = XMVectorGetX(XMVector3LengthSq(Point2 - Center));
							if (d < bestDist)
							{
								bestDist = d;
								bestPoint = Point2;
							}
							d = XMVectorGetX(XMVector3LengthSq(Point3 - Center));
							if (d < bestDist)
							{
								bestDist = d;
								bestPoint = Point3;
							}
						}
						XMVECTOR intersectionVec = Center - bestPoint;
						XMVECTOR intersectionVecLen = XMVector3Length(intersectionVec);

						result.entity = entity;
						result.depth = capsule.radius - XMVectorGetX(intersectionVecLen);
						XMStoreFloat3(&result.position, bestPoint);
						XMStoreFloat3(&result.normal, intersectionVec / intersectionVecLen);
						return result;
					}
				}
				subsetCounter++;
			}

		}
	}

	return result;
}

wiScene::PickResult wiScene::Scene::PickObject(const RAY& ray, uint32_t renderTypeMask, uint32_t layerMask) const
{
	PickResult result;

	if (objects.GetCount() > 0)
	{
		const XMVECTOR rayOrigin = XMLoadFloat3(&ray.origin);
		const XMVECTOR rayDirection = XMVector3Normalize(XMLoadFloat3(&ray.direction));

		for (size_t i = 0; i < aabb_objects.GetCount(); ++i)
		{
			const AABB& aabb = aabb_objects[i];
			if (!ray.intersects(aabb))
			{
				continue;
			}

			const ObjectComponent& object = objects[i];
			if (object.meshID == INVALID_ENTITY)
			{
				continue;
			}
			if (!(renderTypeMask & object.GetRenderTypes()))
			{
				continue;
			}

			Entity entity = aabb_objects.GetEntity(i);
			const LayerComponent* layer = layers.GetComponent(entity);
			if (layer != nullptr && !(layer->GetLayerMask() & layerMask))
			{
				continue;
			}

			const MeshComponent& mesh = *meshes.GetComponent(object.meshID);
			const SoftBodyPhysicsComponent* softbody = softbodies.GetComponent(object.meshID);
			const bool softbody_active = softbody != nullptr && !softbody->vertex_positions_simulation.empty();

			const XMMATRIX objectMat = object.transform_index >= 0 ? XMLoadFloat4x4(&transforms[object.transform_index].world) : XMMatrixIdentity();
			const XMMATRIX objectMat_Inverse = XMMatrixInverse(nullptr, objectMat);

			const XMVECTOR rayOrigin_local = XMVector3Transform(rayOrigin, objectMat_Inverse);
			const XMVECTOR rayDirection_local = XMVector3Normalize(XMVector3TransformNormal(rayDirection, objectMat_Inverse));

			const ArmatureComponent* armature = mesh.IsSkinned() ? armatures.GetComponent(mesh.armatureID) : nullptr;

			int subsetCounter = 0;
			for (auto& subset : mesh.subsets)
			{
				for (size_t i = 0; i < subset.indexCount; i += 3)
				{
					const uint32_t i0 = mesh.indices[subset.indexOffset + i + 0];
					const uint32_t i1 = mesh.indices[subset.indexOffset + i + 1];
					const uint32_t i2 = mesh.indices[subset.indexOffset + i + 2];

					XMVECTOR p0;
					XMVECTOR p1;
					XMVECTOR p2;

					if (softbody_active)
					{
						p0 = softbody->vertex_positions_simulation[i0].LoadPOS();
						p1 = softbody->vertex_positions_simulation[i1].LoadPOS();
						p2 = softbody->vertex_positions_simulation[i2].LoadPOS();
					}
					else
					{
						if (armature == nullptr)
						{
							p0 = XMLoadFloat3(&mesh.vertex_positions[i0]);
							p1 = XMLoadFloat3(&mesh.vertex_positions[i1]);
							p2 = XMLoadFloat3(&mesh.vertex_positions[i2]);
						}
						else
						{
							p0 = SkinVertex(mesh, *armature, i0);
							p1 = SkinVertex(mesh, *armature, i1);
							p2 = SkinVertex(mesh, *armature, i2);
						}
					}

					float distance;
					XMFLOAT2 bary;
					if (wiMath::RayTriangleIntersects(rayOrigin_local, rayDirection_local, p0, p1, p2, distance, bary))
					{
						const XMVECTOR pos = XMVector3Transform(XMVectorAdd(rayOrigin_local, rayDirection_local * distance), objectMat);
						distance = wiMath::Distance(pos, rayOrigin);

						if (distance < result.distance)
						{
							const XMVECTOR nor = XMVector3Normalize(XMVector3TransformNormal(XMVector3Cross(XMVectorSubtract(p2, p1), XMVectorSubtract(p1, p0)), objectMat));

							result.entity = entity;
							XMStoreFloat3(&result.position, pos);
							XMStoreFloat3(&result.normal, nor);
							result.distance = distance;
							result.subsetIndex = subsetCounter;
							result.vertexID0 = (int)i0;
							result.vertexID1 = (int)i1;
							result.vertexID2 = (int)i2;
							result.bary = bary;
						}
					}
				}
				subsetCounter++;
			}

		}
	}

	// Construct a matrix that will orient to position (P) according to surface normal (N):
	XMVECTOR N = XMLoadFloat3(&result.normal);
	XMVECTOR P = XMLoadFloat3(&result.position);
	XMVECTOR E = XMLoadFloat3(&ray.origin);
	XMVECTOR T = XMVector3Normalize(XMVector3Cross(N, P - E));
	XMVECTOR B = XMVector3Normalize(XMVector3Cross(T, N));
	XMMATRIX M = { T, N, B, P };
	XMStoreFloat4x4(&result.orientation, M);

	return result;
}

wiECS::Entity wiScene::LoadModel(const std::string& fileName, wiScene::Scene& scene, const XMMATRIX& transformMatrix, bool attached)
{
	wiArchive archive(fileName, true);
	if (archive.IsOpen())
	{
		// Serialize it from file:
		scene.Serialize(archive);

		// First, create new root:
		Entity root = CreateEntity();
		scene.transforms.Create(root);
		scene.layers.Create(root).layerMask = ~0;

		{
			// Apply the optional transformation matrix to the new scene:

			// Parent all unparented transforms to new root entity
			for (size_t i = 0; i < scene.transforms.GetCount() - 1; ++i) // GetCount() - 1 because the last added was the "root"
			{
				Entity entity = scene.transforms.GetEntity(i);
				if (!scene.hierarchy.Contains(entity))
				{
					scene.Component_Attach(entity, root);
				}
			}

			// The root component is transformed, scene is updated:
			scene.transforms.GetComponent(root)->MatrixTransform(transformMatrix);
			scene.Update(0);
		}

		if (!attached)
		{
			// In this case, we don't care about the root anymore, so delete it. This will simplify overall hierarchy
			scene.Component_DetachChildren(root);
			scene.Entity_Remove(root);
			root = INVALID_ENTITY;
		}

		return root;
	}

	return INVALID_ENTITY;
}
