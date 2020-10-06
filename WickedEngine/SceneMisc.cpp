#include "wiScene.h"
#include "wiGraphics.h"
#include "wiHelper.h"
#include "wiJobSystem.h"

using namespace wiGraphics;
using namespace wiECS;

void wiScene::ObjectComponent::ClearLightmap()
{
	lightmapWidth = 0;
	lightmapHeight = 0;
	globalLightMapMulAdd = XMFLOAT4(0, 0, 0, 0);
	lightmapIterationCount = 0;
	lightmapTextureData.clear();
	SetLightmapRenderRequest(false);
}

void wiScene::ObjectComponent::SaveLightmap()
{
	if (lightmap.IsValid())
	{
		bool success = wiHelper::saveTextureToMemory(lightmap, lightmapTextureData);
		assert(success);
	}
}

FORMAT wiScene::ObjectComponent::GetLightmapFormat()
{
	uint32_t stride = (uint32_t)lightmapTextureData.size() / lightmapWidth / lightmapHeight;

	switch (stride)
	{
		case 4: return FORMAT_R8G8B8A8_UNORM;
		case 8: return FORMAT_R16G16B16A16_FLOAT;
		case 16: return FORMAT_R32G32B32A32_FLOAT;
	}

	return FORMAT_UNKNOWN;
}

void wiScene::Scene::RunObjectUpdateSystem(wiJobSystem::context& ctx)
{
	assert(objects.GetCount() == aabb_objects.GetCount());

	parallel_bounds.clear();
	parallel_bounds.resize((size_t)wiJobSystem::DispatchGroupCount((uint32_t)objects.GetCount(), small_subtask_groupsize));

	wiJobSystem::Dispatch(ctx, (uint32_t)objects.GetCount(), small_subtask_groupsize, [&](wiJobArgs args)
	{
		ObjectComponent& object = objects[args.jobIndex];
		AABB& aabb = aabb_objects[args.jobIndex];

		aabb = AABB();
		object.rendertypeMask = 0;
		object.SetDynamic(false);
		object.SetCastShadow(false);
		object.SetImpostorPlacement(false);
		object.SetRequestPlanarReflection(false);

		if (object.meshID != INVALID_ENTITY)
		{
			Entity entity = objects.GetEntity(args.jobIndex);
			const MeshComponent* mesh = meshes.GetComponent(object.meshID);

			// These will only be valid for a single frame:
			object.transform_index = (int)transforms.GetIndex(entity);
			object.prev_transform_index = (int)prev_transforms.GetIndex(entity);

			const TransformComponent& transform = transforms[object.transform_index];

			if (mesh != nullptr)
			{
				XMMATRIX W = XMLoadFloat4x4(&transform.world);
				aabb = mesh->aabb.transform(W);

				// This is instance bounding box matrix:
				XMFLOAT4X4 meshMatrix;
				XMStoreFloat4x4(&meshMatrix, mesh->aabb.getAsBoxMatrix() * W);

				// We need sometimes the center of the instance bounding box, not the transform position (which can be outside the bounding box)
				object.center = *((XMFLOAT3*)&meshMatrix._41);

				if (mesh->IsSkinned() || mesh->IsDynamic())
				{
					object.SetDynamic(true);
					const ArmatureComponent* armature = armatures.GetComponent(mesh->armatureID);
					if (armature != nullptr)
					{
						aabb = AABB::Merge(aabb, armature->aabb);
					}
				}

				for (auto& subset : mesh->subsets)
				{
					const MaterialComponent* material = materials.GetComponent(subset.materialID);

					if (material != nullptr)
					{
						if (material->IsCustomShader())
						{
							object.rendertypeMask |= RENDERTYPE_ALL;
						}
						else
						{
							if (material->IsTransparent())
							{
								object.rendertypeMask |= RENDERTYPE_TRANSPARENT;
							}
							else
							{
								object.rendertypeMask |= RENDERTYPE_OPAQUE;
							}

							if (material->IsWater())
							{
								object.rendertypeMask |= RENDERTYPE_TRANSPARENT | RENDERTYPE_WATER;
							}
						}

						if (material->HasPlanarReflection())
						{
							object.SetRequestPlanarReflection(true);
						}

						object.SetCastShadow(material->IsCastingShadow());
					}
				}

				ImpostorComponent* impostor = impostors.GetComponent(object.meshID);
				if (impostor != nullptr)
				{
					object.SetImpostorPlacement(true);
					object.impostorSwapDistance = impostor->swapInDistance;
					object.impostorFadeThresholdRadius = aabb.getRadius();

					impostor->aabb = AABB::Merge(impostor->aabb, aabb);
					impostor->color = object.color;
					impostor->fadeThresholdRadius = object.impostorFadeThresholdRadius;

					const SPHERE boundingsphere = mesh->GetBoundingSphere();

					locker.lock();
					impostor->instanceMatrices.emplace_back();
					XMStoreFloat4x4(&impostor->instanceMatrices.back(),
									XMMatrixScaling(boundingsphere.radius, boundingsphere.radius, boundingsphere.radius) *
									XMMatrixTranslation(boundingsphere.center.x, boundingsphere.center.y, boundingsphere.center.z) *
									W
					);
					locker.unlock();
				}

				SoftBodyPhysicsComponent* softbody = softbodies.GetComponent(object.meshID);
				if (softbody != nullptr)
				{
					// this will be registered as soft body in the next physics update
					softbody->_flags |= SoftBodyPhysicsComponent::SAFE_TO_REGISTER;

					// soft body manipulated with the object matrix
					softbody->worldMatrix = transform.world;

					if (softbody->graphicsToPhysicsVertexMapping.empty())
					{
						softbody->CreateFromMesh(*mesh);
					}

					// simulation aabb will be used for soft bodies
					aabb = softbody->aabb;

					// soft bodies have no transform, their vertices are simulated in world space
					object.transform_index = -1;
					object.prev_transform_index = -1;
				}
			}

			// parallel bounds computation using shared memory:
			AABB* shared_bounds = (AABB*)args.sharedmemory;
			if (args.isFirstJobInGroup)
			{
				*shared_bounds = aabb_objects[args.jobIndex];
			}
			else
			{
				*shared_bounds = AABB::Merge(*shared_bounds, aabb_objects[args.jobIndex]);
			}
			if (args.isLastJobInGroup)
			{
				parallel_bounds[args.groupID] = *shared_bounds;
			}
		}

	}, sizeof(AABB));
}

