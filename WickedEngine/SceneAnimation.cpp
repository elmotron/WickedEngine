#include "wiScene.h"
#include "wiJobSystem.h"
#include "wiMath.h"

#include <algorithm>

using namespace wiECS;
using namespace wiScene;

XMVECTOR wiScene::SkinVertex(const wiScene::MeshComponent& mesh, const wiScene::ArmatureComponent& armature, uint32_t index, XMVECTOR* N)
{
	XMVECTOR P = XMLoadFloat3(&mesh.vertex_positions[index]);
	const XMUINT4& ind = mesh.vertex_boneindices[index];
	const XMFLOAT4& wei = mesh.vertex_boneweights[index];

	const XMMATRIX M[] = {
		armature.boneData[ind.x].Load(),
		armature.boneData[ind.y].Load(),
		armature.boneData[ind.z].Load(),
		armature.boneData[ind.w].Load(),
	};

	XMVECTOR skinned;
	skinned = XMVector3Transform(P, M[0]) * wei.x;
	skinned += XMVector3Transform(P, M[1]) * wei.y;
	skinned += XMVector3Transform(P, M[2]) * wei.z;
	skinned += XMVector3Transform(P, M[3]) * wei.w;
	P = skinned;

	if (N != nullptr)
	{
		*N = XMLoadFloat3(&mesh.vertex_normals[index]);
		skinned = XMVector3TransformNormal(*N, M[0]) * wei.x;
		skinned += XMVector3TransformNormal(*N, M[1]) * wei.y;
		skinned += XMVector3TransformNormal(*N, M[2]) * wei.z;
		skinned += XMVector3TransformNormal(*N, M[3]) * wei.w;
		*N = XMVector3Normalize(skinned);
	}

	return P;
}

void wiScene::Scene::RunAnimationUpdateSystem(wiJobSystem::context& ctx, float dt)
{
	for (size_t i = 0; i < animations.GetCount(); ++i)
	{
		AnimationComponent& animation = animations[i];
		if (!animation.IsPlaying() && animation.timer == 0.0f)
		{
			continue;
		}

		for (const AnimationComponent::AnimationChannel& channel : animation.channels)
		{
			assert(channel.samplerIndex < (int)animation.samplers.size());
			AnimationComponent::AnimationSampler& sampler = animation.samplers[channel.samplerIndex];
			if (sampler.data == INVALID_ENTITY)
			{
				// backwards-compatibility mode
				sampler.data = CreateEntity();
				animation_datas.Create(sampler.data) = sampler.backwards_compatibility_data;
				sampler.backwards_compatibility_data.keyframe_times.clear();
				sampler.backwards_compatibility_data.keyframe_data.clear();
			}
			const AnimationDataComponent* animationdata = animation_datas.GetComponent(sampler.data);
			if (animationdata == nullptr)
			{
				continue;
			}

			int keyLeft = 0;
			int keyRight = 0;

			if (animationdata->keyframe_times.back() < animation.timer)
			{
				// Rightmost keyframe is already outside animation, so just snap to last keyframe:
				keyLeft = keyRight = (int)animationdata->keyframe_times.size() - 1;
			}
			else
			{
				// Search for the right keyframe (greater/equal to anim time):
				while (animationdata->keyframe_times[keyRight++] < animation.timer) {}
				keyRight--;

				// Left keyframe is just near right:
				keyLeft = std::max(0, keyRight - 1);
			}

			float left = animationdata->keyframe_times[keyLeft];

			TransformComponent& target_transform = *transforms.GetComponent(channel.target);
			TransformComponent transform = target_transform;

			if (sampler.mode == AnimationComponent::AnimationSampler::Mode::STEP || keyLeft == keyRight)
			{
				// Nearest neighbor method (snap to left):
				switch (channel.path)
				{
					default:
					case AnimationComponent::AnimationChannel::Path::TRANSLATION:
					{
						assert(animationdata->keyframe_data.size() == animationdata->keyframe_times.size() * 3);
						transform.translation_local = ((const XMFLOAT3*)animationdata->keyframe_data.data())[keyLeft];
					}
					break;
					case AnimationComponent::AnimationChannel::Path::ROTATION:
					{
						assert(animationdata->keyframe_data.size() == animationdata->keyframe_times.size() * 4);
						transform.rotation_local = ((const XMFLOAT4*)animationdata->keyframe_data.data())[keyLeft];
					}
					break;
					case AnimationComponent::AnimationChannel::Path::SCALE:
					{
						assert(animationdata->keyframe_data.size() == animationdata->keyframe_times.size() * 3);
						transform.scale_local = ((const XMFLOAT3*)animationdata->keyframe_data.data())[keyLeft];
					}
					break;
				}
			}
			else
			{
				// Linear interpolation method:
				float right = animationdata->keyframe_times[keyRight];
				float t = (animation.timer - left) / (right - left);

				switch (channel.path)
				{
					default:
					case AnimationComponent::AnimationChannel::Path::TRANSLATION:
					{
						assert(animationdata->keyframe_data.size() == animationdata->keyframe_times.size() * 3);
						const XMFLOAT3* data = (const XMFLOAT3*)animationdata->keyframe_data.data();
						XMVECTOR vLeft = XMLoadFloat3(&data[keyLeft]);
						XMVECTOR vRight = XMLoadFloat3(&data[keyRight]);
						XMVECTOR vAnim = XMVectorLerp(vLeft, vRight, t);
						XMStoreFloat3(&transform.translation_local, vAnim);
					}
					break;
					case AnimationComponent::AnimationChannel::Path::ROTATION:
					{
						assert(animationdata->keyframe_data.size() == animationdata->keyframe_times.size() * 4);
						const XMFLOAT4* data = (const XMFLOAT4*)animationdata->keyframe_data.data();
						XMVECTOR vLeft = XMLoadFloat4(&data[keyLeft]);
						XMVECTOR vRight = XMLoadFloat4(&data[keyRight]);
						XMVECTOR vAnim = XMQuaternionSlerp(vLeft, vRight, t);
						vAnim = XMQuaternionNormalize(vAnim);
						XMStoreFloat4(&transform.rotation_local, vAnim);
					}
					break;
					case AnimationComponent::AnimationChannel::Path::SCALE:
					{
						assert(animationdata->keyframe_data.size() == animationdata->keyframe_times.size() * 3);
						const XMFLOAT3* data = (const XMFLOAT3*)animationdata->keyframe_data.data();
						XMVECTOR vLeft = XMLoadFloat3(&data[keyLeft]);
						XMVECTOR vRight = XMLoadFloat3(&data[keyRight]);
						XMVECTOR vAnim = XMVectorLerp(vLeft, vRight, t);
						XMStoreFloat3(&transform.scale_local, vAnim);
					}
					break;
				}
			}

			target_transform.SetDirty();

			const float t = animation.amount;

			const XMVECTOR aS = XMLoadFloat3(&target_transform.scale_local);
			const XMVECTOR aR = XMLoadFloat4(&target_transform.rotation_local);
			const XMVECTOR aT = XMLoadFloat3(&target_transform.translation_local);

			const XMVECTOR bS = XMLoadFloat3(&transform.scale_local);
			const XMVECTOR bR = XMLoadFloat4(&transform.rotation_local);
			const XMVECTOR bT = XMLoadFloat3(&transform.translation_local);

			const XMVECTOR S = XMVectorLerp(aS, bS, t);
			const XMVECTOR R = XMQuaternionSlerp(aR, bR, t);
			const XMVECTOR T = XMVectorLerp(aT, bT, t);

			XMStoreFloat3(&target_transform.scale_local, S);
			XMStoreFloat4(&target_transform.rotation_local, R);
			XMStoreFloat3(&target_transform.translation_local, T);

		}

		if (animation.IsPlaying())
		{
			animation.timer += dt * animation.speed;
		}

		if (animation.IsLooped() && animation.timer > animation.end)
		{
			animation.timer = animation.start;
		}
	}
}

void Scene::RunArmatureUpdateSystem(wiJobSystem::context& ctx)
{
	wiJobSystem::Dispatch(ctx, (uint32_t)armatures.GetCount(), 1, [&](wiJobArgs args)
	{
		ArmatureComponent& armature = armatures[args.jobIndex];
		Entity entity = armatures.GetEntity(args.jobIndex);
		const TransformComponent& transform = *transforms.GetComponent(entity);

		// The transform world matrices are in world space, but skinning needs them in armature-local space, 
		//	so that the skin is reusable for instanced meshes.
		//	We remove the armature's world matrix from the bone world matrix to obtain the bone local transform
		//	These local bone matrices will only be used for skinning, the actual transform components for the bones
		//	remain unchanged.
		//
		//	This is useful for an other thing too:
		//	If a whole transform tree is transformed by some parent (even gltf import does that to convert from RH to LH space)
		//	then the inverseBindMatrices are not reflected in that because they are not contained in the hierarchy system. 
		//	But this will correct them too.
		XMMATRIX R = XMMatrixInverse(nullptr, XMLoadFloat4x4(&transform.world));

		if (armature.boneData.size() != armature.boneCollection.size())
		{
			armature.boneData.resize(armature.boneCollection.size());
		}

		XMFLOAT3 _min = XMFLOAT3(FLT_MAX, FLT_MAX, FLT_MAX);
		XMFLOAT3 _max = XMFLOAT3(-FLT_MAX, -FLT_MAX, -FLT_MAX);

		int boneIndex = 0;
		for (Entity boneEntity : armature.boneCollection)
		{
			const TransformComponent& bone = *transforms.GetComponent(boneEntity);

			XMMATRIX B = XMLoadFloat4x4(&armature.inverseBindMatrices[boneIndex]);
			XMMATRIX W = XMLoadFloat4x4(&bone.world);
			XMMATRIX M = B * W * R;

			armature.boneData[boneIndex++].Store(M);

			const float bone_radius = 1;
			XMFLOAT3 bonepos = bone.GetPosition();
			AABB boneAABB;
			boneAABB.createFromHalfWidth(bonepos, XMFLOAT3(bone_radius, bone_radius, bone_radius));
			_min = wiMath::Min(_min, boneAABB._min);
			_max = wiMath::Max(_max, boneAABB._max);
		}

		armature.aabb = AABB(_min, _max);
	});
}
