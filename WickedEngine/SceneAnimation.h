#pragma once

#include "wiECS.h"
#include "wiGraphics.h"
#include "wiIntersect.h"
#include "CommonInclude.h"

#include <vector>

namespace wiScene
{
	struct ArmatureComponent
	{
		enum FLAGS { EMPTY = 0,	};
		uint32_t _flags = EMPTY;

		std::vector<wiECS::Entity> boneCollection;
		std::vector<XMFLOAT4X4> inverseBindMatrices;

		// Non-serialized attributes:
		AABB aabb;

		struct ShaderBoneType
		{
			XMFLOAT4 pose0;
			XMFLOAT4 pose1;
			XMFLOAT4 pose2;

			inline void Store(const XMMATRIX& M)
			{
				XMFLOAT4X4 mat;
				XMStoreFloat4x4(&mat, M);
				pose0 = XMFLOAT4(mat._11, mat._21, mat._31, mat._41);
				pose1 = XMFLOAT4(mat._12, mat._22, mat._32, mat._42);
				pose2 = XMFLOAT4(mat._13, mat._23, mat._33, mat._43);
			}
			inline XMMATRIX Load() const
			{
				return XMMATRIX(
					pose0.x, pose1.x, pose2.x, 0, 
					pose0.y, pose1.y, pose2.y, 0, 
					pose0.z, pose1.z, pose2.z, 0, 
					pose0.w, pose1.w, pose2.w, 1
				);
			}
		};
		std::vector<ShaderBoneType> boneData;
		wiGraphics::GPUBuffer boneBuffer;

		void Serialize(wiArchive& archive, wiECS::Entity seed = wiECS::INVALID_ENTITY);
	};

	struct AnimationDataComponent
	{
		enum FLAGS
		{
			EMPTY = 0,
		};
		uint32_t _flags = EMPTY;

		std::vector<float> keyframe_times;
		std::vector<float> keyframe_data;

		void Serialize(wiArchive& archive, wiECS::Entity seed = wiECS::INVALID_ENTITY);
	};

	struct AnimationComponent
	{
		enum FLAGS
		{
			EMPTY = 0,
			PLAYING = 1 << 0,
			LOOPED = 1 << 1,
		};
		uint32_t _flags = LOOPED;
		float start = 0;
		float end = 0;
		float timer = 0;
		float amount = 1;	// blend amount
		float speed = 1;

		struct AnimationChannel
		{
			enum FLAGS
			{
				EMPTY = 0,
			};
			uint32_t _flags = LOOPED;

			wiECS::Entity target = wiECS::INVALID_ENTITY;
			int samplerIndex = -1;

			enum Path
			{
				TRANSLATION,
				ROTATION,
				SCALE,
				UNKNOWN,
				TYPE_FORCE_UINT32 = 0xFFFFFFFF
			} path = TRANSLATION;
		};
		struct AnimationSampler
		{
			enum FLAGS
			{
				EMPTY = 0,
			};
			uint32_t _flags = LOOPED;

			wiECS::Entity data = wiECS::INVALID_ENTITY;

			enum Mode
			{
				LINEAR,
				STEP,
				MODE_FORCE_UINT32 = 0xFFFFFFFF
			} mode = LINEAR;

			// The data is now not part of the sampler, so it can be shared. This is kept only for backwards compatibility with previous versions.
			AnimationDataComponent backwards_compatibility_data;
		};
		std::vector<AnimationChannel> channels;
		std::vector<AnimationSampler> samplers;

		inline bool IsPlaying() const { return _flags & PLAYING; }
		inline bool IsLooped() const { return _flags & LOOPED; }
		inline float GetLength() const { return end - start; }
		inline bool IsEnded() const { return timer >= end; }

		inline void Play() { _flags |= PLAYING; }
		inline void Pause() { _flags &= ~PLAYING; }
		inline void Stop() { Pause(); timer = 0.0f; }
		inline void SetLooped(bool value = true) { if (value) { _flags |= LOOPED; } else { _flags &= ~LOOPED; } }

		void Serialize(wiArchive& archive, wiECS::Entity seed = wiECS::INVALID_ENTITY);
	};

	struct MeshComponent;

	// Returns skinned vertex position in armature local space
	//	N : normal (out, optional)
	XMVECTOR SkinVertex(const wiScene::MeshComponent& mesh, const wiScene::ArmatureComponent& armature, uint32_t index, XMVECTOR* N = nullptr);
}
