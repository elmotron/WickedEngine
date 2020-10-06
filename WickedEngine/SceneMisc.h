#pragma once

#include "wiECS.h"
#include "wiArchive.h"
#include "wiGraphics.h"
#include "wiIntersect.h"
#include "CommonInclude.h"

namespace wiScene
{
	struct NameComponent
	{
		std::string name;

		inline void operator=(const std::string& str) { name = str; }
		inline void operator=(std::string&& str) { name = std::move(str); }
		inline bool operator==(const std::string& str) const { return name.compare(str) == 0; }

		void Serialize(wiArchive& archive, wiECS::Entity seed = wiECS::INVALID_ENTITY);
	};

	struct ObjectComponent
	{
		enum FLAGS
		{
			EMPTY = 0,
			RENDERABLE = 1 << 0,
			CAST_SHADOW = 1 << 1,
			DYNAMIC = 1 << 2,
			IMPOSTOR_PLACEMENT = 1 << 3,
			REQUEST_PLANAR_REFLECTION = 1 << 4,
			LIGHTMAP_RENDER_REQUEST = 1 << 5,
		};
		uint32_t _flags = RENDERABLE | CAST_SHADOW;

		wiECS::Entity meshID = wiECS::INVALID_ENTITY;
		uint32_t cascadeMask = 0; // which shadow cascades to skip (0: skip none, 1: skip first, etc...)
		uint32_t rendertypeMask = 0;
		XMFLOAT4 color = XMFLOAT4(1, 1, 1, 1);

		uint32_t lightmapWidth = 0;
		uint32_t lightmapHeight = 0;
		std::vector<uint8_t> lightmapTextureData;

		uint8_t userStencilRef = 0;

		// Non-serialized attributes:

		XMFLOAT4 globalLightMapMulAdd = XMFLOAT4(0, 0, 0, 0);
		wiGraphics::Texture lightmap;
		wiGraphics::RenderPass renderpass_lightmap_clear;
		wiGraphics::RenderPass renderpass_lightmap_accumulate;
		uint32_t lightmapIterationCount = 0;

		XMFLOAT3 center = XMFLOAT3(0, 0, 0);
		float impostorFadeThresholdRadius;
		float impostorSwapDistance;

		// these will only be valid for a single frame:
		int transform_index = -1;
		int prev_transform_index = -1;

		// occlusion result history bitfield (32 bit->32 frame history)
		uint32_t occlusionHistory = ~0;
		// occlusion query pool index
		int occlusionQueryID = -1;

		inline bool IsOccluded() const
		{
			// Perform a conservative occlusion test:
			// If it is visible in any frames in the history, it is determined visible in this frame
			// But if all queries failed in the history, it is occluded.
			// If it pops up for a frame after occluded, it is visible again for some frames
			return ((occlusionQueryID >= 0) && (occlusionHistory & 0xFFFFFFFF) == 0);
		}

		inline void SetRenderable(bool value) { if (value) { _flags |= RENDERABLE; } else { _flags &= ~RENDERABLE; } }
		inline void SetCastShadow(bool value) { if (value) { _flags |= CAST_SHADOW; } else { _flags &= ~CAST_SHADOW; } }
		inline void SetDynamic(bool value) { if (value) { _flags |= DYNAMIC; } else { _flags &= ~DYNAMIC; } }
		inline void SetImpostorPlacement(bool value) { if (value) { _flags |= IMPOSTOR_PLACEMENT; } else { _flags &= ~IMPOSTOR_PLACEMENT; } }
		inline void SetRequestPlanarReflection(bool value) { if (value) { _flags |= REQUEST_PLANAR_REFLECTION; } else { _flags &= ~REQUEST_PLANAR_REFLECTION; } }
		inline void SetLightmapRenderRequest(bool value) { if (value) { _flags |= LIGHTMAP_RENDER_REQUEST; } else { _flags &= ~LIGHTMAP_RENDER_REQUEST; } }

		inline bool IsRenderable() const { return _flags & RENDERABLE; }
		inline bool IsCastingShadow() const { return _flags & CAST_SHADOW; }
		inline bool IsDynamic() const { return _flags & DYNAMIC; }
		inline bool IsImpostorPlacement() const { return _flags & IMPOSTOR_PLACEMENT; }
		inline bool IsRequestPlanarReflection() const { return _flags & REQUEST_PLANAR_REFLECTION; }
		inline bool IsLightmapRenderRequested() const { return _flags & LIGHTMAP_RENDER_REQUEST; }

		inline float GetTransparency() const { return 1 - color.w; }
		inline uint32_t GetRenderTypes() const { return rendertypeMask; }

		// User stencil value can be in range [0, 15]
		//	Values greater than 0 can be used to override userStencilRef of MaterialComponent
		inline void SetUserStencilRef(uint8_t value)
		{
			assert(value < 16);
			userStencilRef = value & 0x0F;
		}

		void ClearLightmap();
		void SaveLightmap();
		wiGraphics::FORMAT GetLightmapFormat();

		void Serialize(wiArchive& archive, wiECS::Entity seed = wiECS::INVALID_ENTITY);
	};
}