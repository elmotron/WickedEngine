#pragma once

#include "wiGraphics.h"
#include "ShaderInterop_Renderer.h"
#include "wiResourceManager.h"
#include "wiRenderer.h"
#include "wiEnums.h"
#include "CommonInclude.h"

namespace wiScene
{
	struct LayerComponent
	{
		uint32_t layerMask = ~0;

		inline uint32_t GetLayerMask() const { return layerMask; }

		void Serialize(wiArchive& archive, wiECS::Entity seed = wiECS::INVALID_ENTITY);
	};

	struct MaterialComponent
	{
		enum FLAGS
		{
			EMPTY = 0,
			DIRTY = 1 << 0,
			CAST_SHADOW = 1 << 1,
			PLANAR_REFLECTION = 1 << 2,
			WATER = 1 << 3,
			FLIP_NORMALMAP = 1 << 4,
			USE_VERTEXCOLORS = 1 << 5,
			SPECULAR_GLOSSINESS_WORKFLOW = 1 << 6,
			OCCLUSION_PRIMARY = 1 << 7,
			OCCLUSION_SECONDARY = 1 << 8,
			USE_WIND = 1 << 9,
		};
		uint32_t _flags = DIRTY | CAST_SHADOW;

		STENCILREF engineStencilRef = STENCILREF_DEFAULT;
		uint8_t userStencilRef = 0;
		BLENDMODE userBlendMode = BLENDMODE_OPAQUE;

		XMFLOAT4 baseColor = XMFLOAT4(1, 1, 1, 1);
		XMFLOAT4 emissiveColor = XMFLOAT4(1, 1, 1, 0);
		XMFLOAT4 texMulAdd = XMFLOAT4(1, 1, 0, 0);
		float roughness = 0.2f;
		float reflectance = 0.02f;
		float metalness = 0.0f;
		float refractionIndex = 0.0f;
		float subsurfaceScattering = 0.0f;
		float normalMapStrength = 1.0f;
		float parallaxOcclusionMapping = 0.0f;
		float displacementMapping = 0.0f;

		float alphaRef = 1.0f;
		wiGraphics::SHADING_RATE shadingRate = wiGraphics::SHADING_RATE_1X1;
		
		XMFLOAT2 texAnimDirection = XMFLOAT2(0, 0);
		float texAnimFrameRate = 0.0f;
		float texAnimElapsedTime = 0.0f;

		std::string baseColorMapName;
		std::string surfaceMapName;
		std::string normalMapName;
		std::string displacementMapName;
		std::string emissiveMapName;
		std::string occlusionMapName;

		uint32_t uvset_baseColorMap = 0;
		uint32_t uvset_surfaceMap = 0;
		uint32_t uvset_normalMap = 0;
		uint32_t uvset_displacementMap = 0;
		uint32_t uvset_emissiveMap = 0;
		uint32_t uvset_occlusionMap = 0;

		// Non-serialized attributes:
		std::shared_ptr<wiResource> baseColorMap;
		std::shared_ptr<wiResource> surfaceMap;
		std::shared_ptr<wiResource> normalMap;
		std::shared_ptr<wiResource> displacementMap;
		std::shared_ptr<wiResource> emissiveMap;
		std::shared_ptr<wiResource> occlusionMap;
		wiGraphics::GPUBuffer constantBuffer;

		int customShaderID = -1; // for now, this is not serialized; need to consider actual proper use case first

		// User stencil value can be in range [0, 15]
		inline void SetUserStencilRef(uint8_t value)
		{
			assert(value < 16);
			userStencilRef = value & 0x0F;
		}
		inline uint32_t GetStencilRef() const
		{
			return wiRenderer::CombineStencilrefs(engineStencilRef, userStencilRef);
		}

		const wiGraphics::Texture* GetBaseColorMap() const;
		const wiGraphics::Texture* GetNormalMap() const;
		const wiGraphics::Texture* GetSurfaceMap() const;
		const wiGraphics::Texture* GetDisplacementMap() const;
		const wiGraphics::Texture* GetEmissiveMap() const;
		const wiGraphics::Texture* GetOcclusionMap() const;

		inline float GetOpacity() const { return baseColor.w; }
		inline float GetEmissiveStrength() const { return emissiveColor.w; }
		inline int GetCustomShaderID() const { return customShaderID; }

		inline void SetDirty(bool value = true) { if (value) { _flags |= DIRTY; } else { _flags &= ~DIRTY; } }
		inline bool IsDirty() const { return _flags & DIRTY; }

		inline void SetCastShadow(bool value) { if (value) { _flags |= CAST_SHADOW; } else { _flags &= ~CAST_SHADOW; } }
		inline void SetPlanarReflections(bool value) { if (value) { _flags |= PLANAR_REFLECTION; } else { _flags &= ~PLANAR_REFLECTION; } }
		inline void SetWater(bool value) { if (value) { _flags |= WATER; } else { _flags &= ~WATER; } }
		inline void SetOcclusionEnabled_Primary(bool value) { SetDirty(); if (value) { _flags |= OCCLUSION_PRIMARY; } else { _flags &= ~OCCLUSION_PRIMARY; } }
		inline void SetOcclusionEnabled_Secondary(bool value) { SetDirty(); if (value) { _flags |= OCCLUSION_SECONDARY; } else { _flags &= ~OCCLUSION_SECONDARY; } }

		inline bool IsTransparent() const { return userBlendMode != BLENDMODE_OPAQUE || IsCustomShader() || IsWater(); }
		inline BLENDMODE GetBlendMode() const { if (userBlendMode == BLENDMODE_OPAQUE && IsTransparent()) { return BLENDMODE_ALPHA; } else return userBlendMode; }
		inline bool IsWater() const { return _flags & WATER; }
		inline bool HasPlanarReflection() const { return (_flags & PLANAR_REFLECTION) || IsWater(); }
		inline bool IsCastingShadow() const { return _flags & CAST_SHADOW; }
		inline bool IsAlphaTestEnabled() const { return alphaRef <= 1.0f - 1.0f / 256.0f; }
		inline bool IsFlipNormalMap() const { return _flags & FLIP_NORMALMAP; }
		inline bool IsUsingVertexColors() const { return _flags & USE_VERTEXCOLORS; }
		inline bool IsUsingWind() const { return _flags & USE_WIND; }
		inline bool IsUsingSpecularGlossinessWorkflow() const { return _flags & SPECULAR_GLOSSINESS_WORKFLOW; }
		inline bool IsOcclusionEnabled_Primary() const { return _flags & OCCLUSION_PRIMARY; }
		inline bool IsOcclusionEnabled_Secondary() const { return _flags & OCCLUSION_SECONDARY; }
		inline bool IsCustomShader() const { return customShaderID >= 0; }

		inline void SetBaseColor(const XMFLOAT4& value) { SetDirty(); baseColor = value; }
		inline void SetEmissiveColor(const XMFLOAT4& value) { SetDirty(); emissiveColor = value; }
		inline void SetRoughness(float value) { SetDirty(); roughness = value; }
		inline void SetReflectance(float value) { SetDirty(); reflectance = value; }
		inline void SetMetalness(float value) { SetDirty(); metalness = value; }
		inline void SetEmissiveStrength(float value) { SetDirty(); emissiveColor.w = value; }
		inline void SetRefractionIndex(float value) { SetDirty(); refractionIndex = value; }
		inline void SetSubsurfaceScattering(float value) { SetDirty(); subsurfaceScattering = value; }
		inline void SetNormalMapStrength(float value) { SetDirty(); normalMapStrength = value; }
		inline void SetParallaxOcclusionMapping(float value) { SetDirty(); parallaxOcclusionMapping = value; }
		inline void SetDisplacementMapping(float value) { SetDirty(); displacementMapping = value; }
		inline void SetOpacity(float value) { SetDirty(); baseColor.w = value; }
		inline void SetAlphaRef(float value) { SetDirty();  alphaRef = value; }
		inline void SetFlipNormalMap(bool value) { SetDirty(); if (value) { _flags |= FLIP_NORMALMAP; } else { _flags &= ~FLIP_NORMALMAP; } }
		inline void SetUseVertexColors(bool value) { SetDirty(); if (value) { _flags |= USE_VERTEXCOLORS; } else { _flags &= ~USE_VERTEXCOLORS; } }
		inline void SetUseWind(bool value) { SetDirty(); if (value) { _flags |= USE_WIND; } else { _flags &= ~USE_WIND; } }
		inline void SetUseSpecularGlossinessWorkflow(bool value) { SetDirty(); if (value) { _flags |= SPECULAR_GLOSSINESS_WORKFLOW; } else { _flags &= ~SPECULAR_GLOSSINESS_WORKFLOW; }  }
		inline void SetCustomShaderID(int id) { customShaderID = id; }
		inline void DisableCustomShader() { customShaderID = -1; }
		inline void SetUVSet_BaseColorMap(uint32_t value) { uvset_baseColorMap = value; SetDirty(); }
		inline void SetUVSet_NormalMap(uint32_t value) { uvset_normalMap = value; SetDirty(); }
		inline void SetUVSet_SurfaceMap(uint32_t value) { uvset_surfaceMap = value; SetDirty(); }
		inline void SetUVSet_DisplacementMap(uint32_t value) { uvset_displacementMap = value; SetDirty(); }
		inline void SetUVSet_EmissiveMap(uint32_t value) { uvset_emissiveMap = value; SetDirty(); }
		inline void SetUVSet_OcclusionMap(uint32_t value) { uvset_occlusionMap = value; SetDirty(); }

		ShaderMaterial CreateShaderMaterial() const;

		void Serialize(wiArchive& archive, wiECS::Entity seed = wiECS::INVALID_ENTITY);
	};

	struct MeshComponent
	{
		enum FLAGS
		{
			EMPTY = 0,
			RENDERABLE = 1 << 0,
			DOUBLE_SIDED = 1 << 1,
			DYNAMIC = 1 << 2,
			TERRAIN = 1 << 3,
		};
		uint32_t _flags = RENDERABLE;

		std::vector<XMFLOAT3> vertex_positions;
		std::vector<XMFLOAT3> vertex_normals;
		std::vector<XMFLOAT2> vertex_uvset_0;
		std::vector<XMFLOAT2> vertex_uvset_1;
		std::vector<XMUINT4> vertex_boneindices;
		std::vector<XMFLOAT4> vertex_boneweights;
		std::vector<XMFLOAT2> vertex_atlas;
		std::vector<uint32_t> vertex_colors;
		std::vector<uint8_t> vertex_windweights;
		std::vector<uint32_t> indices;

		struct MeshSubset
		{
			wiECS::Entity materialID = wiECS::INVALID_ENTITY;
			uint32_t indexOffset = 0;
			uint32_t indexCount = 0;
		};
		std::vector<MeshSubset> subsets;

		float tessellationFactor = 0.0f;
		wiECS::Entity armatureID = wiECS::INVALID_ENTITY;

		// Terrain blend materials:
		//	There are 4 blend materials, the first one (default) being the subset material
		//	Must have TERRAIN flag enabled
		//	Must have vertex colors to blend between materials
		//	extra materials that are not set will use the base subset material
		wiECS::Entity terrain_material1 = wiECS::INVALID_ENTITY;
		wiECS::Entity terrain_material2 = wiECS::INVALID_ENTITY;
		wiECS::Entity terrain_material3 = wiECS::INVALID_ENTITY;

		// Non-serialized attributes:
		AABB aabb;
		wiGraphics::GPUBuffer indexBuffer;
		wiGraphics::GPUBuffer vertexBuffer_POS;
		wiGraphics::GPUBuffer vertexBuffer_UV0;
		wiGraphics::GPUBuffer vertexBuffer_UV1;
		wiGraphics::GPUBuffer vertexBuffer_BON;
		wiGraphics::GPUBuffer vertexBuffer_COL;
		wiGraphics::GPUBuffer vertexBuffer_ATL;
		wiGraphics::GPUBuffer vertexBuffer_PRE;
		wiGraphics::GPUBuffer streamoutBuffer_POS;
		wiGraphics::GPUBuffer vertexBuffer_SUB;
		std::vector<uint8_t> vertex_subsets;

		wiGraphics::RaytracingAccelerationStructure BLAS;
		bool BLAS_build_pending = true;
		uint32_t TLAS_geometryOffset = 0;

		inline void SetRenderable(bool value) { if (value) { _flags |= RENDERABLE; } else { _flags &= ~RENDERABLE; } }
		inline void SetDoubleSided(bool value) { if (value) { _flags |= DOUBLE_SIDED; } else { _flags &= ~DOUBLE_SIDED; } }
		inline void SetDynamic(bool value) { if (value) { _flags |= DYNAMIC; } else { _flags &= ~DYNAMIC; } }
		inline void SetTerrain(bool value) { if (value) { _flags |= TERRAIN; } else { _flags &= ~TERRAIN; } }

		inline bool IsRenderable() const { return _flags & RENDERABLE; }
		inline bool IsDoubleSided() const { return _flags & DOUBLE_SIDED; }
		inline bool IsDynamic() const { return _flags & DYNAMIC; }
		inline bool IsTerrain() const { return _flags & TERRAIN; }

		inline float GetTessellationFactor() const { return tessellationFactor; }
		inline wiGraphics::INDEXBUFFER_FORMAT GetIndexFormat() const { return vertex_positions.size() > 65535 ? wiGraphics::INDEXFORMAT_32BIT : wiGraphics::INDEXFORMAT_16BIT; }
		inline size_t GetIndexStride() const { return GetIndexFormat() == wiGraphics::INDEXFORMAT_32BIT ? sizeof(uint32_t) : sizeof(uint16_t); }
		inline bool IsSkinned() const { return armatureID != wiECS::INVALID_ENTITY; }

		void CreateRenderData();
		enum COMPUTE_NORMALS
		{
			COMPUTE_NORMALS_HARD,		// hard face normals, can result in additional vertices generated
			COMPUTE_NORMALS_SMOOTH,		// smooth per vertex normals, this can remove/simplyfy geometry, but slow
			COMPUTE_NORMALS_SMOOTH_FAST	// average normals, vertex count will be unchanged, fast
		};
		void ComputeNormals(COMPUTE_NORMALS compute);
		void FlipCulling();
		void FlipNormals();
		void Recenter();
		void RecenterToBottom();
		SPHERE GetBoundingSphere() const;

		void Serialize(wiArchive& archive, wiECS::Entity seed = wiECS::INVALID_ENTITY);


		struct Vertex_POS
		{
			XMFLOAT3 pos = XMFLOAT3(0.0f, 0.0f, 0.0f);
			uint32_t normal_wind = 0;

			void FromFULL(const XMFLOAT3& _pos, const XMFLOAT3& _nor, uint8_t wind)
			{
				pos.x = _pos.x;
				pos.y = _pos.y;
				pos.z = _pos.z;
				MakeFromParams(_nor, wind);
			}
			inline XMVECTOR LoadPOS() const
			{
				return XMLoadFloat3(&pos);
			}
			inline XMVECTOR LoadNOR() const
			{
				XMFLOAT3 N = GetNor_FULL();
				return XMLoadFloat3(&N);
			}
			inline void MakeFromParams(const XMFLOAT3& normal)
			{
				normal_wind = normal_wind & 0xFF000000; // reset only the normals

				normal_wind |= (uint32_t)((normal.x * 0.5f + 0.5f) * 255.0f) << 0;
				normal_wind |= (uint32_t)((normal.y * 0.5f + 0.5f) * 255.0f) << 8;
				normal_wind |= (uint32_t)((normal.z * 0.5f + 0.5f) * 255.0f) << 16;
			}
			inline void MakeFromParams(const XMFLOAT3& normal, uint8_t wind)
			{
				normal_wind = 0;

				normal_wind |= (uint32_t)((normal.x * 0.5f + 0.5f) * 255.0f) << 0;
				normal_wind |= (uint32_t)((normal.y * 0.5f + 0.5f) * 255.0f) << 8;
				normal_wind |= (uint32_t)((normal.z * 0.5f + 0.5f) * 255.0f) << 16;
				normal_wind |= (uint32_t)wind << 24;
			}
			inline XMFLOAT3 GetNor_FULL() const
			{
				XMFLOAT3 nor_FULL(0, 0, 0);

				nor_FULL.x = (float)((normal_wind >> 0) & 0x000000FF) / 255.0f * 2.0f - 1.0f;
				nor_FULL.y = (float)((normal_wind >> 8) & 0x000000FF) / 255.0f * 2.0f - 1.0f;
				nor_FULL.z = (float)((normal_wind >> 16) & 0x000000FF) / 255.0f * 2.0f - 1.0f;

				return nor_FULL;
			}
			inline uint8_t GetWind() const
			{
				return (normal_wind >> 24) & 0x000000FF;
			}

			static const wiGraphics::FORMAT FORMAT = wiGraphics::FORMAT::FORMAT_R32G32B32A32_FLOAT;
		};
		struct Vertex_TEX
		{
			XMHALF2 tex = XMHALF2(0.0f, 0.0f);

			void FromFULL(const XMFLOAT2& texcoords)
			{
				tex = XMHALF2(texcoords.x, texcoords.y);
			}

			static const wiGraphics::FORMAT FORMAT = wiGraphics::FORMAT::FORMAT_R16G16_FLOAT;
		};
		struct Vertex_BON
		{
			uint64_t ind = 0;
			uint64_t wei = 0;

			void FromFULL(const XMUINT4& boneIndices, const XMFLOAT4& boneWeights)
			{
				ind = 0;
				wei = 0;

				ind |= (uint64_t)boneIndices.x << 0;
				ind |= (uint64_t)boneIndices.y << 16;
				ind |= (uint64_t)boneIndices.z << 32;
				ind |= (uint64_t)boneIndices.w << 48;

				wei |= (uint64_t)(boneWeights.x * 65535.0f) << 0;
				wei |= (uint64_t)(boneWeights.y * 65535.0f) << 16;
				wei |= (uint64_t)(boneWeights.z * 65535.0f) << 32;
				wei |= (uint64_t)(boneWeights.w * 65535.0f) << 48;
			}
			inline XMUINT4 GetInd_FULL() const
			{
				XMUINT4 ind_FULL(0, 0, 0, 0);

				ind_FULL.x = ((ind >> 0) & 0x0000FFFF);
				ind_FULL.y = ((ind >> 16) & 0x0000FFFF);
				ind_FULL.z = ((ind >> 32) & 0x0000FFFF);
				ind_FULL.w = ((ind >> 48) & 0x0000FFFF);

				return ind_FULL;
			}
			inline XMFLOAT4 GetWei_FULL() const
			{
				XMFLOAT4 wei_FULL(0, 0, 0, 0);

				wei_FULL.x = (float)((wei >> 0) & 0x0000FFFF) / 65535.0f;
				wei_FULL.y = (float)((wei >> 16) & 0x0000FFFF) / 65535.0f;
				wei_FULL.z = (float)((wei >> 32) & 0x0000FFFF) / 65535.0f;
				wei_FULL.w = (float)((wei >> 48) & 0x0000FFFF) / 65535.0f;

				return wei_FULL;
			}
		};
		struct Vertex_COL
		{
			uint32_t color = 0;
			static const wiGraphics::FORMAT FORMAT = wiGraphics::FORMAT::FORMAT_R8G8B8A8_UNORM;
		};

	};

	struct ImpostorComponent
	{
		enum FLAGS
		{
			EMPTY = 0,
			DIRTY = 1 << 0,
		};
		uint32_t _flags = DIRTY;

		float swapInDistance = 100.0f;

		// Non-serialized attributes:
		AABB aabb;
		XMFLOAT4 color;
		float fadeThresholdRadius;
		std::vector<XMFLOAT4X4> instanceMatrices;

		inline void SetDirty(bool value = true) { if (value) { _flags |= DIRTY; } else { _flags &= ~DIRTY; } }
		inline bool IsDirty() const { return _flags & DIRTY; }

		void Serialize(wiArchive& archive, wiECS::Entity seed = wiECS::INVALID_ENTITY);
	};

	struct EnvironmentProbeComponent
	{
		enum FLAGS
		{
			EMPTY = 0,
			DIRTY = 1 << 0,
			REALTIME = 1 << 1,
		};
		uint32_t _flags = DIRTY;

		// Non-serialized attributes:
		int textureIndex = -1;
		XMFLOAT3 position;
		float range;
		XMFLOAT4X4 inverseMatrix;

		inline void SetDirty(bool value = true) { if (value) { _flags |= DIRTY; } else { _flags &= ~DIRTY; } }
		inline void SetRealTime(bool value) { if (value) { _flags |= REALTIME; } else { _flags &= ~REALTIME; } }

		inline bool IsDirty() const { return _flags & DIRTY; }
		inline bool IsRealTime() const { return _flags & REALTIME; }

		void Serialize(wiArchive& archive, wiECS::Entity seed = wiECS::INVALID_ENTITY);
	};

	struct DecalComponent
	{
		enum FLAGS
		{
			EMPTY = 0,
		};
		uint32_t _flags = EMPTY;

		// Non-serialized attributes:
		XMFLOAT4 color;
		float emissive;
		XMFLOAT3 front;
		XMFLOAT3 position;
		float range;
		XMFLOAT4 atlasMulAdd;
		XMFLOAT4X4 world;

		std::shared_ptr<wiResource> texture;
		std::shared_ptr<wiResource> normal;

		inline float GetOpacity() const { return color.w; }

		void Serialize(wiArchive& archive, wiECS::Entity seed = wiECS::INVALID_ENTITY);
	};

	struct WeatherComponent
	{
		enum FLAGS
		{
			EMPTY = 0,
			OCEAN_ENABLED = 1 << 0,
			SIMPLE_SKY = 1 << 1,
		};
		uint32_t _flags = EMPTY;

		inline bool IsOceanEnabled() const { return _flags & OCEAN_ENABLED; }
		inline bool IsSimpleSky() const { return _flags & SIMPLE_SKY; }

		inline void SetOceanEnabled(bool value = true) { if (value) { _flags |= OCEAN_ENABLED; } else { _flags &= ~OCEAN_ENABLED; } }
		inline void SetSimpleSky(bool value = true) { if (value) { _flags |= SIMPLE_SKY; } else { _flags &= ~SIMPLE_SKY; } }

		XMFLOAT3 sunColor = XMFLOAT3(0, 0, 0);
		XMFLOAT3 sunDirection = XMFLOAT3(0, 1, 0);
		float sunEnergy = 0;
		XMFLOAT3 horizon = XMFLOAT3(0.0f, 0.0f, 0.0f);
		XMFLOAT3 zenith = XMFLOAT3(0.0f, 0.0f, 0.0f);
		XMFLOAT3 ambient = XMFLOAT3(0.2f, 0.2f, 0.2f);
		float fogStart = 100;
		float fogEnd = 1000;
		float fogHeight = 0;
		float cloudiness = 0.0f;
		float cloudScale = 0.0003f;
		float cloudSpeed = 0.1f;
		XMFLOAT3 windDirection = XMFLOAT3(0, 0, 0);
		float windRandomness = 5;
		float windWaveSize = 1;
		float windSpeed = 1;

		struct OceanParameters
		{
			// Must be power of 2.
			int dmap_dim = 512;
			// Typical value is 1000 ~ 2000
			float patch_length = 50.0f;

			// Adjust the time interval for simulation.
			float time_scale = 0.3f;
			// Amplitude for transverse wave. Around 1.0
			float wave_amplitude = 1000.0f;
			// Wind direction. Normalization not required.
			XMFLOAT2 wind_dir = XMFLOAT2(0.8f, 0.6f);
			// Around 100 ~ 1000
			float wind_speed = 600.0f;
			// This value damps out the waves against the wind direction.
			// Smaller value means higher wind dependency.
			float wind_dependency = 0.07f;
			// The amplitude for longitudinal wave. Must be positive.
			float choppy_scale = 1.3f;


			XMFLOAT3 waterColor = XMFLOAT3(0.0f, 3.0f / 255.0f, 31.0f / 255.0f);
			float waterHeight = 0.0f;
			uint32_t surfaceDetail = 4;
			float surfaceDisplacementTolerance = 2;
		};
		OceanParameters oceanParameters;

		std::string skyMapName;
		std::shared_ptr<wiResource> skyMap;

		// Non-serialized attributes:
		uint32_t most_important_light_index = ~0;

		void Serialize(wiArchive& archive, wiECS::Entity seed = wiECS::INVALID_ENTITY);
	};

	struct LightComponent
	{
		enum FLAGS
		{
			EMPTY = 0,
			CAST_SHADOW = 1 << 0,
			VOLUMETRICS = 1 << 1,
			VISUALIZER = 1 << 2,
			LIGHTMAPONLY_STATIC = 1 << 3,
		};
		uint32_t _flags = EMPTY;
		XMFLOAT3 color = XMFLOAT3(1, 1, 1);

		enum LightType 
		{
			DIRECTIONAL = ENTITY_TYPE_DIRECTIONALLIGHT,
			POINT = ENTITY_TYPE_POINTLIGHT,
			SPOT = ENTITY_TYPE_SPOTLIGHT,
			SPHERE = ENTITY_TYPE_SPHERELIGHT,
			DISC = ENTITY_TYPE_DISCLIGHT,
			RECTANGLE = ENTITY_TYPE_RECTANGLELIGHT,
			TUBE = ENTITY_TYPE_TUBELIGHT,
			LIGHTTYPE_COUNT,
			ENUM_FORCE_UINT32 = 0xFFFFFFFF,
		};
		LightType type = POINT;
		float energy = 1.0f;
		float range_local = 10.0f;
		float fov = XM_PIDIV4;
		float shadowBias = 0.0001f;
		float radius = 1.0f; // area light
		float width = 1.0f;  // area light
		float height = 1.0f; // area light

		std::vector<std::string> lensFlareNames;

		// Non-serialized attributes:
		XMFLOAT3 position;
		float range_global;
		XMFLOAT3 direction;
		XMFLOAT4 rotation;
		XMFLOAT3 scale;
		XMFLOAT3 front;
		XMFLOAT3 right;
		int shadowMap_index = -1;

		std::vector<std::shared_ptr<wiResource>> lensFlareRimTextures;

		inline void SetCastShadow(bool value) { if (value) { _flags |= CAST_SHADOW; } else { _flags &= ~CAST_SHADOW; } }
		inline void SetVolumetricsEnabled(bool value) { if (value) { _flags |= VOLUMETRICS; } else { _flags &= ~VOLUMETRICS; } }
		inline void SetVisualizerEnabled(bool value) { if (value) { _flags |= VISUALIZER; } else { _flags &= ~VISUALIZER; } }
		inline void SetStatic(bool value) { if (value) { _flags |= LIGHTMAPONLY_STATIC; } else { _flags &= ~LIGHTMAPONLY_STATIC; } }

		inline bool IsCastingShadow() const { return _flags & CAST_SHADOW; }
		inline bool IsVolumetricsEnabled() const { return _flags & VOLUMETRICS; }
		inline bool IsVisualizerEnabled() const { return _flags & VISUALIZER; }
		inline bool IsStatic() const { return _flags & LIGHTMAPONLY_STATIC; }

		inline float GetRange() const { return range_global; }

		inline void SetType(LightType val) {
			type = val;
		}
		inline LightType GetType() const { return type; }

		void Serialize(wiArchive& archive, wiECS::Entity seed = wiECS::INVALID_ENTITY);
	};

	struct TransformComponent;

	struct CameraComponent
	{
		enum FLAGS
		{
			EMPTY = 0,
			DIRTY = 1 << 0,
			CUSTOM_PROJECTION = 1 << 1,
		};
		uint32_t _flags = EMPTY;

		float width = 0.0f;
		float height = 0.0f;
		float zNearP = 0.001f;
		float zFarP = 800.0f;
		float fov = XM_PI / 3.0f;

		// Non-serialized attributes:
		XMFLOAT3 Eye = XMFLOAT3(0, 0, 0);
		XMFLOAT3 At = XMFLOAT3(0, 0, 1);
		XMFLOAT3 Up = XMFLOAT3(0, 1, 0);
		XMFLOAT3X3 rotationMatrix;
		XMFLOAT4X4 View, Projection, VP;
		Frustum frustum;
		XMFLOAT4X4 InvView, InvProjection, InvVP;
		XMFLOAT2 jitter;

		void CreatePerspective(float newWidth, float newHeight, float newNear, float newFar, float newFOV = XM_PI / 3.0f);
		void UpdateCamera();
		void TransformCamera(const TransformComponent& transform);
		void Reflect(const XMFLOAT4& plane = XMFLOAT4(0, 1, 0, 0));

		inline XMVECTOR GetEye() const { return XMLoadFloat3(&Eye); }
		inline XMVECTOR GetAt() const { return XMLoadFloat3(&At); }
		inline XMVECTOR GetUp() const { return XMLoadFloat3(&Up); }
		inline XMVECTOR GetRight() const { return XMVector3Cross(GetAt(), GetUp()); }
		inline XMMATRIX GetView() const { return XMLoadFloat4x4(&View); }
		inline XMMATRIX GetInvView() const { return XMLoadFloat4x4(&InvView); }
		inline XMMATRIX GetProjection() const { return XMLoadFloat4x4(&Projection); }
		inline XMMATRIX GetInvProjection() const { return XMLoadFloat4x4(&InvProjection); }
		inline XMMATRIX GetViewProjection() const { return XMLoadFloat4x4(&VP); }
		inline XMMATRIX GetInvViewProjection() const { return XMLoadFloat4x4(&InvVP); }

		inline void SetDirty(bool value = true) { if (value) { _flags |= DIRTY; } else { _flags &= ~DIRTY; } }
		inline void SetCustomProjectionEnabled(bool value = true) { if (value) { _flags |= CUSTOM_PROJECTION; } else { _flags &= ~CUSTOM_PROJECTION; } }
		inline bool IsDirty() const { return _flags & DIRTY; }
		inline bool IsCustomProjectionEnabled() const { return _flags & CUSTOM_PROJECTION; }

		void Serialize(wiArchive& archive, wiECS::Entity seed = wiECS::INVALID_ENTITY);
	};
}