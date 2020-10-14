#include "wiScene.h"
#include "wiRenderer.h"
#include "wiTextureHelper.h"
#include "wiJobSystem.h"

using namespace wiGraphics;

const wiGraphics::Texture* wiScene::MaterialComponent::GetBaseColorMap() const
{
	if (baseColorMap != nullptr)
	{
		return baseColorMap->texture;
	}
	return wiTextureHelper::getWhite();
}

const Texture* wiScene::MaterialComponent::GetNormalMap() const
{
	if (normalMap != nullptr)
	{
		return normalMap->texture;
	}
	return nullptr;
}

const Texture* wiScene::MaterialComponent::GetSurfaceMap() const
{
	if (surfaceMap != nullptr)
	{
		return surfaceMap->texture;
	}
	return wiTextureHelper::getWhite();
}

const Texture* wiScene::MaterialComponent::GetDisplacementMap() const
{
	if (displacementMap != nullptr)
	{
		return displacementMap->texture;
	}
	return wiTextureHelper::getWhite();
}

const Texture* wiScene::MaterialComponent::GetEmissiveMap() const
{
	if (emissiveMap != nullptr)
	{
		return emissiveMap->texture;
	}
	return wiTextureHelper::getWhite();
}

const Texture* wiScene::MaterialComponent::GetOcclusionMap() const
{
	if (occlusionMap != nullptr)
	{
		return occlusionMap->texture;
	}
	return wiTextureHelper::getWhite();
}

ShaderMaterial wiScene::MaterialComponent::CreateShaderMaterial() const
{
	ShaderMaterial retVal;
	retVal.baseColor = baseColor;
	retVal.emissiveColor = emissiveColor;
	retVal.texMulAdd = texMulAdd;
	retVal.roughness = roughness;
	retVal.reflectance = reflectance;
	retVal.metalness = metalness;
	retVal.refractionIndex = refractionIndex;
	retVal.subsurfaceScattering = subsurfaceScattering;
	retVal.normalMapStrength = (normalMap == nullptr ? 0 : normalMapStrength);
	retVal.normalMapFlip = (_flags & MaterialComponent::FLIP_NORMALMAP ? -1.0f : 1.0f);
	retVal.parallaxOcclusionMapping = parallaxOcclusionMapping;
	retVal.displacementMapping = displacementMapping;
	retVal.uvset_baseColorMap = baseColorMap == nullptr ? -1 : (int)uvset_baseColorMap;
	retVal.uvset_surfaceMap = surfaceMap == nullptr ? -1 : (int)uvset_surfaceMap;
	retVal.uvset_normalMap = normalMap == nullptr ? -1 : (int)uvset_normalMap;
	retVal.uvset_displacementMap = displacementMap == nullptr ? -1 : (int)uvset_displacementMap;
	retVal.uvset_emissiveMap = emissiveMap == nullptr ? -1 : (int)uvset_emissiveMap;
	retVal.uvset_occlusionMap = occlusionMap == nullptr ? -1 : (int)uvset_occlusionMap;
	retVal.options = 0;
	if (IsUsingVertexColors())
	{
		retVal.options |= SHADERMATERIAL_OPTION_BIT_USE_VERTEXCOLORS;
	}
	if (IsUsingSpecularGlossinessWorkflow())
	{
		retVal.options |= SHADERMATERIAL_OPTION_BIT_SPECULARGLOSSINESS_WORKFLOW;
	}
	if (IsOcclusionEnabled_Primary())
	{
		retVal.options |= SHADERMATERIAL_OPTION_BIT_OCCLUSION_PRIMARY;
	}
	if (IsOcclusionEnabled_Secondary())
	{
		retVal.options |= SHADERMATERIAL_OPTION_BIT_OCCLUSION_SECONDARY;
	}
	if (IsUsingWind())
	{
		retVal.options |= SHADERMATERIAL_OPTION_BIT_USE_WIND;
	}

	retVal.baseColorAtlasMulAdd = XMFLOAT4(0, 0, 0, 0);
	retVal.surfaceMapAtlasMulAdd = XMFLOAT4(0, 0, 0, 0);
	retVal.emissiveMapAtlasMulAdd = XMFLOAT4(0, 0, 0, 0);
	retVal.normalMapAtlasMulAdd = XMFLOAT4(0, 0, 0, 0);

	return retVal;
}

void wiScene::MeshComponent::CreateRenderData()
{
	GraphicsDevice* device = wiRenderer::GetDevice();

	// Create index buffer GPU data:
	{
		GPUBufferDesc bd;
		bd.Usage = USAGE_IMMUTABLE;
		bd.CPUAccessFlags = 0;
		bd.BindFlags = BIND_INDEX_BUFFER | BIND_SHADER_RESOURCE;
		bd.MiscFlags = 0;

		SubresourceData initData;

		if (GetIndexFormat() == INDEXFORMAT_32BIT)
		{
			bd.StructureByteStride = sizeof(uint32_t);
			bd.Format = FORMAT_R32_UINT;
			bd.ByteWidth = uint32_t(sizeof(uint32_t) * indices.size());

			// Use indices directly since vector is in correct format
			static_assert(std::is_same<decltype(indices)::value_type, uint32_t>::value, "indices not in INDEXFORMAT_32BIT");
			initData.pSysMem = indices.data();

			device->CreateBuffer(&bd, &initData, &indexBuffer);
			device->SetName(&indexBuffer, "indexBuffer_32bit");
		}
		else
		{
			bd.StructureByteStride = sizeof(uint16_t);
			bd.Format = FORMAT_R16_UINT;
			bd.ByteWidth = uint32_t(sizeof(uint16_t) * indices.size());

			std::vector<uint16_t> gpuIndexData(indices.size());
			std::copy(indices.begin(), indices.end(), gpuIndexData.begin());
			initData.pSysMem = gpuIndexData.data();

			device->CreateBuffer(&bd, &initData, &indexBuffer);
			device->SetName(&indexBuffer, "indexBuffer_16bit");
		}
	}


	XMFLOAT3 _min = XMFLOAT3(FLT_MAX, FLT_MAX, FLT_MAX);
	XMFLOAT3 _max = XMFLOAT3(-FLT_MAX, -FLT_MAX, -FLT_MAX);

	// vertexBuffer - POSITION + NORMAL + WIND:
	{
		std::vector<Vertex_POS> vertices(vertex_positions.size());
		for (size_t i = 0; i < vertices.size(); ++i)
		{
			const XMFLOAT3& pos = vertex_positions[i];
			XMFLOAT3 nor = vertex_normals.empty() ? XMFLOAT3(1, 1, 1) : vertex_normals[i];
			DirectX::XMStoreFloat3(&nor, XMVector3Normalize(XMLoadFloat3(&nor)));
			const uint8_t wind = vertex_windweights.empty() ? 0xFF : vertex_windweights[i];
			vertices[i].FromFULL(pos, nor, wind);

			_min = wiMath::Min(_min, pos);
			_max = wiMath::Max(_max, pos);
		}

		GPUBufferDesc bd;
		bd.Usage = USAGE_DEFAULT;
		bd.CPUAccessFlags = 0;
		bd.BindFlags = BIND_VERTEX_BUFFER | BIND_SHADER_RESOURCE;
		bd.MiscFlags = RESOURCE_MISC_BUFFER_ALLOW_RAW_VIEWS;
		bd.ByteWidth = (uint32_t)(sizeof(Vertex_POS) * vertices.size());

		SubresourceData InitData;
		InitData.pSysMem = vertices.data();
		device->CreateBuffer(&bd, &InitData, &vertexBuffer_POS);
		device->SetName(&vertexBuffer_POS, "vertexBuffer_POS");
	}

	aabb = AABB(_min, _max);

	// skinning buffers:
	if (!vertex_boneindices.empty())
	{
		std::vector<Vertex_BON> vertices(vertex_boneindices.size());
		for (size_t i = 0; i < vertices.size(); ++i)
		{
			XMFLOAT4& wei = vertex_boneweights[i];
			// normalize bone weights
			float len = wei.x + wei.y + wei.z + wei.w;
			if (len > 0)
			{
				wei.x /= len;
				wei.y /= len;
				wei.z /= len;
				wei.w /= len;
			}
			vertices[i].FromFULL(vertex_boneindices[i], wei);
		}

		GPUBufferDesc bd;
		bd.Usage = USAGE_IMMUTABLE;
		bd.BindFlags = BIND_SHADER_RESOURCE;
		bd.CPUAccessFlags = 0;
		bd.MiscFlags = RESOURCE_MISC_BUFFER_ALLOW_RAW_VIEWS;
		bd.ByteWidth = (uint32_t)(sizeof(Vertex_BON) * vertices.size());

		SubresourceData InitData;
		InitData.pSysMem = vertices.data();
		device->CreateBuffer(&bd, &InitData, &vertexBuffer_BON);

		bd.Usage = USAGE_DEFAULT;
		bd.BindFlags = BIND_VERTEX_BUFFER | BIND_UNORDERED_ACCESS | BIND_SHADER_RESOURCE;
		bd.CPUAccessFlags = 0;
		bd.MiscFlags = RESOURCE_MISC_BUFFER_ALLOW_RAW_VIEWS;

		bd.ByteWidth = (uint32_t)(sizeof(Vertex_POS) * vertex_positions.size());
		device->CreateBuffer(&bd, nullptr, &streamoutBuffer_POS);
		device->SetName(&streamoutBuffer_POS, "streamoutBuffer_POS");
	}

	// vertexBuffer - UV SET 0
	if (!vertex_uvset_0.empty())
	{
		std::vector<Vertex_TEX> vertices(vertex_uvset_0.size());
		for (size_t i = 0; i < vertices.size(); ++i)
		{
			vertices[i].FromFULL(vertex_uvset_0[i]);
		}

		GPUBufferDesc bd;
		bd.Usage = USAGE_IMMUTABLE;
		bd.CPUAccessFlags = 0;
		bd.BindFlags = BIND_VERTEX_BUFFER | BIND_SHADER_RESOURCE;
		bd.MiscFlags = 0;
		bd.StructureByteStride = sizeof(Vertex_TEX);
		bd.ByteWidth = (uint32_t)(bd.StructureByteStride * vertices.size());
		bd.Format = Vertex_TEX::FORMAT;

		SubresourceData InitData;
		InitData.pSysMem = vertices.data();
		device->CreateBuffer(&bd, &InitData, &vertexBuffer_UV0);
		device->SetName(&vertexBuffer_UV0, "vertexBuffer_UV0");
	}

	// vertexBuffer - UV SET 1
	if (!vertex_uvset_1.empty())
	{
		std::vector<Vertex_TEX> vertices(vertex_uvset_1.size());
		for (size_t i = 0; i < vertices.size(); ++i)
		{
			vertices[i].FromFULL(vertex_uvset_1[i]);
		}

		GPUBufferDesc bd;
		bd.Usage = USAGE_IMMUTABLE;
		bd.CPUAccessFlags = 0;
		bd.BindFlags = BIND_VERTEX_BUFFER | BIND_SHADER_RESOURCE;
		bd.MiscFlags = 0;
		bd.StructureByteStride = sizeof(Vertex_TEX);
		bd.ByteWidth = (uint32_t)(bd.StructureByteStride * vertices.size());
		bd.Format = Vertex_TEX::FORMAT;

		SubresourceData InitData;
		InitData.pSysMem = vertices.data();
		device->CreateBuffer(&bd, &InitData, &vertexBuffer_UV1);
		device->SetName(&vertexBuffer_UV1, "vertexBuffer_UV1");
	}

	// vertexBuffer - COLORS
	if (!vertex_colors.empty())
	{
		GPUBufferDesc bd;
		bd.Usage = USAGE_IMMUTABLE;
		bd.CPUAccessFlags = 0;
		bd.BindFlags = BIND_VERTEX_BUFFER | BIND_SHADER_RESOURCE;
		bd.MiscFlags = 0;
		bd.StructureByteStride = sizeof(Vertex_COL);
		bd.ByteWidth = (uint32_t)(bd.StructureByteStride * vertex_colors.size());
		bd.Format = FORMAT_R8G8B8A8_UNORM;

		SubresourceData InitData;
		InitData.pSysMem = vertex_colors.data();
		device->CreateBuffer(&bd, &InitData, &vertexBuffer_COL);
		device->SetName(&vertexBuffer_COL, "vertexBuffer_COL");
	}

	// vertexBuffer - ATLAS
	if (!vertex_atlas.empty())
	{
		std::vector<Vertex_TEX> vertices(vertex_atlas.size());
		for (size_t i = 0; i < vertices.size(); ++i)
		{
			vertices[i].FromFULL(vertex_atlas[i]);
		}

		GPUBufferDesc bd;
		bd.Usage = USAGE_IMMUTABLE;
		bd.CPUAccessFlags = 0;
		bd.BindFlags = BIND_VERTEX_BUFFER | BIND_SHADER_RESOURCE;
		bd.MiscFlags = 0;
		bd.StructureByteStride = sizeof(Vertex_TEX);
		bd.ByteWidth = (uint32_t)(bd.StructureByteStride * vertices.size());
		bd.Format = Vertex_TEX::FORMAT;

		SubresourceData InitData;
		InitData.pSysMem = vertices.data();
		device->CreateBuffer(&bd, &InitData, &vertexBuffer_ATL);
		device->SetName(&vertexBuffer_ATL, "vertexBuffer_ATL");
	}

	// vertexBuffer - SUBSETS
	{
		vertex_subsets.resize(vertex_positions.size());

		uint32_t subsetCounter = 0;
		for (auto& subset : subsets)
		{
			for (uint32_t i = 0; i < subset.indexCount; ++i)
			{
				uint32_t index = indices[subset.indexOffset + i];
				vertex_subsets[index] = subsetCounter;
			}
			subsetCounter++;
		}

		GPUBufferDesc bd;
		bd.Usage = USAGE_IMMUTABLE;
		bd.CPUAccessFlags = 0;
		bd.BindFlags = BIND_VERTEX_BUFFER | BIND_SHADER_RESOURCE;
		bd.MiscFlags = 0;
		bd.StructureByteStride = sizeof(uint8_t);
		bd.ByteWidth = (uint32_t)(bd.StructureByteStride * vertex_subsets.size());
		bd.Format = FORMAT_R8_UINT;

		SubresourceData InitData;
		InitData.pSysMem = vertex_subsets.data();
		device->CreateBuffer(&bd, &InitData, &vertexBuffer_SUB);
		device->SetName(&vertexBuffer_SUB, "vertexBuffer_SUB");
	}

	// vertexBuffer_PRE will be created on demand later!
	vertexBuffer_PRE = GPUBuffer();

	if (wiRenderer::GetDevice()->CheckCapability(GRAPHICSDEVICE_CAPABILITY_RAYTRACING))
	{
		BLAS_build_pending = true;

		RaytracingAccelerationStructureDesc desc;
		desc.type = RaytracingAccelerationStructureDesc::BOTTOMLEVEL;

		if (streamoutBuffer_POS.IsValid())
		{
			desc._flags |= RaytracingAccelerationStructureDesc::FLAG_ALLOW_UPDATE;
			desc._flags |= RaytracingAccelerationStructureDesc::FLAG_PREFER_FAST_BUILD;
		}
		else
		{
			desc._flags |= RaytracingAccelerationStructureDesc::FLAG_PREFER_FAST_TRACE;
		}

#if 0
		// Flattened subsets:
		desc.bottomlevel.geometries.emplace_back();
		auto& geometry = desc.bottomlevel.geometries.back();
		geometry.type = RaytracingAccelerationStructureDesc::BottomLevel::Geometry::TRIANGLES;
		geometry.triangles.vertexBuffer = streamoutBuffer_POS.IsValid() ? streamoutBuffer_POS : vertexBuffer_POS;
		geometry.triangles.indexBuffer = indexBuffer;
		geometry.triangles.indexFormat = GetIndexFormat();
		geometry.triangles.indexCount = (uint32_t)indices.size();
		geometry.triangles.indexOffset = 0;
		geometry.triangles.vertexCount = (uint32_t)vertex_positions.size();
		geometry.triangles.vertexFormat = FORMAT_R32G32B32_FLOAT;
		geometry.triangles.vertexStride = sizeof(MeshComponent::Vertex_POS);
#else
		// One geometry per subset:
		for (auto& subset : subsets)
		{
			desc.bottomlevel.geometries.emplace_back();
			auto& geometry = desc.bottomlevel.geometries.back();
			geometry.type = RaytracingAccelerationStructureDesc::BottomLevel::Geometry::TRIANGLES;
			geometry.triangles.vertexBuffer = streamoutBuffer_POS.IsValid() ? streamoutBuffer_POS : vertexBuffer_POS;
			geometry.triangles.indexBuffer = indexBuffer;
			geometry.triangles.indexFormat = GetIndexFormat();
			geometry.triangles.indexCount = subset.indexCount;
			geometry.triangles.indexOffset = subset.indexOffset;
			geometry.triangles.vertexCount = (uint32_t)vertex_positions.size();
			geometry.triangles.vertexFormat = FORMAT_R32G32B32_FLOAT;
			geometry.triangles.vertexStride = sizeof(MeshComponent::Vertex_POS);
		}
#endif

		bool success = device->CreateRaytracingAccelerationStructure(&desc, &BLAS);
		assert(success);
		device->SetName(&BLAS, "BLAS");
	}
}

void wiScene::MeshComponent::ComputeNormals(COMPUTE_NORMALS compute)
{
	// Start recalculating normals:

	switch (compute)
	{
		case wiScene::MeshComponent::COMPUTE_NORMALS_HARD:
		{
			// Compute hard surface normals:

			std::vector<uint32_t> newIndexBuffer;
			std::vector<XMFLOAT3> newPositionsBuffer;
			std::vector<XMFLOAT3> newNormalsBuffer;
			std::vector<XMFLOAT2> newUV0Buffer;
			std::vector<XMFLOAT2> newUV1Buffer;
			std::vector<XMFLOAT2> newAtlasBuffer;
			std::vector<XMUINT4> newBoneIndicesBuffer;
			std::vector<XMFLOAT4> newBoneWeightsBuffer;
			std::vector<uint32_t> newColorsBuffer;

			for (size_t face = 0; face < indices.size() / 3; face++)
			{
				uint32_t i0 = indices[face * 3 + 0];
				uint32_t i1 = indices[face * 3 + 1];
				uint32_t i2 = indices[face * 3 + 2];

				XMFLOAT3& p0 = vertex_positions[i0];
				XMFLOAT3& p1 = vertex_positions[i1];
				XMFLOAT3& p2 = vertex_positions[i2];

				XMVECTOR U = XMLoadFloat3(&p2) - XMLoadFloat3(&p0);
				XMVECTOR V = XMLoadFloat3(&p1) - XMLoadFloat3(&p0);

				XMVECTOR N = XMVector3Cross(U, V);
				N = XMVector3Normalize(N);

				XMFLOAT3 normal;
				DirectX::XMStoreFloat3(&normal, N);

				newPositionsBuffer.push_back(p0);
				newPositionsBuffer.push_back(p1);
				newPositionsBuffer.push_back(p2);

				newNormalsBuffer.push_back(normal);
				newNormalsBuffer.push_back(normal);
				newNormalsBuffer.push_back(normal);

				if (!vertex_uvset_0.empty())
				{
					newUV0Buffer.push_back(vertex_uvset_0[i0]);
					newUV0Buffer.push_back(vertex_uvset_0[i1]);
					newUV0Buffer.push_back(vertex_uvset_0[i2]);
				}

				if (!vertex_uvset_1.empty())
				{
					newUV1Buffer.push_back(vertex_uvset_1[i0]);
					newUV1Buffer.push_back(vertex_uvset_1[i1]);
					newUV1Buffer.push_back(vertex_uvset_1[i2]);
				}

				if (!vertex_atlas.empty())
				{
					newAtlasBuffer.push_back(vertex_atlas[i0]);
					newAtlasBuffer.push_back(vertex_atlas[i1]);
					newAtlasBuffer.push_back(vertex_atlas[i2]);
				}

				if (!vertex_boneindices.empty())
				{
					newBoneIndicesBuffer.push_back(vertex_boneindices[i0]);
					newBoneIndicesBuffer.push_back(vertex_boneindices[i1]);
					newBoneIndicesBuffer.push_back(vertex_boneindices[i2]);
				}

				if (!vertex_boneweights.empty())
				{
					newBoneWeightsBuffer.push_back(vertex_boneweights[i0]);
					newBoneWeightsBuffer.push_back(vertex_boneweights[i1]);
					newBoneWeightsBuffer.push_back(vertex_boneweights[i2]);
				}

				if (!vertex_colors.empty())
				{
					newColorsBuffer.push_back(vertex_colors[i0]);
					newColorsBuffer.push_back(vertex_colors[i1]);
					newColorsBuffer.push_back(vertex_colors[i2]);
				}

				newIndexBuffer.push_back(static_cast<uint32_t>(newIndexBuffer.size()));
				newIndexBuffer.push_back(static_cast<uint32_t>(newIndexBuffer.size()));
				newIndexBuffer.push_back(static_cast<uint32_t>(newIndexBuffer.size()));
			}

			// For hard surface normals, we created a new mesh in the previous loop through faces, so swap data:
			vertex_positions = newPositionsBuffer;
			vertex_normals = newNormalsBuffer;
			vertex_uvset_0 = newUV0Buffer;
			vertex_uvset_1 = newUV1Buffer;
			vertex_atlas = newAtlasBuffer;
			vertex_colors = newColorsBuffer;
			if (!vertex_boneindices.empty())
			{
				vertex_boneindices = newBoneIndicesBuffer;
			}
			if (!vertex_boneweights.empty())
			{
				vertex_boneweights = newBoneWeightsBuffer;
			}
			indices = newIndexBuffer;
		}
		break;

		case wiScene::MeshComponent::COMPUTE_NORMALS_SMOOTH:
		{
			// Compute smooth surface normals:

			// 1.) Zero normals, they will be averaged later
			for (size_t i = 0; i < vertex_normals.size(); i++)
			{
				vertex_normals[i] = XMFLOAT3(0, 0, 0);
			}

			// 2.) Find identical vertices by POSITION, accumulate face normals
			for (size_t i = 0; i < vertex_positions.size(); i++)
			{
				XMFLOAT3& v_search_pos = vertex_positions[i];

				for (size_t ind = 0; ind < indices.size() / 3; ++ind)
				{
					uint32_t i0 = indices[ind * 3 + 0];
					uint32_t i1 = indices[ind * 3 + 1];
					uint32_t i2 = indices[ind * 3 + 2];

					XMFLOAT3& v0 = vertex_positions[i0];
					XMFLOAT3& v1 = vertex_positions[i1];
					XMFLOAT3& v2 = vertex_positions[i2];

					bool match_pos0 =
						fabs(v_search_pos.x - v0.x) < FLT_EPSILON &&
						fabs(v_search_pos.y - v0.y) < FLT_EPSILON &&
						fabs(v_search_pos.z - v0.z) < FLT_EPSILON;

					bool match_pos1 =
						fabs(v_search_pos.x - v1.x) < FLT_EPSILON &&
						fabs(v_search_pos.y - v1.y) < FLT_EPSILON &&
						fabs(v_search_pos.z - v1.z) < FLT_EPSILON;

					bool match_pos2 =
						fabs(v_search_pos.x - v2.x) < FLT_EPSILON &&
						fabs(v_search_pos.y - v2.y) < FLT_EPSILON &&
						fabs(v_search_pos.z - v2.z) < FLT_EPSILON;

					if (match_pos0 || match_pos1 || match_pos2)
					{
						XMVECTOR U = XMLoadFloat3(&v2) - XMLoadFloat3(&v0);
						XMVECTOR V = XMLoadFloat3(&v1) - XMLoadFloat3(&v0);

						XMVECTOR N = XMVector3Cross(U, V);
						N = XMVector3Normalize(N);

						XMFLOAT3 normal;
						DirectX::XMStoreFloat3(&normal, N);

						vertex_normals[i].x += normal.x;
						vertex_normals[i].y += normal.y;
						vertex_normals[i].z += normal.z;
					}

				}
			}

			// 3.) Find duplicated vertices by POSITION and UV0 and UV1 and ATLAS and SUBSET and remove them:
			for (auto& subset : subsets)
			{
				for (uint32_t i = 0; i < subset.indexCount - 1; i++)
				{
					uint32_t ind0 = indices[subset.indexOffset + (uint32_t)i];
					const XMFLOAT3& p0 = vertex_positions[ind0];
					const XMFLOAT2& u00 = vertex_uvset_0.empty() ? XMFLOAT2(0, 0) : vertex_uvset_0[ind0];
					const XMFLOAT2& u10 = vertex_uvset_1.empty() ? XMFLOAT2(0, 0) : vertex_uvset_1[ind0];
					const XMFLOAT2& at0 = vertex_atlas.empty() ? XMFLOAT2(0, 0) : vertex_atlas[ind0];

					for (uint32_t j = i + 1; j < subset.indexCount; j++)
					{
						uint32_t ind1 = indices[subset.indexOffset + (uint32_t)j];

						if (ind1 == ind0)
						{
							continue;
						}

						const XMFLOAT3& p1 = vertex_positions[ind1];
						const XMFLOAT2& u01 = vertex_uvset_0.empty() ? XMFLOAT2(0, 0) : vertex_uvset_0[ind1];
						const XMFLOAT2& u11 = vertex_uvset_1.empty() ? XMFLOAT2(0, 0) : vertex_uvset_1[ind1];
						const XMFLOAT2& at1 = vertex_atlas.empty() ? XMFLOAT2(0, 0) : vertex_atlas[ind1];

						const bool duplicated_pos =
							fabs(p0.x - p1.x) < FLT_EPSILON &&
							fabs(p0.y - p1.y) < FLT_EPSILON &&
							fabs(p0.z - p1.z) < FLT_EPSILON;

						const bool duplicated_uv0 =
							fabs(u00.x - u01.x) < FLT_EPSILON &&
							fabs(u00.y - u01.y) < FLT_EPSILON;

						const bool duplicated_uv1 =
							fabs(u10.x - u11.x) < FLT_EPSILON &&
							fabs(u10.y - u11.y) < FLT_EPSILON;

						const bool duplicated_atl =
							fabs(at0.x - at1.x) < FLT_EPSILON &&
							fabs(at0.y - at1.y) < FLT_EPSILON;

						if (duplicated_pos && duplicated_uv0 && duplicated_uv1 && duplicated_atl)
						{
							// Erase vertices[ind1] because it is a duplicate:
							if (ind1 < vertex_positions.size())
							{
								vertex_positions.erase(vertex_positions.begin() + ind1);
							}
							if (ind1 < vertex_normals.size())
							{
								vertex_normals.erase(vertex_normals.begin() + ind1);
							}
							if (ind1 < vertex_uvset_0.size())
							{
								vertex_uvset_0.erase(vertex_uvset_0.begin() + ind1);
							}
							if (ind1 < vertex_uvset_1.size())
							{
								vertex_uvset_1.erase(vertex_uvset_1.begin() + ind1);
							}
							if (ind1 < vertex_atlas.size())
							{
								vertex_atlas.erase(vertex_atlas.begin() + ind1);
							}
							if (ind1 < vertex_boneindices.size())
							{
								vertex_boneindices.erase(vertex_boneindices.begin() + ind1);
							}
							if (ind1 < vertex_boneweights.size())
							{
								vertex_boneweights.erase(vertex_boneweights.begin() + ind1);
							}

							// The vertices[ind1] was removed, so each index after that needs to be updated:
							for (auto& index : indices)
							{
								if (index > ind1 && index > 0)
								{
									index--;
								}
								else if (index == ind1)
								{
									index = ind0;
								}
							}

						}

					}
				}

			}

		}
		break;

		case wiScene::MeshComponent::COMPUTE_NORMALS_SMOOTH_FAST:
		{
			for (size_t i = 0; i < vertex_normals.size(); i++)
			{
				vertex_normals[i] = XMFLOAT3(0, 0, 0);
			}
			for (size_t i = 0; i < indices.size() / 3; ++i)
			{
				uint32_t index1 = indices[i * 3];
				uint32_t index2 = indices[i * 3 + 1];
				uint32_t index3 = indices[i * 3 + 2];

				XMVECTOR side1 = XMLoadFloat3(&vertex_positions[index1]) - XMLoadFloat3(&vertex_positions[index3]);
				XMVECTOR side2 = XMLoadFloat3(&vertex_positions[index1]) - XMLoadFloat3(&vertex_positions[index2]);
				XMVECTOR N = XMVector3Normalize(XMVector3Cross(side1, side2));
				XMFLOAT3 normal;
				DirectX::XMStoreFloat3(&normal, N);

				vertex_normals[index1].x += normal.x;
				vertex_normals[index1].y += normal.y;
				vertex_normals[index1].z += normal.z;

				vertex_normals[index2].x += normal.x;
				vertex_normals[index2].y += normal.y;
				vertex_normals[index2].z += normal.z;

				vertex_normals[index3].x += normal.x;
				vertex_normals[index3].y += normal.y;
				vertex_normals[index3].z += normal.z;
			}
		}
		break;

	}

	CreateRenderData(); // <- normals will be normalized here!
}

void wiScene::MeshComponent::FlipCulling()
{
	for (size_t face = 0; face < indices.size() / 3; face++)
	{
		uint32_t i0 = indices[face * 3 + 0];
		uint32_t i1 = indices[face * 3 + 1];
		uint32_t i2 = indices[face * 3 + 2];

		indices[face * 3 + 0] = i0;
		indices[face * 3 + 1] = i2;
		indices[face * 3 + 2] = i1;
	}

	CreateRenderData();
}

void wiScene::MeshComponent::FlipNormals()
{
	for (auto& normal : vertex_normals)
	{
		normal.x *= -1;
		normal.y *= -1;
		normal.z *= -1;
	}

	CreateRenderData();
}

void wiScene::MeshComponent::Recenter()
{
	XMFLOAT3 center = aabb.getCenter();

	for (auto& pos : vertex_positions)
	{
		pos.x -= center.x;
		pos.y -= center.y;
		pos.z -= center.z;
	}

	CreateRenderData();
}

void wiScene::MeshComponent::RecenterToBottom()
{
	XMFLOAT3 center = aabb.getCenter();
	center.y -= aabb.getHalfWidth().y;

	for (auto& pos : vertex_positions)
	{
		pos.x -= center.x;
		pos.y -= center.y;
		pos.z -= center.z;
	}

	CreateRenderData();
}

SPHERE wiScene::MeshComponent::GetBoundingSphere() const
{
	XMFLOAT3 halfwidth = aabb.getHalfWidth();

	SPHERE sphere;
	sphere.center = aabb.getCenter();
	sphere.radius = std::max(halfwidth.x, std::max(halfwidth.y, halfwidth.z));

	return sphere;
}

void wiScene::CameraComponent::CreatePerspective(float newWidth, float newHeight, float newNear, float newFar, float newFOV)
{
	zNearP = newNear;
	zFarP = newFar;
	width = newWidth;
	height = newHeight;
	fov = newFOV;

	SetCustomProjectionEnabled(false);

	UpdateCamera();
}

void wiScene::CameraComponent::UpdateCamera()
{
	if (!IsCustomProjectionEnabled())
	{
		XMStoreFloat4x4(&Projection, XMMatrixPerspectiveFovLH(fov, width / height, zFarP, zNearP)); // reverse zbuffer!
		Projection.m[2][0] = jitter.x;
		Projection.m[2][1] = jitter.y;
	}

	XMVECTOR _Eye = XMLoadFloat3(&Eye);
	XMVECTOR _At = XMLoadFloat3(&At);
	XMVECTOR _Up = XMLoadFloat3(&Up);

	XMMATRIX _V = XMMatrixLookToLH(_Eye, _At, _Up);
	XMStoreFloat4x4(&View, _V);

	XMMATRIX _P = XMLoadFloat4x4(&Projection);
	XMMATRIX _InvP = XMMatrixInverse(nullptr, _P);
	XMStoreFloat4x4(&InvProjection, _InvP);

	XMMATRIX _VP = XMMatrixMultiply(_V, _P);
	XMStoreFloat4x4(&View, _V);
	XMStoreFloat4x4(&VP, _VP);
	XMStoreFloat4x4(&InvView, XMMatrixInverse(nullptr, _V));
	XMStoreFloat4x4(&InvVP, XMMatrixInverse(nullptr, _VP));
	XMStoreFloat4x4(&Projection, _P);
	XMStoreFloat4x4(&InvProjection, XMMatrixInverse(nullptr, _P));

	frustum.Create(_VP);
}

void wiScene::CameraComponent::TransformCamera(const TransformComponent& transform)
{
	XMVECTOR S, R, T;
	XMMatrixDecompose(&S, &R, &T, XMLoadFloat4x4(&transform.world));

	XMVECTOR _Eye = T;
	XMVECTOR _At = XMVectorSet(0, 0, 1, 0);
	XMVECTOR _Up = XMVectorSet(0, 1, 0, 0);

	XMMATRIX _Rot = XMMatrixRotationQuaternion(R);
	_At = XMVector3TransformNormal(_At, _Rot);
	_Up = XMVector3TransformNormal(_Up, _Rot);
	XMStoreFloat3x3(&rotationMatrix, _Rot);

	XMMATRIX _V = XMMatrixLookToLH(_Eye, _At, _Up);
	XMStoreFloat4x4(&View, _V);

	XMStoreFloat3(&Eye, _Eye);
	XMStoreFloat3(&At, _At);
	XMStoreFloat3(&Up, _Up);
}

void wiScene::CameraComponent::Reflect(const XMFLOAT4& plane)
{
	XMVECTOR _Eye = XMLoadFloat3(&Eye);
	XMVECTOR _At = XMLoadFloat3(&At);
	XMVECTOR _Up = XMLoadFloat3(&Up);
	XMMATRIX _Ref = XMMatrixReflect(XMLoadFloat4(&plane));

	_Eye = XMVector3Transform(_Eye, _Ref);
	_At = XMVector3TransformNormal(_At, _Ref);
	_Up = XMVector3TransformNormal(_Up, _Ref);

	XMStoreFloat3(&Eye, _Eye);
	XMStoreFloat3(&At, _At);
	XMStoreFloat3(&Up, _Up);

	UpdateCamera();
}

void wiScene::Scene::RunCameraUpdateSystem(wiJobSystem::context& ctx)
{
	wiJobSystem::Dispatch(ctx, (uint32_t)cameras.GetCount(), small_subtask_groupsize, [&](wiJobArgs args)
	{
		CameraComponent& camera = cameras[args.jobIndex];
		wiECS::Entity entity = cameras.GetEntity(args.jobIndex);
		const TransformComponent* transform = transforms.GetComponent(entity);
		if (transform != nullptr)
		{
			camera.TransformCamera(*transform);
		}
		camera.UpdateCamera();
	});
}

void wiScene::Scene::RunMeshUpdateSystem(wiJobSystem::context& ctx)
{
	geometryOffset.store(0);

	wiJobSystem::Dispatch(ctx, (uint32_t)meshes.GetCount(), small_subtask_groupsize, [&](wiJobArgs args)
	{
		MeshComponent& mesh = meshes[args.jobIndex];
		mesh.TLAS_geometryOffset = geometryOffset.fetch_add((uint32_t)mesh.subsets.size());

		GraphicsDevice* device = wiRenderer::GetDevice();
		uint32_t subsetIndex = 0;
		for (auto& subset : mesh.subsets)
		{
			const MaterialComponent* material = materials.GetComponent(subset.materialID);
			if (material != nullptr)
			{
				if (descriptorTable.IsValid())
				{
					uint32_t global_geometryIndex = mesh.TLAS_geometryOffset + subsetIndex;
					device->WriteDescriptor(
						&descriptorTable,
						DESCRIPTORTABLE_ENTRY_SUBSETS_MATERIAL,
						global_geometryIndex,
						&material->constantBuffer
					);
					device->WriteDescriptor(
						&descriptorTable,
						DESCRIPTORTABLE_ENTRY_SUBSETS_TEXTURE_BASECOLOR,
						global_geometryIndex,
						material->baseColorMap ? material->baseColorMap->texture : nullptr
					);
					device->WriteDescriptor(
						&descriptorTable,
						DESCRIPTORTABLE_ENTRY_SUBSETS_INDEXBUFFER,
						global_geometryIndex,
						&mesh.indexBuffer,
						-1, subset.indexOffset * mesh.GetIndexStride()
					);
					device->WriteDescriptor(
						&descriptorTable,
						DESCRIPTORTABLE_ENTRY_SUBSETS_VERTEXBUFFER_UV0,
						global_geometryIndex,
						&mesh.vertexBuffer_UV0
					);
					device->WriteDescriptor(
						&descriptorTable,
						DESCRIPTORTABLE_ENTRY_SUBSETS_VERTEXBUFFER_UV1,
						global_geometryIndex,
						&mesh.vertexBuffer_UV1
					);
				}

				if (mesh.BLAS.IsValid())
				{
					uint32_t flags = mesh.BLAS.desc.bottomlevel.geometries[subsetIndex]._flags;
					if (material->IsAlphaTestEnabled() || material->IsTransparent())
					{
						mesh.BLAS.desc.bottomlevel.geometries[subsetIndex]._flags &=
							~RaytracingAccelerationStructureDesc::BottomLevel::Geometry::FLAG_OPAQUE;
					}
					else
					{
						mesh.BLAS.desc.bottomlevel.geometries[subsetIndex]._flags =
							RaytracingAccelerationStructureDesc::BottomLevel::Geometry::FLAG_OPAQUE;
					}
					if (flags != mesh.BLAS.desc.bottomlevel.geometries[subsetIndex]._flags)
					{
						// New flags invalidate BLAS
						mesh.BLAS_build_pending = true;
					}
				}
			}
			subsetIndex++;
		}

	});
}

void wiScene::Scene::RunMaterialUpdateSystem(wiJobSystem::context& ctx, float dt)
{
	wiJobSystem::Dispatch(ctx, (uint32_t)materials.GetCount(), small_subtask_groupsize, [&](wiJobArgs args)
	{
		MaterialComponent& material = materials[args.jobIndex];

		material.texAnimElapsedTime += dt * material.texAnimFrameRate;
		if (material.texAnimElapsedTime >= 1.0f)
		{
			material.texMulAdd.z = fmodf(material.texMulAdd.z + material.texAnimDirection.x, 1);
			material.texMulAdd.w = fmodf(material.texMulAdd.w + material.texAnimDirection.y, 1);
			material.texAnimElapsedTime = 0.0f;

			material.SetDirty(); // will trigger constant buffer update later on
		}

		material.engineStencilRef = STENCILREF_DEFAULT;
		if (material.subsurfaceScattering > 0)
		{
			material.engineStencilRef = STENCILREF_SKIN;
		}

	});
}

void wiScene::Scene::RunImpostorUpdateSystem(wiJobSystem::context& ctx)
{
	wiJobSystem::Dispatch(ctx, (uint32_t)impostors.GetCount(), 1, [&](wiJobArgs args)
	{
		ImpostorComponent& impostor = impostors[args.jobIndex];
		impostor.aabb = AABB();
		impostor.instanceMatrices.clear();
	});
}


void wiScene::Scene::RunDecalUpdateSystem(wiJobSystem::context& ctx)
{
	assert(decals.GetCount() == aabb_decals.GetCount());

	wiJobSystem::Dispatch(ctx, (uint32_t)decals.GetCount(), small_subtask_groupsize, [&](wiJobArgs args)
	{
		DecalComponent& decal = decals[args.jobIndex];
		wiECS::Entity entity = decals.GetEntity(args.jobIndex);
		const TransformComponent& transform = *transforms.GetComponent(entity);
		decal.world = transform.world;

		XMMATRIX W = XMLoadFloat4x4(&decal.world);
		XMVECTOR front = XMVectorSet(0, 0, 1, 0);
		front = XMVector3TransformNormal(front, W);
		XMStoreFloat3(&decal.front, front);

		XMVECTOR S, R, T;
		XMMatrixDecompose(&S, &R, &T, W);
		XMStoreFloat3(&decal.position, T);
		XMFLOAT3 scale;
		XMStoreFloat3(&scale, S);
		decal.range = std::max(scale.x, std::max(scale.y, scale.z)) * 2;

		AABB& aabb = aabb_decals[args.jobIndex];
		aabb.createFromHalfWidth(XMFLOAT3(0, 0, 0), XMFLOAT3(1, 1, 1));
		aabb = aabb.transform(transform.world);

		const MaterialComponent& material = *materials.GetComponent(entity);
		decal.color = material.baseColor;
		decal.emissive = material.GetEmissiveStrength();
		decal.texture = material.baseColorMap;
		decal.normal = material.normalMap;
	});
}

void wiScene::Scene::RunProbeUpdateSystem(wiJobSystem::context& ctx)
{
	assert(probes.GetCount() == aabb_probes.GetCount());

	wiJobSystem::Dispatch(ctx, (uint32_t)probes.GetCount(), small_subtask_groupsize, [&](wiJobArgs args)
	{

		EnvironmentProbeComponent& probe = probes[args.jobIndex];
		wiECS::Entity entity = probes.GetEntity(args.jobIndex);
		const TransformComponent& transform = *transforms.GetComponent(entity);

		probe.position = transform.GetPosition();

		XMMATRIX W = XMLoadFloat4x4(&transform.world);
		XMStoreFloat4x4(&probe.inverseMatrix, XMMatrixInverse(nullptr, W));

		XMVECTOR S, R, T;
		XMMatrixDecompose(&S, &R, &T, W);
		XMFLOAT3 scale;
		XMStoreFloat3(&scale, S);
		probe.range = std::max(scale.x, std::max(scale.y, scale.z)) * 2;

		AABB& aabb = aabb_probes[args.jobIndex];
		aabb.createFromHalfWidth(XMFLOAT3(0, 0, 0), XMFLOAT3(1, 1, 1));
		aabb = aabb.transform(transform.world);
	});
}

void wiScene::Scene::RunLightUpdateSystem(wiJobSystem::context& ctx)
{
	assert(lights.GetCount() == aabb_lights.GetCount());

	wiJobSystem::Dispatch(ctx, (uint32_t)lights.GetCount(), small_subtask_groupsize, [&](wiJobArgs args)
	{
		LightComponent& light = lights[args.jobIndex];
		wiECS::Entity entity = lights.GetEntity(args.jobIndex);
		const TransformComponent& transform = *transforms.GetComponent(entity);
		AABB& aabb = aabb_lights[args.jobIndex];

		XMMATRIX W = XMLoadFloat4x4(&transform.world);
		XMVECTOR S, R, T;
		XMMatrixDecompose(&S, &R, &T, W);

		XMStoreFloat3(&light.position, T);
		XMStoreFloat4(&light.rotation, R);
		XMStoreFloat3(&light.scale, S);
		XMStoreFloat3(&light.direction, XMVector3TransformNormal(XMVectorSet(0, 1, 0, 0), W));

		light.range_global = light.range_local * std::max(XMVectorGetX(S), std::max(XMVectorGetY(S), XMVectorGetZ(S)));

		switch (light.type)
		{
			default:
			case LightComponent::DIRECTIONAL:
				aabb.createFromHalfWidth(wiRenderer::GetCamera().Eye, XMFLOAT3(10000, 10000, 10000));
				locker.lock();
				if (args.jobIndex < weather->most_important_light_index)
				{
					weather->most_important_light_index = args.jobIndex;
					weather->sunColor = light.color;
					weather->sunDirection = light.direction;
					weather->sunEnergy = light.energy;
				}
				locker.unlock();
				break;
			case LightComponent::SPOT:
				aabb.createFromHalfWidth(light.position, XMFLOAT3(light.GetRange(), light.GetRange(), light.GetRange()));
				break;
			case LightComponent::POINT:
				aabb.createFromHalfWidth(light.position, XMFLOAT3(light.GetRange(), light.GetRange(), light.GetRange()));
				break;
			case LightComponent::SPHERE:
			case LightComponent::DISC:
			case LightComponent::RECTANGLE:
			case LightComponent::TUBE:
				XMStoreFloat3(&light.right, XMVector3TransformNormal(XMVectorSet(-1, 0, 0, 0), W));
				XMStoreFloat3(&light.front, XMVector3TransformNormal(XMVectorSet(0, 0, -1, 0), W));
				// area lights have no bounds, just like directional lights (todo: but they should have real bounds)
				aabb.createFromHalfWidth(wiRenderer::GetCamera().Eye, XMFLOAT3(10000, 10000, 10000));
				break;
		}

	});
}
void wiScene::Scene::RunParticleUpdateSystem(wiJobSystem::context& ctx, float dt)
{
	wiJobSystem::Dispatch(ctx, (uint32_t)emitters.GetCount(), small_subtask_groupsize, [&](wiJobArgs args)
	{
		wiEmittedParticle& emitter = emitters[args.jobIndex];
		wiECS::Entity entity = emitters.GetEntity(args.jobIndex);
		const TransformComponent& transform = *transforms.GetComponent(entity);
		emitter.UpdateCPU(transform, dt);
	});

	wiJobSystem::Dispatch(ctx, (uint32_t)hairs.GetCount(), small_subtask_groupsize, [&](wiJobArgs args)
	{
		wiHairParticle& hair = hairs[args.jobIndex];

		if (hair.meshID != wiECS::INVALID_ENTITY)
		{
			wiECS::Entity entity = hairs.GetEntity(args.jobIndex);
			const MeshComponent* mesh = meshes.GetComponent(hair.meshID);

			if (mesh != nullptr)
			{
				const TransformComponent& transform = *transforms.GetComponent(entity);

				hair.UpdateCPU(transform, *mesh, dt);
			}
		}

	});
}

void wiScene::Scene::RunWeatherUpdateSystem(wiJobSystem::context& ctx)
{
	if (weathers.GetCount() > 0)
	{
		weather = &weathers[0];
		weather->most_important_light_index = ~0;
	}
}
