#pragma once

#include "wiECS.h"
#include "wiArchive.h"
#include "CommonInclude.h"

namespace wiScene
{
	struct TransformComponent
	{
		enum FLAGS
		{
			EMPTY = 0,
			DIRTY = 1 << 0,
		};
		uint32_t _flags = DIRTY;

		XMFLOAT3 scale_local = XMFLOAT3(1, 1, 1);
		XMFLOAT4 rotation_local = XMFLOAT4(0, 0, 0, 1);
		XMFLOAT3 translation_local = XMFLOAT3(0, 0, 0);

		// Non-serialized attributes:
		XMFLOAT4X4 world = IDENTITYMATRIX;

		inline void SetDirty(bool value = true) { if (value) { _flags |= DIRTY; } else { _flags &= ~DIRTY; } }
		inline bool IsDirty() const { return _flags & DIRTY; }

		XMFLOAT3 GetPosition() const;
		XMFLOAT4 GetRotation() const;
		XMFLOAT3 GetScale() const;
		XMVECTOR GetPositionV() const;
		XMVECTOR GetRotationV() const;
		XMVECTOR GetScaleV() const;
		XMMATRIX GetLocalMatrix() const;
		void UpdateTransform();
		void UpdateTransform_Parented(const TransformComponent& parent);
		void ApplyTransform();
		void ClearTransform();
		void Translate(const XMFLOAT3& value);
		void Translate(const XMVECTOR& value);
		void RotateRollPitchYaw(const XMFLOAT3& value);
		void Rotate(const XMFLOAT4& quaternion);
		void Rotate(const XMVECTOR& quaternion);
		void Scale(const XMFLOAT3& value);
		void Scale(const XMVECTOR& value);
		void MatrixTransform(const XMFLOAT4X4& matrix);
		void MatrixTransform(const XMMATRIX& matrix);
		void Lerp(const TransformComponent& a, const TransformComponent& b, float t);
		void CatmullRom(const TransformComponent& a, const TransformComponent& b, const TransformComponent& c, const TransformComponent& d, float t);

		void Serialize(wiArchive& archive, wiECS::Entity seed = wiECS::INVALID_ENTITY);
	};

	struct PreviousFrameTransformComponent
	{
		// Non-serialized attributes:
		XMFLOAT4X4 world_prev;

		void Serialize(wiArchive& archive, wiECS::Entity seed = wiECS::INVALID_ENTITY);
	};

	struct HierarchyComponent
	{
		wiECS::Entity parentID = wiECS::INVALID_ENTITY;
		uint32_t layerMask_bind; // saved child layermask at the time of binding

		void Serialize(wiArchive& archive, wiECS::Entity seed = wiECS::INVALID_ENTITY);
	};
}