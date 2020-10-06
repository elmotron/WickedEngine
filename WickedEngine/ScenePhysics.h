#pragma once

#include "wiECS.h"
#include "wiArchive.h"
#include "wiIntersect.h"
#include "CommonInclude.h"
#include "SceneGraphics.h"
#include "ShaderInterop_Renderer.h"
#include "wiJobSystem.h"

namespace wiScene
{
	struct RigidBodyPhysicsComponent
	{
		enum FLAGS
		{
			EMPTY = 0,
			DISABLE_DEACTIVATION = 1 << 0,
			KINEMATIC = 1 << 1,
		};
		uint32_t _flags = EMPTY;

		enum CollisionShape
		{
			BOX,
			SPHERE,
			CAPSULE,
			CONVEX_HULL,
			TRIANGLE_MESH,
			ENUM_FORCE_UINT32 = 0xFFFFFFFF
		};
		CollisionShape shape;
		float mass = 1.0f;
		float friction = 1.0f;
		float restitution = 1.0f;
		float damping = 1.0f;

		// Non-serialized attributes:
		void* physicsobject = nullptr;

		inline void SetDisableDeactivation(bool value) { if (value) { _flags |= DISABLE_DEACTIVATION; } else { _flags &= ~DISABLE_DEACTIVATION; } }
		inline void SetKinematic(bool value) { if (value) { _flags |= KINEMATIC; } else { _flags &= ~KINEMATIC; } }

		inline bool IsDisableDeactivation() const { return _flags & DISABLE_DEACTIVATION; }
		inline bool IsKinematic() const { return _flags & KINEMATIC; }

		void Serialize(wiArchive& archive, wiECS::Entity seed = wiECS::INVALID_ENTITY);
	};

	struct SoftBodyPhysicsComponent
	{
		enum FLAGS
		{
			EMPTY = 0,
			SAFE_TO_REGISTER = 1 << 0,
			DISABLE_DEACTIVATION = 1 << 1,
			FORCE_RESET = 1 << 2,
		};
		uint32_t _flags = DISABLE_DEACTIVATION;

		float mass = 1.0f;
		float friction = 1.0f;
		std::vector<uint32_t> physicsToGraphicsVertexMapping; // maps graphics vertex index to physics vertex index of the same position
		std::vector<uint32_t> graphicsToPhysicsVertexMapping; // maps a physics vertex index to first graphics vertex index of the same position
		std::vector<float> weights; // weight per physics vertex controlling the mass. (0: disable weight (no physics, only animation), 1: default weight)

		// Non-serialized attributes:
		void* physicsobject = nullptr;
		XMFLOAT4X4 worldMatrix = IDENTITYMATRIX;
		std::vector<MeshComponent::Vertex_POS> vertex_positions_simulation; // graphics vertices after simulation (world space)
		AABB aabb;

		inline void SetDisableDeactivation(bool value) { if (value) { _flags |= DISABLE_DEACTIVATION; } else { _flags &= ~DISABLE_DEACTIVATION; } }

		inline bool IsDisableDeactivation() const { return _flags & DISABLE_DEACTIVATION; }

		// Create physics represenation of graphics mesh
		void CreateFromMesh(const MeshComponent& mesh);

		void Serialize(wiArchive& archive, wiECS::Entity seed = wiECS::INVALID_ENTITY);
	};

	struct FlexiBoneChainComponent
	{
		struct ConstraintParams 
		{
			float softness = 1.f;
			float biasFactor = 0.3f;
			float relaxationFactor = 1.f;
		};

		float capsuleRadius = 1.0f;

		// Chain definition: bones are ordered parent -> child
		std::vector<wiECS::Entity> bones;
		std::vector<float> boneLengths;
		std::vector<void*> physicsBodies;
		std::vector<ConstraintParams> constraints; // size is bones.size()-2!

		void Serialize(wiArchive& archive, wiECS::Entity seed = wiECS::INVALID_ENTITY) {/*TODO*/}
	};

	struct ForceFieldComponent
	{
		enum FLAGS
		{
			EMPTY = 0,
		};
		uint32_t _flags = EMPTY;

		int type = ENTITY_TYPE_FORCEFIELD_POINT;
		float gravity = 0.0f; // negative = deflector, positive = attractor
		float range_local = 0.0f; // affection range

		// Non-serialized attributes:
		XMFLOAT3 position;
		float range_global;
		XMFLOAT3 direction;

		inline float GetRange() const { return range_global; }

		void Serialize(wiArchive& archive, wiECS::Entity seed = wiECS::INVALID_ENTITY);
	};

	struct InverseKinematicsComponent
	{
		enum FLAGS
		{
			EMPTY = 0,
			DISABLED = 1 << 0,
		};
		uint32_t _flags = EMPTY;

		wiECS::Entity target = wiECS::INVALID_ENTITY; // which entity to follow (must have a transform component)
		uint32_t chain_length = ~0; // ~0 means: compute until the root
		uint32_t iteration_count = 1;

		inline void SetDisabled(bool value = true) { if (value) { _flags |= DISABLED; } else { _flags &= ~DISABLED; } }
		inline bool IsDisabled() const { return _flags & DISABLED; }

		void Serialize(wiArchive& archive, wiECS::Entity seed = wiECS::INVALID_ENTITY);
	};

	struct SpringComponent
	{
		enum FLAGS
		{
			EMPTY = 0,
			RESET = 1 << 0,
			DISABLED = 1 << 1,
			STRETCH_ENABLED = 1 << 2,
			GRAVITY_ENABLED = 1 << 3,
		};
		uint32_t _flags = RESET;

		float stiffness = 100;
		float damping = 0.8f;
		float wind_affection = 0;

		// Non-serialized attributes:
		XMFLOAT3 center_of_mass;
		XMFLOAT3 velocity;

		inline void Reset(bool value = true) { if (value) { _flags |= RESET; } else { _flags &= ~RESET; } }
		inline void SetDisabled(bool value = true) { if (value) { _flags |= DISABLED; } else { _flags &= ~DISABLED; } }
		inline void SetStretchEnabled(bool value) { if (value) { _flags |= STRETCH_ENABLED; } else { _flags &= ~STRETCH_ENABLED; } }
		inline void SetGravityEnabled(bool value) { if (value) { _flags |= GRAVITY_ENABLED; } else { _flags &= ~GRAVITY_ENABLED; } }

		inline bool IsResetting() const { return _flags & RESET; }
		inline bool IsDisabled() const { return _flags & DISABLED; }
		inline bool IsStretchEnabled() const { return _flags & STRETCH_ENABLED; }
		inline bool IsGravityEnabled() const { return _flags & GRAVITY_ENABLED; }

		void Serialize(wiArchive& archive, wiECS::Entity seed = wiECS::INVALID_ENTITY);
	};
}
