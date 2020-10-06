#include "wiScene.h"
#include "wiJobSystem.h"

using namespace wiECS;

XMFLOAT3 wiScene::TransformComponent::GetPosition() const
{
	return *((XMFLOAT3*)&world._41);
}
XMFLOAT4 wiScene::TransformComponent::GetRotation() const
{
	XMFLOAT4 rotation;
	XMStoreFloat4(&rotation, GetRotationV());
	return rotation;
}
XMFLOAT3 wiScene::TransformComponent::GetScale() const
{
	XMFLOAT3 scale;
	XMStoreFloat3(&scale, GetScaleV());
	return scale;
}
XMVECTOR wiScene::TransformComponent::GetPositionV() const
{
	return XMLoadFloat3((XMFLOAT3*)&world._41);
}
XMVECTOR wiScene::TransformComponent::GetRotationV() const
{
	XMVECTOR S, R, T;
	XMMatrixDecompose(&S, &R, &T, XMLoadFloat4x4(&world));
	return R;
}
XMVECTOR wiScene::TransformComponent::GetScaleV() const
{
	XMVECTOR S, R, T;
	XMMatrixDecompose(&S, &R, &T, XMLoadFloat4x4(&world));
	return S;
}
XMMATRIX wiScene::TransformComponent::GetLocalMatrix() const
{
	XMVECTOR S_local = XMLoadFloat3(&scale_local);
	XMVECTOR R_local = XMLoadFloat4(&rotation_local);
	XMVECTOR T_local = XMLoadFloat3(&translation_local);
	return
		XMMatrixScalingFromVector(S_local) *
		XMMatrixRotationQuaternion(R_local) *
		XMMatrixTranslationFromVector(T_local);
}
void wiScene::TransformComponent::UpdateTransform()
{
	if (IsDirty())
	{
		SetDirty(false);

		XMStoreFloat4x4(&world, GetLocalMatrix());
	}
}
void wiScene::TransformComponent::UpdateTransform_Parented(const TransformComponent& parent)
{
	XMMATRIX W = GetLocalMatrix();
	XMMATRIX W_parent = XMLoadFloat4x4(&parent.world);
	W = W * W_parent;

	XMStoreFloat4x4(&world, W);
}
void wiScene::TransformComponent::ApplyTransform()
{
	SetDirty();

	XMVECTOR S, R, T;
	XMMatrixDecompose(&S, &R, &T, XMLoadFloat4x4(&world));
	XMStoreFloat3(&scale_local, S);
	XMStoreFloat4(&rotation_local, R);
	XMStoreFloat3(&translation_local, T);
}
void wiScene::TransformComponent::ClearTransform()
{
	SetDirty();
	scale_local = XMFLOAT3(1, 1, 1);
	rotation_local = XMFLOAT4(0, 0, 0, 1);
	translation_local = XMFLOAT3(0, 0, 0);
}
void wiScene::TransformComponent::Translate(const XMFLOAT3& value)
{
	SetDirty();
	translation_local.x += value.x;
	translation_local.y += value.y;
	translation_local.z += value.z;
}
void wiScene::TransformComponent::Translate(const XMVECTOR& value)
{
	XMFLOAT3 translation;
	XMStoreFloat3(&translation, value);
	Translate(translation);
}
void wiScene::TransformComponent::RotateRollPitchYaw(const XMFLOAT3& value)
{
	SetDirty();

	// This needs to be handled a bit differently
	XMVECTOR quat = XMLoadFloat4(&rotation_local);
	XMVECTOR x = XMQuaternionRotationRollPitchYaw(value.x, 0, 0);
	XMVECTOR y = XMQuaternionRotationRollPitchYaw(0, value.y, 0);
	XMVECTOR z = XMQuaternionRotationRollPitchYaw(0, 0, value.z);

	quat = XMQuaternionMultiply(x, quat);
	quat = XMQuaternionMultiply(quat, y);
	quat = XMQuaternionMultiply(z, quat);
	quat = XMQuaternionNormalize(quat);

	XMStoreFloat4(&rotation_local, quat);
}
void wiScene::TransformComponent::Rotate(const XMFLOAT4& quaternion)
{
	SetDirty();

	XMVECTOR result = XMQuaternionMultiply(XMLoadFloat4(&rotation_local), XMLoadFloat4(&quaternion));
	result = XMQuaternionNormalize(result);
	XMStoreFloat4(&rotation_local, result);
}
void wiScene::TransformComponent::Rotate(const XMVECTOR& quaternion)
{
	XMFLOAT4 rotation;
	XMStoreFloat4(&rotation, quaternion);
	Rotate(rotation);
}
void wiScene::TransformComponent::Scale(const XMFLOAT3& value)
{
	SetDirty();
	scale_local.x *= value.x;
	scale_local.y *= value.y;
	scale_local.z *= value.z;
}
void wiScene::TransformComponent::Scale(const XMVECTOR& value)
{
	XMFLOAT3 scale;
	XMStoreFloat3(&scale, value);
	Scale(scale);
}
void wiScene::TransformComponent::MatrixTransform(const XMFLOAT4X4& matrix)
{
	MatrixTransform(XMLoadFloat4x4(&matrix));
}
void wiScene::TransformComponent::MatrixTransform(const XMMATRIX& matrix)
{
	SetDirty();

	XMVECTOR S;
	XMVECTOR R;
	XMVECTOR T;
	XMMatrixDecompose(&S, &R, &T, GetLocalMatrix() * matrix);

	XMStoreFloat3(&scale_local, S);
	XMStoreFloat4(&rotation_local, R);
	XMStoreFloat3(&translation_local, T);
}
void wiScene::TransformComponent::Lerp(const TransformComponent& a, const TransformComponent& b, float t)
{
	SetDirty();

	XMVECTOR aS, aR, aT;
	XMMatrixDecompose(&aS, &aR, &aT, XMLoadFloat4x4(&a.world));

	XMVECTOR bS, bR, bT;
	XMMatrixDecompose(&bS, &bR, &bT, XMLoadFloat4x4(&b.world));

	XMVECTOR S = XMVectorLerp(aS, bS, t);
	XMVECTOR R = XMQuaternionSlerp(aR, bR, t);
	XMVECTOR T = XMVectorLerp(aT, bT, t);

	XMStoreFloat3(&scale_local, S);
	XMStoreFloat4(&rotation_local, R);
	XMStoreFloat3(&translation_local, T);
}
void wiScene::TransformComponent::CatmullRom(const TransformComponent& a, const TransformComponent& b, const TransformComponent& c, const TransformComponent& d, float t)
{
	SetDirty();

	XMVECTOR aS, aR, aT;
	XMMatrixDecompose(&aS, &aR, &aT, XMLoadFloat4x4(&a.world));

	XMVECTOR bS, bR, bT;
	XMMatrixDecompose(&bS, &bR, &bT, XMLoadFloat4x4(&b.world));

	XMVECTOR cS, cR, cT;
	XMMatrixDecompose(&cS, &cR, &cT, XMLoadFloat4x4(&c.world));

	XMVECTOR dS, dR, dT;
	XMMatrixDecompose(&dS, &dR, &dT, XMLoadFloat4x4(&d.world));

	XMVECTOR T = XMVectorCatmullRom(aT, bT, cT, dT, t);

	XMVECTOR setupA;
	XMVECTOR setupB;
	XMVECTOR setupC;

	aR = XMQuaternionNormalize(aR);
	bR = XMQuaternionNormalize(bR);
	cR = XMQuaternionNormalize(cR);
	dR = XMQuaternionNormalize(dR);

	XMQuaternionSquadSetup(&setupA, &setupB, &setupC, aR, bR, cR, dR);
	XMVECTOR R = XMQuaternionSquad(bR, setupA, setupB, setupC, t);

	XMVECTOR S = XMVectorCatmullRom(aS, bS, cS, dS, t);

	XMStoreFloat3(&translation_local, T);
	XMStoreFloat4(&rotation_local, R);
	XMStoreFloat3(&scale_local, S);
}

void wiScene::Scene::RunHierarchyUpdateSystem(wiJobSystem::context& ctx)
{
	// This needs serialized execution because there are dependencies enforced by component order!

	for (size_t i = 0; i < hierarchy.GetCount(); ++i)
	{
		const HierarchyComponent& parentcomponent = hierarchy[i];
		Entity entity = hierarchy.GetEntity(i);

		TransformComponent* transform_child = transforms.GetComponent(entity);
		TransformComponent* transform_parent = transforms.GetComponent(parentcomponent.parentID);
		if (transform_child != nullptr && transform_parent != nullptr)
		{
			transform_child->UpdateTransform_Parented(*transform_parent);
		}


		LayerComponent* layer_child = layers.GetComponent(entity);
		LayerComponent* layer_parent = layers.GetComponent(parentcomponent.parentID);
		if (layer_child != nullptr && layer_parent != nullptr)
		{
			layer_child->layerMask = parentcomponent.layerMask_bind & layer_parent->GetLayerMask();
		}

	}
}

void wiScene::Scene::RunTransformUpdateSystem(wiJobSystem::context& ctx)
{
	wiJobSystem::Dispatch(ctx, (uint32_t)transforms.GetCount(), small_subtask_groupsize, [&](wiJobArgs args)
	{
		TransformComponent& transform = transforms[args.jobIndex];
		transform.UpdateTransform();
	});
}

void wiScene::Scene::RunPreviousFrameTransformUpdateSystem(wiJobSystem::context& ctx)
{
	wiJobSystem::Dispatch(ctx, (uint32_t)prev_transforms.GetCount(), small_subtask_groupsize, [&](wiJobArgs args)
	{
		PreviousFrameTransformComponent& prev_transform = prev_transforms[args.jobIndex];
		Entity entity = prev_transforms.GetEntity(args.jobIndex);
		const TransformComponent& transform = *transforms.GetComponent(entity);

		prev_transform.world_prev = transform.world;
	});
}
