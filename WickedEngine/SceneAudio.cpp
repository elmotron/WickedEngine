#include "wiScene.h"
#include "wiRenderer.h"

using namespace wiECS;

void wiScene::Scene::RunSoundUpdateSystem(wiJobSystem::context& ctx)
{
	const CameraComponent& camera = wiRenderer::GetCamera();
	wiAudio::SoundInstance3D instance3D;
	instance3D.listenerPos = camera.Eye;
	instance3D.listenerUp = camera.Up;
	instance3D.listenerFront = camera.At;

	for (size_t i = 0; i < sounds.GetCount(); ++i)
	{
		SoundComponent& sound = sounds[i];
		Entity entity = sounds.GetEntity(i);
		const TransformComponent* transform = transforms.GetComponent(entity);
		if (transform != nullptr)
		{
			instance3D.emitterPos = transform->GetPosition();
			wiAudio::Update3D(&sound.soundinstance, instance3D);
		}

		if (sound.IsPlaying())
			wiAudio::Play(&sound.soundinstance);
		else
			wiAudio::Stop(&sound.soundinstance);

		if (!sound.IsLooped())
			wiAudio::ExitLoop(&sound.soundinstance);

		wiAudio::SetVolume(sound.volume, &sound.soundinstance);
	}
}
