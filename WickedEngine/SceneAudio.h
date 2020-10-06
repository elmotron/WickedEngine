#pragma once

#include "wiECS.h"
#include "wiArchive.h"
#include "wiAudio.h"
#include "wiResourceManager.h"
#include "CommonInclude.h"

#include <memory>

namespace wiScene
{
	struct SoundComponent
	{
		enum FLAGS
		{
			EMPTY = 0,
			PLAYING = 1 << 0,
			LOOPED = 1 << 1,
		};
		uint32_t _flags = LOOPED;

		std::string filename;
		std::shared_ptr<wiResource> soundResource;
		wiAudio::SoundInstance soundinstance;
		float volume = 1;

		inline bool IsPlaying() const { return _flags & PLAYING; }
		inline bool IsLooped() const { return _flags & LOOPED; }

		inline void Play() { _flags |= PLAYING; }
		inline void Stop() { _flags &= ~PLAYING; }
		inline void SetLooped(bool value = true) { if (value) { _flags |= LOOPED; } else { _flags &= ~LOOPED; } }

		void Serialize(wiArchive& archive, wiECS::Entity seed = wiECS::INVALID_ENTITY);
	};
}