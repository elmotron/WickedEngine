#pragma once

#include "wiECS.h"
#include <string>

namespace wiScene
{
	class Scene;
}

namespace AssetUtil
{
	void gltfImport(const std::string& fileName, wiScene::Scene& scene);
}
