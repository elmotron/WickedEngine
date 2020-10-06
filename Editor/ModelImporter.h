#pragma once
#include <string>

namespace wiScene
{
	class Scene;
}

void ImportModel_OBJ(const std::string& fileName, wiScene::Scene& scene);
void ImportModel_GLTF(const std::string& fileName, wiScene::Scene& scene);

