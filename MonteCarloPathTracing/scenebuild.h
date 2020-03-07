#pragma once

#include "raygeneration.h"
#include "thirdpartywrapper.h"



namespace MCPT::SceneBuild {
	

	class SceneBase {
	public:
		SceneBase() = default;

		virtual void intersect(MCPT::RayGeneration::RayBase* rays) = 0;
		virtual void shade(MCPT::RayGeneration::RayBase* rays, cl::Buffer& colorBuffer) = 0;

		virtual ~SceneBase() = default;
	};

	class SceneCL : public SceneBase {
	public:
		SceneCL(std::vector<MCPT::Triangle>, std::vector<MCPT::Material>, std::vector<int>);
		virtual void intersect(MCPT::RayGeneration::RayBase* rays);
		virtual void shade(MCPT::RayGeneration::RayBase* rays, cl::Buffer& colorBuffer);

	};



	std::unique_ptr< SceneBase > buildScene(std::vector<MCPT::Triangle>, std::vector<MCPT::Material>, std::vector<int>);

	void init();

}