

#include "GEMLoader.h"
#include "Renderer.h"
#include "SceneLoader.h"
#define NOMINMAX
#include "GamesEngineeringBase.h"
#include <unordered_map>

void runTests()
{
	// Add test code here
}

int main(int argc, char *argv[])
{
	// Add call to tests if required
	// runTests()
	
	// Initialize default parameters
	std::string sceneName = "scenes/staircase2";
	//sceneName = "scenes/cornell-box";
	std::string filename = "GI.hdr";
	unsigned int SPP = 8192;

	if (argc > 1)
	{
		std::unordered_map<std::string, std::string> args;
		for (int i = 1; i < argc; ++i)
		{
			std::string arg = argv[i];
			if (!arg.empty() && arg[0] == '-')
			{
				std::string argName = arg;
				if (i + 1 < argc)
				{
					std::string argValue = argv[++i];
					args[argName] = argValue;
				} else
				{
					std::cerr << "Error: Missing value for argument '" << arg << "'\n";
				}
			} else
			{
				std::cerr << "Warning: Ignoring unexpected argument '" << arg << "'\n";
			}
		}
		for (const auto& pair : args)
		{
			if (pair.first == "-scene")
			{
				sceneName = pair.second;
			}
			if (pair.first == "-outputFilename")
			{
				filename = pair.second;
			}
			if (pair.first == "-SPP")
			{
				SPP = stoi(pair.second);
			}
		}
	}

	SYSTEM_INFO sysInfo;
	GetSystemInfo(&sysInfo);
	std::vector<TaskThread*> threads;
	threads.resize(sysInfo.dwNumberOfProcessors);
	for (int i = 0; i < threads.size(); i++)
	{
		threads[i] = new TaskThread();
	}

	Scene* scene = loadScene(sceneName);
	GamesEngineeringBase::Window canvas;
	canvas.create((unsigned int)scene->camera.width, (unsigned int)scene->camera.height, "Tracer", false);
	RayTracer rt;
	rt.init(scene, &canvas, threads);
	bool running = true;
	GamesEngineeringBase::Timer timer;

	bool fast = false;
	float fastDebounce = 0;
	int xL = 0;
	int xR = scene->camera.width;
	int yT = 0;
	int yB = scene->camera.height;

	while (running)
	{
		canvas.checkInput();
		canvas.clear();

		if (canvas.keyPressed(VK_ESCAPE))
		{
			break;
		}
		if (canvas.keyPressed('W'))
		{
			viewcamera.forward(canvas.keyPressed(VK_CONTROL));
			rt.clear();
		}
		if (canvas.keyPressed('S'))
		{
			viewcamera.back(canvas.keyPressed(VK_CONTROL));
			rt.clear();
		}
		if (canvas.keyPressed('A'))
		{
			viewcamera.left(canvas.keyPressed(VK_CONTROL));
			rt.clear();
		}
		if (canvas.keyPressed('D'))
		{
			viewcamera.right(canvas.keyPressed(VK_CONTROL));
			rt.clear();
		}
		if (canvas.keyPressed('E'))
		{
			viewcamera.flyUp(canvas.keyPressed(VK_CONTROL));
			rt.clear();
		}
		if (canvas.keyPressed('Q'))
		{
			viewcamera.flyDown(canvas.keyPressed(VK_CONTROL));
			rt.clear();
		}

		if (canvas.keyPressed(VK_LEFT))
		{
			viewcamera.rotLeft();
			rt.clear();
		}
		if (canvas.keyPressed(VK_RIGHT))
		{
			viewcamera.rotRight();
			rt.clear();
		}
		if (canvas.keyPressed(VK_UP))
		{
			viewcamera.rotUp();
			rt.clear();
		}
		if (canvas.keyPressed(VK_DOWN))
		{
			viewcamera.rotDown();
			rt.clear();
		}

		if (canvas.keyPressed('J'))
		{
			xR -= 8 * (canvas.keyPressed(VK_SHIFT) ? -1 : 1);
			if (xR - 16 < xL)
				xR = xL + 16;
			if (xR > rt.film->width)
				xR = rt.film->width;
			rt.clear();
		}
		if (canvas.keyPressed('L'))
		{
			xL += 8 * (canvas.keyPressed(VK_SHIFT) ? -1 : 1);
			if (xL + 16 > xR)
				xL = xR - 16;
			if (xL < 0)
				xL = 0;
			rt.clear();
		}
		if (canvas.keyPressed('I'))
		{
			yB -= 8 * (canvas.keyPressed(VK_SHIFT) ? -1 : 1);
			if (yB - 16 < yT)
				yB = yT + 16;
			if (yB > rt.film->width)
				yB = rt.film->width;
			rt.clear();
		}
		if (canvas.keyPressed('K'))
		{
			yT += 8 * (canvas.keyPressed(VK_SHIFT) ? -1 : 1);
			if (yT + 16 > yB)
				yT = yB - 16;
			if (xL < 0)
				yT = 0;
			rt.clear();
		}

		if (canvas.keyPressed('R'))
		{
			xL = 0;
			xR = rt.film->width;
			yT = 0;
			yB = rt.film->height;
			rt.clear();
		}

		if (canvas.keyPressed('F') && fastDebounce <= 0)
		{
			fast = !fast;
			rt.clear();
			fastDebounce = 2;
		}

		// Time how long a render call takes
		timer.reset();
		rt.render(threads, fast, xL, xR, yT, yB);
		float t = timer.dt();
		fastDebounce -= t;
		// Write
#ifdef ADDITIVESAMPLES
		std::cout << rt.film->SPP << ": " << t << std::endl;
#else
		std::cout << t << std::endl;
#endif
		if (canvas.keyPressed('P'))
		{
			rt.saveHDR(filename);
		}
		if (canvas.keyPressed('O'))
		{
			size_t pos = filename.find_last_of('.');
			std::string ldrFilename = filename.substr(0, pos) + ".png";
			rt.savePNG(ldrFilename);
		}
		if (SPP == rt.getSPP())
		{
			rt.saveHDR(filename);
			break;
		}
		canvas.present();
	}
	return 0;
}