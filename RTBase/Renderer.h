#pragma once

#include "Core.h"
#include "Sampling.h"
#include "Geometry.h"
#include "Imaging.h"
#include "Materials.h"
#include "Lights.h"
#include "Scene.h"
#include "GamesEngineeringBase.h"
#include "ThreadHelper.h"
#include <thread>
#include <functional>
#include <mutex>

class RayTracer
{
public:
	Scene* scene;
	GamesEngineeringBase::Window* canvas;
	Film* film;
	MTRandom *samplers;
	std::vector<TaskThread*> threads;
	unsigned int numProcs;
	std::atomic<int> tileCount = 0;
	unsigned int tileSize = 16;
	unsigned int maxTiles;

	void init(Scene* _scene, GamesEngineeringBase::Window* _canvas)
	{
		scene = _scene;
		canvas = _canvas;
		film = new Film();
		film->init((unsigned int)scene->camera.width, (unsigned int)scene->camera.height, new MitchellNetravaliFilter());
		SYSTEM_INFO sysInfo;
		GetSystemInfo(&sysInfo);
		maxTiles = std::ceil(film->width / (float)tileSize) * std::ceil(film->height / (float)tileSize);
		numProcs = sysInfo.dwNumberOfProcessors;
		threads.resize(numProcs);
		for (int i = 0; i < numProcs; i++)
		{
			threads[i] = new TaskThread();
		}
		samplers = new MTRandom[numProcs];
		clear();
	}
	void clear()
	{
		film->clear();
	}
	Colour computeDirect(ShadingData shadingData, Sampler* sampler)
	{
		// Is surface is specular we cannot computing direct lighting
		if (shadingData.bsdf->isPureSpecular() == false)
		{
			float pmf;
			Colour c = Colour{ 0,0,0 };
			Light* light = scene->sampleLight(sampler, pmf);
			if (light->isArea())
			{
				float pdf;
				Colour emittedColour;

				Vec3 lightPoint = light->sample(shadingData, sampler, emittedColour, pdf);
				Vec3 surfaceToLight = lightPoint - shadingData.x;
				Vec3 wi = surfaceToLight.normalize();

				float cosTheta = wi.dot(shadingData.sNormal);
				if (cosTheta > 0)
				{
					float cosThetaPrime = -(wi.dot(light->normal(shadingData, wi)));
					if (cosThetaPrime > 0)
					{
						if (scene->visible(shadingData.x, lightPoint))
						{
							float geoTerm = cosTheta * cosThetaPrime / (surfaceToLight.lengthSq());
							Colour bsdfColour = shadingData.bsdf->evaluate(shadingData, wi);
							c = emittedColour * bsdfColour * geoTerm / (pdf * pmf);
						}
					}
				}
			}
			else
			{
				//light->sampleDirectionFromLight(sampler, pdf);
				//geometry term
				//visibility
				//bsdf
			}
			return c;
		}
		// Compute direct lighting here
		return Colour(0.0f, 0.0f, 0.0f);
	}

	Colour pathTraceWrapper(Ray& r, Sampler* sampler)
	{
		Colour c;
		for (int i = 0; i < 32; i++)
		{
			Colour pathThroughput = { 1,1,1 };
			c = c + pathTrace(r, pathThroughput, 0, sampler);
		}
		return c / 32;
	}

	Colour pathTrace(Ray& r, Colour& pathThroughput, int depth, Sampler* sampler)
	{
		IntersectionData intersection = scene->traverse(r);
		ShadingData shadingData = scene->calculateShadingData(intersection, r);
		if (shadingData.t < FLT_MAX)
		{
			if (shadingData.bsdf->isLight())
			{
				return depth == 0 ? shadingData.bsdf->emit(shadingData, shadingData.wo) : Colour{ 0,0,0 };
			}
			Colour direct = pathThroughput * computeDirect(shadingData, sampler);

			Ray indirect;
			Vec3 rayDir = SamplingDistributions::cosineSampleHemisphere(sampler->next(), sampler->next());
			rayDir = shadingData.frame.toWorld(rayDir);
			indirect.init(shadingData.x + rayDir * EPSILON, rayDir);

			float cosOverPdf = rayDir.dot(shadingData.sNormal) / SamplingDistributions::cosineHemispherePDF(rayDir);
			pathThroughput *= shadingData.bsdf->evaluate(shadingData, rayDir) * cosOverPdf;

			float q = std::min(pathThroughput.Lum(), 0.99f);
			if (sampler->next() < q)
			{
				pathThroughput = pathThroughput / q;
				return direct + pathTrace(indirect, pathThroughput, depth + 1, sampler);
			}
			else
			{
				return direct + Colour{ 0,0,0 };
			}
		}
		return scene->background->evaluate(r.dir);
	}
	Colour direct(Ray& r, Sampler* sampler)
	{
		IntersectionData intersection = scene->traverse(r);
		ShadingData shadingData = scene->calculateShadingData(intersection, r);
		if (shadingData.t < FLT_MAX)
		{
			if (shadingData.bsdf->isLight())
			{
				return shadingData.bsdf->emit(shadingData, shadingData.wo);
			}
			return computeDirect(shadingData, sampler);
		}
		return scene->background->evaluate(r.dir);
	}
	Colour albedo(Ray& r)
	{
		IntersectionData intersection = scene->traverse(r);
		ShadingData shadingData = scene->calculateShadingData(intersection, r);
		if (shadingData.t < FLT_MAX)
		{
			if (shadingData.bsdf->isLight())
			{
				return shadingData.bsdf->emit(shadingData, shadingData.wo);
			}
			return shadingData.bsdf->evaluate(shadingData, Vec3(0, 1, 0));
		}
		return scene->background->evaluate(r.dir);
	}
	Colour viewNormals(Ray& r)
	{
		IntersectionData intersection = scene->traverse(r);
		if (intersection.t < FLT_MAX)
		{
			ShadingData shadingData = scene->calculateShadingData(intersection, r);
			return Colour(fabsf(shadingData.sNormal.x), fabsf(shadingData.sNormal.y), fabsf(shadingData.sNormal.z));
		}
		return Colour(0.0f, 0.0f, 0.0f);
	}
	Colour fakeShading(Ray& r, float percentMultNormal = 0.2, float percentAddNormal = 0.01)
	{
		IntersectionData intersection = scene->traverse(r);
		if (intersection.t < FLT_MAX)
		{
			ShadingData shadingData = scene->calculateShadingData(intersection, r);
			Colour normal = Colour(fabsf(shadingData.sNormal.x), fabsf(shadingData.sNormal.y), fabsf(shadingData.sNormal.z));
			Colour albedo;
			if (shadingData.bsdf->isLight())
				albedo = shadingData.bsdf->emit(shadingData, shadingData.wo);
			else
				albedo = shadingData.bsdf->evaluate(shadingData, Vec3(0, 1, 0));

			//return normal * percentNormal + albedo * (1 - percentNormal);
			return albedo * (1 - percentMultNormal - percentAddNormal) + albedo * normal * percentMultNormal + normal * percentAddNormal;
		}
		return Colour(0.0f, 0.0f, 0.0f);
	}

	void renderPixel(unsigned int x, unsigned int y, Sampler* sampler)
	{
		float px = x + 0.5f;
		float py = y + 0.5f;
		Ray ray = scene->camera.generateRay(px, py);
		//Colour col = fakeShading(ray, 0.2, 0.003);
		Colour col = pathTraceWrapper(ray, sampler);
		film->splat(px, py, col);
	}

	void renderST()
	{
		film->incrementSPP();
		for (unsigned int y = 0; y < film->height; y++)
		{
			for (unsigned int x = 0; x < film->width; x++)
			{
				renderPixel(x, y, samplers);
			}
		}
	}

	void renderMT()
	{
		tileCount = 0;

		std::vector<unsigned int> taskIDs;
		taskIDs.resize(numProcs);
		for (int i = 0; i < numProcs; i++)
		{
			auto renderThread = [&](std::stop_token stop)
				{
					unsigned int myTile;
					while ((myTile = tileCount++) < maxTiles && !stop.stop_requested())
					{
						renderTile(myTile, samplers);
					}
				};

			taskIDs[i] = threads[i]->QueueTask(renderThread);
		}
		for (int i = 0; i < numProcs; i++)
		{
			threads[i]->Join(taskIDs[i]);
		}
	}

	void render()
	{
		renderMT();
		for (unsigned int y = 0; y < film->height; y++)
		{
			for (unsigned int x = 0; x < film->width; x++)
			{
				unsigned char r, g, b;

				film->tonemap(x, y, r, g, b);
				canvas->draw(x, y, r, g, b);
			}
		}
	}

	void renderTile(unsigned int index, Sampler* sampler)
	{
		unsigned int tilesPerRow = (film->width / tileSize);

		unsigned int ystart = index / tilesPerRow * tileSize;
		unsigned int xstart = index % tilesPerRow * tileSize;
		unsigned int xend = std::min(xstart + tileSize, film->width);
		unsigned int yend = std::min(ystart + tileSize, film->height);

		film->incrementSPP();
		for (unsigned int y = ystart; y < yend; y++)
		{
			for (unsigned int x = xstart; x < xend; x++)
			{
				renderPixel(x, y, sampler);
			}
		}
	}

	int getSPP()
	{
		return film->SPP;
	}
	void saveHDR(std::string filename)
	{
		film->save(filename);
	}
	void savePNG(std::string filename)
	{
		stbi_write_png(filename.c_str(), canvas->getWidth(), canvas->getHeight(), 3, canvas->getBackBuffer(), canvas->getWidth() * 3);
	}
};