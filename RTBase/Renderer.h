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
	std::atomic<int> tileCount = 0;
	unsigned int tileSize = 16;
	unsigned int maxTiles;

	void init(Scene* _scene, GamesEngineeringBase::Window* _canvas, std::vector<TaskThread*>& threads)
	{
		scene = _scene;
		canvas = _canvas;
		film = new Film();
		film->init((unsigned int)scene->camera.width, (unsigned int)scene->camera.height, new MitchellNetravaliFilter());

		maxTiles = std::ceil(film->width / (float)tileSize) * std::ceil(film->height / (float)tileSize);
		samplers = new MTRandom[threads.size()];
		for (unsigned int i = 0; i < threads.size(); i++)
		{
			samplers[i] = MTRandom{ i + 1 };
		}
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
			//Light* light = scene->sampleLight(sampler, pmf);
			Light* light = scene->sampleLightWeightedDistance(sampler, shadingData.x, pmf);
			float pdf;
			if (light->isArea())
			{
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
							return emittedColour * bsdfColour * geoTerm / (pdf * pmf);
						}
					}
				}
			}
			else
			{
				Vec3 wi = light->sampleDirectionFromLight(sampler, pdf);

				float cosTheta = wi.dot(shadingData.sNormal);
				if (cosTheta > 0)
				{
					float t;
					Ray lightRay = Ray(shadingData.x, wi);
					scene->bounds.rayAABB(lightRay, t);
					Vec3 oobPoint = lightRay.at(t + 1);
					if (scene->visible(shadingData.x, oobPoint))
					{
						float geoTerm = cosTheta;
						Colour bsdfColour = shadingData.bsdf->evaluate(shadingData, wi);
						return light->evaluate(wi) * bsdfColour * geoTerm / (pdf * pmf);
					}
				}
			}
		}
		// Compute direct lighting here
		return Colour(0.0f, 0.0f, 0.0f);
	}

	Colour pathTraceWrapper(Ray& r, Sampler* sampler)
	{
		Colour pathThroughput = { 1,1,1 };
		Colour result = pathTrace(r, pathThroughput, 0, sampler);
		return result;
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

/*			if (depth == 0 && shadingData.bsdf->PDF(shadingData, shadingData.wo) != 0)
				return computeDirect(shadingData, sampler);*/

			Colour direct = pathThroughput * computeDirect(shadingData, sampler);

			Ray indirect;
			Colour bsdfColour;
			float rayPdf;
			Vec3 rayDir = shadingData.bsdf->sample(shadingData, sampler, bsdfColour, rayPdf);
			indirect.init(shadingData.x + rayDir * EPSILON, rayDir);

			float cosOverPdf = fabsf(rayDir.dot(shadingData.sNormal)) / rayPdf;

			pathThroughput *= bsdfColour * cosOverPdf;

			float q = depth > 4 ? std::min(pathThroughput.Lum(), 0.9f) : 1.0f;
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

		return scene->background->evaluate(r.dir) * pathThroughput;
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

	void renderPixel(unsigned int x, unsigned int y, Sampler* sampler, bool fast)
	{
		float px = x + 0.5f;
		float py = y + 0.5f;
		Ray ray = scene->camera.generateRay(px, py);
		//Colour col = viewNormals(ray);
		Colour col{};
		for (int i = 0; i < SAMPLESPP; i++)
		{
			if (fast)
				col = col + direct(ray, sampler);
			else
				col = col + pathTraceWrapper(ray, sampler);
		}
		film->splat(px, py, col);
	}

	void renderST(bool fast)
	{
		if (fast)
		{
			for (unsigned int y = 0; y < film->height; y += 4)
			{
				for (unsigned int x = 0; x < film->width; x += 4)
				{
					renderPixel(x, y, samplers, fast);
				}
			}
		}
		else
		{
			for (unsigned int y = 0; y < film->height; y++)
			{
				for (unsigned int x = 0; x < film->width; x++)
				{
					renderPixel(x, y, samplers, fast);
				}
			}
		}
	}

	void renderMT(std::vector<TaskThread*>& threads, bool fast)
	{
		tileCount = 0;

		static std::vector<unsigned int> taskIDs;
		taskIDs.resize(threads.size());
		for (int i = 0; i < threads.size(); i++)
		{
			auto renderThread = [&,i,fast](std::stop_token stop)
				{
					unsigned int myTile;
					while ((myTile = tileCount++) < maxTiles && !stop.stop_requested())
					{
						renderTile(myTile, &(samplers[i]), fast);
					}
				};

			taskIDs[i] = threads[i]->QueueTask(renderThread);
		}
		for (int i = 0; i < threads.size(); i++)
		{
			threads[i]->Join(taskIDs[i]);
		}
	}

	void render(std::vector<TaskThread*>& threads, bool fast = false)
	{
#ifndef ADDITIVESAMPLES
		clear();
#endif
		film->SPP += SAMPLESPP;
#ifdef USETHREADS
		renderMT(threads, fast);
#else
		renderST(fast);
#endif
		if (fast)
		{
			for (unsigned int y = 0; y < film->height; y += 4)
			{
				for (unsigned int x = 0; x < film->width; x += 4)
				{
					unsigned char r, g, b;

					film->tonemap(x, y, r, g, b);
					canvas->draw(x, y, r, g, b);
				}
			}
		}
		else
		{
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
	}

	void renderTile(unsigned int index, Sampler* sampler, bool fast)
	{
		unsigned int tilesPerRow = (film->width / tileSize);

		unsigned int ystart = index / tilesPerRow * tileSize;
		unsigned int xstart = index % tilesPerRow * tileSize;
		unsigned int xend = std::min(xstart + tileSize, film->width);
		unsigned int yend = std::min(ystart + tileSize, film->height);

		if (fast)
		{
			for (unsigned int y = ystart; y < yend; y += 4)
			{
				for (unsigned int x = xstart; x < xend; x += 4)
				{
					renderPixel(x, y, sampler, fast);
				}
			}
		}
		else
		{
			for (unsigned int y = ystart; y < yend; y++)
			{
				for (unsigned int x = xstart; x < xend; x++)
				{
					renderPixel(x, y, sampler, fast);
				}
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