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


class TestSampler : public Sampler
{
	// Inherited via Sampler
	float next()
	{
		return 0.5f;
	}
};

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

	Colour computeDirectMIS(ShadingData shadingData, Sampler* sampler)
	{
		float pmf;
		float pdf;
		Vec3 wi;
		// Is surface is specular we cannot computing direct lighting
		if (shadingData.bsdf->isPureSpecular() == false)
		{
			Light* light = scene->sampleLightWeightedDistance(sampler, shadingData.x, pmf);
			if (light->isArea())
			{
				Colour emittedColour;

				Vec3 lightPoint = light->sample(shadingData, sampler, emittedColour, pdf);
				Vec3 surfaceToLight = lightPoint - shadingData.x;
				wi = surfaceToLight.normalize();

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
							return emittedColour * bsdfColour * geoTerm * powerHeuristic(pdf * pmf, shadingData.bsdf->PDF(shadingData, wi)) / (pdf * pmf);
						}
					}
				}
			}
			else
			{
				wi = light->sampleDirectionFromLight(sampler, pdf);

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
						return light->evaluate(wi) * bsdfColour * geoTerm * powerHeuristic(pdf * pmf, shadingData.bsdf->PDF(shadingData, wi)) / (pdf * pmf);
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
		Colour result = pathTrace(r, pathThroughput, 0, sampler, true);
		return result;
	}

	Colour fastPathTrace(Ray& r, Colour& pathThroughput, int depthRemaining, Sampler* sampler, bool wasSpecular)
	{
		IntersectionData intersection = scene->traverse(r);
		ShadingData shadingData = scene->calculateShadingData(intersection, r);
		if (shadingData.t < FLT_MAX)
		{
			if (shadingData.bsdf->isLight())
			{
				return wasSpecular ? shadingData.bsdf->emit(shadingData, shadingData.wo) * pathThroughput : Colour{ 0,0,0 };
			}

			Colour direct = pathThroughput * computeDirect(shadingData, sampler);

			Ray indirect;
			Colour bsdfColour;
			float rayPdf;
			Vec3 rayDir = shadingData.bsdf->sample(shadingData, sampler, bsdfColour, rayPdf);
			indirect.init(shadingData.x + rayDir * EPSILON, rayDir);

			float cosOverPdf = fabsf(rayDir.dot(shadingData.sNormal)) / rayPdf;

			pathThroughput *= bsdfColour * cosOverPdf;

			if (depthRemaining > 0)
			{
				return direct + fastPathTrace(indirect, pathThroughput, depthRemaining - 1, sampler, shadingData.bsdf->isPureSpecular());
			}
			else
			{
				return direct + Colour{ 0,0,0 };
			}
		}

		return scene->background->evaluate(r.dir) * pathThroughput;
	}

	Colour pathTrace(Ray& r, Colour& pathThroughput, int depth, Sampler* sampler, bool wasSpecular)
	{
		IntersectionData intersection = scene->traverse(r);
		ShadingData shadingData = scene->calculateShadingData(intersection, r);
		if (shadingData.t < FLT_MAX)
		{
			if (shadingData.bsdf->isLight())
			{
				return wasSpecular ? shadingData.bsdf->emit(shadingData, shadingData.wo) * pathThroughput : Colour{ 0,0,0 };
			}

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
				return direct + pathTrace(indirect, pathThroughput, depth + 1, sampler, shadingData.bsdf->isPureSpecular());
			}
			else
			{
				return direct + Colour{ 0,0,0 };
			}
		}

		return scene->background->evaluate(r.dir) * pathThroughput;
	}

	float powerHeuristic(float a, float b)
	{
		float a2 = a * a;
		float b2 = b * b;
		return a2 / (a2 + b2);
	}

	Colour pathTraceMIS(Ray r, Sampler* sampler)
	{
		Colour pathThroughput = { 1,1,1 };
		int depth = 0;
		Colour result = { 0,0,0 };

		IntersectionData intersection = scene->traverse(r);
		ShadingData shadingData = scene->calculateShadingData(intersection, r);

		if (shadingData.t == FLT_MAX)
			return scene->background->evaluate(r.dir);

		if (shadingData.bsdf->isLight())
			return shadingData.bsdf->emit(shadingData, shadingData.wo);


		while (true)
		{
			if (!shadingData.bsdf->isPureSpecular())
				result = result + computeDirectMIS(shadingData, sampler) * pathThroughput;

			Colour bsdfColour;
			float rayPdf;
			Vec3 rayDir = shadingData.bsdf->sample(shadingData, sampler, bsdfColour, rayPdf);
			r.init(shadingData.x + rayDir * EPSILON, rayDir);

			float cosOverPdf = fabsf(rayDir.dot(shadingData.sNormal)) / rayPdf;
			pathThroughput *= bsdfColour * cosOverPdf;

			float q = depth > 4 ? std::min(pathThroughput.Lum(), 0.9f) : 1.0f;
			if (sampler->next() < q)
			{
				pathThroughput = pathThroughput / q;

				intersection = scene->traverse(r);
				ShadingData newShadingData = scene->calculateShadingData(intersection, r);

				if (newShadingData.t == FLT_MAX)
				{
/*					result = result + scene->background->evaluate(rayDir) * pathThroughput;
					break;*/
					result = result + scene->background->evaluate(rayDir)
									* powerHeuristic(rayPdf,
													scene->pdfLightWeightedDistance(shadingData, r, true))
									* pathThroughput;
					break;
				}
				else if (newShadingData.bsdf->isLight())
				{
/*					if (shadingData.bsdf->isPureSpecular())
						result = result + newShadingData.bsdf->emit(newShadingData, newShadingData.wo) * pathThroughput;
					break;*/
					result = result + newShadingData.bsdf->emit(newShadingData, newShadingData.wo)
									* powerHeuristic(rayPdf,
													 scene->pdfLightWeightedDistance(shadingData, r, false))
									* pathThroughput;
					break;
				}
				else
				{
					depth++;
					shadingData = newShadingData;
				}
			}
			else
			{
				break;
			}
		}

		return result;
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
			else if (shadingData.bsdf->isPureSpecular())
			{
				Colour pt{ 1,1,1 };
				return fastPathTrace(r, pt, 2, sampler, true);
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
				col = col + pathTraceMIS(ray, sampler);
		}
		if (fast)
			(*film)(x, y) = (*film)(x, y) + col;
		else
			film->splat(px, py, col);
	}

	void renderST(bool fast, unsigned int xL, unsigned int xR, unsigned int yT, unsigned int yB)
	{
		if (fast)
		{
			for (unsigned int y = yT; y < yB; y += 4)
			{
				for (unsigned int x = xL; x < xR; x += 4)
				{
					renderPixel(x, y, samplers, fast);
				}
			}
		}
		else
		{
			for (unsigned int y = yT; y < yB; y++)
			{
				for (unsigned int x = xL; x < xR; x++)
				{
					renderPixel(x, y, samplers, fast);
				}
			}
		}
	}

	void renderMT(std::vector<TaskThread*>& threads, bool fast, unsigned int xL, unsigned int xR, unsigned int yT, unsigned int yB)
	{
		tileCount = 0;

		static std::vector<unsigned int> taskIDs;
		taskIDs.resize(threads.size());
		for (int i = 0; i < threads.size(); i++)
		{
			auto renderThread = [&,i,fast,xL,xR,yT,yB](std::stop_token stop)
				{
					unsigned int myTile;
					while ((myTile = tileCount++) < maxTiles && !stop.stop_requested())
					{
						renderTile(myTile, &(samplers[i]), fast, xL, xR, yT, yB);
					}
				};

			taskIDs[i] = threads[i]->QueueTask(renderThread);
		}
		for (int i = 0; i < threads.size(); i++)
		{
			threads[i]->Join(taskIDs[i]);
		}
	}

	void render(std::vector<TaskThread*>& threads, bool fast = false, unsigned int xL = 0, unsigned int xR = INT32_MAX, unsigned int yT = 0, unsigned int yB = INT32_MAX)
	{
#ifndef ADDITIVESAMPLES
		clear();
#endif
		film->SPP += SAMPLESPP;

		xL = std::max(0u, xL);
		xR = std::min(xR, film->width);

		yT = std::max(0u, yT);
		yB = std::min(yB, film->height);

#ifdef USETHREADS
		renderMT(threads, fast, xL, xR, yT, yB);
#else
		renderST(fast, xL, xR, yT, yB);
#endif

		if (fast)
		{
			for (unsigned int y = yT; y < yB; y += 4)
			{
				for (unsigned int x = xL; x < xR; x += 4)
				{
					unsigned char r, g, b;
					film->tonemap(x, y, r, g, b);

					for (int i = 0; i < 4; i++)
					{
						for (int j = 0; j < 4; j++)
						{
							canvas->draw(x + j, y + i, r, g, b);
						}
					}
				}
			}
		}
		else
		{
			for (unsigned int y = yT; y < yB; y++)
			{
				for (unsigned int x = xL; x < xR; x++)
				{
					unsigned char r, g, b;

					film->tonemap(x, y, r, g, b);
					canvas->draw(x, y, r, g, b);
				}
			}
		}
	}

	void renderTile(unsigned int index, Sampler* sampler, bool fast, unsigned int xL, unsigned int xR, unsigned int yT, unsigned int yB)
	{
		unsigned int width = xR - xL;
		unsigned int height = yB - yT;

		unsigned int tilesPerRow = (width / tileSize);

		unsigned int ystart = index / tilesPerRow * tileSize + yT;
		unsigned int xstart = index % tilesPerRow * tileSize + xL;
		unsigned int xend = std::min(xstart + tileSize, xR);
		unsigned int yend = std::min(ystart + tileSize, yB);

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