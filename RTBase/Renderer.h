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
	std::atomic<int> pathCount = 0;
	unsigned int tileSize = 16;
	unsigned int maxTiles;

	void init(Scene* _scene, GamesEngineeringBase::Window* _canvas, std::vector<TaskThread*>& threads)
	{
		scene = _scene;
		canvas = _canvas;
		film = new Film();
		unsigned int width = (unsigned int)scene->camera.width;
		unsigned int height = (unsigned int)scene->camera.height;

#ifdef Denoise
		film->oidnDevice = oidn::newDevice(oidn::DeviceType::CPU);
		film->oidnDevice.commit();

		film->albedos = new Colour[width * height];
		film->normals = new Vec3[width * height];

		film->colorBuff = new float[width * height * 3];
		film->albedoBuff = new float[width * height * 3];
		film->normalBuff = new float[width * height * 3];
		film->denoisedBuff = new float[width * height * 3];
#ifdef DefaultDenoised
		film->currentRenderBuff = film->denoisedBuff;
#else
		film->currentRenderBuff = film->colorBuff;
#endif

		film->oidnFilter = film->oidnDevice.newFilter("RT");
		film->oidnFilter.setImage("color", film->colorBuff, oidn::Format::Float3, width, height);
		film->oidnFilter.setImage("albedo", film->albedoBuff, oidn::Format::Float3, width, height);
		film->oidnFilter.setImage("normal", film->normalBuff, oidn::Format::Float3, width, height);
		film->oidnFilter.setImage("output", film->denoisedBuff, oidn::Format::Float3, width, height);
		film->oidnFilter.set("hdr", true);
#ifdef DenoiseCleanAux
		film->oidnFilter.set("cleanAux", true);
#endif
		film->oidnFilter.commit();

		film->init(width, height, new BoxFilter);
#else
		film->init(width, height, new MitchellNetravaliFilter);
#endif

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
#ifdef DenoiseCleanAux
		for (int y = 0; y < film->height; y++)
		{
			for (int x = 0; x < film->width; x++)
			{
				Ray ray = scene->camera.generateRay(x + 0.5f, y + 0.5f);
				IntersectionData intersection = scene->traverse(ray);
				Vec3 normal;
				Colour albedo;
				if (intersection.t < FLT_MAX)
				{
					ShadingData shadingData = scene->calculateShadingData(intersection, ray);
					normal = shadingData.sNormal;
					albedo;
					if (shadingData.bsdf->isLight())
						albedo = shadingData.bsdf->emit(shadingData, shadingData.wo);
					else
						albedo = shadingData.bsdf->evaluate(shadingData, shadingData.sNormal);
				}
				else
				{
					normal = -ray.dir;
					albedo = scene->background->evaluate(ray.dir);
				}
				unsigned int buffIndex = film->xyToIndex(x, y) * 3;
				film->albedoBuff[buffIndex + 0] = albedo.r;
				film->albedoBuff[buffIndex + 1] = albedo.g;
				film->albedoBuff[buffIndex + 2] = albedo.b;

				film->normalBuff[buffIndex + 0] = normal.x;
				film->normalBuff[buffIndex + 1] = normal.y;
				film->normalBuff[buffIndex + 2] = normal.z;
			}
		}
#endif
	}

	void connectToCamera(Vec3 p, Vec3 n, Colour col)
	{
		float x, y;
		if (!scene->camera.projectOntoCamera(p, x, y) || !scene->visible(p, scene->camera.origin))
		{
			return;
		}

		Vec3 camNormal = scene->camera.viewDirection;
		Vec3 camPos = scene->camera.origin;
		Vec3 dirToCam = (camPos - p).normalize();

		float cosTheta = camNormal.dot(-dirToCam);
		float dist2 = (camPos - p).lengthSq();

		float We = 1 / (scene->camera.Afilm * dist2 * SQ(SQ(cosTheta)));
		
		film->splat(x, y, col * We, film->lightTraceFilm);
	}

	void lightTrace(Sampler* sampler)
	{
		float pmf, pdfPosition, pdfDirection;
		Light* light = scene->sampleLightWeighted(sampler, pmf);

		Vec3 p = light->samplePositionFromLight(sampler, pdfPosition);
		Vec3 wi = light->sampleDirectionFromLight(sampler, pdfDirection);

		Colour col = light->evaluate(-wi) / (pdfPosition * pmf * pdfDirection);

		Ray r;
		r.init(p + wi * EPSILON, wi);
		Colour pt{ 1,1,1 };

		lightTracePath(r, pt, col, sampler);
	}

	void lightTracePath(Ray& r, Colour pathThroughput, Colour Le, Sampler* sampler)
	{
		IntersectionData intersection = scene->traverse(r);
		ShadingData shadingData = scene->calculateShadingData(intersection, r);

		if (shadingData.t < FLT_MAX && !shadingData.bsdf->isLight())
		{
			Vec3 wi = (scene->camera.origin - shadingData.x).normalize();
			Colour col = pathThroughput * shadingData.bsdf->evaluate(shadingData, wi) * Le * shadingData.gNormal.dot(wi);

			if (!shadingData.bsdf->isPureSpecular())
			{
				connectToCamera(shadingData.x, shadingData.gNormal, col);
			}

			Ray newRay;
			Colour sampleEffect;
			float pdf;
			Vec3 wo = shadingData.bsdf->sample(shadingData, sampler, sampleEffect, pdf);
			float cosTheta = shadingData.gNormal.dot(wo);
			if (cosTheta < 0)
			{ //refracted
				sampleEffect = sampleEffect * fabsf(wo.dot(shadingData.sNormal) / wo.dot(shadingData.gNormal))
											* fabsf(wi.dot(shadingData.gNormal) / wi.dot(shadingData.sNormal));
				cosTheta = -cosTheta;
			}

			newRay.init(shadingData.x + wo * EPSILON, wo);
			pathThroughput = pathThroughput * sampleEffect * cosTheta / pdf;

			float q = std::min(pathThroughput.Lum(), 0.9f);
			if (sampler->next() < q)
			{
				pathThroughput = pathThroughput / q;
				lightTracePath(newRay, pathThroughput, Le, sampler);
			}
		}
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
							float MISw = powerHeuristic(pdf * pmf * surfaceToLight.lengthSq() / cosThetaPrime, shadingData.bsdf->PDF(shadingData, wi));
							return emittedColour * bsdfColour * geoTerm * MISw / (pdf * pmf);
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
						float MISw = powerHeuristic(pdf * pmf, shadingData.bsdf->PDF(shadingData, wi));
						return light->evaluate(wi) * bsdfColour * geoTerm * MISw / (pdf * pmf);
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

			if (bsdfColour.Lum() == 0 || rayPdf == 0)
				return direct;

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

			if (bsdfColour.Lum() == 0 || rayPdf == 0)
				break;

			float cosTheta = fabsf(rayDir.dot(shadingData.sNormal));
			float cosOverPdf = cosTheta / rayPdf;
			pathThroughput *= bsdfColour * cosOverPdf;

			float q = depth > 4 ? std::min(pathThroughput.Lum(), 0.9f) : 1.0f;
			if (sampler->next() < q)
			{
				pathThroughput = pathThroughput / q;

				intersection = scene->traverse(r);
				ShadingData newShadingData = scene->calculateShadingData(intersection, r);

				if (newShadingData.t == FLT_MAX)
				{
					float lightPdf = scene->pdfLightWeightedDistance(shadingData, newShadingData, r, true);
					float MISw = powerHeuristic(rayPdf, lightPdf);
					result = result + scene->background->evaluate(rayDir)
									* MISw
									* pathThroughput;
					break;
				}
				else if (newShadingData.bsdf->isLight())
				{
					float cos = newShadingData.wo.dot(newShadingData.sNormal);
					float lightPdf = scene->pdfLightWeightedDistance(shadingData, newShadingData, r, false);
					lightPdf *= newShadingData.t * newShadingData.t / cos;
					float MISw = powerHeuristic(rayPdf, lightPdf);
					result = result + newShadingData.bsdf->emit(newShadingData, newShadingData.wo)
									* MISw
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
			return computeDirect(shadingData, sampler);
		}
		return scene->background->evaluate(r.dir);
	}

	void sampleVPLsFromLight(Light* light, ShadingData& shadingData, Sampler* sampler, Colour& result)
	{
#ifdef InstantRadiositySampleX
		int i = 0;
		std::vector<VPL>& vec = scene->vpls[light];
		while (sampler->next() > InstantRadiositySampleX && i < vec.size())
			i++;
		Colour temp;
		while (i < vec.size())
		{
			Colour c = scene->evaluateVPL(vec[i], shadingData) / InstantRadiositySampleX;
			temp = temp + c;
			while (sampler->next() > InstantRadiositySampleX && i < vec.size())
				i++;
		}
		result = result + temp / (InstantRadiosity * InstantRadiositySampleX);
#elif defined(InstantRadiosity)
		Colour temp;
		for (VPL& vpl : scene->vpls[light])
		{
			Colour c = scene->evaluateVPL(vpl, shadingData);
			temp = temp + c;
		}
		result = result + temp / InstantRadiosity;
#endif
	}

	Colour instantRadiosity(Ray r, Sampler* sampler)
	{
		IntersectionData intersection = scene->traverse(r);
		ShadingData shadingData = scene->calculateShadingData(intersection, r);
		Colour pt = { 1,1,1 };
		while (shadingData.t < FLT_MAX && shadingData.bsdf->isPureSpecular())
		{
			float mult;
			Colour colourMult;
			Vec3 wi = shadingData.bsdf->sample(shadingData, sampler, colourMult, mult);
			float cosT = fabsf(wi.dot(shadingData.sNormal));
			pt = pt * colourMult * cosT / mult;

			r.init(shadingData.x + wi * EPSILON, wi);

			intersection = scene->traverse(r);
			shadingData = scene->calculateShadingData(intersection, r);
		}

		if (shadingData.t < FLT_MAX)
		{
			if (shadingData.bsdf->isLight())
			{
				return pt * shadingData.bsdf->emit(shadingData, shadingData.wo);
			}
			Colour direct = computeDirect(shadingData, sampler);
			Colour indirect;
#ifdef InstantRadiosityRandomLight
			float pmf;
			Light* light = scene->sampleLightUniform(sampler, pmf);
			sampleVPLsFromLight(light, shadingData, sampler, indirect);
			indirect = indirect / scene->lights.size();
#else
			for (Light* light : scene->lights)
			{
				sampleVPLsFromLight(light, shadingData, sampler, indirect);
			}
			indirect = indirect / scene->lights.size();
#endif
			return pt * (direct + indirect);
		}
		return pt * scene->background->evaluate(r.dir);
	}

	Colour approxTrace(Ray& r, Sampler* sampler)
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
		float px = x + sampler->next();
		float py = y + sampler->next();
		Ray ray = scene->camera.generateRay(px, py);
		//Colour col = direct(ray, sampler);
		Colour col{};
		for (int i = 0; i < SAMPLESPP; i++)
		{
			if (fast)
				col = col + approxTrace(ray, sampler);
			else
#ifdef InstantRadiosity
				col = col + instantRadiosity(ray, sampler);
#else
#ifdef MultipleImportanceSampling
				col = col + pathTraceMIS(ray, sampler);
#else
				col = col + pathTraceWrapper(ray, sampler);
#endif
#endif
		}

		/*if (std::isnan(col.r) || std::isnan(col.g) || std::isnan(col.b))
		{
			std::cout << "NaN!" << std::endl;
			return;
		}*/

		if (col.r < 0 || col.g < 0 || col.b < 0)
		{
			std::cout << "Negative colour!" << std::endl;
			return;
		}

#if defined(Denoise) && !defined(DenoiseCleanAux)
		if (!fast)
		{
			IntersectionData intersection = scene->traverse(ray);
			Vec3 normal;
			Colour albedo;
			if (intersection.t < FLT_MAX)
			{
				ShadingData shadingData = scene->calculateShadingData(intersection, ray);
				normal = shadingData.sNormal;
				albedo;
				if (shadingData.bsdf->isLight())
					albedo = shadingData.bsdf->emit(shadingData, shadingData.wo);
				else
					albedo = shadingData.bsdf->evaluate(shadingData, shadingData.sNormal);
			}
			else
			{
				normal = -ray.dir;
				albedo = scene->background->evaluate(ray.dir);
			}
			film->denoiseData(x, y, albedo, normal);
		}
#endif

		if (fast)
			(*film)(x, y) = (*film)(x, y) + col;
		else
			film->splat(px, py, col, film->film);
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
#ifdef RayTrace
			for (unsigned int y = yT; y < yB; y++)
			{
				for (unsigned int x = xL; x < xR; x++)
				{
					renderPixel(x, y, samplers, fast);
				}
			}
#endif
#ifdef LightTrace
			for (unsigned int i = 0; i < LightTrace; i++)
			{
				lightTrace(samplers);
			}
#endif
		}
	}

	void renderMT(std::vector<TaskThread*>& threads, bool fast, unsigned int xL, unsigned int xR, unsigned int yT, unsigned int yB)
	{
		tileCount = 0;
		pathCount = 0;

		static std::vector<unsigned int> taskIDs;
		taskIDs.resize(threads.size());
		for (int i = 0; i < threads.size(); i++)
		{
			auto renderThread = [&, i, fast, xL, xR, yT, yB](std::stop_token stop)
				{
					unsigned int myTile;
					while ((myTile = tileCount++) < maxTiles && !stop.stop_requested())
					{
#ifdef CountTiles
						if (myTile % 250 == 0)
							std::cout << "Drawing tile: " << myTile << std::endl;
#endif
#ifndef RayTrace
						if (fast)
#endif
							renderTile(myTile, &(samplers[i]), fast, xL, xR, yT, yB);
					}
#ifdef LightTrace
					if (!fast)
					{
						unsigned int myPath;
						while ((myPath = pathCount++) < LightTrace && !stop.stop_requested())
						{
#ifdef CountTiles
							if (myPath % (250 * 16) == 0)
								std::cout << "Drawing path: " << myPath << std::endl;
#endif
							lightTrace(&(samplers[i]));
						}
					}
#endif
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
#ifdef Denoise
			for (unsigned int y = yT; y < yB; y++)
			{
				for (unsigned int x = xL; x < xR; x++)
				{
					unsigned int index = film->xyToIndex(x, y);
					unsigned int buffIndex = index * 3;

					film->colorBuff[buffIndex + 0] = film->film[index].r / film->SPP;
					film->colorBuff[buffIndex + 1] = film->film[index].g / film->SPP;
					film->colorBuff[buffIndex + 2] = film->film[index].b / film->SPP;
				}
			}
#endif
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
#ifdef Denoise
			for (unsigned int y = yT; y < yB; y++)
			{
				for (unsigned int x = xL; x < xR; x++)
				{
					unsigned int index = film->xyToIndex(x, y);
					unsigned int buffIndex = index * 3;

					Colour c = film->getCombinedColour(index);

					film->colorBuff[buffIndex + 0] = c.r;
					film->colorBuff[buffIndex + 1] = c.g;
					film->colorBuff[buffIndex + 2] = c.b;

#ifndef DenoiseCleanAux
					film->albedoBuff[buffIndex + 0] = film->albedos[index].r / film->SPP;
					film->albedoBuff[buffIndex + 1] = film->albedos[index].g / film->SPP;
					film->albedoBuff[buffIndex + 2] = film->albedos[index].b / film->SPP;

					film->normalBuff[buffIndex + 0] = film->normals[index].x / film->SPP;
					film->normalBuff[buffIndex + 1] = film->normals[index].y / film->SPP;
					film->normalBuff[buffIndex + 2] = film->normals[index].z / film->SPP;
#endif
				}
			}

			film->oidnDevice.sync();
			film->oidnFilter.execute();
			film->oidnDevice.sync();
			const char* errorMessage;
			if (film->oidnDevice.getError(errorMessage) != oidn::Error::None)
				printf("Error: %s\n", errorMessage);
#endif

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