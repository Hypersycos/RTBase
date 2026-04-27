#pragma once

#include "Core.h"
#include "Sampling.h"
#include "Geometry.h"
#include "Imaging.h"
#include "Materials.h"
#include "Lights.h"
#include <unordered_map>
#include <queue>

class Camera
{
public:
	Matrix projectionMatrix;
	Matrix inverseProjectionMatrix;
	Matrix camera;
	Matrix cameraToView;
	float width = 0;
	float height = 0;
	Vec3 origin;
	Vec3 viewDirection;
	float Afilm;
	void init(Matrix ProjectionMatrix, int screenwidth, int screenheight)
	{
		projectionMatrix = ProjectionMatrix;
		inverseProjectionMatrix = ProjectionMatrix.invert();
		width = (float)screenwidth;
		height = (float)screenheight;
		float Wlens = (2.0f / ProjectionMatrix.a[1][1]);
		float aspect = ProjectionMatrix.a[0][0] / ProjectionMatrix.a[1][1];
		float Hlens = Wlens * aspect;
		Afilm = Wlens * Hlens;
	}
	void updateView(Matrix V)
	{
		camera = V;
		cameraToView = V.invert();
		origin = camera.mulPoint(Vec3(0, 0, 0));
		viewDirection = inverseProjectionMatrix.mulPointAndPerspectiveDivide(Vec3(0, 0, 1));
		viewDirection = camera.mulVec(viewDirection);
		viewDirection = viewDirection.normalize();
	}
	// Add code here
	Ray generateRay(float x, float y)
	{
		x /= width;
		y = 1 - y / height;

		x = x * 2 - 1;
		y = y * 2 - 1;

		Vec3 clipDir = { x, y, 1 };
		Vec3 cameraDir = inverseProjectionMatrix.mulPointAndPerspectiveDivide(clipDir);
		Vec3 worldDir = camera.mulVec(cameraDir);
		worldDir = worldDir.normalize();

		return Ray(origin, worldDir);
	}

	bool projectOntoCamera(const Vec3& p, float& x, float& y)
	{
		Vec3 pview = cameraToView.mulPoint(p);
		Vec3 pproj = projectionMatrix.mulPointAndPerspectiveDivide(pview);
		x = (pproj.x + 1.0f) * 0.5f;
		y = (pproj.y + 1.0f) * 0.5f;
		if (x < 0 || x > 1.0f || y < 0 || y > 1.0f)
		{
			return false;
		}
		x = x * width;
		y = 1.0f - y;
		y = y * height;
		return true;
	}
};

struct VPL
{
	ShadingData shadingData;
	Colour Le;
};

class FakeSampler : public MTRandom
{
public:
	std::queue<float> seeded;
	float next()
	{
		if (seeded.empty())
			return MTRandom::next();
		float val = seeded.front();
		seeded.pop();
		return val;
	}
	void seed(float val)
	{
		seeded.push(val);
	}
};

class Scene
{
public:
	std::vector<Triangle> triangles;
	std::vector<BSDF*> materials;
	std::vector<Light*> lights;
	std::unordered_map<Light*, std::vector<VPL>> vpls;
	std::unordered_map<BSDF*, Light*> bsdfToLight;
	Light* background = NULL;
	float lightPMF = 0;
	BVHNode* bvh = NULL;
	Camera camera;
	AABB bounds;
	void build()
	{
		// Add BVH building code here
		bvh = new BVHNode();
		bvh->buildRoot(triangles);
		
		// Do not touch the code below this line!
		// Build light list
		for (int i = 0; i < triangles.size(); i++)
		{
			if (materials[triangles[i].materialIndex]->isLight())
			{
				AreaLight* light = new AreaLight();
				light->triangle = &triangles[i];
				light->emission = materials[triangles[i].materialIndex]->emission;
				lights.push_back(light);
				bsdfToLight.emplace(materials[triangles[i].materialIndex], light);

				lightPMF += light->totalIntegratedPower();
			}
		}
#ifdef InstantRadiosity
		sampleVPLs(InstantRadiosity);
#endif
	}

	void sampleVPLs(unsigned int nvpl)
	{
		FakeSampler sampler;
		MTRandom baseSampler;
		int i = 0;
		for (Light* light : lights)
		{
			vpls.emplace(light, std::vector<VPL>{});
			std::vector<VPL>& vec = vpls[light];
			for (unsigned int i = 0; i < nvpl; i++)
			{
				float pct = (float)i / nvpl;
				float vary = 1.0f / nvpl;
				float pdfDirection;
				float pdfPosition;
				if (light->isArea())
				{
					sampler.seed(std::clamp(baseSampler.next() * vary + pct, 0.0f, 1.0f));
					sampler.seed(baseSampler.next() * 0.25f + (i % 4) * 0.25f);
				}
				else
				{
					sampler.seed(std::clamp(baseSampler.next() * vary + pct, 0.0f, 1.0f));
					sampler.seed(baseSampler.next() * 0.25f + (i % 4) * 0.25f);
				}

				Vec3 wi = light->sampleDirectionFromLight(&sampler, pdfDirection);
				Vec3 p = light->samplePositionFromLight(&baseSampler, pdfPosition);

				Colour col = light->evaluate(-wi) / (pdfPosition * pdfDirection);

				Ray r;
				r.init(p + wi * EPSILON, wi);
				Colour pt{ 1,1,1 };

				buildVPL(r, pt, InstantRadiosityMaxBounce, col, &baseSampler, vec);
			}
			std::cout << vec.size() << " VPLs generated for light " << i++ << std::endl;
		}
	}

	void buildVPL(Ray& r, Colour pathThroughput, int depth, Colour Le, Sampler* sampler, std::vector<VPL>& vec)
	{
		IntersectionData intersection = traverse(r);
		ShadingData shadingData = calculateShadingData(intersection, r);

		if (shadingData.t < FLT_MAX && !shadingData.bsdf->isLight())
		{
			if (!shadingData.bsdf->isPureSpecular())
			{
				vec.push_back({ shadingData, Le * pathThroughput });
			}

			Ray newRay;
			Colour sampleEffect;
			float pdf;
			Vec3 wo = shadingData.bsdf->sample(shadingData, sampler, sampleEffect, pdf);
			float cosTheta = fabsf(shadingData.sNormal.dot(wo));

			newRay.init(shadingData.x + wo * EPSILON, wo);
			pathThroughput = pathThroughput * sampleEffect * cosTheta / pdf;

			if ((pathThroughput * Le).Lum() > InstantRadiosityRelevance && depth > 0)
			{
				buildVPL(newRay, pathThroughput, depth - 1, Le, sampler, vec);
			}
		}
	}

	Colour evaluateVPL(const VPL& vpl, const ShadingData& hitData)
	{
		if (!visible(vpl.shadingData.x, hitData.x))
			return Colour{ 0,0,0 };
		Vec3 pointToLight = vpl.shadingData.x - hitData.x;
		Vec3 wi = pointToLight.normalize();
		float cosT = vpl.shadingData.sNormal.dot(-wi);
		if (cosT < 0)
			return Colour{ 0,0,0 };

		float cosT2 = hitData.sNormal.dot(wi);
		if (cosT2 < 0)
			return Colour{ 0,0,0 };
#ifdef InstantRadiosityClamp
		float dist = std::max<float>(pointToLight.lengthSq(), InstantRadiosityClamp);
#else
		float dist = pointToLight.lengthSq();
#endif
		Colour vplBsdf = vpl.shadingData.bsdf->evaluate(vpl.shadingData, -wi);
		Colour hitBsdf = hitData.bsdf->evaluate(hitData, wi);

		if (std::isinf(vplBsdf.Lum()) || std::isinf(hitBsdf.Lum()) || std::isnan(vplBsdf.Lum()) || std::isnan(hitBsdf.Lum()))
			return Colour{ 0,0,0 };

		return vpl.Le * vplBsdf * hitBsdf * cosT * cosT2 / dist;
	}

	IntersectionData traverse(const Ray& ray)
	{
		return bvh->traverse(ray, triangles);
	}

	Light* sampleLightUniform(Sampler* sampler, float& pmf)
	{
		static float PMF = 1.0f / lights.size();
		pmf = PMF;
		return lights[std::min<unsigned int>(lights.size() - 1, std::floor(sampler->next() * lights.size()))];
	}

	Light* sampleLightWeighted(Sampler* sampler, float& pmf)
	{
		//TODO: binary search tree?
		if (lightPMF == 0)
			return sampleLightUniform(sampler, pmf);

		float target = sampler->next() * lightPMF - lights[0]->totalIntegratedPower();
		unsigned int i = 0;
		while (i < lights.size() - 1 && target > 0)
		{
			target -= lights[++i]->totalIntegratedPower();
		}
		pmf = lights[i]->totalIntegratedPower() / lightPMF;
		return lights[i];
	}

	Light* sampleLightWeightedDistance(Sampler* sampler, Vec3& position, float& pmf)
	{
		if (lightPMF == 0 || lights.size() > 100)
			return sampleLightUniform(sampler, pmf);

		thread_local static std::vector<float> weights;
		weights.resize(lights.size());
		float currentTarget = 0;
		float currentWeight = 0;
		float sum = 0;
		float rand = sampler->next();

		for (int i = 0; i < lights.size(); i++)
		{
			float weight;
			if (lights[i]->isArea())
			{
				weight = lights[i]->totalIntegratedPower() / (dynamic_cast<AreaLight*>(lights[i])->triangle->centre() - position).lengthSq();
			}
			else
			{
				weight = lights[i]->totalIntegratedPower();
			}
			weights[i] = weight;
			sum += weight;
			currentWeight += weight * rand;
			while (currentWeight > weights[currentTarget])
			{
				currentWeight -= weights[currentTarget++];
			}
		}

		if (currentTarget >= lights.size())
			currentTarget = lights.size() - 1;

		pmf = weights[currentTarget] / sum;
		return lights[currentTarget];
	}

	float pdfLightWeightedDistance(ShadingData& data, ShadingData& newData, Ray r, bool environment)
	{
		if (lightPMF == 0 || lights.size() > 100)
			if (environment)
				return 1.0f / lights.size() * background->PDF(newData, -r.dir);
			else
				return 1.0f / lights.size() * bsdfToLight[newData.bsdf]->PDF(newData, -r.dir);

		float targetWeight = 0;
		float targetPdf = 0;
		float sum = 0;

		for (int i = 0; i < lights.size(); i++)
		{
			float weight;
			if (lights[i]->isArea())
			{
				AreaLight* areaLight = dynamic_cast<AreaLight*>(lights[i]);
				weight = lights[i]->totalIntegratedPower() / (areaLight->triangle->centre() - data.x).lengthSq();
				float t, u, v;
				if (lights[i] == bsdfToLight[newData.bsdf])
				{
					targetWeight = weight;
					targetPdf = lights[i]->PDF(data, r.dir);
				}
			}
			else
			{
				weight = lights[i]->totalIntegratedPower();
				if (environment)
				{
					targetWeight = weight;
					targetPdf = lights[i]->PDF(data, r.dir);
				}
			}
			sum += weight;
		}

		return targetWeight * targetPdf / sum;
	}

	Light* sampleLight(Sampler* sampler, float& pmf)
	{
		return sampleLightWeighted(sampler, pmf);
	}
	// Do not modify any code below this line
	void init(std::vector<Triangle> meshTriangles, std::vector<BSDF*> meshMaterials, Light* _background)
	{
		for (int i = 0; i < meshTriangles.size(); i++)
		{
			triangles.push_back(meshTriangles[i]);
			bounds.extend(meshTriangles[i].vertices[0].p);
			bounds.extend(meshTriangles[i].vertices[1].p);
			bounds.extend(meshTriangles[i].vertices[2].p);
		}
		for (int i = 0; i < meshMaterials.size(); i++)
		{
			materials.push_back(meshMaterials[i]);
		}
		background = _background;
		if (background->totalIntegratedPower() > 0)
		{
			lights.push_back(background);
			lightPMF += background->totalIntegratedPower();
		}
	}
	bool visible(const Vec3& p1, const Vec3& p2)
	{
		Ray ray;
		Vec3 dir = p2 - p1;
		float maxT = dir.length() - (2.0f * EPSILON);
		dir = dir.normalize();
		ray.init(p1 + (dir * EPSILON), dir);
		return bvh->traverseVisible(ray, maxT, triangles);
	}
	Colour emit(Triangle* light, ShadingData shadingData, Vec3 wi)
	{
		return materials[light->materialIndex]->emit(shadingData, wi);
	}
	ShadingData calculateShadingData(IntersectionData intersection, Ray& ray)
	{
		ShadingData shadingData = {};
		if (intersection.t < FLT_MAX)
		{
			shadingData.x = ray.at(intersection.t);
			shadingData.gNormal = triangles[intersection.ID].gNormal();
			triangles[intersection.ID].interpolateAttributes(intersection.alpha, intersection.beta, intersection.gamma, shadingData.sNormal, shadingData.tu, shadingData.tv);
			shadingData.bsdf = materials[triangles[intersection.ID].materialIndex];
			shadingData.wo = -ray.dir;
			if (shadingData.bsdf->isTwoSided())
			{
				if (Dot(shadingData.wo, shadingData.sNormal) < 0)
				{
					shadingData.sNormal = -shadingData.sNormal;
				}
				if (Dot(shadingData.wo, shadingData.gNormal) < 0)
				{
					shadingData.gNormal = -shadingData.gNormal;
				}
			}
			shadingData.frame.fromVector(shadingData.sNormal);
			shadingData.t = intersection.t;
		} else
		{
			shadingData.wo = -ray.dir;
			shadingData.t = intersection.t;
		}
		return shadingData;
	}
};