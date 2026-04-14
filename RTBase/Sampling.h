#pragma once

#include "Core.h"
#include <random>
#include <algorithm>

class Sampler
{
public:
	virtual float next() = 0;
};

class MTRandom : public Sampler
{
public:
	std::mt19937 generator;
	std::uniform_real_distribution<float> dist;
	MTRandom(unsigned int seed = 1) : dist(0.0f, 1.0f)
	{
		generator.seed(seed);
	}
	float next()
	{
		return dist(generator);
	}
};

// Note all of these distributions assume z-up coordinate system
class SamplingDistributions
{
public:
	static Vec3 uniformSampleHemisphere(float r1, float r2)
	{
		float theta = std::acosf(r1);
		float phi = 2 * M_PI * r2;
		return SphericalCoordinates::sphericalToWorld(theta, phi);
	}
	static float uniformHemispherePDF(const Vec3 wi)
	{
		return M_1_PI / 2;
	}
	static Vec3 cosineSampleHemisphere(float r1, float r2)
	{
		float theta = std::acosf(std::sqrtf(r1));
		float phi = 2 * M_PI * r2;
		return SphericalCoordinates::sphericalToWorld(theta, phi);
	}
	static float cosineHemispherePDF(const Vec3 wi)
	{
		// Add code here
		return std::cosf(SphericalCoordinates::sphericalTheta(wi)) * M_1_PI;
	}
	static Vec3 uniformSampleSphere(float r1, float r2)
	{
		// Add code here
		float theta = std::acos(1 - 2 * r1);
		float phi = 2 * M_PI * r2;
		return SphericalCoordinates::sphericalToWorld(theta, phi);
	}
	static float uniformSpherePDF(const Vec3& wi)
	{
		// Add code here
		return M_1_PI / 4;
	}

	static void uniformSampleTriangle(float r1, float r2, float& a, float& b)
	{
		a = 1 - std::sqrtf(r1);
		b = r2 * std::sqrtf(r1);
	}
};