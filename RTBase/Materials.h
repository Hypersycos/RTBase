#pragma once

#include "Core.h"
#include "Imaging.h"
#include "Sampling.h"

#pragma warning( disable : 4244)
#pragma warning( disable : 4305) // Double to float

class BSDF;

class ShadingData
{
public:
	Vec3 x;
	Vec3 wo;
	Vec3 sNormal;
	Vec3 gNormal;
	float tu;
	float tv;
	Frame frame;
	BSDF* bsdf;
	float t;
	ShadingData() {}
	ShadingData(Vec3 _x, Vec3 n)
	{
		x = _x;
		gNormal = n;
		sNormal = n;
		bsdf = NULL;
	}
};

class ShadingHelper
{
public:
	static float schlick(float cosIncidence, float iorInt, float iorExt)
	{
		float F0 = (iorInt - iorExt) / (iorInt + iorExt);
		return F0 * F0 + (1 - F0) * cosf(cosIncidence);
	}

	static float fresnelDielectric(float cosIncidence, float iorInt, float iorExt)
	{ //going from Ext to Int
		float n = iorExt / iorInt;
		float cosTIRCheck = 1 - (n * n) * (1 - cosIncidence * cosIncidence);
		if (cosTIRCheck < 0)
			return -1;

		float cosTransmission = sqrt(cosTIRCheck);
		float F_parallel = (cosIncidence - n * cosTransmission) / (cosIncidence + n * cosTransmission);
		float F_perp = (n * cosIncidence - cosTransmission) / (n * cosIncidence + cosTransmission);
		// Add code here
		return (F_parallel * F_parallel + F_perp * F_perp) / 2;
	}
	static Colour fresnelConductor(float cosTheta, Colour ior, Colour k)
	{ //going from Ext to Int
		float sin2Theta = 1 - cosTheta * cosTheta;
		float cos2Theta = cosTheta * cosTheta;
		Colour sin2ThetaC = { sin2Theta, sin2Theta, sin2Theta };
		Colour cos2ThetaC = { cos2Theta, cos2Theta, cos2Theta };
		Colour ior2 = ior * ior;
		Colour k2 = k * k;

		Colour F_parallel2 = ((ior2 + k2) * cos2Theta - ior * 2 * cosTheta + sin2ThetaC)
						   / ((ior2 + k2) * cos2Theta + ior * 2 * cosTheta + sin2ThetaC);
		Colour F_perp2 = (ior2 + k2 - ior * 2 * cosTheta + cos2ThetaC)
					   / (ior2 + k2 + ior * 2 * cosTheta + cos2ThetaC);
		// Add code here
		return (F_parallel2 + F_perp2) / 2;
	}
	static float lambdaGGX(Vec3 wi, float alpha)
	{
		float cos2Theta = wi.z * wi.z;
		if (cos2Theta == 0)
			return 0;
		float sin2Theta = 1 - cos2Theta;
		float tan2Theta = sin2Theta / cos2Theta;
		// Add code here
		return (std::sqrtf(1 + alpha * alpha * tan2Theta) - 1) / 2;
	}
	static float Gggx(Vec3 wi, Vec3 wo, float alpha)
	{
		return 1 / (1 + lambdaGGX(wi, alpha) + lambdaGGX(wo, alpha));
	}
	static float Dggx(Vec3 h, float alpha)
	{
		float alpha2 = alpha * alpha;
		float cosT = h.z;
		float denom = cosT * cosT * (alpha2 - 1) + 1;
		return alpha2 / (M_PI * denom * denom);
	}
};

class BSDF
{
public:
	Colour emission;
	virtual Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf) = 0;
	virtual Colour evaluate(const ShadingData& shadingData, const Vec3& wi) = 0;
	virtual float PDF(const ShadingData& shadingData, const Vec3& wi) = 0;
	virtual bool isPureSpecular() = 0;
	virtual bool isTwoSided() = 0;
	bool isLight()
	{
		return emission.Lum() > 0 ? true : false;
	}
	void addLight(Colour _emission)
	{
		emission = _emission;
	}
	Colour emit(const ShadingData& shadingData, const Vec3& wi)
	{
		return emission;
	}
	virtual float mask(const ShadingData& shadingData) = 0;
};


class DiffuseBSDF : public BSDF
{
public:
	Texture* albedo;
	DiffuseBSDF() = default;
	DiffuseBSDF(Texture* _albedo)
	{
		albedo = _albedo;
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		// Add correct sampling code here
		Vec3 wi = SamplingDistributions::cosineSampleHemisphere(sampler->next(), sampler->next());
		pdf = SamplingDistributions::cosineHemispherePDF(wi);
		reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) * M_1_PI;
		wi = shadingData.frame.toWorld(wi);
		return wi;
	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		return albedo->sample(shadingData.tu, shadingData.tv) * M_1_PI;
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		// Add correct PDF code here
		return SamplingDistributions::cosineHemispherePDF(wi);;
	}
	bool isPureSpecular()
	{
		return false;
	}
	bool isTwoSided()
	{
		return true;
	}
	float mask(const ShadingData& shadingData)
	{
		return albedo->sampleAlpha(shadingData.tu, shadingData.tv);
	}
};

class MirrorBSDF : public BSDF
{
public:
	Texture* albedo;
	MirrorBSDF() = default;
	MirrorBSDF(Texture* _albedo)
	{
		albedo = _albedo;
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		// Replace this with Mirror sampling code
		Vec3 wi = shadingData.frame.toLocal(shadingData.wo);
		wi.x = -wi.x;
		wi.y = -wi.y;
		pdf = 1;
		reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) / wi.z;
		wi = shadingData.frame.toWorld(wi);
		return wi;
	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with Mirror evaluation code
		return albedo->sample(shadingData.tu, shadingData.tv) / wi.dot(shadingData.sNormal);
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		return 0;
	}
	bool isPureSpecular()
	{
		return true;
	}
	bool isTwoSided()
	{
		return true;
	}
	float mask(const ShadingData& shadingData)
	{
		return albedo->sampleAlpha(shadingData.tu, shadingData.tv);
	}
};


class ConductorBSDF : public BSDF
{
public:
	Texture* albedo;
	Colour eta;
	Colour k;
	float alpha;
	ConductorBSDF() = default;
	ConductorBSDF(Texture* _albedo, Colour _eta, Colour _k, float roughness)
	{
		albedo = _albedo;
		eta = _eta;
		k = _k;
		alpha = 1.62142f * sqrtf(roughness);
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		if (alpha < EPSILON)
		{
			Vec3 wi = shadingData.frame.toLocal(shadingData.wo);
			wi.x = -wi.x;
			wi.y = -wi.y;
			pdf = 1;
			reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) / wi.z;
			wi = shadingData.frame.toWorld(wi);
			return wi;
		}

		float e1 = sampler->next();
		float e2 = sampler->next();
		float theta = std::acosf(std::sqrtf((1 - e1) / (e1 * (alpha * alpha - 1) + 1)));
		float phi = 2 * M_PI * e2;


		Vec3 woLocal = shadingData.frame.toLocal(shadingData.wo);

		Vec3 wm = SphericalCoordinates::sphericalToWorld(theta, phi);
		Vec3 wi = -woLocal + wm * 2 * wm.dot(woLocal);

		float D = ShadingHelper::Dggx(wm, alpha);

		pdf = D * wm.z / (4 * woLocal.dot(wm));
		if (wi.z <= 0)
		{
			reflectedColour = { 0,0,0 };
			return shadingData.sNormal;
		}

		float G = ShadingHelper::Gggx(wi, woLocal, alpha);
		Colour fresnel = ShadingHelper::fresnelConductor(woLocal.z, eta, k);
		reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) * fresnel * (G * D / (4 * wi.z * woLocal.z));

		return shadingData.frame.toWorld(wi);
	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		if (alpha < EPSILON)
		{
			return albedo->sample(shadingData.tu, shadingData.tv) / wi.dot(shadingData.sNormal);
		}

		Vec3 wiLocal = shadingData.frame.toLocal(wi);
		Vec3 woLocal = shadingData.frame.toLocal(shadingData.wo);

		Vec3 h = (wiLocal + woLocal).normalize();

		float D = ShadingHelper::Dggx(h, alpha);
		float G = ShadingHelper::Gggx(wiLocal, woLocal, alpha);
		Colour fresnel = ShadingHelper::fresnelConductor(woLocal.z, eta, k);
		return albedo->sample(shadingData.tu, shadingData.tv) * fresnel * (G * D / (4 * wiLocal.z * woLocal.z));
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		if (alpha < EPSILON)
			return 0;

		Vec3 wiLocal = shadingData.frame.toLocal(wi);
		Vec3 woLocal = shadingData.frame.toLocal(shadingData.wo);

		Vec3 wm = (wiLocal + woLocal).normalize();
		return ShadingHelper::Dggx(wm, alpha) * wm.z / (4 * woLocal.dot(wm));
	}
	bool isPureSpecular()
	{
		return false;
	}
	bool isTwoSided()
	{
		return true;
	}
	float mask(const ShadingData& shadingData)
	{
		return albedo->sampleAlpha(shadingData.tu, shadingData.tv);
	}
};

class GlassBSDF : public BSDF
{
public:
	Texture* albedo;
	float intIOR;
	float extIOR;
	GlassBSDF() = default;
	GlassBSDF(Texture* _albedo, float _intIOR, float _extIOR)
	{
		albedo = _albedo;
		intIOR = _intIOR;
		extIOR = _extIOR;
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		Vec3 rayDir = shadingData.frame.toLocal(-shadingData.wo);
		bool entering = rayDir.z < 0;
		float outerIndex = entering ? extIOR : intIOR;
		float innerIndex = entering ? intIOR : extIOR;

		float cosTheta = fabsf(rayDir.z);
		float fresnel = ShadingHelper::fresnelDielectric(cosTheta, innerIndex, outerIndex);

		if (fresnel == -1)
			fresnel = 1;

		if (sampler->next() <= fresnel)
		{ //reflect
			pdf = fresnel;
			Vec3 wr = -rayDir;
			wr.z = -wr.z;

			reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) / fabsf(wr.z) * fresnel;
			return shadingData.frame.toWorld(wr);
		}
		else
		{ //refract
			pdf = (1 - fresnel);

			rayDir = -rayDir;

			float theta_in = SphericalCoordinates::sphericalTheta(rayDir);
			float phi_in = SphericalCoordinates::sphericalPhi(rayDir);
			float n = outerIndex / innerIndex;
			float sin_theta_out = n * sinf(theta_in);

			float theta_out = asinf(sin_theta_out);
			float phi_out = phi_in + M_PI;
			
			Vec3 wt = { sinf(theta_out) * cosf(phi_out), sinf(theta_out) * sinf(phi_out), -cosf(theta_out) * (entering ? 1 : -1)};
			reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) / fabsf(wt.z) * n * n * (1 - fresnel);
			return shadingData.frame.toWorld(wt);
		}
	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		Vec3 rayDir = shadingData.frame.toLocal(-shadingData.wo);
		bool entering = rayDir.z < 0;

		bool reflect = wi.dot(shadingData.wo) > 0;
		float outerIndex = entering ? extIOR : intIOR;
		float innerIndex = entering ? intIOR : extIOR;

		float cosTheta = fabsf(rayDir.z);
		float fresnel = ShadingHelper::fresnelDielectric(cosTheta, innerIndex, outerIndex);
		
		if (reflect)
		{
			return albedo->sample(shadingData.tu, shadingData.tv) / fabsf(wi.dot(shadingData.sNormal)) * fresnel;
		}
		else
		{
			float n = outerIndex / innerIndex;
			return albedo->sample(shadingData.tu, shadingData.tv) / fabsf(wi.dot(shadingData.sNormal)) * n * n * (1 - fresnel);
		}
	}

	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		return 0;
	}
	bool isPureSpecular()
	{
		return true;
	}
	bool isTwoSided()
	{
		return false;
	}
	float mask(const ShadingData& shadingData)
	{
		return albedo->sampleAlpha(shadingData.tu, shadingData.tv);
	}
};

class DielectricBSDF : public BSDF
{
public:
	Texture* albedo;
	float intIOR;
	float extIOR;
	float alpha;
	DielectricBSDF() = default;
	DielectricBSDF(Texture* _albedo, float _intIOR, float _extIOR, float roughness)
	{
		albedo = _albedo;
		intIOR = _intIOR;
		extIOR = _extIOR;
		alpha = 1.62142f * sqrtf(roughness);
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		// Replace this with Dielectric sampling code
		Vec3 wi = SamplingDistributions::cosineSampleHemisphere(sampler->next(), sampler->next());
		pdf = wi.z / M_PI;
		reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
		wi = shadingData.frame.toWorld(wi);
		return wi;
	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with Dielectric evaluation code
		return albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with Dielectric PDF
		Vec3 wiLocal = shadingData.frame.toLocal(wi);
		return SamplingDistributions::cosineHemispherePDF(wiLocal);
	}
	bool isPureSpecular()
	{
		return false;
	}
	bool isTwoSided()
	{
		return false;
	}
	float mask(const ShadingData& shadingData)
	{
		return albedo->sampleAlpha(shadingData.tu, shadingData.tv);
	}
};

class OrenNayarBSDF : public BSDF
{
public:
	Texture* albedo;
	float sigma;
	OrenNayarBSDF() = default;
	OrenNayarBSDF(Texture* _albedo, float _sigma)
	{
		albedo = _albedo;
		sigma = _sigma;
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		// Replace this with OrenNayar sampling code
		Vec3 wi = SamplingDistributions::cosineSampleHemisphere(sampler->next(), sampler->next());
		Vec3 wo = shadingData.frame.toLocal(shadingData.wo);
		pdf = SamplingDistributions::cosineHemispherePDF(wi);

		float sigma2 = sigma * sigma;
		float A = 1 - sigma2 / (2 * (sigma2 + 0.33f));
		float B = 0.45 * sigma2 / (sigma2 + 0.09);

		float thetaI = SphericalCoordinates::sphericalTheta(wi);
		float thetaO = SphericalCoordinates::sphericalTheta(wo);
		float phiI = SphericalCoordinates::sphericalPhi(wi);
		float phiO = SphericalCoordinates::sphericalPhi(wo);

		float C = cosf(phiI - phiO) * sinf(std::max(thetaI, thetaO)) * tanf(std::min(thetaI, thetaO));

		reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) / M_PI * (A + B * std::max(0.0f, C));

		return shadingData.frame.toWorld(wi);
	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with OrenNayar evaluation code

		Vec3 wo = shadingData.frame.toLocal(shadingData.wo);
		Vec3 wiLocal = shadingData.frame.toLocal(wi);

		float sigma2 = sigma * sigma;
		float A = 1 - sigma2 / (2 * (sigma2 + 0.33f));
		float B = 0.45 * sigma2 / (sigma2 + 0.09);

		float thetaI = SphericalCoordinates::sphericalTheta(wiLocal);
		float thetaO = SphericalCoordinates::sphericalTheta(wo);
		float phiI = SphericalCoordinates::sphericalPhi(wiLocal);
		float phiO = SphericalCoordinates::sphericalPhi(wo);

		float C = cosf(phiI - phiO) * sinf(std::max(thetaI, thetaO)) * tanf(std::min(thetaI, thetaO));

		return albedo->sample(shadingData.tu, shadingData.tv) / M_PI * (A + B * std::max(0.0f, C));
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with OrenNayar PDF
		Vec3 wiLocal = shadingData.frame.toLocal(wi);
		return SamplingDistributions::cosineHemispherePDF(wiLocal);
	}
	bool isPureSpecular()
	{
		return false;
	}
	bool isTwoSided()
	{
		return true;
	}
	float mask(const ShadingData& shadingData)
	{
		return albedo->sampleAlpha(shadingData.tu, shadingData.tv);
	}
};

class PlasticBSDF : public BSDF
{
public:
	Texture* albedo;
	float intIOR;
	float extIOR;
	float alpha;
	PlasticBSDF() = default;
	PlasticBSDF(Texture* _albedo, float _intIOR, float _extIOR, float roughness)
	{
		albedo = _albedo;
		intIOR = _intIOR;
		extIOR = _extIOR;
		alpha = 1.62142f * sqrtf(roughness);
	}
	float alphaToPhongExponent()
	{
		return (2.0f / SQ(std::max(alpha, 0.001f))) - 2.0f;
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		Vec3 wr = shadingData.frame.toLocal(shadingData.wo);
		wr.x = -wr.x;
		wr.y = -wr.y;
		Frame wrFrame;
		wrFrame.fromVector(wr);

		float e = alphaToPhongExponent();
		float theta = std::acosf(std::powf(sampler->next(), 1 / (e + 1)));
		float phi = 2 * M_PI * sampler->next();
		Vec3 wi = SphericalCoordinates::sphericalToWorld(theta, phi);

		float maxCalc = M_1_PI * std::powf(std::max(0.0f, wi.z), e);
		pdf = (e + 1) * maxCalc;

		wi = wrFrame.toWorld(wi);
		if (wi.z <= 0)
		{
			reflectedColour = { 0,0,0 };
			return shadingData.sNormal;
		}

		reflectedColour = albedo->sample(shadingData.tu, shadingData.tv)* (e + 2) / 2 * maxCalc;

		return shadingData.frame.toWorld(wi);
	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		Vec3 wr = shadingData.frame.toLocal(shadingData.wo);
		wr.x = -wr.x;
		wr.y = -wr.y;

		Vec3 wiLocal = shadingData.frame.toWorld(wi);

		float e = alphaToPhongExponent();
		float maxCalc = M_1_PI * std::powf(std::max(0.0f, wr.dot(wiLocal)), e);

		return albedo->sample(shadingData.tu, shadingData.tv) * (e + 2) / 2 * maxCalc;
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		Vec3 wr = shadingData.frame.toLocal(shadingData.wo);
		wr.x = -wr.x;
		wr.y = -wr.y;

		Vec3 wiLocal = shadingData.frame.toWorld(wi);

		float e = alphaToPhongExponent();
		float maxCalc = M_1_PI * std::powf(std::max(0.0f, wr.dot(wiLocal)), e);
		return (e + 1) * maxCalc;
	}
	bool isPureSpecular()
	{
		return false;
	}
	bool isTwoSided()
	{
		return true;
	}
	float mask(const ShadingData& shadingData)
	{
		return albedo->sampleAlpha(shadingData.tu, shadingData.tv);
	}
};

class LayeredBSDF : public BSDF
{
public:
	BSDF* base;
	Colour sigmaa;
	float thickness;
	float intIOR;
	float extIOR;
	LayeredBSDF() = default;
	LayeredBSDF(BSDF* _base, Colour _sigmaa, float _thickness, float _intIOR, float _extIOR)
	{
		base = _base;
		sigmaa = _sigmaa;
		thickness = _thickness;
		intIOR = _intIOR;
		extIOR = _extIOR;
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		// Add code to include layered sampling
		return base->sample(shadingData, sampler, reflectedColour, pdf);
	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		// Add code for evaluation of layer
		return base->evaluate(shadingData, wi);
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		// Add code to include PDF for sampling layered BSDF
		return base->PDF(shadingData, wi);
	}
	bool isPureSpecular()
	{
		return base->isPureSpecular();
	}
	bool isTwoSided()
	{
		return true;
	}
	float mask(const ShadingData& shadingData)
	{
		return base->mask(shadingData);
	}
};