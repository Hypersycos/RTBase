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
		float cosInv = 1 - cosIncidence;
		float cosI2 = cosInv * cosInv;
		float cosI4 = cosI2 * cosI2;
		return F0 * F0 + (1 - F0) * cosI4 * cosInv;
	}

	static float schlickWithF0(float cosIncidence, float iorInt, float iorExt, float F0)
	{
		float cosInv = 1 - cosIncidence;
		float cosI2 = cosInv * cosInv;
		float cosI4 = cosI2 * cosI2;
		return F0 * F0 + (1 - F0) * cosI4 * cosInv;
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
		if (h.z < 0)
			return 0;
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
		return SamplingDistributions::cosineHemispherePDF(shadingData.frame.toLocal(wi));;
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
		reflectedColour = Colour{ 1,1,1 } / wi.z;
		wi = shadingData.frame.toWorld(wi);
		return wi;
	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with Mirror evaluation code
		return Colour{ 1,1,1 } / wi.dot(shadingData.sNormal);
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
		return alpha < EPSILON;
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
			Vec3 wr = rayDir;
			wr.z = -wr.z;

			if (wr.z == 0)
			{
				reflectedColour = Colour{ 0,0,0 };
				return shadingData.sNormal;
			}
			reflectedColour = Colour{ 1,1,1 } / fabsf(wr.z) * fresnel;
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
			if (wt.z == 0)
			{
				reflectedColour = Colour{ 0,0,0 };
				return shadingData.sNormal;
			}
			reflectedColour = Colour{ 1,1,1 } / fabsf(wt.z) * n * n * (1 - fresnel);
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
			return Colour{ 1,1,1 } / fabsf(wi.dot(shadingData.sNormal)) * fresnel;
		}
		else
		{
			float n = outerIndex / innerIndex;
			return Colour{ 1,1,1 } / fabsf(wi.dot(shadingData.sNormal)) * n * n * (1 - fresnel);
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

#ifdef DielectricImpl
class DielectricBSDF : public BSDF
#else
class DielectricBSDF : public DiffuseBSDF
#endif
{
public:
	Texture* albedo;
	Colour sigma;
	float intIOR;
	float extIOR;
	float alpha;
	GlassBSDF* glass;
#ifdef DielecNoTransmit
	DiffuseBSDF* diffuse;
#endif
	DielectricBSDF() = default;
	DielectricBSDF(Colour sigma, float _intIOR, float _extIOR, float roughness)
	{
		intIOR = _intIOR;
		extIOR = _extIOR;
		alpha = 1.62142f * sqrtf(roughness);
		if (alpha < EPSILON)
			glass = new GlassBSDF(nullptr, _intIOR, _extIOR);
		this->sigma = sigma;
	}
	DielectricBSDF(Texture* _albedo, float _intIOR, float _extIOR, float roughness)
	{
		albedo = _albedo;
		intIOR = _intIOR;
		extIOR = _extIOR;
		alpha = 1.62142f * sqrtf(roughness);
#ifdef DielecNoTransmit
		diffuse = new DiffuseBSDF(_albedo);
#endif
		if (alpha < EPSILON)
			glass = new GlassBSDF(_albedo, _intIOR, _extIOR);
	}
#ifdef DielectricImpl
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		if (alpha < EPSILON)
		{
			return glass->sample(shadingData, sampler, reflectedColour, pdf);
		}

		Vec3 wo = shadingData.frame.toLocal(shadingData.wo);

		Vec3 wh = Vec3(alpha * wo.x, alpha * wo.y, wo.z).normalize();
		if (wh.z < 0)
			wh = -wh;

		Vec3 T1 = (wh.z < 0.99999f) ? wh.cross(Vec3(0, 0, 1)).normalize() : Vec3(1, 0, 0);
		Vec3 T2 = wh.cross(T1);
		float r = sqrtf(sampler->next());
		float theta = 2 * M_PI * sampler->next();
		float px = r * std::cosf(theta);
		float py = r * std::sinf(theta);

		float h = std::sqrt(1 - px * px);
		py = std::lerp((1 + wh.z) / 2, h, py);

		float pz = sqrtf(std::max(0.0f, 1 - px * py));
		Vec3 nh = T1 * px + T2 * py + wh * pz;

		Vec3 wm = Vec3(alpha * nh.x, alpha * nh.y, std::max(1e-6f, nh.z)).normalize();

		float cosTheta = wm.dot(wo);

		bool entering = cosTheta >= 0;

		cosTheta = fabsf(cosTheta);

		float outerIndex = entering ? extIOR : intIOR;
		float innerIndex = entering ? intIOR : extIOR;

		float n = innerIndex / outerIndex;

		float fresnel = ShadingHelper::fresnelDielectric(cosTheta, innerIndex, outerIndex);

		Colour passthrough = { 1,1,1 };
		if (entering)
			passthrough = { 1,1,1 };
		else if (sigma.Lum() != 0)
		{
			passthrough.r = std::powf(M_E, -sigma.r * shadingData.t);
			passthrough.g = std::powf(M_E, -sigma.g * shadingData.t);
			passthrough.b = std::powf(M_E, -sigma.b * shadingData.t);
		}
#if !defined(DielecNoTransmit) && !defined(DielecNoAbsorb)
		else
		{
			float thickness = 0.1;
			Colour sigma = albedo->sample(shadingData.tu, shadingData.tv);
			sigma.r = -std::log(std::max(sigma.r, 0.001f)) / thickness;
			sigma.g = -std::log(std::max(sigma.g, 0.001f)) / thickness;
			sigma.b = -std::log(std::max(sigma.b, 0.001f)) / thickness;

			passthrough.r = std::powf(M_E, -sigma.r * shadingData.t);
			passthrough.g = std::powf(M_E, -sigma.g * shadingData.t);
			passthrough.b = std::powf(M_E, -sigma.b * shadingData.t);
		}
#endif

		if (fresnel == -1)
			fresnel = 1;

		if (sampler->next() <= fresnel)
		{ //reflect
			Vec3 wi = -wo + wm * wo.dot(wm) * 2;
			if (wi.z * wo.z < 0)
			{
				reflectedColour = { 0,0,0 };
				return shadingData.sNormal;
			}

			//float D = ShadingHelper::Dggx(wm, alpha);
			float D;
			float DTan2CosTheta = wm.z * wm.z;
			if (DTan2CosTheta < 1)
			{
				float DTan2SinTheta = 1 - DTan2CosTheta;
				float DTan2Theta = DTan2SinTheta / DTan2CosTheta;
				float cos4Theta = DTan2CosTheta * DTan2CosTheta;
				float sinTheta = std::sqrt(DTan2SinTheta);
				float cosPhi = (sinTheta == 0) ? 1 : std::clamp<float>(wm.x / sinTheta, -1, 1);
				float sinPhi = (sinTheta == 0) ? 0 : std::clamp<float>(wm.y / sinTheta, -1, 1);
				float e = DTan2Theta * ((cosPhi * cosPhi / alpha) + (sinPhi * sinPhi / alpha));
				D = 1 / (M_PI * alpha * alpha * cos4Theta * (1 + e) * (1 + e));
			}
			else
				D = 0;

			float G1 = 1 / (1 + ShadingHelper::lambdaGGX(wo, alpha));
			float D2 = G1 / fabsf(wo.z) * D * cosTheta;

			pdf = D2 / (4 * cosTheta) * fresnel;

			float G = ShadingHelper::Gggx(wi, wo, alpha);
			reflectedColour = passthrough * fresnel * D * G / (4 * wi.z * wo.z);

			return shadingData.frame.toWorld(wi);
		}
		else
		{ //refract
#ifdef DielecNoTransmit
			Vec3 wi = diffuse->sample(shadingData, sampler, reflectedColour, pdf);
			pdf *= (1 - fresnel);
			reflectedColour = reflectedColour * (1 - fresnel);
			return wi;
#else
			if (!entering)
				wm = -wm;

			float sin2ThetaI = std::max(0.0f, 1 - cosTheta * cosTheta);
			float sin2ThetaT = sin2ThetaI / (n * n);

			if (sin2ThetaT >= 1)
			{ // total internal reflection somehow
				reflectedColour = { 0,0,0 };
				return shadingData.sNormal;
			}

			float cosThetaT = sqrtf(1 - sin2ThetaT);

			Vec3 wi = -wo / n + wm * (cosTheta / n - cosThetaT);

			if (wi.z * wo.z > 0 || wi.z == 0)
			{ //refract but still leaving material
				reflectedColour = { 0,0,0 };
				return shadingData.sNormal;
			}

			if (!entering)
				wm = -wm;

			float denom = wi.dot(wm) + wo.dot(wm) / n;
			float denom2 = denom * denom;
			float dwm_dwi = fabsf(wi.dot(wm)) / denom2;

			//float D = ShadingHelper::Dggx(wm, alpha);
			float D;
			float DTan2CosTheta = wm.z * wm.z;
			if (DTan2CosTheta < 1)
			{
				float DTan2SinTheta = 1 - DTan2CosTheta;
				float DTan2Theta = DTan2SinTheta / DTan2CosTheta;
				float cos4Theta = DTan2CosTheta * DTan2CosTheta;
				float sinTheta = std::sqrt(DTan2SinTheta);
				float cosPhi = (sinTheta == 0) ? 1 : std::clamp<float>(wm.x / sinTheta, -1, 1);
				float sinPhi = (sinTheta == 0) ? 0 : std::clamp<float>(wm.y / sinTheta, -1, 1);
				float e = DTan2Theta * ((cosPhi * cosPhi / alpha) + (sinPhi * sinPhi / alpha));
				D = 1 / (M_PI * alpha * alpha * cos4Theta * (1 + e) * (1 + e));
			}
			else
				D = 0;


			float G1 = 1 / (1 + ShadingHelper::lambdaGGX(wo, alpha));
			float D2 = G1 / fabsf(wo.z) * D * cosTheta;

			pdf = D2 * dwm_dwi * (1 - fresnel);

			float G = ShadingHelper::Gggx(wi, wo, alpha);
#ifdef DielecNoAbsorb
			if (sigma.Lum() == 0)
				passthrough = albedo->sample(shadingData.tu, shadingData.tv);
#endif
			reflectedColour = passthrough * (1 - fresnel) * D * G * fabsf(wi.dot(wm) * wo.dot(wm) / (wi.z * wo.z * denom2));

			return shadingData.frame.toWorld(wi);
#endif
		}
	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		if (alpha < EPSILON)
		{
			return glass->evaluate(shadingData, wi);
		}

		Vec3 woLocal = shadingData.frame.toLocal(shadingData.wo);
		Vec3 wiLocal = shadingData.frame.toLocal(wi);

		float cosThetaO = woLocal.z;
		float cosThetaI = wiLocal.z;
		bool reflect = cosThetaI * cosThetaO > 0;
		float outerIndex = cosThetaO > 0 ? extIOR : intIOR;
		float innerIndex = cosThetaO > 0 ? intIOR : extIOR;

		float n = outerIndex / innerIndex;

		Vec3 wm = wiLocal * n + woLocal;

		if (cosThetaI == 0 || cosThetaO == 0 || wm.lengthSq() == 0)
			return Colour{ 0,0,0 };

		wm = wm.normalize();
		if (wm.z < 0)
			wm = -wm;

		if (wm.dot(wiLocal) * cosThetaI < 0 || wm.dot(woLocal) * cosThetaO < 0)
			return Colour{ 0,0,0 };

		float fresnel = ShadingHelper::fresnelDielectric(fabsf(woLocal.dot(wm)), innerIndex, outerIndex);
		if (fresnel == -1)
			fresnel = 1;
		if (reflect)
		{ //reflect
			//float D = ShadingHelper::Dggx(wm, alpha);
			float D;
			float DTan2CosTheta = wm.z * wm.z;
			if (DTan2CosTheta < 1)
			{
				float DTan2SinTheta = 1 - DTan2CosTheta;
				float DTan2Theta = DTan2SinTheta / DTan2CosTheta;
				float cos4Theta = DTan2CosTheta * DTan2CosTheta;
				float sinTheta = std::sqrt(DTan2SinTheta);
				float cosPhi = (sinTheta == 0) ? 1 : std::clamp<float>(wm.x / sinTheta, -1, 1);
				float sinPhi = (sinTheta == 0) ? 0 : std::clamp<float>(wm.y / sinTheta, -1, 1);
				float e = DTan2Theta * ((cosPhi * cosPhi / alpha) + (sinPhi * sinPhi / alpha));
				D = 1 / (M_PI * alpha * alpha * cos4Theta * (1 + e) * (1 + e));
			}
			else
				D = 0;
			float G = ShadingHelper::Gggx(wiLocal, woLocal, alpha);
			Colour highlight = Colour{ 1,1,1 } * fresnel * D * G / fabsf(4 * wiLocal.z * woLocal.z);
#ifdef DielecNoTransmit
			Colour diffuseC = diffuse->evaluate(shadingData, wi);
			return highlight + diffuseC * M_1_PI * (1 - fresnel);
#else
			return highlight;
#endif;
		}
		else
		{ //refract
			float denom = wiLocal.dot(wm) + woLocal.dot(wm) / n;
			float denom2 = denom * denom;

			//float D = ShadingHelper::Dggx(wm, alpha);
			float D;
			float DTan2CosTheta = wm.z * wm.z;
			if (DTan2CosTheta < 1)
			{
				float DTan2SinTheta = 1 - DTan2CosTheta;
				float DTan2Theta = DTan2SinTheta / DTan2CosTheta;
				float cos4Theta = DTan2CosTheta * DTan2CosTheta;
				float sinTheta = std::sqrt(DTan2SinTheta);
				float cosPhi = (sinTheta == 0) ? 1 : std::clamp<float>(wm.x / sinTheta, -1, 1);
				float sinPhi = (sinTheta == 0) ? 0 : std::clamp<float>(wm.y / sinTheta, -1, 1);
				float e = DTan2Theta * ((cosPhi * cosPhi / alpha) + (sinPhi * sinPhi / alpha));
				D = 1 / (M_PI * alpha * alpha * cos4Theta * (1 + e) * (1 + e));
			}
			else
				D = 0;
			float G = ShadingHelper::Gggx(wiLocal, woLocal, alpha);
			return Colour{ 1,1,1 } * (1 - fresnel) * D * G * fabsf(wiLocal.dot(wm) * woLocal.dot(wm) / (wiLocal.z * woLocal.z * denom2));
		}

	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		if (alpha < EPSILON)
			return 0;

		Vec3 woLocal = shadingData.frame.toLocal(shadingData.wo);
		Vec3 wiLocal = shadingData.frame.toLocal(wi);

		float cosThetaO = woLocal.z;
		float cosThetaI = wiLocal.z;
		bool reflect = cosThetaI * cosThetaO > 0;
		float outerIndex = cosThetaO > 0 ? extIOR : intIOR;
		float innerIndex = cosThetaO > 0 ? intIOR : extIOR;

		float n = outerIndex / innerIndex;

		Vec3 wm = wiLocal * n + woLocal;

		if (cosThetaI == 0 || cosThetaO == 0 || wm.lengthSq() == 0)
			return 0;

		wm = wm.normalize();
		if (wm.z < 0)
			wm = -wm;

		if (wm.dot(wiLocal) * cosThetaI < 0 || wm.dot(woLocal) * cosThetaO < 0)
			return 0;

		float fresnel = ShadingHelper::fresnelDielectric(fabsf(woLocal.dot(wm)), innerIndex, outerIndex);
		if (fresnel == -1)
			fresnel = 1;
		if (reflect)
		{
			//float D = ShadingHelper::Dggx(wm, alpha);
			float D;
			float DTan2CosTheta = wm.z * wm.z;
			if (DTan2CosTheta < 1)
			{
				float DTan2SinTheta = 1 - DTan2CosTheta;
				float DTan2Theta = DTan2SinTheta / DTan2CosTheta;
				float cos4Theta = DTan2CosTheta * DTan2CosTheta;
				float sinTheta = std::sqrt(DTan2SinTheta);
				float cosPhi = (sinTheta == 0) ? 1 : std::clamp<float>(wm.x / sinTheta, -1, 1);
				float sinPhi = (sinTheta == 0) ? 0 : std::clamp<float>(wm.y / sinTheta, -1, 1);
				float e = DTan2Theta * ((cosPhi * cosPhi / alpha) + (sinPhi * sinPhi / alpha));
				D = 1 / (M_PI * alpha * alpha * cos4Theta * (1 + e) * (1 + e));
			}
			else
				D = 0;
			float G1 = 1 / (1 + ShadingHelper::lambdaGGX(woLocal, alpha));
			float D2 = G1 / fabsf(woLocal.z) * D * wm.dot(woLocal);

#ifdef DielecNoTransmit
			return D2 / (4 * fabsf(woLocal.dot(wm))) * fresnel + diffuse->PDF(shadingData, wi) * (1 - fresnel);
#else
			return D2 / (4 * fabsf(woLocal.dot(wm))) * fresnel;
#endif
		}
		else
		{
			float denom = wi.dot(wm) + woLocal.dot(wm) / n;
			float denom2 = denom * denom;
			float dwm_dwi = fabsf(wi.dot(wm) / denom2);

			//float D = ShadingHelper::Dggx(wm, alpha);
			float D;
			float DTan2CosTheta = wm.z * wm.z;
			if (DTan2CosTheta < 1)
			{
				float DTan2SinTheta = 1 - DTan2CosTheta;
				float DTan2Theta = DTan2SinTheta / DTan2CosTheta;
				float cos4Theta = DTan2CosTheta * DTan2CosTheta;
				float sinTheta = std::sqrt(DTan2SinTheta);
				float cosPhi = (sinTheta == 0) ? 1 : std::clamp<float>(wm.x / sinTheta, -1, 1);
				float sinPhi = (sinTheta == 0) ? 0 : std::clamp<float>(wm.y / sinTheta, -1, 1);
				float e = DTan2Theta * ((cosPhi * cosPhi / alpha) + (sinPhi * sinPhi / alpha));
				D = 1 / (M_PI * alpha * alpha * cos4Theta * (1 + e) * (1 + e));
			}
			else
				D = 0;
			float G1 = 1 / (1 + ShadingHelper::lambdaGGX(woLocal, alpha));
			float D2 = G1 / fabsf(woLocal.z) * D * fabsf(woLocal.dot(wm));

			return D2 * dwm_dwi * (1 - fresnel);
		}
	}
#endif
	bool isPureSpecular()
	{
		return alpha < EPSILON;
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

class PlasticBSDF : public DiffuseBSDF
{
public:
	Texture* albedo;
	float intIOR;
	float extIOR;
	float alpha;
	PlasticBSDF() = default;
#ifdef PlasticPhong
	PlasticBSDF(Texture* _albedo, float _intIOR, float _extIOR, float roughness) : DiffuseBSDF(_albedo)
#else
	MirrorBSDF* mirror;
	PlasticBSDF(Texture* _albedo, float _intIOR, float _extIOR, float roughness) : DiffuseBSDF(_albedo)
#endif
	{
		albedo = _albedo;
		intIOR = _intIOR;
		extIOR = _extIOR;
		alpha = 1.62142f * sqrtf(roughness);
#ifndef PlasticPhong
		if (alpha < EPSILON)
			mirror = new MirrorBSDF(_albedo);
#endif
	}
	float alphaToPhongExponent()
	{
		return (2.0f / SQ(std::max(alpha, 0.001f))) - 2.0f;
	}
#ifdef PlasticPhong
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
/*		if (alpha < EPSILON)
			MirrorBSDF::sample(shadingData, sampler, reflectedColour, pdf);*/

		Vec3 woLocal = shadingData.frame.toLocal(shadingData.wo);

		float cosTheta = woLocal.z;
		float specularWeight = ShadingHelper::schlickWithF0(cosTheta, intIOR, extIOR, 0.04);
		float diffuseWeight = 1 - specularWeight;

		Vec3 wi;
		float e = alphaToPhongExponent();
		float maxCalc;

		Vec3 wr = shadingData.frame.toLocal(shadingData.wo);
		wr.x = -wr.x;
		wr.y = -wr.y;

		if (sampler->next() < specularWeight)
		{ //specular
			Frame wrFrame;
			//wrFrame.fromVector(wr);

			float theta = std::acosf(std::powf(sampler->next(), 1 / (e + 1)));
			float phi = 2 * M_PI * sampler->next();
			Vec3 wi = SphericalCoordinates::sphericalToWorld(theta, phi);

			maxCalc = M_1_PI / 2 * std::powf(std::max(0.0f, wi.z), e);

			wi = wrFrame.toWorld(wi);
			if (wi.z <= 0)
			{
				pdf = 1;
				reflectedColour = { 0,0,0 };
				return shadingData.sNormal;
			}

			pdf = (e + 1) * maxCalc;
			reflectedColour = Colour{ 1,1,1 } * (e + 2) * maxCalc;
			return shadingData.frame.toWorld(wi);
		}
		else
		{ //diffuse
			return DiffuseBSDF::sample(shadingData, sampler, reflectedColour, pdf);
		}
	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
/*		if (alpha < EPSILON)
			MirrorBSDF::evaluate(shadingData, wi);*/

		Vec3 woLocal = shadingData.frame.toLocal(shadingData.wo);

		Vec3 wr = woLocal;
		wr.x = -wr.x;
		wr.y = -wr.y;

		Vec3 wiLocal = shadingData.frame.toLocal(wi);

		float e = alphaToPhongExponent();
		float maxCalc = M_1_PI / 2 * std::powf(std::max(0.0f, wr.dot(wiLocal)), e);

		float cosTheta = woLocal.z;
		float specularWeight = ShadingHelper::schlickWithF0(cosTheta, intIOR, extIOR, 0.04);
		specularWeight = 0.5f;
		float diffuseWeight = 1 - specularWeight;

		Colour phong = Colour{ 1,1,1 } * (e + 2) * maxCalc;

		Colour lambert = albedo->sample(shadingData.tu, shadingData.tv) * M_1_PI;

		return phong * 0.04 + lambert * 0.96;
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		Vec3 woLocal = shadingData.frame.toLocal(shadingData.wo);

		Vec3 wr = shadingData.frame.toLocal(shadingData.wo);
		wr.x = -wr.x;
		wr.y = -wr.y;

		Vec3 wiLocal = shadingData.frame.toLocal(wi);

		float cosTheta = woLocal.z;
		float specularWeight = ShadingHelper::schlickWithF0(cosTheta, intIOR, extIOR, 0.04);
		float diffuseWeight = 1 - specularWeight;

		float e = alphaToPhongExponent();
		float maxCalc = M_1_PI / 2 * std::powf(std::max(0.0f, wr.dot(wiLocal)), e);

		float phongPdf = (e + 1) * maxCalc * specularWeight;
		float lambertPdf = SamplingDistributions::cosineHemispherePDF(wi) * diffuseWeight;

		return phongPdf + lambertPdf;
	}
	bool isPureSpecular()
	{
		return false;
	}
	bool isTwoSided()
	{
		return true;
	}
#else
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		Vec3 wo = shadingData.frame.toLocal(shadingData.wo);

		if (alpha < EPSILON)
		{
			float fresnel = ShadingHelper::fresnelDielectric(wo.z, intIOR, extIOR);
			if (fresnel == -1)
				fresnel = 1;
			if (sampler->next() <= fresnel)
			{
				Vec3 wi = mirror->sample(shadingData, sampler, reflectedColour, pdf);
				reflectedColour = reflectedColour * fresnel;
				pdf *= fresnel;
				return wi;
			}
			else
			{
				Vec3 wi = DiffuseBSDF::sample(shadingData, sampler, reflectedColour, pdf);
				pdf *= (1 - fresnel);
				reflectedColour = reflectedColour * (1 - fresnel);
				return wi;
			}
		}

		Vec3 wh = Vec3(alpha * wo.x, alpha * wo.y, wo.z).normalize();
		if (wh.z < 0)
			wh = -wh;

		Vec3 T1 = (wh.z < 0.99999f) ? wh.cross(Vec3(0, 0, 1)).normalize() : Vec3(1, 0, 0);
		Vec3 T2 = wh.cross(T1);
		float r = sqrtf(sampler->next());
		float theta = 2 * M_PI * sampler->next();
		float px = r * std::cosf(theta);
		float py = r * std::sinf(theta);

		float h = std::sqrt(1 - px * px);
		py = std::lerp((1 + wh.z) / 2, h, py);

		float pz = sqrtf(std::max(0.0f, 1 - px * py));
		Vec3 nh = T1 * px + T2 * py + wh * pz;

		Vec3 wm = Vec3(alpha * nh.x, alpha * nh.y, std::max(1e-6f, nh.z)).normalize();

		float cosTheta = wm.dot(wo);

		bool entering = cosTheta >= 0;

		cosTheta = fabsf(cosTheta);

		float outerIndex = entering ? extIOR : intIOR;
		float innerIndex = entering ? intIOR : extIOR;

		float n = innerIndex / outerIndex;

		float fresnel = ShadingHelper::fresnelDielectric(cosTheta, innerIndex, outerIndex);

		if (fresnel == -1)
			fresnel = 1;

		if (sampler->next() <= fresnel)
		{ //reflect
			Vec3 wi = -wo + wm * wo.dot(wm) * 2;
			if (wi.z * wo.z < 0)
			{
				reflectedColour = { 0,0,0 };
				return shadingData.sNormal;
			}

			//float D = ShadingHelper::Dggx(wm, alpha);
			float D;
			float DTan2CosTheta = wm.z * wm.z;
			if (DTan2CosTheta < 1)
			{
				float DTan2SinTheta = 1 - DTan2CosTheta;
				float DTan2Theta = DTan2SinTheta / DTan2CosTheta;
				float cos4Theta = DTan2CosTheta * DTan2CosTheta;
				float sinTheta = std::sqrt(DTan2SinTheta);
				float cosPhi = (sinTheta == 0) ? 1 : std::clamp<float>(wm.x / sinTheta, -1, 1);
				float sinPhi = (sinTheta == 0) ? 0 : std::clamp<float>(wm.y / sinTheta, -1, 1);
				float e = DTan2Theta * ((cosPhi * cosPhi / alpha) + (sinPhi * sinPhi / alpha));
				D = 1 / (M_PI * alpha * alpha * cos4Theta * (1 + e) * (1 + e));
			}
			else
				D = 0;

			float G1 = 1 / (1 + ShadingHelper::lambdaGGX(wo, alpha));
			float D2 = G1 / fabsf(wo.z) * D * cosTheta;

			pdf = D2 / (4 * cosTheta) * fresnel;

			float G = ShadingHelper::Gggx(wi, wo, alpha);
			reflectedColour = Colour{ 1,1,1 } * fresnel * D * G / (4 * wi.z * wo.z);

			return shadingData.frame.toWorld(wi);
		}
		else
		{ //"refract"
			Vec3 wi = DiffuseBSDF::sample(shadingData, sampler, reflectedColour, pdf);
			pdf *= (1 - fresnel);
			reflectedColour = reflectedColour * (1 - fresnel);
			return wi;
		}
	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		Vec3 woLocal = shadingData.frame.toLocal(shadingData.wo);
		Vec3 wiLocal = shadingData.frame.toLocal(wi);

		float cosThetaO = woLocal.z;
		float cosThetaI = wiLocal.z;
		bool reflect = cosThetaI * cosThetaO > 0;
		float outerIndex = cosThetaO > 0 ? extIOR : intIOR;
		float innerIndex = cosThetaO > 0 ? intIOR : extIOR;

		float n = innerIndex / outerIndex;

		Vec3 wm = wiLocal * n + woLocal;

		if (cosThetaI == 0 || cosThetaO == 0 || wm.lengthSq() == 0)
			return Colour{ 0,0,0 };

		wm = wm.normalize();
		if (wm.z < 0)
			wm = -wm;

		if (wm.dot(wiLocal) * cosThetaI < 0 || wm.dot(woLocal) * cosThetaO < 0)
			return Colour{ 0,0,0 };

		float fresnel = ShadingHelper::fresnelDielectric(fabsf(woLocal.dot(wm)), innerIndex, outerIndex);
		if (fresnel == -1)
			fresnel = 1;

		if (alpha < EPSILON)
		{
			return mirror->evaluate(shadingData, wi) * fresnel + DiffuseBSDF::evaluate(shadingData, wi) * (1 - fresnel);
		}

		if (reflect)
		{ //reflect
			//float D = ShadingHelper::Dggx(wm, alpha);
			float D;
			float DTan2CosTheta = wm.z * wm.z;
			if (DTan2CosTheta < 1)
			{
				float DTan2SinTheta = 1 - DTan2CosTheta;
				float DTan2Theta = DTan2SinTheta / DTan2CosTheta;
				float cos4Theta = DTan2CosTheta * DTan2CosTheta;
				float sinTheta = std::sqrt(DTan2SinTheta);
				float cosPhi = (sinTheta == 0) ? 1 : std::clamp<float>(wm.x / sinTheta, -1, 1);
				float sinPhi = (sinTheta == 0) ? 0 : std::clamp<float>(wm.y / sinTheta, -1, 1);
				float e = DTan2Theta * ((cosPhi * cosPhi / alpha) + (sinPhi * sinPhi / alpha));
				D = 1 / (M_PI * alpha * alpha * cos4Theta * (1 + e) * (1 + e));
			}
			else
				D = 0;
			float G = ShadingHelper::Gggx(wiLocal, woLocal, alpha);
			Colour highlight = Colour{ 1,1,1 } * fresnel * D * G / fabsf(4 * wiLocal.z * woLocal.z);
			Colour diffuseC = DiffuseBSDF::evaluate(shadingData, wi);
			return highlight + diffuseC * (1 - fresnel);
		}
		else
		{ //refract
			float denom = wiLocal.dot(wm) + woLocal.dot(wm) / n;
			float denom2 = denom * denom;

			//float D = ShadingHelper::Dggx(wm, alpha);
			float D;
			float DTan2CosTheta = wm.z * wm.z;
			if (DTan2CosTheta < 1)
			{
				float DTan2SinTheta = 1 - DTan2CosTheta;
				float DTan2Theta = DTan2SinTheta / DTan2CosTheta;
				float cos4Theta = DTan2CosTheta * DTan2CosTheta;
				float sinTheta = std::sqrt(DTan2SinTheta);
				float cosPhi = (sinTheta == 0) ? 1 : std::clamp<float>(wm.x / sinTheta, -1, 1);
				float sinPhi = (sinTheta == 0) ? 0 : std::clamp<float>(wm.y / sinTheta, -1, 1);
				float e = DTan2Theta * ((cosPhi * cosPhi / alpha) + (sinPhi * sinPhi / alpha));
				D = 1 / (M_PI * alpha * alpha * cos4Theta * (1 + e) * (1 + e));
			}
			else
				D = 0;
			float G = ShadingHelper::Gggx(wiLocal, woLocal, alpha);
			return Colour{ 1,1,1 } * (1 - fresnel) * D * G * fabsf(wiLocal.dot(wm) * woLocal.dot(wm) / (wiLocal.z * woLocal.z * denom2));
		}

	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		if (alpha < EPSILON)
			return DiffuseBSDF::PDF(shadingData, wi);

		Vec3 woLocal = shadingData.frame.toLocal(shadingData.wo);
		Vec3 wiLocal = shadingData.frame.toLocal(wi);

		float cosThetaO = woLocal.z;
		float cosThetaI = wiLocal.z;
		bool reflect = cosThetaI * cosThetaO > 0;
		float outerIndex = cosThetaO > 0 ? extIOR : intIOR;
		float innerIndex = cosThetaO > 0 ? intIOR : extIOR;

		float n = innerIndex / outerIndex;

		Vec3 wm = wiLocal * n + woLocal;

		if (cosThetaI == 0 || cosThetaO == 0 || wm.lengthSq() == 0)
			return 0;

		wm = wm.normalize();
		if (wm.z < 0)
			wm = -wm;

		if (wm.dot(wiLocal) * cosThetaI < 0 || wm.dot(woLocal) * cosThetaO < 0)
			return 0;

		float fresnel = ShadingHelper::fresnelDielectric(fabsf(woLocal.dot(wm)), innerIndex, outerIndex);
		if (fresnel == -1)
			fresnel = 1;
		if (reflect)
		{
			//float D = ShadingHelper::Dggx(wm, alpha);
			float D;
			float DTan2CosTheta = wm.z * wm.z;
			if (DTan2CosTheta < 1)
			{
				float DTan2SinTheta = 1 - DTan2CosTheta;
				float DTan2Theta = DTan2SinTheta / DTan2CosTheta;
				float cos4Theta = DTan2CosTheta * DTan2CosTheta;
				float sinTheta = std::sqrt(DTan2SinTheta);
				float cosPhi = (sinTheta == 0) ? 1 : std::clamp<float>(wm.x / sinTheta, -1, 1);
				float sinPhi = (sinTheta == 0) ? 0 : std::clamp<float>(wm.y / sinTheta, -1, 1);
				float e = DTan2Theta * ((cosPhi * cosPhi / alpha) + (sinPhi * sinPhi / alpha));
				D = 1 / (M_PI * alpha * alpha * cos4Theta * (1 + e) * (1 + e));
			}
			else
				D = 0;
			float G1 = 1 / (1 + ShadingHelper::lambdaGGX(woLocal, alpha));
			float D2 = G1 / fabsf(woLocal.z) * D * wm.dot(woLocal);
			return D2 / (4 * fabsf(woLocal.dot(wm))) * fresnel + DiffuseBSDF::PDF(shadingData, wi) * (1 - fresnel);
		}
		else
		{
			float denom = wi.dot(wm) + woLocal.dot(wm) / n;
			float denom2 = denom * denom;
			float dwm_dwi = fabsf(wi.dot(wm) / denom2);

			//float D = ShadingHelper::Dggx(wm, alpha);
			float D;
			float DTan2CosTheta = wm.z * wm.z;
			if (DTan2CosTheta < 1)
			{
				float DTan2SinTheta = 1 - DTan2CosTheta;
				float DTan2Theta = DTan2SinTheta / DTan2CosTheta;
				float cos4Theta = DTan2CosTheta * DTan2CosTheta;
				float sinTheta = std::sqrt(DTan2SinTheta);
				float cosPhi = (sinTheta == 0) ? 1 : std::clamp<float>(wm.x / sinTheta, -1, 1);
				float sinPhi = (sinTheta == 0) ? 0 : std::clamp<float>(wm.y / sinTheta, -1, 1);
				float e = DTan2Theta * ((cosPhi * cosPhi / alpha) + (sinPhi * sinPhi / alpha));
				D = 1 / (M_PI * alpha * alpha * cos4Theta * (1 + e) * (1 + e));
			}
			else
				D = 0;
			float G1 = 1 / (1 + ShadingHelper::lambdaGGX(woLocal, alpha));
			float D2 = G1 / fabsf(woLocal.z) * D * fabsf(woLocal.dot(wm));

			return D2 * dwm_dwi * (1 - fresnel);
		}
	}
#endif
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
	GlassBSDF glass;
	LayeredBSDF() = default;
	LayeredBSDF(BSDF* _base, Colour _sigmaa, float _thickness, float _intIOR, float _extIOR) : glass(nullptr, _intIOR, _extIOR)
	{
		base = _base;
		sigmaa = _sigmaa;
		thickness = _thickness;
		intIOR = _intIOR;
		extIOR = _extIOR;
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
#ifndef LayeredImpl
		return base->sample(shadingData, sampler, reflectedColour, pdf);
#endif

		Vec3 wi = glass.sample(shadingData, sampler, reflectedColour, pdf);
		Vec3 localWi = shadingData.frame.toLocal(wi);
		bool refracted = localWi.z < 0;

		if (refracted)
		{ //pass through outer layer

			float cos = -localWi.z;
			//cos = adjacent / hypotenuse
			//hypotenuse = adjacent / cos
			float dist = thickness / cos;
			ShadingData copy = ShadingData{ shadingData };
			copy.sNormal = -copy.sNormal;
			copy.wo = -wi;

			Colour col = reflectedColour;
			float p = pdf;

			Vec3 wi2 = glass.sample(copy, sampler, reflectedColour, pdf);

			reflectedColour.r *= std::powf(M_E, -sigmaa.r * dist);
			reflectedColour.g *= std::powf(M_E, -sigmaa.g * dist);
			reflectedColour.b *= std::powf(M_E, -sigmaa.b * dist);

			Vec3 localWi2 = shadingData.frame.toLocal(wi2);
			bool refracted = localWi2.z < 0;

			if (refracted)
			{ //pass through to base layer
				col = col * reflectedColour;
				p *= pdf;

				copy.sNormal = shadingData.sNormal;
				copy.wo = -wi2;

				Vec3 wi3 = base->sample(copy, sampler, reflectedColour, pdf);
				Vec3 localWi3 = shadingData.frame.toLocal(wi3);

				//pass through medium again

				float cos = -localWi3.z;
				//cos = adjacent / hypotenuse
				//hypotenuse = adjacent / cos
				dist = thickness / cos;

				reflectedColour.r *= std::powf(M_E, -sigmaa.r * dist);
				reflectedColour.g *= std::powf(M_E, -sigmaa.g * dist);
				reflectedColour.b *= std::powf(M_E, -sigmaa.b * dist);

				reflectedColour = col * reflectedColour;
				pdf *= p;
				
				//manual to ensure no reflection
				float theta_in = SphericalCoordinates::sphericalTheta(-localWi3);
				float phi_in = SphericalCoordinates::sphericalPhi(-localWi3);
				float n = intIOR / extIOR;
				float sin_theta_out = n * sinf(theta_in);

				float theta_out = asinf(sin_theta_out);
				float phi_out = phi_in + M_PI;

				Vec3 wt = { sinf(theta_out) * cosf(phi_out), sinf(theta_out) * sinf(phi_out), -cosf(theta_out) * -1 };

				if (wt.z < 0)
				{
					reflectedColour = { 0,0,0 };
					pdf = 0;
					return shadingData.sNormal;
				}

				reflectedColour = reflectedColour * col;
				pdf = p;

				return shadingData.frame.toWorld(wt);
			}
			else
			{
				//reflect back through medium

				reflectedColour.r *= std::powf(M_E, -sigmaa.r * dist);
				reflectedColour.g *= std::powf(M_E, -sigmaa.g * dist);
				reflectedColour.b *= std::powf(M_E, -sigmaa.b * dist);

				float theta_in = SphericalCoordinates::sphericalTheta(-localWi2);
				float phi_in = SphericalCoordinates::sphericalPhi(-localWi2);
				float n = intIOR / extIOR;
				float sin_theta_out = n * sinf(theta_in);

				float theta_out = asinf(sin_theta_out);
				float phi_out = phi_in + M_PI;

				Vec3 wt = { sinf(theta_out) * cosf(phi_out), sinf(theta_out) * sinf(phi_out), -cosf(theta_out) * -1 };

				if (wt.z < 0)
				{
					reflectedColour = { 0,0,0 };
					pdf = 0;
					return shadingData.sNormal;
				}

				reflectedColour = reflectedColour * col;
				pdf = p;

				return shadingData.frame.toWorld(wt);
			}
		}
		else
		{
			if (localWi.z < 0)
			{
				reflectedColour = { 0,0,0 };
				pdf = 0;
				return shadingData.sNormal;
			}

			return wi;
		}
	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
#ifndef LayeredImpl
		return base->evaluate(shadingData, wi);
#endif

		Vec3 wiLocal = shadingData.frame.toLocal(wi);
		Vec3 woLocal = shadingData.frame.toLocal(shadingData.wo);

		if (woLocal.z < 0 || wiLocal.z < 0)
			return { 0,0,0 };

		float n = intIOR / extIOR;
		Vec3 wm = wiLocal * n + woLocal;

		float sin2ThetaIWo = std::max(0.0f, 1 - woLocal.z * woLocal.z);
		float sin2ThetaTWo = sin2ThetaIWo / (n * n);

		float cosThetaTWo = sqrtf(1 - sin2ThetaTWo);
		float distWo = thickness / cosThetaTWo;

		Vec3 woInternal = (- woLocal / n + wm * (woLocal.z / n - cosThetaTWo)) * -1;

		float sin2ThetaIWi = std::max(0.0f, 1 - wiLocal.z * wiLocal.z);
		float sin2ThetaTWi = sin2ThetaIWi / (n * n);
		float cosThetaTWi = sqrtf(1 - sin2ThetaTWi);

		float distWi = thickness / cosThetaTWi;

		Vec3 wiInternal = (- wiLocal / n + wm * (wiLocal.z / n - cosThetaTWi)) * -1;

		ShadingData copy = ShadingData{ shadingData };
		copy.wo = shadingData.frame.toWorld(woInternal);
		Vec3 wiInternalWorld = shadingData.frame.toWorld(wiInternal);

		Colour col = base->evaluate(copy, wiInternalWorld);
		Colour passthrough;
		passthrough.r = std::powf(M_E, -sigmaa.r * (distWo + distWi));
		passthrough.g = std::powf(M_E, -sigmaa.g * (distWo + distWi));
		passthrough.b = std::powf(M_E, -sigmaa.b * (distWo + distWi));
		// Add code for evaluation of layer

		float fresnelWo = ShadingHelper::fresnelDielectric(woLocal.z, intIOR, extIOR);
		float fresnelWi = ShadingHelper::fresnelDielectric(wiLocal.z, extIOR, intIOR);

		if (fresnelWo == -1)
			fresnelWo = 1;
		if (fresnelWi == -1)
			fresnelWi = 1;

		float mults = 1/n * (1 - fresnelWi) / fabsf(wiLocal.z) *
					  1/n * (1 - fresnelWo) / fabsf(wiInternal.z);

		return col * passthrough * mults;
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
#ifndef LayeredImpl
		return base->PDF(shadingData, wi);
#endif

		Vec3 wiLocal = shadingData.frame.toLocal(wi);
		Vec3 woLocal = shadingData.frame.toLocal(shadingData.wo);

		if (woLocal.z < 0 || wiLocal.z < 0)
			return 0;

		float fresnelWo = ShadingHelper::fresnelDielectric(woLocal.z, intIOR, extIOR);
		float fresnelWi = ShadingHelper::fresnelDielectric(wiLocal.z, extIOR, intIOR);

		if (fresnelWo == -1)
			fresnelWo = 1;
		if (fresnelWi == -1)
			fresnelWi = 1;

		float n = intIOR / extIOR;

		Vec3 wm = wiLocal * n + woLocal;
		float sin2ThetaIWi = std::max(0.0f, 1 - wiLocal.z * wiLocal.z);
		float sin2ThetaTWi = sin2ThetaIWi / (n * n);
		float cosThetaTWi = sqrtf(1 - sin2ThetaTWi);

		float distWi = thickness / cosThetaTWi;

		Vec3 wiInternal = (- wiLocal / n + wm * (wiLocal.z / n - cosThetaTWi)) * -1;

		float mults = 1 / n * (1 - fresnelWi) / fabsf(wiLocal.z) *
			1 / n * (1 - fresnelWo) / fabsf(wiInternal.z);

		// Add code to include PDF for sampling layered BSDF
		return mults * base->PDF(shadingData, wi);
	}
	bool isPureSpecular()
	{
#ifndef LayeredImpl
		return base->isPureSpecular();
#endif
		return false;
	}
	bool isTwoSided()
	{
		return base->isTwoSided();
	}
	float mask(const ShadingData& shadingData)
	{
		return base->mask(shadingData);
	}
};