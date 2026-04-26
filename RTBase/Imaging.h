#pragma once

#include "Core.h"
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
#define STB_IMAGE_WRITE_IMPLEMENTATION
#define __STDC_LIB_EXT1__
#include "stb_image_write.h"
#include <OpenImageDenoise/oidn.hpp>

#define ADDITIVESAMPLES
#define RayTrace
//#define LightTrace film->width * film->height
#define DielectricImpl
#define LayeredImpl
//#define DielecNoTransmit
#define DielecNoAbsorb
//#define PlasticPhong
//#define MultipleImportanceSampling
//#define Denoise
//#define DenoiseCleanAux
//#define CountTiles

#if defined(LightTrace) && defined(RayTrace)
#define LightTWeight 0.5
#define RayTWeight 0.5
#elif defined(LightTrace)
#define LightTWeight 1
#define RayTWeight 0
#else
#define LightTWeight 0
#define RayTWeight 1
#endif



#if defined(NDEBUG)
#define SAMPLESPP 1
#define USETHREADS
#else
#define SAMPLESPP 1
#endif

// Stop warnings about buffer overruns if size is zero. Size should never be zero and if it is the code handles it.
#pragma warning( disable : 6386)

constexpr float texelScale = 1.0f / 255.0f;

class Texture
{
public:
	Colour* texels;
	float* alpha;
	int width;
	int height;
	int channels;
	void loadDefault()
	{
		width = 1;
		height = 1;
		channels = 3;
		texels = new Colour[1];
		texels[0] = Colour(1.0f, 1.0f, 1.0f);
	}
	void load(std::string filename)
	{
		alpha = NULL;
		if (filename.find(".hdr") != std::string::npos)
		{
			float* textureData = stbi_loadf(filename.c_str(), &width, &height, &channels, 0);
			if (width == 0 || height == 0)
			{
				loadDefault();
				return;
			}
			texels = new Colour[width * height];
			for (int i = 0; i < (width * height); i++)
			{
				texels[i] = Colour(textureData[i * channels], textureData[(i * channels) + 1], textureData[(i * channels) + 2]);
			}
			stbi_image_free(textureData);
			return;
		}
		unsigned char* textureData = stbi_load(filename.c_str(), &width, &height, &channels, 0);
		if (width == 0 || height == 0)
		{
			loadDefault();
			return;
		}
		texels = new Colour[width * height];
		for (int i = 0; i < (width * height); i++)
		{
			texels[i] = Colour(textureData[i * channels] / 255.0f, textureData[(i * channels) + 1] / 255.0f, textureData[(i * channels) + 2] / 255.0f);
		}
		if (channels == 4)
		{
			alpha = new float[width * height];
			for (int i = 0; i < (width * height); i++)
			{
				alpha[i] = textureData[(i * channels) + 3] / 255.0f;
			}
		}
		stbi_image_free(textureData);
	}
	Colour sample(const float tu, const float tv) const
	{
		Colour tex;
		float u = std::max(0.0f, fabsf(tu)) * width;
		float v = std::max(0.0f, fabsf(tv)) * height;
		int x = (int)floorf(u);
		int y = (int)floorf(v);
		float frac_u = u - x;
		float frac_v = v - y;
		float w0 = (1.0f - frac_u) * (1.0f - frac_v);
		float w1 = frac_u * (1.0f - frac_v);
		float w2 = (1.0f - frac_u) * frac_v;
		float w3 = frac_u * frac_v;
		x = x % width;
		y = y % height;
		Colour s[4];
		s[0] = texels[y * width + x];
		s[1] = texels[y * width + ((x + 1) % width)];
		s[2] = texels[((y + 1) % height) * width + x];
		s[3] = texels[((y + 1) % height) * width + ((x + 1) % width)];
		tex = (s[0] * w0) + (s[1] * w1) + (s[2] * w2) + (s[3] * w3);
		return tex;
	}
	float sampleAlpha(const float tu, const float tv) const
	{
		if (alpha == NULL)
		{
			return 1.0f;
		}
		float tex;
		float u = std::max(0.0f, fabsf(tu)) * width;
		float v = std::max(0.0f, fabsf(tv)) * height;
		int x = (int)floorf(u);
		int y = (int)floorf(v);
		float frac_u = u - x;
		float frac_v = v - y;
		float w0 = (1.0f - frac_u) * (1.0f - frac_v);
		float w1 = frac_u * (1.0f - frac_v);
		float w2 = (1.0f - frac_u) * frac_v;
		float w3 = frac_u * frac_v;
		x = x % width;
		y = y % height;
		float s[4];
		s[0] = alpha[y * width + x];
		s[1] = alpha[y * width + ((x + 1) % width)];
		s[2] = alpha[((y + 1) % height) * width + x];
		s[3] = alpha[((y + 1) % height) * width + ((x + 1) % width)];
		tex = (s[0] * w0) + (s[1] * w1) + (s[2] * w2) + (s[3] * w3);
		return tex;
	}
	~Texture()
	{
		delete[] texels;
		if (alpha != NULL)
		{
			delete alpha;
		}
	}
};

class ImageFilter
{
public:
	virtual float filter(const float x, const float y) const = 0;
	virtual unsigned int size() const = 0;
};

class BoxFilter : public ImageFilter
{
public:
	float filter(float x, float y) const
	{
		if (fabsf(x) < 0.5f && fabs(y) < 0.5f)
		{
			return 1.0f;
		}
		return 0;
	}
	unsigned int size() const
	{
		return 0;
	}
};

class GaussianFilter : public ImageFilter
{
	float radius;
	float alpha;
	float limit;

	float G(float d) const
	{
		return std::pow(M_E, -alpha * d * d) - limit;
	}

public:
	GaussianFilter(float alpha, float radius) : alpha(alpha), radius(radius)
	{
		limit = std::pow(M_E, -alpha * radius * radius);
	}

	float filter(float x, float y) const
	{
		return G(x) * G(y);
	}

	unsigned int size() const
	{
		return std::ceil(radius);
	}
};

class MitchellNetravaliFilter : public ImageFilter
{
	float B;
	float C;

	float h(float x) const
	{
		float absX = fabsf(x);
		float absX2 = absX * absX;
		float absX3 = absX2 * absX;
		if (absX < 1)
		{
			return 1.0f / 6.0f * ((12 - 9 * B - 6 * C) * absX3
				+ (-18 + 12 * B + 6 * C) * absX2
				+ (6 - 2 * B));
		}
		else if (absX < 2)
		{
			return 1.0f / 6.0f * ((-B - 6 * C) * absX3
				+ (6 * B + 30 * C) * absX2
				+ (-12 * B - 48 * C) * absX
				+ (8 * B + 24 * C));
		}
		else
		{
			return 0;
		}
	}

public:
	MitchellNetravaliFilter(float B = 1.0f/3.0f, float C = 1.0f/3.0f) : B(B), C(C)
	{
	}

	float filter(float x, float y) const
	{
		return h(x) * h(y);
	}

	unsigned int size() const
	{
		return 2;
	}
};

class Film
{
public:
	Colour* film;
	Colour* lightTraceFilm;
	float* colorBuff;
	float* albedoBuff;
	float* normalBuff;
	float* denoisedBuff;

	float* currentRenderBuff;

	oidn::FilterRef oidnFilter;
	Colour* albedos;
	Vec3* normals;
	oidn::DeviceRef oidnDevice;

	unsigned int width;
	unsigned int height;
	int SPP;
	int lightPaths;
	ImageFilter* filter;

	unsigned int xyToIndex(unsigned int x, unsigned int y)
	{
		return x + y * width;
	}

	Colour& operator()(int x, int y)
	{
		return film[xyToIndex(x, y)];
	}

	const Colour& getCombinedColour(unsigned int index)
	{
		return film[index] / SPP * RayTWeight + lightTraceFilm[index] / SPP * LightTWeight;
	}

	void denoiseData(const float x, const float y, const Colour& albedo, const Vec3& normal)
	{
		unsigned int index = xyToIndex(x, y);
		albedos[index] = albedos[index] + albedo;
		normals[index] = normals[index] + normal;
	}

	void splat(const float x, const float y, const Colour& L, Colour* dest)
	{
		// Code to splat a smaple with colour L into the image plane using an ImageFilter

		float total = 0;
		int size = filter->size();

		std::vector<unsigned int> indices;
		indices.reserve(size * size);
		std::vector<float> filterWeights;
		filterWeights.reserve(size * size);

		for (int i = -size; i <= size; i++)
		{
			for (int j = -size; j <= size; j++)
			{
				int px = (int)x + j;
				int py = (int)y + i;
				if (px >= 0 && px < width && py >= 0 && py < height)
				{
					indices.push_back((py * width) + px);
					float weight = filter->filter(px - x + .5f, py - y + .5f);
					filterWeights.push_back(weight);
					total += weight;
				}
			}
		}

		if (total <= 0)
			return;

		for (int i = 0; i < indices.size(); i++)
		{
			dest[indices[i]] = dest[indices[i]] + (L * filterWeights[i] / total);
		}
	}

	float tonemapLinearWithExposure(Colour& c, float L_in, float exposure = 1.0f)
	{
		float L_out = L_in * std::powf(2, exposure);
		return L_out;
	}

	float tonemapReinhard(Colour& c, float L_in, float exposure = 1.0f)
	{
		float L_out = L_in / (1 + L_in);
		return L_out;
	}

	float tonemapFilmic(Colour& c, float L_in, float exposure = 1.0f)
	{
		static auto C = [](float x, float A, float B, float C, float D, float E, float F)
			{
				return (x * (A * x + C * B) + D * E) / (x * (A * x + B) + D * F) - E / F;
			};
		static auto C2 = [&](float x)
			{
				return C(x, 0.15f, 0.5f, 0.1f, 0.2f, 0.02f, 0.3f);
			};

		float L_out = C2(L_in) / C2(11.2);
		return L_out;
	}

	float tonemapPassthrough(Colour& c, float L_in, float exposure = 1.0f)
	{
		return L_in;
	}


	void tonemap(int x, int y, unsigned char& r, unsigned char& g, unsigned char& b, float exposure = 1.0f)
	{
#ifdef Denoise
		unsigned int index = xyToIndex(x, y) * 3;

		Colour c = Colour(currentRenderBuff[index + 0], currentRenderBuff[index + 1], currentRenderBuff[index + 2]);
#else
		Colour& filmC = operator()(x, y);
		Colour c = filmC / SPP;
#endif

		float L_in = c.Lum();

		if (L_in <= 0)
		{
			r = b = g = 0;
			return;
		}

		float L_out = tonemapFilmic(c, L_in, exposure);

		float scalar = L_out / L_in;

		c.r = std::min(1.0f, powf(c.r * scalar, 1.0f / 2.2f));
		c.g = std::min(1.0f, powf(c.g * scalar, 1.0f / 2.2f));
		c.b = std::min(1.0f, powf(c.b * scalar, 1.0f / 2.2f));

		r = std::round(c.r * 255);
		g = std::round(c.g * 255);
		b = std::round(c.b * 255);
	}

	// Do not change any code below this line
	void init(int _width, int _height, ImageFilter* _filter)
	{
		width = _width;
		height = _height;
		film = new Colour[width * height];
		lightTraceFilm = new Colour[width * height];

		clear();
		filter = _filter;
	}
	void clear()
	{
		memset(film, 0, width * height * sizeof(Colour));
		memset(lightTraceFilm, 0, width * height * sizeof(Colour));
#ifdef Denoise
#ifndef DenoiseCleanAux
		memset(albedos, 0, width * height * sizeof(Colour));
		memset(normals, 0, width * height * sizeof(Vec3));
#endif
#endif
		SPP = 0;
		lightPaths = 0;
	}
	void incrementSPP()
	{
		SPP++;
	}
	void save(std::string filename)
	{
		Colour* hdrpixels = new Colour[width * height];
		for (unsigned int i = 0; i < (width * height); i++)
		{
			hdrpixels[i] = film[i] / (float)SPP;
		}
		stbi_write_hdr(filename.c_str(), width, height, 3, (float*)hdrpixels);
		delete[] hdrpixels;
	}
};