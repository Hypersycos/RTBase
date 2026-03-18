#pragma once

#include "Core.h"
#include "Sampling.h"

class Ray
{
public:
	Vec3 o;
	Vec3 dir;
	Vec3 invDir;
	Ray()
	{
	}
	Ray(Vec3 _o, Vec3 _d)
	{
		init(_o, _d);
	}
	void init(Vec3 _o, Vec3 _d)
	{
		o = _o;
		dir = _d;
		invDir = Vec3(1.0f / dir.x, 1.0f / dir.y, 1.0f / dir.z);
	}
	Vec3 at(const float t) const
	{
		return (o + (dir * t));
	}
};

class Plane
{
public:
	Vec3 n;
	float d;

	Plane(Vec3&& _n, Vec3&& p)
	{
		init(_n, p);
	}

	Plane(Vec3&& _n, float _d)
	{
		init(_n, _d);
	}

	void init(Vec3& _n, Vec3& p)
	{
		n = _n;
		d = -(_n.x * p.x + _n.y * p.y + _n.z * p.z);
	}

	void init(Vec3& _n, float _d)
	{
		n = _n;
		d = _d;
	}
	// Add code here

	bool rayIntersect(Ray& r, float& t)
	{
		float denom = n.dot(r.dir);
		if (denom == 0)
			return false;
		t = -(n.dot(r.o) + d) / denom;
		return t > 0 && true;
	}

	// Add code here
};

#define EPSILON 0.001f

class Triangle
{
public:
	Vertex vertices[3];
	Vec3 e1; // Edge 1
	Vec3 e2; // Edge 2
	Vec3 n; // Geometric Normal
	float area; // Triangle area
	float d; // For ray triangle if needed
	unsigned int materialIndex;

	Triangle()
	{

	}

	Triangle(Vec3 _v0, Vec3 _v1, Vec3 _v2)
	{
		Vec3 e1 = _v2 - _v1;
		Vec3 e2 = _v0 - _v2;
		Vec3 n = e1.cross(e2).normalize();

		Vertex v0 = { _v0, n, 0, 0 };
		Vertex v1 = { _v1, n, 0, 0 };
		Vertex v2 = { _v2, n, 0, 0 };

		init(v0, v1, v2, 0);
	}

	void init(Vertex v0, Vertex v1, Vertex v2, unsigned int _materialIndex)
	{
		materialIndex = _materialIndex;
		vertices[0] = v0;
		vertices[1] = v1;
		vertices[2] = v2;
		e1 = vertices[2].p - vertices[1].p;
		e2 = vertices[0].p - vertices[2].p;
		n = e1.cross(e2).normalize();
		area = e1.cross(e2).length() * 0.5f;
		d = Dot(n, vertices[0].p);
	}
	Vec3 centre() const
	{
		return (vertices[0].p + vertices[1].p + vertices[2].p) / 3.0f;
	}
	// Add code here
	bool rayIntersectSimple(const Ray& r, float& t, float& u, float& v) const
	{
		//plane intersection
		float denom = n.dot(r.dir);
		if (denom == 0)
			return false;
		t = (d - n.dot(r.o)) / denom;

		Vec3 p = r.at(t);
		float invArea = .5f / area;

		u = e1.cross(p - vertices[1].p).dot(n) * invArea;
		if (u < 0.0f || u > 1.0f)
			return false;

		v = e2.cross(p - vertices[2].p).dot(n) * invArea;
		if (v < 0.0f || (u + v) > 1.0f)
			return false;

		return true;
	}

	bool rayIntersectMoller(const Ray& r, float& t, float& u, float& v) const
	{
		//Vec3 e1 = vertices[1].p - vertices[0].p;
		//Vec3 e2 = vertices[2].p - vertices[0].p;

		Vec3 e1 = -this->e1;

		Vec3 T = r.o - vertices[2].p;
		Vec3 rayCrossE2 = r.dir.cross(e2);

		float invdet = e1.dot(rayCrossE2);
		if (fabsf(invdet) < 1e-6)
			return false;
		
		invdet = 1 / invdet;

		u = T.dot(rayCrossE2) * invdet;
		if (u < 0 || u > 1)
			return false;

		Vec3 TCrossE1 = T.cross(e1);

		v = r.dir.dot(TCrossE1) * invdet;
		if (v < 0 || (u + v) > 1)
			return false;

		t = e2.dot(TCrossE1) * invdet;

		return t >= 0;
	}

	bool rayIntersect(const Ray& r, float& t, float& u, float& v) const
	{
		return rayIntersectMoller(r, t, u, v);
	}

	void interpolateAttributes(const float alpha, const float beta, const float gamma, Vec3& interpolatedNormal, float& interpolatedU, float& interpolatedV) const
	{
		interpolatedNormal = vertices[0].normal * alpha + vertices[1].normal * beta + vertices[2].normal * gamma;
		interpolatedNormal = interpolatedNormal.normalize();
		interpolatedU = vertices[0].u * alpha + vertices[1].u * beta + vertices[2].u * gamma;
		interpolatedV = vertices[0].v * alpha + vertices[1].v * beta + vertices[2].v * gamma;
	}
	// Add code here
	Vec3 sample(Sampler* sampler, float& pdf)
	{
		return Vec3(0, 0, 0);
	}
	Vec3 gNormal()
	{
		return (n * (Dot(vertices[0].normal, n) > 0 ? 1.0f : -1.0f));
	}
};

class AABB
{
public:
	Vec3 max;
	Vec3 min;
	AABB()
	{
		reset();
	}

	AABB(Vec3 min, Vec3 max) : max(max), min(min)
	{
	}

	void reset()
	{
		max = Vec3(-FLT_MAX, -FLT_MAX, -FLT_MAX);
		min = Vec3(FLT_MAX, FLT_MAX, FLT_MAX);
	}
	void extend(const Vec3 p)
	{
		max = Max(max, p);
		min = Min(min, p);
	}
	// Add code here
	bool rayAABB(const Ray& r, float& t)
	{
		Vec3 t1 = (min - r.o) * r.invDir;
		Vec3 t2 = (max - r.o) * r.invDir;

		Vec3 TEntry = Min(t1, t2);
		Vec3 TExit = Max(t1, t2);

		float tEntry = std::max(TEntry.x, std::max(TEntry.y, TEntry.z));
		float tExit = std::min(TExit.x, std::min(TExit.y, TExit.z));

		t = std::min(tEntry, tExit);

		return tEntry <= tExit && tExit > 0;
	}
	// Add code here
	bool rayAABB(const Ray& r)
	{
		Vec3 t1 = (min - r.o) * r.invDir;
		Vec3 t2 = (max - r.o) * r.invDir;

		Vec3 TEntry = Min(t1, t2);
		Vec3 TExit = Max(t1, t2);

		float tEntry = std::max(TEntry.x, std::max(TEntry.y, TEntry.z));
		float tExit = std::min(TExit.x, std::min(TExit.y, TExit.z));

		return tEntry <= tExit && tExit > 0;
	}
	// Add code here
	float area()
	{
		Vec3 size = max - min;
		return ((size.x * size.y) + (size.y * size.z) + (size.x * size.z)) * 2.0f;
	}
};

class Sphere
{
public:
	Vec3 centre;
	float radius;
	void init(Vec3& _centre, float _radius)
	{
		centre = _centre;
		radius = _radius;
	}
	// Add code here
	bool rayIntersect(Ray& r, float& t)
	{
		Vec3 l = r.o - centre;
		float b = l.dot(r.dir);
		float c = l.dot(l) - radius * radius;
		float discriminant = b * b - c;

		if (discriminant < 0)
			return false;
		else if (discriminant == 0)
		{
			t = -b;
			return true;
		}
		else
		{
			float sqrRtDiscriminant = std::sqrtf(discriminant);
			t = -b - sqrRtDiscriminant;
			if (t < 0)
				t = -b + sqrRtDiscriminant;
			return true;
		}
	}
};

struct IntersectionData
{
	unsigned int ID;
	float t;
	float alpha;
	float beta;
	float gamma;
};

#define MAXNODE_TRIANGLES 8
#define TRAVERSE_COST 1.0f
#define TRIANGLE_COST 2.0f
#define BUILD_BINS 32

class BVHNode
{
public:
	AABB bounds;
	BVHNode* r;
	BVHNode* l;
	// This can store an offset and number of triangles in a global triangle list for example
	// But you can store this however you want!
	// unsigned int offset;
	// unsigned char num;
	BVHNode()
	{
		r = NULL;
		l = NULL;
	}
	// Note there are several options for how to implement the build method. Update this as required
	void build(std::vector<Triangle>& inputTriangles)
	{
		// Add BVH building code here
	}
	void traverse(const Ray& ray, const std::vector<Triangle>& triangles, IntersectionData& intersection)
	{
		// Add BVH Traversal code here
	}
	IntersectionData traverse(const Ray& ray, const std::vector<Triangle>& triangles)
	{
		IntersectionData intersection;
		intersection.t = FLT_MAX;
		traverse(ray, triangles, intersection);
		return intersection;
	}
	bool traverseVisible(const Ray& ray, const std::vector<Triangle>& triangles, const float maxT)
	{
		// Add visibility code here
		return true;
	}
};
