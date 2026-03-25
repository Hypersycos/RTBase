#pragma once

#include "Core.h"
#include "Sampling.h"
#include <numeric>
#include <stack>

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
	Vec3 e0;
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
		e0 = vertices[1].p - vertices[0].p;
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
		Vec3 e1 = -this->e2;

		Vec3 T = r.o - vertices[0].p;
		Vec3 rayCrossE1 = r.dir.cross(e1);

		float invdet = e0.dot(rayCrossE1);
		if (fabsf(invdet) < 1e-6)
			return false;
		
		invdet = 1 / invdet;

		u = T.dot(rayCrossE1) * invdet;
		if (u < 0 || u > 1)
			return false;

		Vec3 TCrossE0 = T.cross(e0);

		v = r.dir.dot(TCrossE0) * invdet;
		if (v < 0 || (u + v) > 1)
			return false;

		t = e1.dot(TCrossE0) * invdet;

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
#define BUILD_BINS 10

class BVHNode
{
private:
	bool getSplit(const std::vector<Triangle>& triangles, std::vector<unsigned int>& indices, unsigned int start, unsigned int count, float& splitPos, unsigned int& axis)
	{
		float bestCost = FLT_MAX;
		for (unsigned int i = 0; i < 3; i++)
		{
			float axisSplitPos;
			float cost = getSplitForAxis(triangles, indices, start, count, i, axisSplitPos);
			if (cost < bestCost)
			{
				splitPos = axisSplitPos;
				axis = i;
				bestCost = cost;
			}
		}

		float parentCost = count * bounds.area();// - C_bounds / C_isect * area_parent

		if (bestCost > parentCost)
			return false;

		return true;
	}

	float getSplitForAxis(const std::vector<Triangle>& triangles, std::vector<unsigned int>& indices, unsigned int start, unsigned int count, unsigned int axis, float& splitPos)
	{
		float bestCost = FLT_MAX;

		if (BUILD_BINS * 2 < count)
		{
			float binSize = (getValue(bounds.max, axis) - getValue(bounds.min, axis)) / BUILD_BINS;
			float min = getValue(bounds.min, axis);

			for (unsigned int i = 0; i < BUILD_BINS; i++)
			{
				float pos = min + binSize * i;
				float cost = getCostForSplit(triangles, indices, start, count, pos, axis);
				if (cost < bestCost)
				{
					bestCost = cost;
					splitPos = pos;
				}
			}
		}
		else
		{
			for (unsigned int i = start; i < start + count; i++)
			{
				float pos = getValue(triangles[indices[i]].centre(), axis);
				float cost = getCostForSplit(triangles, indices, start, count, pos, axis);
				if (cost < bestCost)
				{
					bestCost = cost;
					splitPos = pos;
				}
			}
		}

		return bestCost;
	}

	float getValue(const Vec3& v, unsigned int axis)
	{
		switch (axis)
		{
		case 0:
			return v.x;
		case 1:
			return v.y;
		case 2:
			return v.z;
		default:
			return 0.0f;
		}
	}

	float getCostForSplit(const std::vector<Triangle>& triangles, std::vector<unsigned int>& indices, unsigned int start, unsigned int count, float split_pos, unsigned int axis)
	{
		AABB leftBox{}, rightBox{};
		unsigned int leftCount = 0, rightCount = 0;

		for (unsigned int i = start; i < start + count; i++)
		{
			if (getValue(triangles[indices[i]].centre(), axis) <= split_pos)
			{
				leftBox.extend(triangles[indices[i]].centre());
				leftCount++;
			}
			else
			{
				rightBox.extend(triangles[indices[i]].centre());
				rightCount++;
			}
		}

		if (leftCount == 0 || rightCount == 0)
			return FLT_MAX;

		return leftCount * leftBox.area() + rightCount * rightBox.area();
	}

	// Note there are several options for how to implement the build method. Update this as required
	void build(const std::vector<Triangle>& triangles, std::vector<unsigned int>& indices, unsigned int start, unsigned int count, unsigned int depth = 0)
	{
		// Add BVH building code here
		for (unsigned int i = start; i < start + count; i++)
		{
			for (unsigned int j = 0; j < 3; j++)
				bounds.extend(triangles[indices[i]].vertices[j].p);
		}

		unsigned int axis;
		float splitPos;

		offset = start;
		num = count;

		if (getSplit(triangles, indices, start, count, splitPos, axis))
		{
			auto midIterator = std::partition(indices.begin() + start, indices.begin() + start + count, [&](const unsigned int t) { return getValue(triangles[t].centre(), axis) <= splitPos; });
			unsigned int mid = midIterator - indices.begin();

			if (mid == start || mid == start + count)
			{
				return;
			}

			l = new BVHNode();
			if (depth <= 5)
			{
				std::cout << "Depth: " << depth << ", start: " << start << ", mid: " << mid << ", end : " << start + count << std::endl;
			}

			l->build(triangles, indices, start, mid - start, depth + 1);

			r = new BVHNode();
			r->build(triangles, indices, mid, start + count - mid, depth + 1);
		}
	}

	void print(unsigned int depth, std::vector<unsigned int>& indices)
	{
		if (l != nullptr)
		{
			l->print(depth + 1, indices);
			r->print(depth + 1, indices);
		}
		else
		{
			std::cout << std::string(depth, '-') << "[";
			for (int i = offset; i < offset + num; i++)
			{
				std::cout << indices[i] << ", ";
			}
			std::cout << "]" << std::endl;
		}
	}

	void stats(unsigned int& leafCount, unsigned int& nodeCount)
	{
		if (l == nullptr)
		{
			leafCount++;
			nodeCount++;
		}
		else
		{
			nodeCount++;
			l->stats(leafCount, nodeCount);
			r->stats(leafCount, nodeCount);
		}
	}

	std::vector<unsigned int> indices;
public:
	AABB bounds;
	BVHNode* r;
	BVHNode* l;
	// This can store an offset and number of triangles in a global triangle list for example
	// But you can store this however you want!
	unsigned int offset = 0;
	unsigned int num = 0;

	BVHNode()
	{
		r = NULL;
		l = NULL;
	}

	~BVHNode()
	{
		if (r != nullptr)
			delete r;
		if (l != nullptr)
			delete l;
	}

	void buildRoot(const std::vector<Triangle>& triangles)
	{
		indices.resize(triangles.size());
		for (int i = 0; i < triangles.size(); i++)
		{
			indices[i] = i;
		}
		build(triangles, indices, 0, triangles.size());
		unsigned int nodecount = 0, leafcount = 0;
		stats(leafcount, nodecount);
		std::cout << nodecount << " nodes, " << leafcount << " leaves." << std::endl;
		std::cout << (float)nodecount / triangles.size() << " nodes per tri, " << (float)leafcount / triangles.size() << " leaves per tri" << std::endl;
	}

	void print()
	{
		print(0, indices);
	}

	void traverse(const Ray& ray, IntersectionData& intersection, const std::vector<Triangle>& triangles)
	{
		// Add BVH Traversal code here
		std::stack<BVHNode*> nodeStack;
		nodeStack.push(this);

		while (!nodeStack.empty())
		{
			BVHNode* node = nodeStack.top();
			nodeStack.pop();

			if (!node->bounds.rayAABB(ray))
			{
				continue;
			}

			if (node->l == nullptr)
			{
				for (int i = node->offset; i < node->offset + node->num; i++)
				{
					float t, alpha, beta;
					if (triangles[indices[i]].rayIntersect(ray, t, alpha, beta))
					{
						if (t < intersection.t)
						{
							intersection.t = t;
							intersection.alpha = alpha;
							intersection.beta = beta;
							intersection.gamma = 1 - alpha - beta;
							intersection.ID = indices[i];
						}
					}
				}
			}
			else
			{
				nodeStack.push(node->l);
				nodeStack.push(node->r);
			}
		}
	}
	IntersectionData traverse(const Ray& ray, const std::vector<Triangle>& triangles)
	{
		IntersectionData intersection;
		intersection.t = FLT_MAX;
		traverse(ray, intersection, triangles);
		return intersection;
	}
	bool traverseVisible(const Ray& ray, const float maxT, const std::vector<Triangle>& triangles)
	{
		// Add visibility code here
		std::stack<BVHNode*> nodeStack;
		nodeStack.push(this);

		while (!nodeStack.empty())
		{
			BVHNode* node = nodeStack.top();

			if (!node->bounds.rayAABB(ray))
			{
				nodeStack.pop();
				continue;
			}

			if (node->l == nullptr)
			{
				for (int i = node->offset; i < node->offset + node->num; i++)
				{
					float t, alpha, beta;
					if (triangles[indices[i]].rayIntersect(ray, t, alpha, beta))
					{
						return false;
					}
				}
				return true;
			}

			nodeStack.pop();
			nodeStack.push(node->l);
			nodeStack.push(node->r);
		}
		return true;
	}
};
