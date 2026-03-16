#include "pch.h"
#include "CppUnitTest.h"
#include "Geometry.h"

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace UnitTest1
{
	TEST_CLASS(TriangleTests)
	{
	public:
		TEST_METHOD(RayIntersectsTriangleCentre)
		{
			Ray r = { {0, 0.5f, 0}, {1, 0, 0} };
			Triangle T { Vec3{ 1, 0, 1 }, Vec3{ 1, 1, 0 }, Vec3{ 1, 0, -1 } };

			float t, u, v;
			Assert::IsTrue(T.rayIntersect(r, t, u, v));
		}

		TEST_METHOD(RayIntersectsTriangleEdge)
		{
			Ray r = { {0, 0, 0}, {1, 0, 0} };
			Triangle T{ Vec3{ 1, 0, 1 }, Vec3{ 1, 1, 0 }, Vec3{ 1, 0, -1 } };

			float t, u, v;
			Assert::IsTrue(T.rayIntersect(r, t, u, v));
		}

		TEST_METHOD(CorrectUInCentre)
		{
			Ray r = { {0, 0.5f, 0}, {1, 0, 0} };
			Triangle T{ Vec3{ 1, 0, 1 }, Vec3{ 1, 1, 0 }, Vec3{ 1, 0, -1 } };

			float t, u, v;
			T.rayIntersect(r, t, u, v);
			Assert::AreEqual(0.25f, u);
		}

		TEST_METHOD(CorrectVInCentre)
		{
			Ray r = { {0, 0.5f, 0}, {1, 0, 0} };
			Triangle T{ Vec3{ 1, 0, 1 }, Vec3{ 1, 1, 0 }, Vec3{ 1, 0, -1 } };

			float t, u, v;
			T.rayIntersect(r, t, u, v);
			Assert::AreEqual(0.5f, v);
		}

		TEST_METHOD(CorrectTInTopLeft)
		{
			Ray r = { {0, 1, 6.80000019}, {-0.166818127, 0.166818127, -0.971773326} };
			Triangle T{ Vec3{ -1, 2, -1 }, Vec3{ -1, 2, 1 }, Vec3{ 1, 2, 1 } };

			float t, u, v;
			Assert::IsTrue(T.rayIntersect(r, t, u, v));
			//5.9683062358509975 or 5.995203836930456
			Assert::IsTrue(t < 6.0f && t > 5.96f);
		}
	};
}
