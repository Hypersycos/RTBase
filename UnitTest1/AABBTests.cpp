#include "pch.h"
#include "CppUnitTest.h"
#include "Scene.h"

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace UnitTest1
{
	TEST_CLASS(AABBTests)
	{
	public:
		/*TEST_METHOD(RayIntersectsPlane)
		{
			Ray r = { {0, 0, 0}, {0, 0, 1} };
			Plane p = { {0, 0, 1}, {0, 0, 2} };

			float t;
			Assert::IsTrue(p.rayIntersect(r, t));
		}

		TEST_METHOD(RayIntersectsOppositePlane)
		{
			Ray r = { {0, 0, 0}, {0, 0, 1} };
			Plane p = { {0, 0, -1}, {0, 0, 2} };

			float t;
			Assert::IsTrue(p.rayIntersect(r, t));
		}

		TEST_METHOD(RayIntersectsPlaneWithCorrectT)
		{
			Ray r = { {0, 0, 0}, {0, 0, 1} };
			Plane p = { {0, 0, 1}, {0, 0, 2} };

			float t;
			p.rayIntersect(r, t);
			Assert::AreEqual(2.0f, t);
		}

		TEST_METHOD(RayNoIntersectBehind)
		{
			Ray r = { {0, 0, 0}, {0, 0, 1} };
			Plane p = { {0, 0, 1}, {0, 0, -2} };

			float t;
			Assert::IsFalse(p.rayIntersect(r, t));
		}

		TEST_METHOD(RayNoIntersectParallel)
		{
			Ray r = { {0, 0, 0}, {0, 0, 1} };
			Plane p = { {0, 0, 1}, {0, 1, 0} };

			float t;
			Assert::IsFalse(p.rayIntersect(r, t));
		}

		TEST_METHOD(RayNoIntersectStartingInside)
		{
			Ray r = { {0, 0, 0}, {0, 0, 1} };
			Plane p = { {0, 0, 0}, {0, 0, 1} };

			float t;
			Assert::IsFalse(p.rayIntersect(r, t));
		}*/
	};
}
