#pragma once

#include "CppUnitTest.h"
#include <concepts>
using namespace Microsoft::VisualStudio::CppUnitTestFramework;

template <std::floating_point T>
void AssertFuzzyEqual(T expected, T value, T epsilon = FLT_EPSILON)
{
	Assert::IsTrue(fabsf(expected - value) < epsilon, (L"Expected " + std::to_wstring(expected) + L", got " + std::to_wstring(value)).c_str());
}

template <std::floating_point T>
void AssertWithinRange(T min, T max, T value)
{
	Assert::IsTrue(value >= min && value <= max, (std::to_wstring(value) + L" is not between "+std::to_wstring(min)+L" and "+std::to_wstring(max)).c_str());
}