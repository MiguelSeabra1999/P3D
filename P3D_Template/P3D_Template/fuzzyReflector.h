#pragma once
#define FUZZY_REFLECTOR_H

#include "vector.h"
#include "maths.h"

using namespace std;

class FuzzyReflector
{
private:
	float roughness = 0.3f;
public:
	FuzzyReflector() {};
	bool calculateFuzzyRayDirection(Vector actualHitPoint, Vector& reflectionRay, Vector normal) {
		Vector S = actualHitPoint + reflectionRay + Vector(rand_float(), rand_float(), rand_float()) * 0.3f;
		Vector Sdir = S - actualHitPoint;
		if (Sdir * normal > 0) {
			reflectionRay = Sdir;
			return true;
		}
		return false;
	}
};