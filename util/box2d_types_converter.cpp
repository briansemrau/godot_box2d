#include "box2d_types_converter.h"

#include <core/config/project_settings.h>

/**
* @author Brian Semrau
*/

// Box2D to Godot

void b2_to_gd(b2Vec2 const &inVal, Vector2 &outVal) {
	const float factor = B2_TO_GD;
	outVal[0] = inVal.x * factor;
	outVal[1] = inVal.y * factor;
}

Vector2 b2_to_gd(b2Vec2 const &inVal) {
	return Vector2(inVal.x, inVal.y) * B2_TO_GD;
}

Transform2D b2_to_gd(b2Transform const &inVal) {
	Transform2D outVal;
	outVal[0][0] = inVal.q.c;
	outVal[0][1] = inVal.q.s;
	outVal[1][0] = -inVal.q.s;
	outVal[1][1] = inVal.q.c;
	b2_to_gd(inVal.p, outVal[2]);
	return outVal;
}

// Godot to Box2D

void gd_to_b2(Vector2 const &inVal, b2Vec2 &outVal) {
	const float factor = GD_TO_B2;
	outVal.x = inVal[0] * factor;
	outVal.y = inVal[1] * factor;
}

b2Vec2 gd_to_b2(Vector2 const &inVal) {
	const float factor = GD_TO_B2;
	return b2Vec2(inVal.x * factor, inVal.y * factor);
}

b2Transform gd_to_b2(Transform2D const& inVal) {
	return b2Transform(gd_to_b2(inVal.get_origin()), b2Rot(inVal.get_rotation()));
}

b2AABB gd_to_b2(Rect2 const& inVal) {
	b2AABB outVal;
	outVal.lowerBound = gd_to_b2(inVal.get_position());
	outVal.upperBound = gd_to_b2(inVal.get_position() + inVal.get_size());
	return outVal;
}
