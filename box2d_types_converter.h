#ifndef BOX2D_TYPES_CONVERTER_H
#define BOX2D_TYPES_CONVERTER_H

#include "core/math/vector2.h"
#include "core/math/vector3.h"
#include "core/math/transform_2d.h"
#include "core/typedefs.h"

#include "box2d/b2_math.h"
#include "box2d/b2_collision.h"

/**
* @author Brian Semrau
*/

extern void b2_to_gd(b2Vec2 const &inVal, Vector2 &outVal);
extern Vector2 b2_to_gd(b2Vec2 const &inVal);
extern Transform2D b2_to_gd(b2Transform const &inVal);

extern void gd_to_b2(Vector2 const &inVal, b2Vec2 &outVal);
extern b2Vec2 gd_to_b2(Vector2 const &inVal);
extern b2Transform gd_to_b2(Transform2D const &inVal);
extern b2AABB gd_to_b2(Rect2 const &inVal);

#endif //BOX2D_TYPES_CONVERTER_H
