#ifndef B2_USER_SETTINGS_H
#define B2_USER_SETTINGS_H

#include "box2d/b2_api.h"
#include "box2d/b2_types.h"

#include <stdarg.h>
#include <stdint.h>

#include <core/os/memory.h>

/**
* @author Brian Semrau
*
* Following template given by b2_settings.h
*/

// Tunable Constants

/// You can use this to change the length scale used by your game.
/// For example for inches you could use 39.4.
#define b2_lengthUnitsPerMeter 1.0f

/// The maximum number of vertices on a convex polygon. You cannot increase
/// this too much because b2BlockAllocator has a maximum object size.
#define b2_maxPolygonVertices 8

// User data

class Box2DCollisionObject;
class Box2DFixture;
class Box2DJoint;

struct B2_API b2BodyUserData {
	b2BodyUserData() :
			owner(NULL) {}

	Box2DCollisionObject *owner;
	//bool is_area;
};

struct B2_API b2FixtureUserData {
	b2FixtureUserData() :
			owner(NULL) {}

	Box2DFixture *owner;
};

struct B2_API b2JointUserData {
	b2JointUserData() :
			owner(NULL) {}

	Box2DJoint *owner;
};

// Memory Allocation

/// Default allocation functions
B2_API void *b2Alloc_Default(int32 size);
B2_API void b2Free_Default(void *mem);

/// Implement this function to use your own memory allocator.
inline void *b2Alloc(int32 size) {
	return memalloc(size);
	//return b2Alloc_Default(size);
}

/// If you implement b2Alloc, you should also implement this function.
inline void b2Free(void *mem) {
	memfree(mem);
	//b2Free_Default(mem);
}

/// Default logging function
B2_API void b2Log_Default(const char *string, va_list args);

/// Implement this to use your own logging.
inline void b2Log(const char *string, ...) {
	va_list args;
	va_start(args, string);
	b2Log_Default(string, args);
	va_end(args);
}

#endif // B2_USER_SETTINGS_H
