
#ifndef BOX2D_WORLD_H
#define BOX2D_WORLD_H

#include "core/object.h"
#include "core/reference.h"
#include "core/resource.h"

#include "scene/2d/node_2d.h"

#include "box2d/b2_fixture.h"
#include "box2d/b2_joint.h"
#include "box2d/b2_world.h"
#include "box2d/b2_world_callbacks.h"

#include "box2d_child_object.h"
#include "box2d_types_converter.h"

/**
* @author Brian Semrau
*/

class Box2DWorld : public Node2D, b2DestructionListener {
	GDCLASS(Box2DWorld, Node2D);

	friend class Box2DPhysicsBody;
	friend class Box2DJoint;

private:
	Vector2 gravity;
	b2World *world;

	Set<IBox2DChildObject *> box2d_children;

	virtual void SayGoodbye(b2Joint *joint) override;
	virtual void SayGoodbye(b2Fixture *fixture) override;

	// TODO Refactor this callback garbage.
	//      It may make sense to do this when/if shape queries are implemented.
	//      These at least need renamed.
	class QueryCallback : public b2QueryCallback {
	public:
		Vector<b2Fixture *> results;

		virtual bool ReportFixture(b2Fixture *fixture) override;
	};

	class IntersectPointCallback : public b2QueryCallback {
	public:
		Vector<b2Fixture *> results;
		b2Vec2 point;

		virtual bool ReportFixture(b2Fixture *fixture) override;
	};

	QueryCallback aabbCallback;
	IntersectPointCallback pointCallback;

protected:
	void _notification(int p_what);
	static void _bind_methods();

public:
	void step(real_t p_step);

	void set_gravity(const Vector2 &gravity);
	Vector2 get_gravity() const;

	//bool isLocked() const;

	Array intersect_point(const Point2 &p_point);
	Array query_aabb(const Rect2 &p_bounds); // TODO add more parameters like Physics2DDirectSpaceState::_intersect_point

	//void shiftOrigin(const Vector2 &newOrigin);

	// debugDraw

	Box2DWorld();
	~Box2DWorld();
};

#endif // BOX2D_WORLD_H
