#ifndef BOX2D_CHILD_OBJECT_H
#define BOX2D_CHILD_OBJECT_H

#include <scene/main/node.h>

class Box2DWorld;
class Box2DPhysicsBody;

class IBox2DChildObject {
	friend class Box2DWorld;
	friend class Box2DPhysicsBody;

protected:
	virtual void on_parent_created(Node *p_parent) = 0;
};

#endif
