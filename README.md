# Box2D for Godot

An unofficial [Box2D](https://github.com/erincatto/box2d) module for the Godot game engine.

## **This is a work in progress!**
**⚠ Expect frequent renames, reworks, crashes, and bugs. ⚠**

Using Box2D v2.4.1

This should work on all platforms.

This supports Godot 3.2.
This should work with 4.0, but is untested.

# Purpose

This module aims to provide the awesome features of the Box2D physics library to the Godot Engine in an easy to use way, similar to the existing 2D physics engine in Godot.

Hopefully the usage of this module is clear whether you're coming with experience from Box2D or from Godot.

# Documentation

There is currently no real documentation for this module. However, most features are meant to align with the Box2D API, so the [Box2D documentation](https://box2d.org/documentation/) should provide a strong conceptual guide.

## Basic Usage

Unlike Godot 2D physics, this module does not add a physics server. Instead, everything must be done in the scene tree. The `Box2DWorld` node controls all the physics processing. (This may change in the far future)

To use this module, first add a `Box2DWorld` node to the scene. This node must be the "ancestor" to any and all `Box2D` nodes you use within this world. (It's okay to have multiple `Box2DWorld` nodes in one scene, but they will not interact, nor their bodies nor joints.)

To create a body, add a `Box2DPhysicsBody` in the node hierarchy beneath the world. This node will do nothing until you add a `Box2DFixture` node as a direct child. The body type (rigid/static/kinematic) is selected with the property `Box2DPhysicsBody.type`.

Here is an example of a functional scene tree:

<pre>
root
└─ MyGame [Node]
    ├─ some control nodes or whatever
    └─ <span style="color:#3cc24a">Box2DWorld</span>
        ├─ <span style="color:#a5b7f3">Box2DPhysicsBody1</span>
        │   ├─ MySprite
        │   └─ <span style="color:#54d1c6">Box2DCircleFixture</span>
        ├─ <span style="color:#a5b7f3">Box2DPhysicsBody2</span>
        │   ├─ <span style="color:#54d1c6">Box2DRectFixture1</span>
        │   └─ <span style="color:#54d1c6">Box2DRectFixture2</span>
        ├─ <span style="color:#e5b23b">Box2DWeldJoint</span>
        └─ WheelOnAStick
            ├─ <span style="color:#a5b7f3">Box2DPhysicsBody1</span>
            │   └─ <span style="color:#54d1c6">Box2DRectFixture</span>
            ├─ <span style="color:#a5b7f3">Box2DPhysicsBody2</span>
            │   └─ <span style="color:#54d1c6">Box2DCircleFixture</span>
            └─ <span style="color:#e5b23b">Box2DRevoluteJoint</span>
</pre>

## Building the module:

### Pre-built releases:

Right now, there are no pre-built binaries. This will change when this module becomes more stable.

To use this module, it helps to already be familiar with compiling the engine on your own.

### Building it yourself:

1. Clone the engine source code (Godot 3.2):

```
cd /your/documents/folder/wherever/
git clone -b 3.2 https://github.com/godotengine/godot.git godot
```

For Godot 4.0 use: `git clone https://github.com/godotengine/godot.git godot`

2. Clone this module inside the modules folder:
```
cd ./godot/modules
git clone https://github.com/briansemrau/godot_box2d
```

3. Compile the engine.
See the official Godot documentation:
https://docs.godotengine.org/en/latest/development/compiling/index.html

## Comparing to Godot 2D Physics

### Why use Box2D?:
Box2D has...
-  A _lot_ of new joints:
    - [Distance joint](https://box2d.org/documentation/md__d_1__git_hub_box2d_docs_dynamics.html#autotoc_md85) (Equivalent to `DampedSpringJoint2D`) (⚠ unimplemented)
    - [Revolute joint](https://box2d.org/documentation/md__d_1__git_hub_box2d_docs_dynamics.html#autotoc_md86) (`PinJoint2D`, but with limits and a motor)
    - [Prismatic joint](https://box2d.org/documentation/md__d_1__git_hub_box2d_docs_dynamics.html#autotoc_md87) (Similar to `GrooveJoint2D`, I think?) (⚠ unimplemented)
    - [Pulley joint](https://box2d.org/documentation/md__d_1__git_hub_box2d_docs_dynamics.html#autotoc_md88) (⚠ unimplemented)
    - [Gear joint](https://box2d.org/documentation/md__d_1__git_hub_box2d_docs_dynamics.html#autotoc_md89) (⚠ unimplemented)
    - [Wheel joint](https://box2d.org/documentation/md__d_1__git_hub_box2d_docs_dynamics.html#autotoc_md91) (⚠ unimplemented)
    - [Weld joint](https://box2d.org/documentation/md__d_1__git_hub_box2d_docs_dynamics.html#autotoc_md92)
    - [Rope](https://box2d.org/documentation/md__d_1__git_hub_box2d_docs_dynamics.html#autotoc_md93) (⚠ unimplemented)
    - [Friction joint](https://box2d.org/documentation/md__d_1__git_hub_box2d_docs_dynamics.html#autotoc_md94) (⚠ unimplemented)
    - [Motor joint](https://box2d.org/documentation/md__d_1__git_hub_box2d_docs_dynamics.html#autotoc_md95) (⚠ unimplemented)
- Improved physics stability
- Additional small features that Godot doesn't have that may be critical for some games, such as:
    - Automatically calculated mass properties (body center of mass, body mass given material density, etc.)
    - Joints report what forces they're exerting (allows for breakable joints)
    - Contacts report collision impulse (necessary for destructible bodies)
    - Material property settings *per shape*, not just per body
    

### Guide for Switching from Godot Physics to Box2D:

Many features of Box2D have very clear parallels to Godot physics. Here are a few, just to help get started:

| Godot Physics | Godot Box2D module |
|-|-|
| `PhysicsBody2D` (Rigid, Static, Kinematic) | Use `Box2DPhysicsBody`. Use `type` property to change body type. Material properties are set using fixtures. |
| `Area2D` | *Not yet implemented.* |
| `CollisionShape2D`/`CollisionPolygon2D` | Use `Box2DFixture` and set the `shape` property. |
| `Joint2D` (Pin, Groove, Spring) | Use variants of `Box2DJoint`. |

## Additional features this module provides:
(Features not provided by the Box2D library)
### Breakable joints
Gives joints new properties `breaking_enabled`, `max_force`, and `max_torque` to configure maximum linear force and/or torque
### Concave polygons
Godot physics provides this, but the Box2D library doesn't. This module bridges the gap so you don't have to think about polygon concavity or vertex count.

# Development Plans:

This is a list of all things that Box2D can do that this module doesn't provide (yet!):

- Edge shape
- Chain shape
- Lots of joints
- Contact monitoring
- Sensor fixtures
- Collision filtering/exclusion

If this list is missing anything important, feel free to submit an issue so it can be tracked.

# Contributing

If you would like to contribute to the development or maintenance of this module, please start by submitting an issue.

# License

The [Box2D library](https://github.com/erincatto/box2d) is developed and maintained by Erin Catto and is provided under the MIT license.

All code in this repository is provided under the MIT license.
