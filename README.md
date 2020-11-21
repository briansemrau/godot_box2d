# Box2D for Godot

An unofficial [Box2D](https://github.com/erincatto/box2d) module for the Godot game engine.

## **This is a work in progress!**

Using Box2D v2.4.1

This should work on all platforms.

This supports Godot 3.2.
This may work with 4.0, but is untested.


# Documentation

There is currently no real documentation for this module. However, most features are meant to mimic the Box2D API, so the [Box2D documentation](https://box2d.org/documentation/) should provide a strong conceptual guide.

## Basic Usage

Unlike Godot 2D physics, this module does not add a physics server. Instead, everything must be done in the scene tree. The `Box2DWorld` node controls all the physics processing.

All physics happens in the physics step, and there is no issue with modifying physics outside of `_physics_process`.

To hopefully make usage clear without too many words, this is an example of a functional scene:

```
root
└─ MyGame [Node]
    ├─ [stuff]
    └─ Box2DWorld
        ├─ Box2DPhysicsBody1
        │   ├─ [sprite]
        │   └─ Box2DCircleFixture
        └─ Box2DPhysicsBody2
        │   ├─ [sprite]
        │   ├─ Box2DRectFixture1
        │   └─ Box2DRectFixture2
        └─ Box2DWeldJoint
```

While Godot uses distinct nodes for physics object types (RigidBody2D, StaticBody2D, etc.), Box2DPhysicsBody has a `type` property that allows you to pick what kind of body to use.

# Building

Right now, there are no pre-built binaries. This will change when this module becomes more stable.

To use this module, it helps to already be familiar with compiling the engine on your own.

## To build:

1. Clone the engine source code

```
cd /your/documents/folder/wherever/
```
For Godot 3.2: `git clone -b 3.2 https://github.com/godotengine/godot.git godot`

For Godot 4.0: `git clone https://github.com/godotengine/godot.git godot`

2. Clone this module
```
cd ./godot/modules
git clone https://github.com/briansemrau/godot_box2d
```

3. Compile the engine.
See the official Godot documentation:
https://docs.godotengine.org/en/latest/development/compiling/index.html

# Comparing to Godot Physics

Box2D provides many features that parallel what is available in Godot, but many things may look different.

### Broad feature comparison:

| Godot Physics                     | Godot Box2D module                                                                                  |
|-----------------------------------|-----------------------------------------------------------------------------------------------------|
| Physics2DServer                   | Physics step is calculated by `Box2DWorld`. Each body, fixture and joint are managed by their node. |
| Bodies (static, rigid, kinematic) | Provided by Box2DPhysicsBody node                                                                               |
| Joints                            | This module offers same constraints + more                                                                           |
| Areas                             | Fixtures with `sensor` flag offer a similar function (⚠ unimplemented)                                |
| Shapes                            | All Godot shapes can be recreated with combinations of fixture shapes (⚠ WIP)                              |

### Why use Box2D?:

-  A _lot_ of new joints
    - [Distance joint](https://box2d.org/documentation/md__d_1__git_hub_box2d_docs_dynamics.html#autotoc_md85) (Equivalent to DampedSpringJoint2D) (⚠ unimplemented)
    - [Revolute joint](https://box2d.org/documentation/md__d_1__git_hub_box2d_docs_dynamics.html#autotoc_md86) (PinJoint2D, but with limits and a motor)
    - [Prismatic joint](https://box2d.org/documentation/md__d_1__git_hub_box2d_docs_dynamics.html#autotoc_md87) (Similar to GrooveJoint2D, I think?) (⚠ unimplemented)
    - [Pulley joint](https://box2d.org/documentation/md__d_1__git_hub_box2d_docs_dynamics.html#autotoc_md88) (⚠ unimplemented)
    - [Gear joint](https://box2d.org/documentation/md__d_1__git_hub_box2d_docs_dynamics.html#autotoc_md89) (⚠ unimplemented)
    - [Wheel joint](https://box2d.org/documentation/md__d_1__git_hub_box2d_docs_dynamics.html#autotoc_md91) (⚠ unimplemented)
    - [Weld joint](https://box2d.org/documentation/md__d_1__git_hub_box2d_docs_dynamics.html#autotoc_md92)
    - [Rope](https://box2d.org/documentation/md__d_1__git_hub_box2d_docs_dynamics.html#autotoc_md93) (⚠ unimplemented)
    - [Friction joint](https://box2d.org/documentation/md__d_1__git_hub_box2d_docs_dynamics.html#autotoc_md94) (⚠ unimplemented)
    - [Motor joint](https://box2d.org/documentation/md__d_1__git_hub_box2d_docs_dynamics.html#autotoc_md95) (⚠ unimplemented)
- Probably more stable physics, but I have no way to support this claim
- ??? Probably more

### What Box2D can't do:

- Instantaneous shape intersection querying. (Instead, create a sensor fixture and wait 1 physics step for it to report.)
- ? possibly more

# ⚠ Planned Features:

This is a list of all things that Box2D can do that this module doesn't provide (yet!).

- Fixtures with certain shapes:
    - Polygon (Except rectangles - those are implemented)
    - Edge
    - Chain
- Lots of joints
- Contact monitoring
- Sensor fixtures

If this list is missing anything important, feel free to submit an issue.

# Contributing

Contributions would be appreciated.
If you would like to contribute to the development or maintenance of this module, please start by submitting an issue.

# License

The Box2D library is developed by Erin Catto. Like Box2D, godot_box2d uses the MIT license.
