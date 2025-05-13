# Physics Concepts

This document explains the core physics concepts implemented in the PhysEngine library.

## Rigid Body Dynamics

### Rigid Bodies

A rigid body is an idealized solid that doesn't deform under applied forces. It has the following properties:

- **Position**: The location of the body's center of mass
- **Orientation**: The rotation of the body relative to a reference frame
- **Linear Velocity**: The rate of change of position
- **Angular Velocity**: The rate of change of orientation
- **Mass**: The amount of matter in the body
- **Inertia Tensor**: Describes how mass is distributed and how it resists rotational acceleration

### Types of Rigid Bodies

- **Dynamic**: Fully simulated, affected by forces and collisions
- **Kinematic**: Moved manually, affect dynamic bodies but are not affected by forces
- **Static**: Never move, used for immovable objects like terrain

### Forces and Torques

- **Force**: Causes linear acceleration according to F = ma
- **Torque**: Causes angular acceleration according to τ = Iα
- **Impulse**: An instantaneous change in momentum (force applied over an infinitesimal time)

## Collision Detection

Collision detection is typically divided into phases:

### Broad Phase

Efficiently identifies potential collisions to reduce the number of detailed checks:

- **Spatial Hashing**: Divides space into a grid and only checks objects in the same or neighboring cells
- **Sweep and Prune**: Sorts objects along axes and checks overlapping intervals

### Narrow Phase

Performs detailed collision checks between pairs identified in the broad phase:

- **GJK (Gilbert-Johnson-Keerthi)**: An algorithm for determining if two convex shapes intersect
- **EPA (Expanding Polytope Algorithm)**: Finds the penetration depth and contact normal after GJK detects a collision

### Contact Generation

Once a collision is detected, contact points are generated:

- **Contact Points**: The points where bodies touch
- **Contact Normal**: The direction perpendicular to the contact surface
- **Penetration Depth**: How deeply the bodies are intersecting

## Collision Response

After detecting collisions, the physics engine needs to respond appropriately:

### Constraint-Based Physics

- **Constraints**: Equations that limit how bodies can move relative to each other
- **Constraint Solver**: Iteratively solves all constraints to find a valid configuration
- **Sequential Impulse**: A method for solving constraints by applying impulses at contact points

### Collision Resolution Properties

- **Restitution**: Determines bounciness (0 = no bounce, 1 = perfect bounce)
- **Friction**: Resistance to sliding motion between surfaces
- **Rolling Friction**: Resistance to rolling motion

## Numerical Integration

Integration methods update the position and orientation of bodies over time:

### Integration Methods

- **Euler**: Simple first-order method (least accurate)
- **Symplectic Euler**: Modified Euler that preserves energy better (good balance of accuracy and performance)
- **Verlet**: Second-order method with good numerical stability
- **Runge-Kutta**: Higher-order methods for more accurate results (more computationally expensive)

## Optimization Techniques

### Sleeping

Bodies that haven't moved significantly for a period are put to sleep to save processing power:

- **Velocity Threshold**: Bodies with velocities below this threshold are candidates for sleeping
- **Sleep Time**: How long a body must be inactive before being put to sleep

### Island Management

Connected bodies are grouped into "islands" for more efficient simulation:

- **Island**: A set of bodies connected by constraints or contacts
- **Island Solver**: Solves constraints within an island independently from others

## Advanced Features

### Continuous Collision Detection (CCD)

Prevents fast-moving objects from tunneling through thin objects:

- **Sweep Tests**: Check for collisions along the path of a moving object
- **Time of Impact**: Calculate exactly when a collision occurs within a time step

### Constraint Types

- **Contact Constraints**: Prevent bodies from penetrating each other
- **Joint Constraints**: Connect bodies with specific types of joints:
  - **Distance**: Maintains a fixed distance between points on two bodies
  - **Hinge/Revolute**: Allows rotation around a single axis, like a door hinge
  - **Ball Socket/Spherical**: Allows rotation in all directions, like a shoulder joint
  - **Slider/Prismatic**: Allows movement along a single axis
  - **Fixed**: Rigidly attaches two bodies together

### Shapes

- **Primitive Shapes**: Simple geometric shapes (sphere, box, capsule)
- **Compound Shapes**: Multiple shapes combined for more complex objects
- **Convex Shapes**: Shapes where any line connecting two points stays inside the shape
- **Concave Shapes**: Typically decomposed into convex parts for simulation

## Physics Materials

Materials define surface properties for interactions:

- **Density**: Determines the mass based on the shape's volume
- **Restitution**: How bouncy a surface is (0-1)
- **Friction**: How much resistance there is to sliding (0-1)
- **Rolling Friction**: How much resistance there is to rolling

## Time Stepping

Managing the simulation time is critical for stability and performance:

- **Fixed Time Step**: Updates the simulation at a constant rate
- **Variable Time Step**: Adapts to the actual frame rate
- **Sub-stepping**: Divides large time steps into smaller ones for stability