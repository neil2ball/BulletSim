# BulletSim Development Repository

Copy of the C++ wrapper of the [Bullet Physics Engine](https://github.com/bulletphysics/bullet3)
for the BulletSim physics engine for
[OpenSimulator](http://opensimulator.org)
.

This repository has been created for radical enhancement work.

The official source for BulletSim are kept in the OpenSim Libs repository at `git://opensimulator.org/git/opensim-libs`
in the `trunk/unmanaged/BulletSim` sub-directory.

This is a copy of that directory (without the Git history) that is used for major mangling and testing
of new BulletSim configurations. The projects that have been considered:

- separating the physics engine into a separate process with some API ([Thrift](https://thrift.apache.org/)
was one consideration but gRPC or FlatBuffers might be better these days);
- automated building for all the different target machines (ARM, IOS, X86, ...);
- other new physics feature development and testing

As of January 2023, there is the `dotnet6` branch that contains
multi-architecture building development,
versioning, and 
and automated building.
This is all targeted to the `dotnet6` branch of the OpenSimulator development sources.
