# BUILDING THINGS

The [Bullet physics engine](https://github.com/bulletphysics/bullet3) is
available for OpenSimulator using the BulletSim plugin. This functionality
is provided by several pre-built binary executables for various target
architectures. These executables include DLL's, SO's, and DYNLIB's.

BulletSim consists of the C# code that is included in the OpenSimulator
sources, this C++ "glue" code which provides the interface between the C# code
and the Bullet physics engine, and the Bullet physics engine itself.

The steps are to fetch the Bullet physics engine sources, build it, then
build the BulletSim C++ glue code and staticlly link it with the built
Bullet physics engine.

Since Bullet is supplied as a binary, there are separate versions built
for different target operating systems and machine architectures. Thus
the built binary filename includes the version of Bullet used, the build
date, and the target machine architecture. Expect to see filenames like:

- `libBulletSim-3.25-20230122-x86_64.so` (Intel arch, Linux)
- `libBulletSim-3.25-20230122-aarch64.so` (ARM 64 bit arch, Linux)
- `libBulletSim-3.25-20230122-universal.dynlib` (either x86 or ARM, IOS)
- `libBulletSim-3.25-20230122-x86.64.dll` (Intel arch, Windows)

The selection of which binary to use must be configured in OpenSimulator.
This either requires copying the correct file to as default name or
editing a `.config` file.

# NOTES

- This builds with Bullet physics engine version 3+. Before 2023, 
  `BulletSim.dll` was built with Bullet version 2.86. The 3.25 version
  of Bullet has been tested and does not seem to make any
  difference to OpenSimulator operation as most of the Bullet changes
  have to do with APIs and integration with Python (thus PyBullet).
  Refer to some of the `.sh` files for the process of building the
  BulletSim binary with the previous version of Bullet.

- Only 64 bit architectures are supported.

# BUILDING

The file `makeBullets.sh` is a script that encompasses these steps.

1) Fetch the latest version from GitHub: https://github.com/bulletphysics/bullet3.

```
    cd trunk/unmanaged/BulletSim
    git clone --depth 1 https://github.com/bulletphysics/bullet3.git
```

2) Apply all the patches for bullet:

```
	cd bullet3 ; for file in ../*.patch ; do cat $file | patch -p1 ; done
```

There are some small changes that are needed for the using Bullet for
distributed physics (physics simulation in both the serve and the client).

There are separate patch files for Bullet version 2.86. These all start
with the string "2.86-". Refer to `makeBullets.sh` for an example of pulling
and patching the 2.86 version of Bullet.

3) Build the Bullet physics engine

  a) Windows:

    <b>Windows not done yet</b>
    <strike>
    The Bullet distribution has an instance of PreMake (https://premake.github.io/)
    to build the Visual Studio project files. 
    The script buildBulletVS.bat will call premake and generate the project files.
    As of August 2017, premake version 4 worked but would only generate for VS2010 (v100).
    I build the BulletSim libraries with VS2012 (v110) for downward compatibility.
    I let VS2012 upgrade the VS2010 project files for VS2012.

    Once the project files have been built, open
    "bullet-2/build3/vs2010/0_Bullet3Solution.sln" with VS2012
    and do a batch compile for a Release version for 'Win32' and 'x64'.
    </strike>

  b) Linux and IOS:
    
    The script "buildBulletCMake.sh" has the appropriate cmake and compilation
    commands for Linux and IOS.
    The script builds bullet static libraries and copies them into local directories.

```
    ./buildBulletCMake.sh
```

4) Build BulletSim

  a) Windows:
    <b>Windows not done yet</b>
    <strike>
    Use VS2012 to open "BulletSim.sln". Build Release version for 'Win32'
    and 'x64'. The resulting DLLs will be in "Release/BulletSim.dll"
    and "x64/Release/BulletSim.dll". These files are copied to
    "bin/lib32/BulletSim.dll" and "bin/lib64/BulletSim.dll" in the
    OpenSimulator execution tree.
    </strike>

  b) Linux and IOS:

    Run BulletSim compile and link script:

```
    ./makeBulletSim.sh
```

    This builds a file with a name like: `libBulletSim-3.25-20230111-x86_64.so`.
    Copy this file in to the OpenSimulator `bin/lib64` directory and edit
    `OpenSim.Region.PhysicsModule.BulletS.dll.config` to point to this file for
    the machine architecture you are running on.

