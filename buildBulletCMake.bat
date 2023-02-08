rem Script to build Bullet on a target system.

set MACH=x64
set BULLETDIR=bullet3

set BUILDDIR=bullet-build

cd %BULLETDIR%
mkdir -p "${BUILDDIR}"
cd %BUILDDIR%

echo "=== Building Bullet in dir %BULLETDIR% for arch %MACH% into %BUILDDIR%"

cmake .. -G "Visual Studio 17 2022" -A %MACH% \
        -DBUILD_BULLET3=ON \
        -DBUILD_EXTRAS=ON \
            -DBUILD_INVERSE_DYNAMIC_EXTRA=OFF \
            -DBUILD_BULLET_ROBOTICS_GUI_EXTRA=OFF \
            -DBUILD_BULLET_ROBOTICS_EXTRA=OFF \
            -DBUILD_OBJ2SDF_EXTRA=OFF \
            -DBUILD_SERIALIZE_EXTRA=OFF \
            -DBUILD_CONVEX_DECOMPOSITION_EXTRA=ON \
            -DBUILD_HACD_EXTRA=ON \
            -DBUILD_GIMPACTUTILS_EXTRA=OFF \
        -DBUILD_CPU_DEMOS=OFF \
        -DBUILD_BULLET2_DEMOS=OFF \
        -DBUILD_ENET=OFF \
        -DBUILD_PYBULLET=OFF \
        -DBUILD_UNIT_TESTS=OFF \
        -DBUILD_SHARED_LIBS=OFF \
        -DINSTALL_EXTRA_LIBS=ON \
        -DINSTALL_LIBS=ON \
        -DCMAKE_CXX_FLAGS="-fPIC" \
        -DCMAKE_BUILD_TYPE=Release

make -j4

make install
