#! /bin/bash -e

# Check that valid parameters have been specified.
if [ $# -ne 2 ] || ([ "$1" != "11" ] && [ "$1" != "12" ] && [ "$1" != "14" ]) || ([ "$2" != "Debug" ] && [ "$2" != "Release" ])
then
  echo "Usage: build-win.sh {11|12|14} {Debug|Release}"
  exit
fi

# Check that msbuild is on the system path.
./require-msbuild.sh

# Download and extract freeglut if it's not already present.
if [ ! -d freeglut ]
then
  /bin/rm -fR tmp
  mkdir tmp
  cd tmp

  echo "[InfiniTAM] Downloading freeglut..."
  curl -L http://files.transmissionzero.co.uk/software/development/GLUT/older/freeglut-MSVC-2.8.1-1.mp.zip > freeglut-MSVC-2.8.1-1.mp.zip

  echo "[InfiniTAM] Extracting freeglut..."
  unzip freeglut-MSVC-2.8.1-1.mp.zip

  mv freeglut ..
  cd ..
  /bin/rm -fR tmp
fi

# Build InfiniTAM itself.
echo "[InfiniTAM] Building InfiniTAM"

if [ ! -d build ]
then
  mkdir build
  cd build

  # Note: We need to configure twice to handle conditional building.
  echo "[InfiniTAM] ...Configuring using CMake..."
  cmake -G "Visual Studio $1 Win64" ..
  cmake ..

  cd ..
fi

cd build

echo "[InfiniTAM] ...Running build..."
cmd //c "msbuild /p:Configuration=$2 InfiniTAM.sln"

echo "[InfiniTAM] ...Finished building InfiniTAM."
