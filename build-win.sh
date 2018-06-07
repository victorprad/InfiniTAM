#! /bin/bash -e

# Check that valid parameters have been specified.
if [ $# -ne 2 ] || ([ "$1" != "11" ] && [ "$1" != "12" ] && [ "$1" != "14" ] && [ "$1" != "15" ]) || ([ "$2" != "Debug" ] && [ "$2" != "Release" ])
then
  echo "Usage: build-win.sh {11|12|14|15} {Debug|Release}"
  exit
fi

# Check that msbuild is on the system path.
./require-msbuild.sh

# Determine the toolset and generator to use.
CMAKE_GENERATOR="Visual Studio $1 Win64"
CMAKE_TOOLSET_STRING=""

if [ "$1" == "15" ]
then
  CMAKE_GENERATOR="Visual Studio 15 2017 Win64"
  CMAKE_TOOLSET_STRING="-T v140"
fi

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
  cmake -G "$CMAKE_GENERATOR" $CMAKE_TOOLSET_STRING ..
  cmake ..

  cd ..
fi

cd build

echo "[InfiniTAM] ...Running build..."
cmd //c "msbuild /p:Configuration=$2 InfiniTAM.sln"

echo "[InfiniTAM] ...Finished building InfiniTAM."
