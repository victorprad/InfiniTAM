#! /bin/bash -e

# Check that valid parameters have been specified.
if [ $# -ne 2 ] || ([ "$1" != "11" ] && [ "$1" != "12" ]) || ([ "$2" != "Debug" ] && [ "$2" != "Release" ])
then
  echo "Usage: build-win.sh {11|12} {Debug|Release}"
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
  curl http://files.transmissionzero.co.uk/software/development/GLUT/older/freeglut-MSVC-2.8.1-1.mp.zip > freeglut-MSVC-2.8.1-1.mp.zip

  echo "[InfiniTAM] Extracting freeglut..."
  unzip freeglut-MSVC-2.8.1-1.mp.zip

  mv freeglut ..
  cd ..
  /bin/rm -fR tmp
fi

#cd libraries
#./build-boost_1_56_0-win.sh "msvc-$1.0"
#./build-glew-1.12.0-win.sh
#./build-opencv-2.4.9-win.sh "Visual Studio $1 Win64"
#./build-SDL2-2.0.3-win.sh "Visual Studio $1 Win64"
#./extract-Eigen-3.2.2.sh
#cd ..

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
