#! /bin/bash -e

# Check that a valid build type has been specified.
if [ $# -ne 2 ] || ([ "$1" != "Ninja" ] && [ "$1" != "Unix Makefiles" ] && [ "$1" != "Xcode" ]) || ([ $2 != "Debug" ] && [ $2 != "Release" ])
then
  echo "Usage: build-nix.sh {Ninja|Unix Makefiles|Xcode} {Debug|Release}"
  exit
fi

# Detect whether this is being run on Linux or Mac OS X.
PLATFORM=linux
if [ "$(uname)" == "Darwin" ]
then
  PLATFORM=mac
fi

# Build InfiniTAM itself.
echo "[InfiniTAM] Building InfiniTAM"

if [ ! -d build ]
then
  mkdir build
  cd build

  echo "[InfiniTAM] ...Configuring using CMake..."

  if [ "$1" == "Ninja" ] && [ $PLATFORM == "mac" ]
  then
    cmake -G"$1" -DCMAKE_BUILD_TYPE=$2 -DCMAKE_MAKE_PROGRAM="/usr/local/Cellar/ninja/1.7.2/bin/ninja" ..
  else
    cmake -G"$1" -DCMAKE_BUILD_TYPE=$2 ..
  fi

  # Note: We need to configure twice to handle conditional building.
  cmake ..

  cd ..
fi

cd build

echo "[InfiniTAM] ...Running build..."

if [ "$1" == "Ninja" ]
then
  ninja
else
  make -j2
fi

echo "[InfiniTAM] ...Finished building InfiniTAM."
