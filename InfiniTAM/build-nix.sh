#! /bin/bash -e

# Check that a valid build type has been specified.
if [ $# -ne 2 ] || ([ "$1" != "Unix Makefiles" ] && [ "$1" != "Xcode" ]) || ([ $2 != "Debug" ] && [ $2 != "Release" ])
then
  echo "Usage: build-nix.sh {Unix Makefiles|Xcode} {Debug|Release}"
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

  # Note: We need to configure twice to handle conditional building.
  echo "[InfiniTAM] ...Configuring using CMake..."
  cmake -G"$1" -DCMAKE_BUILD_TYPE=$2 ..
  cmake ..

  cd ..
fi

cd build

echo "[InfiniTAM] ...Running build..."
make -j2

echo "[InfiniTAM] ...Finished building InfiniTAM."
