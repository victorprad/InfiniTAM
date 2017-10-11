#! /bin/bash -e

# Check that valid parameters have been specified.
if [ $# -ne 2 ] || ([ "$1" != "12" ] && [ "$1" != "14" ]) || ([ "$2" != "Debug" ] && [ "$2" != "Release" ])
then
  echo "Usage: build-realsense-win.sh {12|14} {Debug|Release}"
  exit
fi

# Clone the librealsense repository if it's not already present.
if [ ! -d librealsense ]
then
  /bin/rm -fR tmp
  git clone git@github.com:sgolodetz/librealsense.git tmp > /dev/null
  mv tmp librealsense
fi

# Build librealsense. If this fails with Visual Studio 2013, it's generally because you haven't installed VS Update 5.
cd librealsense/librealsense.vc$1
cmd //c "msbuild /p:Configuration=$2 /p:Platform=x64 realsense.sln"
