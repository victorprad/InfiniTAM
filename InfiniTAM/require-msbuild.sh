#! /bin/bash

result=`cmd //c msbuild 2>&1`
if [[ $result == *"not recognized"* ]]
then
  echo "Error: msbuild not found. You probably need to add it to your system path."
  echo "The VS2013 version is generally in C:\Program Files (x86)\MSBuild\12.0\Bin."
  echo "The VS2015 version is generally in C:\Program Files (x86)\MSBuild\14.0\Bin."
  exit 1
fi
