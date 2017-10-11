/*
* Copyright (C) 2011 Florian Rathgeber, florian.rathgeber@gmail.com
*
* This code is licensed under the MIT License.  See the FindCUDA.cmake script
* for the text of the license.
*
* Based on code by Christopher Bruns published on Stack Overflow (CC-BY):
* http://stackoverflow.com/questions/2285185
*/

#include <stdio.h>
#include <cuda_runtime.h>

#include <iostream>
#include <set>
#include <sstream>

int main() {
    int deviceCount;
    std::set<std::string> computeCapabilities;

    if (cudaGetDeviceCount(&deviceCount) != cudaSuccess)
    {
        printf("Couldn't get device count: %s\n", cudaGetErrorString(cudaGetLastError()));
        return 1;
    }
    /* machines with no GPUs can still report one emulation device */
    for (int device = 0; device < deviceCount; ++device)
    {
        cudaDeviceProp currentProperties;
        cudaGetDeviceProperties(&currentProperties, device);
        if (currentProperties.major != 9999) {/* 9999 means emulation only */
            std::stringstream ss;
            ss << currentProperties.major;
            if(currentProperties.major == 2 && currentProperties.minor == 1)
            {
                ss << 0; // There is no compute_21 architecture.
            }
            else
            {
                ss << currentProperties.minor;
            }

            computeCapabilities.insert(ss.str());
        }
    }

    /* don't just return the number of gpus, because other runtime cuda
    errors can also yield non-zero return values */
    for(std::set<std::string>::const_iterator it = computeCapabilities.begin(); it != computeCapabilities.end(); ++it)
    {
        // Add a semicolon if we have already printed some output.
        if(it != computeCapabilities.begin()) std::cout << ';';
        std::cout << *it;
    }

    return computeCapabilities.size() == 0; /* 0 devices -> failure */
}
