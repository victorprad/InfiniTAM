// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Scene/ITMVoxelBlockHash.h"
#include "../../../ORUtils/Image.h"

#include <stdlib.h>

namespace ITMLib
{
	class ITMMesh
	{
	public:
		struct Triangle
		{
			Vector4u c0, c1, c2;
			Vector3f p0, p1, p2;
		};

		MemoryDeviceType memoryType;

		uint noTotalTriangles;
		static const uint noMaxTriangles_default = SDF_LOCAL_BLOCK_NUM * 32 * 16;
		uint noMaxTriangles;

		ORUtils::MemoryBlock<Triangle> *triangles;

		explicit ITMMesh(MemoryDeviceType memoryType, uint maxTriangles = noMaxTriangles_default)
		{
			this->memoryType = memoryType;
			this->noTotalTriangles = 0;
			this->noMaxTriangles = maxTriangles;

			triangles = new ORUtils::MemoryBlock<Triangle>(noMaxTriangles, memoryType);
		}

		void WriteOBJ(const char *fileName)
		{
			ORUtils::MemoryBlock<Triangle> *cpu_triangles;
			bool shouldDelete = false;

			if (memoryType == MEMORYDEVICE_CUDA)
			{
				cpu_triangles = new ORUtils::MemoryBlock<Triangle>(noMaxTriangles, MEMORYDEVICE_CPU);
				cpu_triangles->SetFrom(triangles, ORUtils::MemoryBlock<Triangle>::CUDA_TO_CPU);
				shouldDelete = true;
			}
			else cpu_triangles = triangles;

			Triangle *triangleArray = cpu_triangles->GetData(MEMORYDEVICE_CPU);

			FILE *f = fopen(fileName, "w+");
			if (f != NULL)
			{
				for (uint i = 0; i < noTotalTriangles; i++)
				{
					fprintf(f, "v %f %f %f %f %f %f\n", triangleArray[i].p0.x, triangleArray[i].p0.y, triangleArray[i].p0.z, triangleArray[i].c0.x / 255.0f, triangleArray[i].c0.y / 255.0f, triangleArray[i].c0.z / 255.0f);
					fprintf(f, "v %f %f %f %f %f %f\n", triangleArray[i].p1.x, triangleArray[i].p1.y, triangleArray[i].p1.z, triangleArray[i].c1.x / 255.0f, triangleArray[i].c1.y / 255.0f, triangleArray[i].c1.z / 255.0f);
					fprintf(f, "v %f %f %f %f %f %f\n", triangleArray[i].p2.x, triangleArray[i].p2.y, triangleArray[i].p2.z, triangleArray[i].c2.x / 255.0f, triangleArray[i].c2.y / 255.0f, triangleArray[i].c2.z / 255.0f);
				}

				for (uint i = 0; i<noTotalTriangles; i++) fprintf(f, "f %d %d %d\n", i * 3 + 2 + 1, i * 3 + 1 + 1, i * 3 + 0 + 1);
				fclose(f);
			}

			if (shouldDelete) delete cpu_triangles;
		}

		void WritePLY(const char *fileName)
		{
			ORUtils::MemoryBlock<Triangle> *cpu_triangles;
			bool shouldDelete = false;

			if (memoryType == MEMORYDEVICE_CUDA)
			{
				cpu_triangles = new ORUtils::MemoryBlock<Triangle>(noMaxTriangles, MEMORYDEVICE_CPU);
				cpu_triangles->SetFrom(triangles, ORUtils::MemoryBlock<Triangle>::CUDA_TO_CPU);
				shouldDelete = true;
			}
			else cpu_triangles = triangles;

			Triangle *triangleArray = cpu_triangles->GetData(MEMORYDEVICE_CPU);

			// Open the file in binary mode to prevent the insertion of \r\n on new lines.
			// The PLY format wants raw "\n" as separators.
			FILE *f = fopen(fileName, "wb");
			if (f != NULL)
			{
				fprintf(f, "ply\n");
				fprintf(f, "format binary_little_endian 1.0\n");
				fprintf(f, "comment created with InfiniTAM\n");
				fprintf(f, "element vertex %d\n", noTotalTriangles * 3);
				fprintf(f, "property float x\n");
				fprintf(f, "property float y\n");
				fprintf(f, "property float z\n");
				fprintf(f, "property uchar red\n");
				fprintf(f, "property uchar green\n");
				fprintf(f, "property uchar blue\n");
				fprintf(f, "element face %d\n", noTotalTriangles);
				fprintf(f, "property list int int vertex_index\n");
				fprintf(f, "end_header\n");

				// Write the vertices.
				for (uint i = 0; i < noTotalTriangles; ++i)
				{
					const Triangle& triangle = triangleArray[i];
					fwrite(triangle.p0.v, sizeof(float), 3, f);
					fwrite(triangle.c0.v, sizeof(uchar), 3, f);
					fwrite(triangle.p1.v, sizeof(float), 3, f);
					fwrite(triangle.c1.v, sizeof(uchar), 3, f);
					fwrite(triangle.p2.v, sizeof(float), 3, f);
					fwrite(triangle.c2.v, sizeof(uchar), 3, f);
				}

				// Write the triangles.
				for (int i = 0; i < static_cast<int>(noTotalTriangles); ++i)
				{
					const int indices[] = { 3, i * 3 + 2, i * 3 + 1, i * 3 + 0 };
					fwrite(indices, sizeof(int), 4, f);
				}

				fclose(f);
			}

			if (shouldDelete) delete cpu_triangles;
		}

		void WriteSTL(const char *fileName)
		{
			ORUtils::MemoryBlock<Triangle> *cpu_triangles;
			bool shouldDelete = false;

			if (memoryType == MEMORYDEVICE_CUDA)
			{
				cpu_triangles = new ORUtils::MemoryBlock<Triangle>(noMaxTriangles, MEMORYDEVICE_CPU);
				cpu_triangles->SetFrom(triangles, ORUtils::MemoryBlock<Triangle>::CUDA_TO_CPU);
				shouldDelete = true;
			}
			else cpu_triangles = triangles;

			Triangle *triangleArray = cpu_triangles->GetData(MEMORYDEVICE_CPU);

			FILE *f = fopen(fileName, "wb+");

			if (f != NULL) {
				for (int i = 0; i < 80; i++) fwrite(" ", sizeof(char), 1, f);

				fwrite(&noTotalTriangles, sizeof(int), 1, f);

				float zero = 0.0f; short attribute = 0;
				for (uint i = 0; i < noTotalTriangles; i++)
				{
					fwrite(&zero, sizeof(float), 1, f); fwrite(&zero, sizeof(float), 1, f); fwrite(&zero, sizeof(float), 1, f);

					fwrite(&triangleArray[i].p2.x, sizeof(float), 1, f); 
					fwrite(&triangleArray[i].p2.y, sizeof(float), 1, f); 
					fwrite(&triangleArray[i].p2.z, sizeof(float), 1, f);

					fwrite(&triangleArray[i].p1.x, sizeof(float), 1, f); 
					fwrite(&triangleArray[i].p1.y, sizeof(float), 1, f); 
					fwrite(&triangleArray[i].p1.z, sizeof(float), 1, f);

					fwrite(&triangleArray[i].p0.x, sizeof(float), 1, f);
					fwrite(&triangleArray[i].p0.y, sizeof(float), 1, f);
					fwrite(&triangleArray[i].p0.z, sizeof(float), 1, f);

					fwrite(&attribute, sizeof(short), 1, f);

					//fprintf(f, "v %f %f %f\n", triangleArray[i].p0.x, triangleArray[i].p0.y, triangleArray[i].p0.z);
					//fprintf(f, "v %f %f %f\n", triangleArray[i].p1.x, triangleArray[i].p1.y, triangleArray[i].p1.z);
					//fprintf(f, "v %f %f %f\n", triangleArray[i].p2.x, triangleArray[i].p2.y, triangleArray[i].p2.z);
				}

				//for (uint i = 0; i<noTotalTriangles; i++) fprintf(f, "f %d %d %d\n", i * 3 + 2 + 1, i * 3 + 1 + 1, i * 3 + 0 + 1);
				fclose(f);
			}

			if (shouldDelete) delete cpu_triangles;
		}

		~ITMMesh()
		{
			delete triangles;
		}

		// Suppress the default copy constructor and assignment operator
		ITMMesh(const ITMMesh&);
		ITMMesh& operator=(const ITMMesh&);
	};
}
