#include <iostream>
#include <fstream>
#include <array>

#include "Eigen.h"
#include "VirtualSensor.h"

struct Vertex
{
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	// position stored as 4 floats (4th component is supposed to be 1.0)
	Vector4f position;
	// color stored as 4 unsigned char
	Vector4uc color;
};

float dist(Vertex v1, Vertex v2)
{
	return sqrt(pow(v1.position[0] - v2.position[0], 2) + pow(v1.position[1] - v2.position[1], 2) + pow(v1.position[2] - v2.position[2], 2));
}

bool IsFaceValid(Vertex v1, Vertex v2, Vertex v3, float edgeThreshold)
{

	if (v1.position[0] == MINF || v2.position[0] == MINF || v3.position[0] == MINF) return false;


	if (dist(v1, v2) > edgeThreshold || dist(v1, v3) > edgeThreshold || dist(v2, v3) > edgeThreshold) return false;


	return true;

}

bool WriteMesh(Vertex* vertices, unsigned int width, unsigned int height, const std::string& filename)
{
	float edgeThreshold = 0.01f; // 1cm

	// TODO 2: use the OFF file format to save the vertices grid (http://www.geomview.org/docs/html/OFF.html)
	// - have a look at the "off_sample.off" file to see how to store the vertices and triangles
	// - for debugging we recommend to first only write out the vertices (set the number of faces to zero)
	// - for simplicity write every vertex to file, even if it is not valid (position.x() == MINF) (note that all vertices in the off file have to be valid, thus, if a point is not valid write out a dummy point like (0,0,0))
	// - use a simple triangulation exploiting the grid structure (neighboring vertices build a triangle, two triangles per grid cell)
	// - you can use an arbitrary triangulation of the cells, but make sure that the triangles are consistently oriented
	// - only write triangles with valid vertices and an edge length smaller then edgeThreshold

	// TODO: Get number of vertices
	unsigned int nVertices = width*height;

	// TODO: Determine number of valid faces
	unsigned nFaces = 0;


	for (int i = 0; i < height - 1; i++)
	{
		for (int j = 0; j < width - 1; j++)
		{

			int index0 = i * width + j;
			int index1 = index0 + 1;
			int index2 = (i + 1) * width + j;
			int index3 = index2 + 1;

			if (IsFaceValid(vertices[index0], vertices[index1], vertices[index2], edgeThreshold))

				nFaces++;

			if (IsFaceValid(vertices[index1], vertices[index2], vertices[index3], edgeThreshold))

				nFaces++;

		}

	}

	// Write off file
	std::ofstream outFile(filename);
	if (!outFile.is_open()) return false;

	// write header
	outFile << "COFF" << std::endl;

	outFile << "# numVertices numFaces numEdges" << std::endl;

	outFile << nVertices << " " << nFaces << " 0" << std::endl;

	// TODO: save vertices
	for (int i = 0; i < height; i++)
	{
		for (int j = 0; j < width; j++)
		{

			int index = i * width + j;

			if (vertices[index].position[0] == MINF)
			{
				outFile << "0.0 0.0 0.0 0 0 0 0" << std::endl;
			}
			else
			{

				outFile << vertices[index].position[0] << " " << vertices[index].position[1] << " " << vertices[index].position[2] << " ";
				outFile << (int)vertices[index].color[0] << " " << (int)vertices[index].color[1] << " " << (int)vertices[index].color[2] << " " << (int)vertices[index].color[3] << std::endl;

			}

		}

	}

	// TODO: save valid faces
	std::cout << "# list of faces" << std::endl;
	std::cout << "# nVerticesPerFace idx0 idx1 idx2 ..." << std::endl;


	for (int i = 0; i < height - 1; i++)
	{
		for (int j = 0; j < width - 1; j++)
		{

			int index0 = i * width + j;
			int index1 = index0 + 1;
			int index2 = (i + 1) * width + j;
			int index3 = index2 + 1;

			if (IsFaceValid(vertices[index0], vertices[index1], vertices[index2], edgeThreshold))
				outFile << "3 " << index0 << " " << index2 << " " << index1 << std::endl;

			if (IsFaceValid(vertices[index1], vertices[index2], vertices[index3], edgeThreshold))
				outFile << "3 " << index2 << " " << index3 << " " << index1 << std::endl;
		}
	}

	// close file
	outFile.close();

	return true;
}

int main()
{
	// Make sure this path points to the data folder
	std::string filenameIn = "../../Data/rgbd_dataset_freiburg1_xyz/";
	std::string filenameBaseOut = "mesh_";

	// load video
	std::cout << "Initialize virtual sensor..." << std::endl;
	VirtualSensor sensor;
	if (!sensor.Init(filenameIn))
	{
		std::cout << "Failed to initialize the sensor!\nCheck file path!" << std::endl;
		return -1;
	}

	// convert video to meshes
	while (sensor.ProcessNextFrame())
	{
		// get ptr to the current depth frame
		// depth is stored in row major (get dimensions via sensor.GetDepthImageWidth() / GetDepthImageHeight())
		float* depthMap = sensor.GetDepth();
		// get ptr to the current color frame
		// color is stored as RGBX in row major (4 byte values per pixel, get dimensions via sensor.GetColorImageWidth() / GetColorImageHeight())
		BYTE* colorMap = sensor.GetColorRGBX();

		// get depth intrinsics
		Matrix3f depthIntrinsics = sensor.GetDepthIntrinsics();
		Matrix3f depthIntrinsicsInv = depthIntrinsics.inverse();

		float fX = depthIntrinsics(0, 0);
		float fY = depthIntrinsics(1, 1);
		float cX = depthIntrinsics(0, 2);
		float cY = depthIntrinsics(1, 2);

		// compute inverse depth extrinsics
		Matrix4f depthExtrinsicsInv = sensor.GetDepthExtrinsics().inverse();

		Matrix4f trajectory = sensor.GetTrajectory();
		Matrix4f trajectoryInv = sensor.GetTrajectory().inverse();

		// TODO 1: back-projection
		// write result to the vertices array below, keep pixel ordering!
		// if the depth value at idx is invalid (MINF) write the following values to the vertices array
		// vertices[idx].position = Vector4f(MINF, MINF, MINF, MINF);
		// vertices[idx].color = Vector4uc(0,0,0,0);
		// otherwise apply back-projection and transform the vertex to world space, use the corresponding color from the colormap
		Vertex* vertices = new Vertex[sensor.GetDepthImageWidth() * sensor.GetDepthImageHeight()];

		int color_image_width = sensor.GetColorImageWidth(); //640
		int color_image_height = sensor.GetColorImageHeight();//480

		for (int i = 0; i < color_image_height; i++)
		{
			for (int j = 0; j < color_image_width; j++)
			{
				//get the 1D index for 2D image
				int idx = i * color_image_width + j;

				//check if deopth value at idx is invalid
				if (depthMap[idx] == MINF)
				{

					vertices[idx].position = Vector4f(MINF, MINF, MINF, MINF);
					vertices[idx].color = Vector4uc(0, 0, 0, 0);

				}
				else {

					float x = ((float)j - cX) / fX;
					float y = ((float)i - cY) / fY;
					float z = depthMap[idx];
					Vector4f pos = Vector4f(x * z, y * z, z, 1.0);
					pos = trajectoryInv * depthExtrinsicsInv * pos;
					vertices[idx].position = pos;

					Vector4uc v_color = Vector4uc(0, 0, 0, 0);
					for (int t = 0; t < 4; t++)
						v_color[t] = colorMap[4 * idx + t];
					vertices[idx].color = v_color;

				}
			}
		}

		// write mesh file
		std::stringstream ss;
		ss << filenameBaseOut << sensor.GetCurrentFrameCnt() << ".off";
		if (!WriteMesh(vertices, sensor.GetDepthImageWidth(), sensor.GetDepthImageHeight(), ss.str()))
		{
			std::cout << "Failed to write mesh!\nCheck file path!" << std::endl;
			return -1;
		}

		// free mem
		delete[] vertices;
	}

	return 0;
}