// main function to obtain the smoothing : use the output of the other one to know the weight

// save the normal => read it => perform the filtering => store the results as input for the filtering 

#include <iostream>
#include "fastBilateral.h"
#include "../Engine/ITMVisualisationEngine.h"
#include "../../Utils/FileUtils.h"

using namespace std;

#define MEAN_SIGMA_L 1.2232f
#define MEAN_SIGMA_COLOR 1.0f/2.0f  // difference between 2 pixel of the filtered map 

// for the normals 
_CPU_AND_GPU_CODE_ inline void depthBilaFilter(DEVICEPTR(Vector4f) *imageData_out, DEVICEPTR(Vector4f) *imageData_in,  int x, 
int y, Vector2i imDims, std::vector<uchar>& filtering_in)
{
	float z, dz, w, w_sum = 0.0f, tmpz, dc; int thisPos  = x + y * imDims.x ;
	Vector4f thisNormal, final_normal = Vector4f(0.0f, 0.0f, 0.0f, 0.0f);

    z = float(filtering_in[x + y * imDims.x]);

	// normals 
	thisNormal = imageData_in[x + y * imDims.x] ;
	if (thisNormal.w < 0.0f) {
		imageData_out[thisPos].x = imageData_out[thisPos].y = imageData_out[thisPos].z = 0.0f ; imageData_out[thisPos].w = -1.0f;
		return ;
	}

	for (int i = -2 ; i <= 2; i++) for (int j = -2; j <= 2; j++ )
	{
		//depth 
		tmpz = float(filtering_in[(x + j) + (y + i) * imDims.x]);
		if (tmpz < 0.0f) continue;
        
        // normal : 
		thisNormal = imageData_in[(x + j) + (y + i) * imDims.x] ;
		if (thisNormal.w < 0.0f) continue;

		// RGB weight 
        dz = (tmpz - z); dz *= dz;
        
        //std::cout << "dz : " << dz << " for tmpz : "<<tmpz << " z : " << z <<std::endl ;

        // only take into account the difference in intensity 
        w = exp(-0.5f * ( dz * MEAN_SIGMA_COLOR * MEAN_SIGMA_COLOR));
        //std::cout << "w : " << w <<std::endl ;
        
		w_sum += w;

		final_normal += w*thisNormal;
    }
    //std::cout << "w_sum : " << w_sum <<std::endl ;
    
	final_normal /= w_sum;

	// normalization in the direction 
	final_normal /= sqrt(final_normal.x*final_normal.x + final_normal.y*final_normal.y + final_normal.z*final_normal.z) ;
	final_normal.w = 1.0f ;
	imageData_out[thisPos] = final_normal;
}

// for the vertices 
_CPU_AND_GPU_CODE_ inline void verticesBilaFilter(DEVICEPTR(Vector4f) *imageData_out, DEVICEPTR(Vector4f) *imageData_in,  int x, 
int y, Vector2i imDims, std::vector<uchar>& filtering_in)
{
	float z, dz, w, w_sum = 0.0f, tmpz, dc; int thisPos  = x + y * imDims.x ;
	Vector4f thisVertex, final_vertex = Vector4f(0.0f, 0.0f, 0.0f, 0.0f);

    z = float(filtering_in[x + y * imDims.x]);

	// normals 
	thisVertex = imageData_in[x + y * imDims.x] ;
	if (thisVertex.w < 0.0f) {
		imageData_out[thisPos].x = imageData_out[thisPos].y = imageData_out[thisPos].z = 0.0f ; imageData_out[thisPos].w = -1.0f;
		return ;
	}

	for (int i = -2 ; i <= 2; i++) for (int j = -2; j <= 2; j++ )
	{
		//depth 
		tmpz = float(filtering_in[(x + j) + (y + i) * imDims.x]);
		if (tmpz < 0.0f) continue;
        
        // normal : 
		thisVertex = imageData_in[(x + j) + (y + i) * imDims.x] ;
		if (thisVertex.w < 0.0f) continue;

		// RGB weight 
        dz = (tmpz - z); dz *= dz;
        
        //std::cout << "dz : " << dz << " for tmpz : "<<tmpz << " z : " << z <<std::endl ;

        // only take into account the difference in intensity 
        w = exp(-0.5f * ( dz * MEAN_SIGMA_COLOR * MEAN_SIGMA_COLOR));
        //std::cout << "w : " << w <<std::endl ;
        
		w_sum += w;

		final_vertex += w*thisVertex;
    }
    //std::cout << "w_sum : " << w_sum <<std::endl ;
    
	final_vertex /= w_sum;

	// normalization in the direction 
	final_vertex.w = 1.0f ;
	imageData_out[thisPos] = final_vertex;
}

// get a depth image in to output a noise free image in return 
_CPU_AND_GPU_CODE_ inline void bilaFilter(ITMFloat4Image *image_out, ITMFloat4Image *image_in, Vector2i imgSize, int number, double sigmaColor, double sigmaSpace)
{
    // save the depth image 
    char str_in[250];

    // 0 for vertices, 1 for normals 
    if (number == 0) sprintf(str_in, "normals/verBefore_%d.ppm", number);
    else if (number == 1) sprintf(str_in, "normals/normBefore_%d.ppm", number);
	ITMUChar4Image *out_imageIn = new ITMUChar4Image(imgSize, true, false); //InfiniTAM_IMAGE_NORMAL_SEGMENTED
    ITMLib::Engine::ITMVisualisationEngine<ITMVoxel, ITMVoxelIndex>::NormalToUchar4(out_imageIn, image_in);
    SaveImageToFile(out_imageIn, str_in); 
    
    // read the depth image
    cv::Mat1b src = cv::imread(str_in, cv::IMREAD_GRAYSCALE);

    cv::imwrite("normals/normCV_B.png", src);

    // perform the filtering 
    cv::Mat1b dst ;
    cv_extend::bilateralFilter(src, dst, sigmaColor, sigmaSpace);
    cv_extend::bilateralFilter(dst, src, sigmaColor, sigmaSpace);
 
    // transform the image dst in a image for InfiniTam 
    std::vector<uchar> gray_filtered(src.rows*src.cols);
    gray_filtered.assign(src.datastart, src.dataend);;

    cv::imwrite("normals/normCV_A.png", src);
    
    // perform the filtering using the output of the previous algorithm 
    image_out->Clear();

    Vector4f *imout = image_out->GetData(MEMORYDEVICE_CPU);
	Vector4f *imin = image_in->GetData(MEMORYDEVICE_CPU);

    for (int y = 2; y < imgSize.y - 2; y++) for (int x = 2; x < imgSize.x - 2; x++) {
        if (number == 0) verticesBilaFilter(imout, imin, x, y, imgSize, gray_filtered);
        else if (number == 1) depthBilaFilter(imout, imin, x, y, imgSize, gray_filtered);
    }
    
    // save the result for comparaison 
    char str_out[250];
    if (number == 0) sprintf(str_out, "normals/verAfter_%d.ppm", number);
    else if (number == 1) sprintf(str_out, "normals/normAfter_%d.ppm", number);
    ITMUChar4Image *out_ImageOut = new ITMUChar4Image(image_in->noDims, true, false); //InfiniTAM_IMAGE_NORMAL_SEGMENTED
    ITMLib::Engine::ITMVisualisationEngine<ITMVoxel, ITMVoxelIndex>::NormalToUchar4(out_ImageOut, image_out);
    SaveImageToFile(out_ImageOut, str_out); 

    // memory management 
    //delete outFilter ; 
    delete out_ImageOut ; delete out_imageIn ;
}