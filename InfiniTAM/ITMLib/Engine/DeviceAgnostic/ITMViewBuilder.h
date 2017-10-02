// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../Utils/ITMLibDefines.h"
#include "../../../ORUtils/Matrix.h"


_CPU_AND_GPU_CODE_ inline void convertDisparityToDepth(DEVICEPTR(float) *d_out, int x, int y, const CONSTPTR(short) *d_in,
	Vector2f disparityCalibParams, float fx_depth, Vector2i imgSize)
{
	int locId = x + y * imgSize.x;

	short disparity = d_in[locId];
	float disparity_tmp = disparityCalibParams.x - (float)(disparity);
	float depth;

	if (disparity_tmp == 0) depth = 0.0;
	else depth = 8.0f * disparityCalibParams.y * fx_depth / disparity_tmp;

	d_out[locId] = (depth > 0) ? depth : -1.0f;
}

_CPU_AND_GPU_CODE_ inline void convertDepthAffineToFloat(DEVICEPTR(float) *d_out, int x, int y, const CONSTPTR(short) *d_in, Vector2i imgSize, Vector2f depthCalibParams)
{
	int locId = x + y * imgSize.x;

	short depth_in = d_in[locId];
	d_out[locId] = ((depth_in <= 0)||(depth_in > 32000)) ? -1.0f : (float)depth_in * depthCalibParams.x + depthCalibParams.y;
}

#define MEAN_SIGMA_L 1.2232f
_CPU_AND_GPU_CODE_ inline void filterDepth(DEVICEPTR(float) *imageData_out, const CONSTPTR(float) *imageData_in, int x, int y, Vector2i imgDims)
{
	float z, tmpz, dz, final_depth = 0.0f, w, w_sum = 0.0f;

	z = imageData_in[x + y * imgDims.x];
	if (z < 0.0f) { imageData_out[x + y * imgDims.x] = -1.0f; return; }

	float sigma_z = 1.0f / (0.0012f + 0.0019f*(z - 0.4f)*(z - 0.4f) + 0.0001f / sqrt(z) * 0.25f);

	for (int i = -2, count = 0; i <= 2; i++) for (int j = -2; j <= 2; j++, count++)
	{
		tmpz = imageData_in[(x + j) + (y + i) * imgDims.x];
		if (tmpz < 0.0f) continue;
		dz = (tmpz - z); dz *= dz;
		w = exp(-0.5f * ((abs(i) + abs(j))*MEAN_SIGMA_L*MEAN_SIGMA_L + dz * sigma_z * sigma_z));
		w_sum += w;
		final_depth += w*tmpz;
	}

	final_depth /= w_sum;
	imageData_out[x + y*imgDims.x] = final_depth;
}

// try to filter and adding RGB information 
#define MEAN_SIGMA_COLOR  1.0f/100.0f // sigma color = 10 
_CPU_AND_GPU_CODE_ inline void filterDepthWithRGB(DEVICEPTR(float) *imageData_out, const CONSTPTR(float) *imageData_in, const CONSTPTR(Vector4u) *rgbData_in, int x, int y, Vector2i imgDims)
{
	float z, tmpz, dz, dc, final_depth = 0.0f, w, w_sum = 0.0f;
	Vector3f thisColor, color_cur ;
	z = imageData_in[x + y * imgDims.x];
	if (z < 0.0f) { imageData_out[x + y * imgDims.x] = -1.0f; return; }

	// for the depth information 
	float sigma_z = 1.0f / (0.0012f + 0.0019f*(z - 0.4f)*(z - 0.4f) + 0.0001f / sqrt(z) * 0.25f);

	// for the color information 
	thisColor.x = float(rgbData_in[x + y * imgDims.x].x) ; thisColor.y = float(rgbData_in[x + y * imgDims.x].y) ;
	thisColor.z = float(rgbData_in[x + y * imgDims.x].z) ;

	for (int i = -2, count = 0; i <= 2; i++) for (int j = -2; j <= 2; j++, count++)
	{
		tmpz = imageData_in[(x + j) + (y + i) * imgDims.x];
		color_cur.x = float(rgbData_in[(x + j) + (y + i) * imgDims.x].x) ; color_cur.y = float(rgbData_in[(x + j) + (y + i) * imgDims.x].y) ;
		color_cur.z = float(rgbData_in[(x + j) + (y + i) * imgDims.x].z) ;

		if (tmpz < 0.0f) continue;
		dz = (tmpz - z); dz *= dz;

		// color differences 
		dc = (thisColor.x - color_cur.x + thisColor.y - color_cur.y + thisColor.z - color_cur.z );
		//std::cout << dc << std::endl ;
		dc *= dc ;

		//std::cout << "distance : " << (abs(i) + abs(j))*MEAN_SIGMA_L*MEAN_SIGMA_L << " depth : " <<  dz * sigma_z * sigma_z << " color : " <<  dc * MEAN_SIGMA_COLOR*MEAN_SIGMA_COLOR << std::endl ;
		w = exp(-0.5f * ((abs(i) + abs(j))*MEAN_SIGMA_L*MEAN_SIGMA_L + dz * sigma_z * sigma_z + dc * MEAN_SIGMA_COLOR*MEAN_SIGMA_COLOR));
		w_sum += w;
		final_depth += w*tmpz;
	}

	final_depth /= w_sum;
	imageData_out[x + y*imgDims.x] = final_depth;
}

_CPU_AND_GPU_CODE_ inline void filterVertexWithRGB(DEVICEPTR(Vector4f) *imageData_out, const CONSTPTR(Vector4f) *imageData_in, const CONSTPTR(Vector4u) *rgbData_in, int x, 
int y, Vector2i imDims,  const CONSTPTR(float) *depth_in)
{
	float z, w, w_sum = 0.0f, tmpz, dz, dc; 
	int thisPos  = x + y * imDims.x ;
	Vector3f thisVertex, current_vertex, final_vertex = Vector3f(0.0f, 0.0f, 0.0f), thisColor, color_cur ;
	
	// check the validity of the vertex 
	z = depth_in[x + y * imDims.x];
	if (z < 0.0f || imageData_in[x + y * imDims.x].w < 0.0f) {
		imageData_out[thisPos].x = imageData_out[thisPos].y = imageData_out[thisPos].z = 0.0f ; imageData_out[thisPos].w = -1.0f;
		return ;
	}

	float sigma_z = 1.0f / (0.0012f + 0.0019f*(z - 0.4f)*(z - 0.4f) + 0.0001f / sqrt(z) * 0.25f);
	
	// information for this vertex 
	thisVertex.x = imageData_in[x + y* imDims.x].x ; thisVertex.y = imageData_in[x + y* imDims.x].y ; thisVertex.z = imageData_in[x + y* imDims.x].z ;
	thisColor.x = float(rgbData_in[x + y * imDims.x].x) ; thisColor.y = float(rgbData_in[x + y * imDims.x].y) ;
	thisColor.z = float(rgbData_in[x + y * imDims.x].z) ;
	
	for (int i = -2 ; i <= 2; i++) for (int j = -2; j <= 2; j++ )
	{
		// current vertex info 
		tmpz = depth_in[(x + j) + (y + i) * imDims.x]; if (tmpz < 0.0f) continue;		
		color_cur.x = float(rgbData_in[(x + j) + (y + i) * imDims.x].x) ; color_cur.y = float(rgbData_in[(x + j) + (y + i) * imDims.x].y) ;
		color_cur.z = float(rgbData_in[(x + j) + (y + i) * imDims.x].z) ;

		// distance weight 
		dz = (tmpz - z); dz *= dz;
		
		// RGB weight 
		dc = (thisColor.x - color_cur.x + thisColor.y - color_cur.y + thisColor.z - color_cur.z );
		dc *= dc ;

		current_vertex.x = imageData_in[(x + j) + (y + i) * imDims.x].x ; current_vertex.y = imageData_in[(x + j) + (y + i) * imDims.x].y ; current_vertex.z = imageData_in[(x + j) + (y + i) * imDims.x].z ;
		if (imageData_in[(x + j) + (y + i) * imDims.x].w < 0.0f) continue;

		w = exp(-0.5f * ((std::abs(i) + std::abs(j))*MEAN_SIGMA_L*MEAN_SIGMA_L + dz * sigma_z * sigma_z + dc * MEAN_SIGMA_COLOR*MEAN_SIGMA_COLOR));
		w_sum += w;
		final_vertex += w*current_vertex;
	}

	final_vertex /= w_sum;

	imageData_out[thisPos].x = final_vertex.x;	imageData_out[thisPos].y = final_vertex.y;	imageData_out[thisPos].z = final_vertex.z ;
	imageData_out[thisPos].w = 1.0f;
}

_CPU_AND_GPU_CODE_ inline void depthNormalFilter(DEVICEPTR(Vector4f) *imageData_out, const CONSTPTR(Vector4f) *imageData_in, int x, 
int y, Vector2i imDims,  const CONSTPTR(float) *depth_in)
{
	float z, dz, w, w_sum = 0.0f, tmpz; int thisPos  = x + y * imDims.x ;
	Vector4f thisNormal, current_Normal, final_normal = Vector4f(0.0f, 0.0f, 0.0f, 0.0f);

	z = depth_in[x + y * imDims.x];
	if (z < 0.0f) {
		imageData_out[thisPos].x = imageData_out[thisPos].y = imageData_out[thisPos].z = 0.0f ; imageData_out[thisPos].w = -1.0f;
		return ;
	}

	thisNormal = imageData_in[x + y * imDims.x] ;
	if (thisNormal.w < 0.0f) {
		imageData_out[thisPos].x = imageData_out[thisPos].y = imageData_out[thisPos].z = 0.0f ; imageData_out[thisPos].w = -1.0f;
		return ;
	}

	float sigma_z = 1.0f / (0.0012f + 0.0019f*(z - 0.4f)*(z - 0.4f) + 0.0001f / sqrt(z) * 0.25f);

	for (int i = -2 ; i <= 2; i++) for (int j = -2; j <= 2; j++ )
	{
		// compute the weight for the bilateral filter 
		tmpz = depth_in[(x + j) + (y + i) * imDims.x];
		if (tmpz < 0.0f) continue;

		// for the normal : 
		thisNormal = imageData_in[(x + j) + (y + i) * imDims.x] ;
		if (thisNormal.w < 0.0f) continue;

		dz = (tmpz - z); dz *= dz;
		w = exp(-0.5f * ((std::abs(i) + std::abs(j))*MEAN_SIGMA_L*MEAN_SIGMA_L + dz * sigma_z * sigma_z));
		w_sum += w;

		final_normal += w*thisNormal;
	}
	final_normal /= w_sum;

	// normalization in the direction 
	final_normal /= sqrt(final_normal.x*final_normal.x + final_normal.y*final_normal.y + final_normal.z*final_normal.z) ;
	final_normal.w = 1.0f ;
	imageData_out[thisPos] = final_normal;
}

_CPU_AND_GPU_CODE_ inline void normalFilterWithRGB(DEVICEPTR(Vector4f) *imageData_out, const CONSTPTR(Vector4f) *imageData_in, const CONSTPTR(Vector4u) *rgbData_in, int x, 
int y, Vector2i imDims,  const CONSTPTR(float) *depth_in)
{
	float z, dz, w, w_sum = 0.0f, tmpz, dc; int thisPos  = x + y * imDims.x ;
	Vector4f thisNormal, current_Normal, final_normal = Vector4f(0.0f, 0.0f, 0.0f, 0.0f);
	Vector3f thisColor, color_cur ;

	// depth 
	z = depth_in[x + y * imDims.x];
	if (z < 0.0f) {
		imageData_out[thisPos].x = imageData_out[thisPos].y = imageData_out[thisPos].z = 0.0f ; imageData_out[thisPos].w = -1.0f;
		return ;
	}
	// normals 
	thisNormal = imageData_in[x + y * imDims.x] ;
	if (thisNormal.w < 0.0f) {
		imageData_out[thisPos].x = imageData_out[thisPos].y = imageData_out[thisPos].z = 0.0f ; imageData_out[thisPos].w = -1.0f;
		return ;
	}
	// color info 
	thisColor.x = float(rgbData_in[x + y * imDims.x].x) ; thisColor.y = float(rgbData_in[x + y * imDims.x].y) ;
	thisColor.z = float(rgbData_in[x + y * imDims.x].z) ;
	
	float sigma_z = 1.0f / (0.0012f + 0.0019f*(z - 0.4f)*(z - 0.4f) + 0.0001f / sqrt(z) * 0.25f);

	for (int i = -2 ; i <= 2; i++) for (int j = -2; j <= 2; j++ )
	{
		//depth 
		tmpz = depth_in[(x + j) + (y + i) * imDims.x];
		if (tmpz < 0.0f) continue;
		// normal : 
		thisNormal = imageData_in[(x + j) + (y + i) * imDims.x] ;
		if (thisNormal.w < 0.0f) continue;
		// color 
		color_cur.x = float(rgbData_in[(x + j) + (y + i) * imDims.x].x) ; color_cur.y = float(rgbData_in[(x + j) + (y + i) * imDims.x].y) ;
		color_cur.z = float(rgbData_in[(x + j) + (y + i) * imDims.x].z) ;

		// distance weight 
		dz = (tmpz - z); dz *= dz;
		
		// RGB weight 
		dc = (thisColor.x - color_cur.x + thisColor.y - color_cur.y + thisColor.z - color_cur.z );
		dc *= dc ;

		w = exp(-0.5f * ((std::abs(i) + std::abs(j))*MEAN_SIGMA_L*MEAN_SIGMA_L + dz * sigma_z * sigma_z) + dc * MEAN_SIGMA_COLOR*MEAN_SIGMA_COLOR );
		w_sum += w;

		final_normal += w*thisNormal;
	}
	final_normal /= w_sum;

	// normalization in the direction 
	final_normal /= sqrt(final_normal.x*final_normal.x + final_normal.y*final_normal.y + final_normal.z*final_normal.z) ;
	final_normal.w = 1.0f ;
	imageData_out[thisPos] = final_normal;
}

_CPU_AND_GPU_CODE_ inline void vertexFilter(DEVICEPTR(Vector4f) *imageData_out, const CONSTPTR(Vector4f) *imageData_in, int x, 
int y, Vector2i imDims,  const CONSTPTR(float) *depth_in)
{
	float z, distance, w, w_sum = 0.0f, tmpz, dz; int thisPos  = x + y * imDims.x ;
	Vector3f thisVertex, current_vertex, final_vertex = Vector3f(0.0f, 0.0f, 0.0f);
	
	// check the validity of the vertex 
	z = depth_in[x + y * imDims.x];
	if (z < 0.0f || imageData_in[x + y * imDims.x].w < 0.0f) {
		imageData_out[thisPos].x = imageData_out[thisPos].y = imageData_out[thisPos].z = 0.0f ; imageData_out[thisPos].w = -1.0f;
		return ;
	}

	float sigma_z = 1.0f / (0.0012f + 0.0019f*(z - 0.4f)*(z - 0.4f) + 0.0001f / sqrt(z) * 0.25f);
	thisVertex.x = imageData_in[x + y* imDims.x].x ; thisVertex.y = imageData_in[x + y* imDims.x].y ; thisVertex.z = imageData_in[x + y* imDims.x].z ;

	for (int i = -2 ; i <= 2; i++) for (int j = -2; j <= 2; j++ )
	{
		// compute the weight for the bilateral filter 
		tmpz = depth_in[(x + j) + (y + i) * imDims.x];
		if (tmpz < 0.0f) continue;

		current_vertex.x = imageData_in[(x + j) + (y + i) * imDims.x].x ; current_vertex.y = imageData_in[(x + j) + (y + i) * imDims.x].y ; current_vertex.z = imageData_in[(x + j) + (y + i) * imDims.x].z ;
		if (imageData_in[(x + j) + (y + i) * imDims.x].w < 0.0f) continue;

		dz = (tmpz - z); dz *= dz;
		w = exp(-0.5f * ((std::abs(i) + std::abs(j))*MEAN_SIGMA_L*MEAN_SIGMA_L + dz * sigma_z * sigma_z));
		w_sum += w;

		final_vertex += w*current_vertex;
	}
	final_vertex /= w_sum;

	imageData_out[thisPos].x = final_vertex.x;	imageData_out[thisPos].y = final_vertex.y;	imageData_out[thisPos].z = final_vertex.z ;
	imageData_out[thisPos].w = 1.0f;
}


_CPU_AND_GPU_CODE_ inline void computeNormalAndWeight(const CONSTPTR(float) *depth_in, DEVICEPTR(Vector4f) *normal_out,DEVICEPTR(Vector4f) *vertex_out, 
	DEVICEPTR(float) *sigmaZ_out, int x, int y, Vector2i imgDims, Vector4f intrinparam)
{
	/*Vector3f outNormal ;

	int idx = x + y*imgDims.x;

	double z = depth_in[x + y * imgDims.x];
	
	if (z < 0.0f)
	{
		normal_out[idx].x = normal_out[idx].y = normal_out[idx].z = 0.0f ;
		normal_out[idx].w = -1.0f;
		sigmaZ_out[idx] = -1;
		vertex_out[idx].x = vertex_out[idx].y = vertex_out[idx].z = 0.0f ;
		vertex_out[idx].w = -1.0f ;
		return;
	}
	
	// ************** START THEIR CODE **************
	// first compute the normal to compare : THEIR CODE 
	Vector3d xp1_y, xm1_y, x_yp1, x_ym1;
	Vector3d diff_x(0.0, 0.0, 0.0), diff_y(0.0, 0.0, 0.0);

	xp1_y.z = double(depth_in[(x + 1) + y * imgDims.x]), x_yp1.z = double(depth_in[x + (y + 1) * imgDims.x]);
	xm1_y.z = double(depth_in[(x - 1) + y * imgDims.x]), x_ym1.z = double(depth_in[x + (y - 1) * imgDims.x]);

	if (xp1_y.z <= 0 || x_yp1.z <= 0 || xm1_y.z <= 0 || x_ym1.z <= 0)
	{
		normal_out[idx].x = normal_out[idx].y = normal_out[idx].z = 0.0f ;
		normal_out[idx].w = -1.0f;
		sigmaZ_out[idx] = -1;
		vertex_out[idx].x = vertex_out[idx].y = vertex_out[idx].z = 0.0f ;
		vertex_out[idx].w = -1.0f ;
		return;
	}

	// unprojected
	xp1_y.x = xp1_y.z * ((x + 1.0f) - intrinparam.z) * intrinparam.x; xp1_y.y = xp1_y.z * (y - intrinparam.w) * intrinparam.y;
	xm1_y.x = xm1_y.z * ((x - 1.0f) - intrinparam.z) * intrinparam.x; xm1_y.y = xm1_y.z * (y - intrinparam.w) * intrinparam.y;
	x_yp1.x = x_yp1.z * (x - intrinparam.z) * intrinparam.x; x_yp1.y = x_yp1.z * ((y + 1.0f) - intrinparam.w) * intrinparam.y;
	x_ym1.x = x_ym1.z * (x - intrinparam.z) * intrinparam.x; x_ym1.y = x_ym1.z * ((y - 1.0f) - intrinparam.w) * intrinparam.y;
	
	// normalization 
	xp1_y.x /= sqrt(xp1_y.x*xp1_y.x + xp1_y.y*xp1_y.y), xp1_y.y /= sqrt(xp1_y.x*xp1_y.x + xp1_y.y*xp1_y.y) ;
	xm1_y.x /= sqrt(xm1_y.x*xm1_y.x + xm1_y.y*xm1_y.y), xm1_y.y /= sqrt(xm1_y.x*xm1_y.x + xm1_y.y*xm1_y.y) ;
	x_yp1.x /= sqrt(x_yp1.x*x_yp1.x + x_yp1.y*x_yp1.y), x_yp1.y /= sqrt(x_yp1.x*x_yp1.x + x_yp1.y*x_yp1.y) ;
	x_ym1.x /= sqrt(x_ym1.x*x_ym1.x + x_ym1.y*x_ym1.y), x_ym1.y /= sqrt(x_ym1.x*x_ym1.x + x_ym1.y*x_ym1.y) ;

	// gradients x and y
	diff_x = xp1_y - xm1_y, diff_y = x_yp1 - x_ym1;

	// cross product
	outNormal.x = float(diff_x.y * diff_y.z - diff_x.z*diff_y.y);
	outNormal.y = float(diff_x.z * diff_y.x - diff_x.x*diff_y.z);
	outNormal.z = float(diff_x.x * diff_y.y - diff_x.y*diff_y.x);

	if (outNormal.x == 0.0f && outNormal.y == 0 && outNormal.z == 0)
	{
		normal_out[idx].x = normal_out[idx].y = normal_out[idx].z = 0.0f ;
		normal_out[idx].w = -1.0f;
		sigmaZ_out[idx] = -1;
		vertex_out[idx].x = vertex_out[idx].y = vertex_out[idx].z = 0.0f ;
		vertex_out[idx].w = -1.0f ;
		return;
	}

    float norm = 1.0f / sqrt(outNormal.x * outNormal.x + outNormal.y * outNormal.y + outNormal.z * outNormal.z);
    outNormal *= norm;

	// the normal + vertex computed to compare  
	normal_out[idx].x = outNormal.x; normal_out[idx].y = outNormal.y; normal_out[idx].z = outNormal.z; normal_out[idx].w = 1.0f;
	
	// ******* Start MY CODE ********

	// compute the vertices 
	Matrix3f calib = Matrix3f(intrinparam.x, 0.0f, 0.0f,0.0f, intrinparam.y,0.0f, intrinparam.z, intrinparam.w , 1.0f) ;
	Matrix3f calib_inv ;
	calib.inv(calib_inv) ;
	
	Vector3f thisVertex , thisVec ;
	thisVec.x = x ; thisVec.y = y ; thisVec.z = 1.0f ;  

	thisVertex = calib_inv*thisVec*double(depth_in[x + y * imgDims.x]) ;

	//vertex_out[idx].x = thisVertex.x; vertex_out[idx].y = thisVertex.y; vertex_out[idx].z = thisVertex.z; vertex_out[idx].w = 1.0f; 

	// fill in the vectors in image plane 
	Vector3f vec1, vec2, vec3 ; 
	thisVec.x = x ; thisVec.y = y ; thisVec.z = 1.0f ;  

	vec1.x = x-2 ; vec1.y = y+2 ; vec2.x = x+2 ; vec2.y = y+2 ; vec3.x = x ; vec3.y = y-2 ; 	
	vec1.z = 1.0f; vec2.z = 1.0f; vec3.z = 1.0f;

	// deduce the vertices :
	Vector3f vertex1, vertex2, vertex3; 

	vertex1 = calib_inv*vec1*double(depth_in[x-2 + (y+2) * imgDims.x]) ;	
	vertex2 = calib_inv*vec2* double(depth_in[(x+2) + (y+2)* imgDims.x]) ;
	vertex3 = calib_inv*vec3*double(depth_in[x + (y-2) * imgDims.x]) ;

	// NEED TO SMOOTH THE VERTEX MAP BEFORE COMPUTING THE NORMAL MAP !!! 

	// determine the difference vectors 
	Vector3f diff_x2(0.0f, 0.0f, 0.0f), diff_y2(0.0f, 0.0f, 0.0f) ;
	
	diff_x2 =  vertex2-vertex1 ; diff_y2 =  vertex3-vertex1;

	// cross product
	outNormal.x = float(diff_x2.y * diff_y2.z - diff_x2.z*diff_y2.y);
	outNormal.y = float(diff_x2.z * diff_y2.x - diff_x2.x*diff_y2.z);
	outNormal.z = float(diff_x2.x * diff_y2.y - diff_x2.y*diff_y2.x);

	if (outNormal.x == 0.0f && outNormal.y == 0.0f && outNormal.z == 0.0f)
	{
		normal_out[idx].x = normal_out[idx].y = normal_out[idx].z = 0.0f ;
		normal_out[idx].w = -1.0f;
		sigmaZ_out[idx] = -1;
		vertex_out[idx].x = vertex_out[idx].y = vertex_out[idx].z = 0.0f ;
		vertex_out[idx].w = -1.0f ;
		return;
	}

	// normalization 
	float norm = 1.0f / sqrt(outNormal.x * outNormal.x + outNormal.y * outNormal.y + outNormal.z * outNormal.z);
    outNormal *= norm;

	// fill in the normal and the vertex vertex_out
	normal_out[idx].x = outNormal.x; normal_out[idx].y = outNormal.y; normal_out[idx].z = outNormal.z; normal_out[idx].w = 1.0f;
	vertex_out[idx].x = thisVertex.x; vertex_out[idx].y = thisVertex.y; vertex_out[idx].z = thisVertex.z; vertex_out[idx].w = 1.0f;

	// ******* END MY CODE ********

	// now compute weight : depth Uncertainty
	float theta = acos(outNormal.z);
	float theta_diff = theta / (PI*0.5f - theta);

	sigmaZ_out[idx] = (0.0012f + 0.0019f * (z - 0.4f) * (z - 0.4f) + 0.0001f / sqrt(z) * theta_diff * theta_diff);*/
}

_CPU_AND_GPU_CODE_ inline void computeVertexMap(const CONSTPTR(float) *depth_in, DEVICEPTR(Vector4f) *vertex_out, 
	int x, int y, Vector2i imgDims, Vector4f intrinparam)
{
	int idx = x + y*imgDims.x;

	Matrix3f calib = Matrix3f(intrinparam.x, 0.0f, 0.0f,0.0f, intrinparam.y,0.0f, intrinparam.z, intrinparam.w , 1.0f) ;
	
	Matrix3f calib_inv ;
	calib.inv(calib_inv) ;
	
	Vector3f thisVertex , thisVec ;
	thisVec.x = x ; thisVec.y = y ; thisVec.z = 1.0f ;  

	// check if the normal is good 
	float z = depth_in[x + y * imgDims.x];
	if (z < 0.0f)
	{
		vertex_out[idx].x = vertex_out[idx].y = vertex_out[idx].z = 0.0f ;
		vertex_out[idx].w = -1.0f ;
		return;
	}
	// create and store the 3D vertex 
	thisVertex = calib_inv*thisVec*z ;
	vertex_out[idx].x = thisVertex.x; vertex_out[idx].y = thisVertex.y; vertex_out[idx].z = thisVertex.z; vertex_out[idx].w = 1.0f; 
}

_CPU_AND_GPU_CODE_ inline void computeNormalAndSigma(const CONSTPTR(Vector4f) *vertex_in, const CONSTPTR(float) *depth_in, DEVICEPTR(Vector4f) *normal_out, 
	DEVICEPTR(float) *sigmaZ_out, int x, int y, Vector2i imgDims, Vector4f intrinparam)
{
	int idx = x + y*imgDims.x;
	double z = depth_in[x + y * imgDims.x];

	// deduce the vertices from the vertex map
	Vector3f vertex1, vertex2, vertex3, vertex4, outNormal; 

	/*vertex1.x = vertex_in[x-2 + (y+2) * imgDims.x].x ;	vertex1.y = vertex_in[x-2 + (y+2) * imgDims.x].y ; vertex1.z = vertex_in[x-2 + (y+2) * imgDims.x].z ;	
	vertex2.x = vertex_in[(x+2) + (y+2)* imgDims.x].x ;	vertex2.y = vertex_in[(x+2) + (y+2)* imgDims.x].y ; vertex2.z = vertex_in[(x+2) + (y+2)* imgDims.x].z ;
	vertex3.x = vertex_in[x + (y-2)* imgDims.x].x ;	vertex3.y = vertex_in[x + (y-2)* imgDims.x].y ; vertex3.z = vertex_in[x + (y-2)* imgDims.x].z ;

	// determine the difference vectors 
	Vector3f diff_x2(0.0f, 0.0f, 0.0f), diff_y2(0.0f, 0.0f, 0.0f) ;
	
	diff_x2 =  vertex2-vertex1 ; diff_y2 =  vertex3-vertex1;*/

	vertex1.x = vertex_in[x + (y-1) * imgDims.x].x ;	vertex1.y = vertex_in[x + (y-1) * imgDims.x].y ; vertex1.z = vertex_in[x + (y-1) * imgDims.x].z ;	
	vertex2.x = vertex_in[(x+1) + y* imgDims.x].x ;	vertex2.y = vertex_in[(x+1) + y* imgDims.x].y ; vertex2.z = vertex_in[(x+1) + y* imgDims.x].z ;
	vertex3.x = vertex_in[x + (y+1)* imgDims.x].x ;	vertex3.y = vertex_in[x + (y+1)* imgDims.x].y ; vertex3.z = vertex_in[x + (y+1)* imgDims.x].z ;
	vertex4.x = vertex_in[x-1 + y* imgDims.x].x ;	vertex4.y = vertex_in[x-1 + y* imgDims.x].y ; vertex4.z = vertex_in[x-1 + y* imgDims.x].z ;

	// determine the difference vectors 
	Vector3f diff_x2(0.0f, 0.0f, 0.0f), diff_y2(0.0f, 0.0f, 0.0f) ;
	
	diff_x2 =  vertex4-vertex2 ; diff_y2 =  vertex3-vertex1;

	// cross product
	outNormal.x = float(diff_x2.y * diff_y2.z - diff_x2.z*diff_y2.y);
	outNormal.y = float(diff_x2.z * diff_y2.x - diff_x2.x*diff_y2.z);
	outNormal.z = float(diff_x2.x * diff_y2.y - diff_x2.y*diff_y2.x);

	//std::cout << "outNormal before norm : " << outNormal << std::endl ;

	if (outNormal.x == 0.0f && outNormal.y == 0.0f && outNormal.z == 0.0f)
	{
		normal_out[idx].x = normal_out[idx].y = normal_out[idx].z = 0.0f ;
		normal_out[idx].w = -1.0f;
		sigmaZ_out[idx] = -1;
		return;
	}

	// normalization 
	float norm = 1.0f / sqrt(outNormal.x * outNormal.x + outNormal.y * outNormal.y + outNormal.z * outNormal.z);
    outNormal *= norm;

	//std::cout << "outNormal after norm : " << outNormal << std::endl ;

	// fill in the normal and the vertex vertex_out
	normal_out[idx].x = outNormal.x; normal_out[idx].y = outNormal.y; normal_out[idx].z = outNormal.z; normal_out[idx].w = 1.0f;

	// now compute weight : depth Uncertainty
	float theta = acos(outNormal.z);
	float theta_diff = theta / (PI*0.5f - theta);

	sigmaZ_out[idx] = (0.0012f + 0.0019f * (z - 0.4f) * (z - 0.4f) + 0.0001f / sqrt(z) * theta_diff * theta_diff);
}
