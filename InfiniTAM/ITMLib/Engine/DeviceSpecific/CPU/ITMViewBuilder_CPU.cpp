// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#include "ITMViewBuilder_CPU.h"
#include "../../../Segment/normalSegmentation.h"
#include "../../../Segment/semanticMerging.h"
#include "../../../Segment/filtering.h"


using namespace ITMLib::Engine;
using namespace ORUtils;

ITMViewBuilder_CPU::ITMViewBuilder_CPU(const ITMRGBDCalib *calib):ITMViewBuilder(calib) { }
ITMViewBuilder_CPU::~ITMViewBuilder_CPU(void) { }

void ITMViewBuilder_CPU::UpdateView(ITMView **view_ptr, ITMUChar4Image *rgbImage, ITMShortImage *rawDepthImage, bool useBilateralFilter, bool modelSensorNoise, bool segmentation)
{ 
	if (*view_ptr == NULL)
	{
		*view_ptr = new ITMView(calib, rgbImage->noDims, rawDepthImage->noDims, false);
		
		if (this->shortImage != NULL) delete this->shortImage;
		this->shortImage = new ITMShortImage(rawDepthImage->noDims, true, false);
		if (this->floatImage != NULL) delete this->floatImage;
		this->floatImage = new ITMFloatImage(rawDepthImage->noDims, true, false);

		// add all the images -> store the changes 
		if (this->probaMap != NULL) delete this->probaMap;
		this->probaMap = new ITMFloatImage(rawDepthImage->noDims, true, false);
		if (this->depthFloat4 != NULL) delete this->depthFloat4;
		this->depthFloat4 = new ITMFloat4Image(rawDepthImage->noDims, true, false);
		if (this->vertexFloat4 != NULL) delete this->vertexFloat4;
		this->vertexFloat4 = new ITMFloat4Image(rawDepthImage->noDims, true, false);
		if (this->depthUncertaintyFloat != NULL) delete this->depthUncertaintyFloat;
		this->depthUncertaintyFloat = new ITMFloatImage(rawDepthImage->noDims, true, false);
		if (this->depthSegmentedBool != NULL) delete this->depthSegmentedBool;
		this->depthSegmentedBool = new ITMBoolImage(rawDepthImage->noDims, true, false);
		if (this->depthLabeledInt != NULL) delete this->depthLabeledInt;
		this->depthLabeledInt = new ITMIntImage(rawDepthImage->noDims, true, false);
	}
	
	ITMView *view = *view_ptr;

	view->rgb->SetFrom(rgbImage, MemoryBlock<Vector4u>::CPU_TO_CPU);
	this->shortImage->SetFrom(rawDepthImage, MemoryBlock<short>::CPU_TO_CPU);
	
	switch (view->calib->disparityCalib.type)
	{
	case ITMDisparityCalib::TRAFO_KINECT:
		this->ConvertDisparityToDepth(view->depth, this->shortImage, &(view->calib->intrinsics_d), view->calib->disparityCalib.params);
		break;
	case ITMDisparityCalib::TRAFO_AFFINE:
		this->ConvertDepthAffineToFloat(view->depth, this->shortImage, view->calib->disparityCalib.params);
		break;
	default:
		break;
	}
	
	// 2 steps of bilateral filtering for the depth 
	this->DepthFiltering(this->floatImage, view->depth);
	this->DepthFiltering(view->depth, this->floatImage);

	// test by adding the RGB information to the filter 
	/*this->DepthFilteringRGB(this->floatImage, view->depth, view->rgb);
	this->DepthFilteringRGB(view->depth, this->floatImage, view->rgb);
	this->DepthFilteringRGB(this->floatImage, view->depth, view->rgb); 
	this->DepthFilteringRGB(view->depth, this->floatImage, view->rgb);*/

	// *********** OLD VERSION *********** -> normal computed without filtering the vertex map 
	/*this->ComputeNormalAndWeights(view->depthNormal, view->vertex, view->depthUncertainty, view->depth, 
									  view->calib->intrinsics_d.projectionParamsSimple.all);
	this->ComputeNormalAndWeights(this->depthFloat4, this->vertexFloat4, this->depthUncertaintyFloat, this->floatImage, 
									  //view->calib->intrinsics_d.projectionParamsSimple.all);

	// depth filtering using a bilateral filter 							 	 
	//this->DepthNormalFiltering(view->depthNormal, this->depthFloat4, view->depth) ;
	this->DepthNormalFiltering(this->depthFloat4, view->depthNormal, view->depth) ;
	this->DepthNormalFiltering(view->depthNormal, this->depthFloat4, view->depth) ;
	this->DepthNormalFiltering(this->depthFloat4, view->depthNormal, view->depth) ;
	this->DepthNormalFiltering(view->depthNormal, this->depthFloat4, view->depth) ;
	this->DepthNormalFiltering(this->depthFloat4, view->depthNormal, view->depth) ;
	this->DepthNormalFiltering(view->depthNormal, this->depthFloat4, view->depth) ;

	//view->depthNormal->SetFrom(this->depthFloat4, MemoryBlock<Vector4f>::CPU_TO_CPU);
	//view->vertex->SetFrom(this->vertexFloat4, MemoryBlock<Vector4f>::CPU_TO_CPU);
	//view->depthUncertainty->SetFrom(this->depthUncertaintyFloat, MemoryBlock<float>::CPU_TO_CPU);*/

	// *********** NEW VERSION  *********
	//  compute the vertices from the depth map
	ComputeVertices(view->vertex, view->depth, view->calib->intrinsics_d.projectionParamsSimple.all) ; 

	// try to filter the vertices with the bilateral filter implemented 
	//double sigmaColor = 6.0;
    //double sigmaSpace = 4;
	//bilaFilter(view->vertex, this->depthFloat4, rawDepthImage->noDims, 0, sigmaColor, sigmaSpace) ;	
	
	// filter the vertices using a bilateral filter 
	this->VertexFiltering(this->vertexFloat4, view->vertex, view->depth) ;
	this->VertexFiltering(view->vertex, this->vertexFloat4, view->depth) ;

	/*this->VertexFilteringRGB(this->vertexFloat4, view->vertex, view->depth, view->rgb) ;
	this->VertexFilteringRGB(view->vertex, this->vertexFloat4, view->depth, view->rgb) ;
	this->VertexFilteringRGB(this->vertexFloat4, view->vertex, view->depth, view->rgb) ;
	this->VertexFilteringRGB(view->vertex, this->vertexFloat4,  view->depth, view->rgb) ;*/
	//this->VertexFilteringRGB(this->vertexFloat4, view->vertex, view->depth, view->rgb) ;
	//this->VertexFilteringRGB(view->vertex, this->vertexFloat4,  view->depth, view->rgb) ;

	ITMFloat4Image * normal_NF = new ITMFloat4Image(rawDepthImage->noDims, true, false);
	ComputeNormalSigma(normal_NF, view->depthUncertainty, view->vertex, view->depth,view->calib->intrinsics_d.projectionParamsSimple.all) ;

	// filter using the bilateral filter with 1 channel 
	double sigmaColor = 15.0;
	double sigmaSpace = 5.0;
	bilaFilter(view->depthNormal, normal_NF, rawDepthImage->noDims, 1, sigmaColor, sigmaSpace) ;
	delete normal_NF ;

	/*ComputeNormalSigma(view->depthNormal, view->depthUncertainty, view->vertex, view->depth,view->calib->intrinsics_d.projectionParamsSimple.all) ;
	this->DepthNormalFilteringRGB(this->depthFloat4, view->depthNormal, view->depth, view->rgb) ;
	this->DepthNormalFilteringRGB(view->depthNormal, this->depthFloat4, view->depth, view->rgb) ;*/
	
	// add the segmentation part here 
	if (segmentation) {
		float threshold = 0.983 ;
		//this->Segmentation(view->depthSegmented, view->depthLabeled, threshold, view->depthNormal, view->vertex, view->depthUncertainty) ; 
		this->Segmentation(view->depthSegFloat, view->depthSegmented, view->depthLabeled, threshold, view->depthNormal, view->vertex, view->depthUncertainty) ;
	}
}

// we add the semantic segmentation image 
void ITMViewBuilder_CPU::UpdateAllView(ITMView **view_ptr, ITMUChar4Image *rgbImage, ITMShortImage *rawDepthImage, ITMShortImage *semImage, bool useBilateralFilter, bool modelSensorNoise, bool segmentation) {
	
	if (*view_ptr == NULL)
	{
		*view_ptr = new ITMView(calib, rgbImage->noDims, rawDepthImage->noDims, false);
		
		if (this->shortImage != NULL) delete this->shortImage;
		this->shortImage = new ITMShortImage(rawDepthImage->noDims, true, false);
		if (this->floatImage != NULL) delete this->floatImage;
		this->floatImage = new ITMFloatImage(rawDepthImage->noDims, true, false);

		// add all the images -> store the changes 
		if (this->probaMap != NULL) delete this->probaMap;
		this->probaMap = new ITMFloatImage(rawDepthImage->noDims, true, false);
		if (this->depthFloat4 != NULL) delete this->depthFloat4;
		this->depthFloat4 = new ITMFloat4Image(rawDepthImage->noDims, true, false);
		if (this->vertexFloat4 != NULL) delete this->vertexFloat4;
		this->vertexFloat4 = new ITMFloat4Image(rawDepthImage->noDims, true, false);
		if (this->depthUncertaintyFloat != NULL) delete this->depthUncertaintyFloat;
		this->depthUncertaintyFloat = new ITMFloatImage(rawDepthImage->noDims, true, false);
		if (this->depthSegmentedBool != NULL) delete this->depthSegmentedBool;
		this->depthSegmentedBool = new ITMBoolImage(rawDepthImage->noDims, true, false);
		if (this->depthLabeledInt != NULL) delete this->depthLabeledInt;
		this->depthLabeledInt = new ITMIntImage(rawDepthImage->noDims, true, false);
	}
	
	ITMView *view = *view_ptr;

	view->rgb->SetFrom(rgbImage, MemoryBlock<Vector4u>::CPU_TO_CPU);
	this->shortImage->SetFrom(rawDepthImage, MemoryBlock<short>::CPU_TO_CPU);

	//std::cout << view->calib->disparityCalib.type << std::endl ;

	switch (view->calib->disparityCalib.type)
	{
	case ITMDisparityCalib::TRAFO_KINECT:
		this->ConvertDisparityToDepth(view->depth, this->shortImage, &(view->calib->intrinsics_d), view->calib->disparityCalib.params);
		break;
	case ITMDisparityCalib::TRAFO_AFFINE:
		this->ConvertDepthAffineToFloat(view->depth, this->shortImage, view->calib->disparityCalib.params);
		break;
	default:
		break;
	}

	// 2 steps of bilateral filtering
	this->DepthFiltering(this->floatImage, view->depth);
	this->DepthFiltering(view->depth, this->floatImage);
	
	// OLD VERSION  
	/*this->ComputeNormalAndWeights(view->depthNormal, view->vertex, view->depthUncertainty, view->depth, 
									  view->calib->intrinsics_d.projectionParamsSimple.all);
	this->ComputeNormalAndWeights(this->depthFloat4, this->vertexFloat4, this->depthUncertaintyFloat, this->floatImage, 
									  //view->calib->intrinsics_d.projectionParamsSimple.all);

	// depth filtering using a bilateral filter 							 	 
	this->DepthNormalFiltering(view->depthNormal, this->depthFloat4, view->depth) ;
	this->DepthNormalFiltering(this->depthFloat4, view->depthNormal, view->depth) ;
	this->DepthNormalFiltering(view->depthNormal, this->depthFloat4, view->depth) ;
	this->DepthNormalFiltering(this->depthFloat4, view->depthNormal, view->depth) ;
	this->DepthNormalFiltering(view->depthNormal, this->depthFloat4, view->depth) ;
	this->DepthNormalFiltering(this->depthFloat4, view->depthNormal, view->depth) ;
	this->DepthNormalFiltering(view->depthNormal, this->depthFloat4, view->depth) ;

	//view->depthNormal->SetFrom(this->depthFloat4, MemoryBlock<Vector4f>::CPU_TO_CPU);
	//view->vertex->SetFrom(this->vertexFloat4, MemoryBlock<Vector4f>::CPU_TO_CPU);
	//view->depthUncertainty->SetFrom(this->depthUncertaintyFloat, MemoryBlock<float>::CPU_TO_CPU);*/

	// *********** NEW VERSION  *********
	//  compute the vertices from the depth map
	ComputeVertices(view->vertex, view->depth, view->calib->intrinsics_d.projectionParamsSimple.all) ; 
	
	// try to filter the vertices with the bilateral filter implemented 
	/*double sigmaColor = 10.0;
	double sigmaSpace = 1.5;
	bilaFilter(view->vertex, this->depthFloat4, rawDepthImage->noDims, 0, sigmaColor, sigmaSpace) ;	
	*/	
	// filter the vertices using a bilateral filter 
	this->VertexFiltering(this->vertexFloat4, view->vertex, view->depth) ;
	this->VertexFiltering(view->vertex, this->vertexFloat4, view->depth) ;
	this->VertexFiltering(this->vertexFloat4, view->vertex, view->depth) ;
	this->VertexFiltering(view->vertex, this->vertexFloat4,  view->depth) ;

	ITMFloat4Image * normal_NF = new ITMFloat4Image(rawDepthImage->noDims, true, false);
	ComputeNormalSigma(normal_NF, view->depthUncertainty, view->vertex, view->depth,view->calib->intrinsics_d.projectionParamsSimple.all) ;
	double sigmaColor = 15.0;
	double sigmaSpace = 5.0;
	bilaFilter(view->depthNormal, normal_NF, rawDepthImage->noDims, 1, sigmaColor, sigmaSpace) ;
	delete normal_NF ;

	/*ComputeNormalSigma(view->depthNormal, view->depthUncertainty, view->vertex, view->depth,view->calib->intrinsics_d.projectionParamsSimple.all) ;
	this->DepthNormalFilteringRGB(this->depthFloat4, view->depthNormal, view->depth, view->rgb) ;
	this->DepthNormalFilteringRGB(view->depthNormal, this->depthFloat4, view->depth, view->rgb) ;*/
	
	// add the segmentation part here 
	view->sem_seg->SetFrom(semImage, MemoryBlock<short>::CPU_TO_CPU);

	if (segmentation) {
		float threshold = 0.99 ;
		this->SegmentationWithSemantique(view->depthSegFloat, view->depthSegmented, view->depthLabeled, threshold, view->depthNormal, view->vertex, view->depthUncertainty, view->sem_seg) ;
	}
}


void ITMViewBuilder_CPU::UpdateView(ITMView **view_ptr, ITMUChar4Image *rgbImage, ITMFloatImage *depthImage)
{
	if (*view_ptr == NULL)
		*view_ptr = new ITMView(calib, rgbImage->noDims, depthImage->noDims, false);

	ITMView *view = *view_ptr;

	view->rgb->UpdateDeviceFromHost();
	view->depth->UpdateDeviceFromHost();
}

void ITMViewBuilder_CPU::UpdateView(ITMView **view_ptr, ITMUChar4Image *rgbImage, ITMShortImage *depthImage, bool useBilateralFilter, ITMIMUMeasurement *imuMeasurement)
{
	if (*view_ptr == NULL)
	{
		*view_ptr = new ITMViewIMU(calib, rgbImage->noDims, depthImage->noDims, false);
		if (this->shortImage != NULL) delete this->shortImage;
		this->shortImage = new ITMShortImage(depthImage->noDims, true, false);
		if (this->floatImage != NULL) delete this->floatImage;
		this->floatImage = new ITMFloatImage(depthImage->noDims, true, false);
	}

	ITMViewIMU* imuView = (ITMViewIMU*)(*view_ptr);
	imuView->imu->SetFrom(imuMeasurement);

	this->UpdateView(view_ptr, rgbImage, depthImage, useBilateralFilter);
}

void ITMViewBuilder_CPU::ConvertDisparityToDepth(ITMFloatImage *depth_out, const ITMShortImage *depth_in, const ITMIntrinsics *depthIntrinsics,
	Vector2f disparityCalibParams)
{
	Vector2i imgSize = depth_in->noDims;

	const short *d_in = depth_in->GetData(MEMORYDEVICE_CPU);
	float *d_out = depth_out->GetData(MEMORYDEVICE_CPU);

	float fx_depth = depthIntrinsics->projectionParamsSimple.fx;

	for (int y = 0; y < imgSize.y; y++) for (int x = 0; x < imgSize.x; x++)
		convertDisparityToDepth(d_out, x, y, d_in, disparityCalibParams, fx_depth, imgSize);
}

void ITMViewBuilder_CPU::ConvertDepthAffineToFloat(ITMFloatImage *depth_out, const ITMShortImage *depth_in, const Vector2f depthCalibParams)
{
	Vector2i imgSize = depth_in->noDims;

	const short *d_in = depth_in->GetData(MEMORYDEVICE_CPU);
	float *d_out = depth_out->GetData(MEMORYDEVICE_CPU);

	for (int y = 0; y < imgSize.y; y++) for (int x = 0; x < imgSize.x; x++)
		convertDepthAffineToFloat(d_out, x, y, d_in, imgSize, depthCalibParams);
}

void ITMViewBuilder_CPU::DepthFiltering(ITMFloatImage *image_out, const ITMFloatImage *image_in)
{
	Vector2i imgSize = image_in->noDims;

	image_out->Clear();

	float *imout = image_out->GetData(MEMORYDEVICE_CPU);
	const float *imin = image_in->GetData(MEMORYDEVICE_CPU);

	for (int y = 2; y < imgSize.y - 2; y++) for (int x = 2; x < imgSize.x - 2; x++)
		filterDepth(imout, imin, x, y, imgSize);
}

void ITMViewBuilder_CPU::DepthFilteringRGB(ITMFloatImage *image_out, const ITMFloatImage *image_in, const ITMUChar4Image *rgb_in)
{
	Vector2i imgSize = image_in->noDims;

	image_out->Clear();

	float *imout = image_out->GetData(MEMORYDEVICE_CPU);
	const float *imin = image_in->GetData(MEMORYDEVICE_CPU);
	const Vector4u *imrgb = rgb_in->GetData(MEMORYDEVICE_CPU);
	
	for (int y = 2; y < imgSize.y - 2; y++) for (int x = 2; x < imgSize.x - 2; x++)
	filterDepthWithRGB(imout, imin, imrgb, x, y, imgSize);
}

void ITMViewBuilder_CPU::VertexFilteringRGB(ITMFloat4Image *image_out, const ITMFloat4Image *image_in,  const ITMFloatImage *depth_in, const ITMUChar4Image *rgb_in)
{
	Vector2i imgSize = image_in->noDims;

	image_out->Clear();

	Vector4f *imout = image_out->GetData(MEMORYDEVICE_CPU);
	const Vector4f *imin = image_in->GetData(MEMORYDEVICE_CPU);
	const float *depthIn = depth_in->GetData(MEMORYDEVICE_CPU);
	const Vector4u *imrgb = rgb_in->GetData(MEMORYDEVICE_CPU);
	
	for (int y = 2; y < imgSize.y - 2; y++) for (int x = 2; x < imgSize.x - 2; x++)
	filterVertexWithRGB(imout, imin, imrgb, x, y, imgSize, depthIn);
}

// added to filter normals 
void ITMViewBuilder_CPU::DepthNormalFiltering(ITMFloat4Image *image_out, const ITMFloat4Image *image_in, const ITMFloatImage *depth_in)
{
	Vector2i imgSize = image_in->noDims;

	image_out->Clear();

	Vector4f *imout = image_out->GetData(MEMORYDEVICE_CPU);
	const Vector4f *imin = image_in->GetData(MEMORYDEVICE_CPU);
	const float *depthIn = depth_in->GetData(MEMORYDEVICE_CPU);

	for (int y = 2; y < imgSize.y - 2; y++) for (int x = 2; x < imgSize.x - 2; x++)
		depthNormalFilter(imout, imin, x, y, imgSize, depthIn);
}
// added to filter normals with normal information 
void ITMViewBuilder_CPU::DepthNormalFilteringRGB(ITMFloat4Image *image_out, const ITMFloat4Image *image_in, const ITMFloatImage *depth_in, const ITMUChar4Image *rgb_in)
{
	Vector2i imgSize = image_in->noDims;

	image_out->Clear();

	Vector4f *imout = image_out->GetData(MEMORYDEVICE_CPU);
	const Vector4f *imin = image_in->GetData(MEMORYDEVICE_CPU);
	const float *depthIn = depth_in->GetData(MEMORYDEVICE_CPU);
	const Vector4u *imrgb = rgb_in->GetData(MEMORYDEVICE_CPU);
	
	for (int y = 2; y < imgSize.y - 2; y++) for (int x = 2; x < imgSize.x - 2; x++)
		normalFilterWithRGB(imout, imin, imrgb, x, y, imgSize, depthIn);
}

// filter the vertices 
void ITMViewBuilder_CPU::VertexFiltering(ITMFloat4Image *image_out, const ITMFloat4Image *image_in, const ITMFloatImage *depth_in)
{
	Vector2i imgSize = image_in->noDims;

	image_out->Clear();

	Vector4f *imout = image_out->GetData(MEMORYDEVICE_CPU);
	const Vector4f *imin = image_in->GetData(MEMORYDEVICE_CPU);
	const float *depthIn = depth_in->GetData(MEMORYDEVICE_CPU);

	for (int y = 2; y < imgSize.y - 2; y++) for (int x = 2; x < imgSize.x - 2; x++)
		vertexFilter(imout, imin, x, y, imgSize, depthIn);
}

void ITMLib::Engine::ITMViewBuilder_CPU::ComputeNormalAndWeights(ITMFloat4Image *normal_out,ITMFloat4Image *vertex_out, 
	ITMFloatImage *sigmaZ_out, const ITMFloatImage *depth_in, Vector4f intrinsic)
{
	Vector2i imgDims = depth_in->noDims;

	const float *depthData_in = depth_in->GetData(MEMORYDEVICE_CPU);

	float *sigmaZData_out = sigmaZ_out->GetData(MEMORYDEVICE_CPU);
	Vector4f *normalData_out = normal_out->GetData(MEMORYDEVICE_CPU);
	Vector4f *vertexData_out = vertex_out->GetData(MEMORYDEVICE_CPU);

	for (int y = 2; y < imgDims.y - 2; y++) for (int x = 2; x < imgDims.x - 2; x++)
		computeNormalAndWeight(depthData_in, normalData_out, vertexData_out, sigmaZData_out, x, y, imgDims, intrinsic);
}

void ITMLib::Engine::ITMViewBuilder_CPU::ComputeVertices(ITMFloat4Image *vertex_out, 
	const ITMFloatImage *depth_in, Vector4f intrinsic) 
{
	Vector2i imgDims = depth_in->noDims;

	const float *depthData_in = depth_in->GetData(MEMORYDEVICE_CPU);

	Vector4f *vertexData_out = vertex_out->GetData(MEMORYDEVICE_CPU);

	for (int y = 2; y < imgDims.y - 2; y++) for (int x = 2; x < imgDims.x - 2; x++)
		computeVertexMap(depthData_in, vertexData_out, x, y, imgDims, intrinsic) ;
}

void ITMLib::Engine::ITMViewBuilder_CPU::ComputeNormalSigma(ITMFloat4Image *normal_out, 
	ITMFloatImage *sigmaZ_out, ITMFloat4Image *vertex_in, ITMFloatImage *depth_in, Vector4f intrinsic)
{
	Vector2i imgDims = depth_in->noDims;

	// given at previous step 
	const Vector4f *vertexData_in = vertex_in->GetData(MEMORYDEVICE_CPU);
	const float *depthData_in = depth_in->GetData(MEMORYDEVICE_CPU);

	// computed 
	float *sigmaZData_out = sigmaZ_out->GetData(MEMORYDEVICE_CPU);
	Vector4f *normalData_out = normal_out->GetData(MEMORYDEVICE_CPU);

	for (int y = 2; y < imgDims.y - 2; y++) for (int x = 2; x < imgDims.x - 2; x++)
		computeNormalAndSigma(vertexData_in, depthData_in, normalData_out, sigmaZData_out, x, y, imgDims, intrinsic);
}

void ITMLib::Engine::ITMViewBuilder_CPU::Segmentation(ITMFloatImage *propaMapOut, ITMBoolImage *edgeMapOut, ITMIntImage *labelMapOut, float threshold, const ITMFloat4Image *depthNormal,const ITMFloat4Image *vertexMap,const ITMFloatImage *depthUncertainty) {
	Vector2i imgDims = depthUncertainty->noDims;

	// segment the normal map
	Segment(vertexMap, propaMapOut, edgeMapOut, threshold, depthNormal, depthUncertainty) ;

	// create a new one to perfrom dilatation and erosion  
	ITMBoolImage* egdeMap_inter = new ITMBoolImage(imgDims, true, false) ;
	
	Dilate(egdeMap_inter, edgeMapOut, imgDims) ; // 2 closing operations 
	Erosion(edgeMapOut, egdeMap_inter, imgDims) ;
	Dilate(egdeMap_inter, edgeMapOut, imgDims) ;
	Erosion(edgeMapOut, egdeMap_inter, imgDims) ;
	Dilate(egdeMap_inter, edgeMapOut, imgDims) ;
	Erosion(edgeMapOut, egdeMap_inter, imgDims) ;
	delete egdeMap_inter ;	

	// perform the labeling  
	Label(edgeMapOut, labelMapOut, imgDims) ;
}

void ITMLib::Engine::ITMViewBuilder_CPU::SegmentationWithSemantique(ITMFloatImage *propaMapOut, ITMBoolImage *edgeMapOut, ITMIntImage *labelMapOut, float threshold, const ITMFloat4Image *depthNormal,const ITMFloat4Image *vertexMap,const ITMFloatImage *depthUncertainty, const ITMShortImage* sem_sementation) {
	// ********  perform the labeling as before ******* 
	Vector2i imgDims = depthUncertainty->noDims;

	// segment the normal map
	Segment(vertexMap, propaMapOut, edgeMapOut, threshold, depthNormal, depthUncertainty) ;

	// create a new one to perfrom dilatation and erosion  
	ITMBoolImage* egdeMap_inter = new ITMBoolImage(imgDims, true, false) ;
	Dilate(egdeMap_inter, edgeMapOut, imgDims) ; // 2 closing operations 
	Erosion(edgeMapOut, egdeMap_inter, imgDims) ;
	Dilate(egdeMap_inter, edgeMapOut, imgDims) ;
	Erosion(edgeMapOut, egdeMap_inter, imgDims) ;
	Dilate(egdeMap_inter, edgeMapOut, imgDims) ;
	Erosion(edgeMapOut, egdeMap_inter, imgDims)  ;
	delete egdeMap_inter ;	

	// perform the labeling  
	Label(edgeMapOut, labelMapOut, imgDims) ;

	// compare the result with the semantic segmentation and merge it if necessary 
	MergeSemantic(labelMapOut, sem_sementation, imgDims) ;
}


void ITMLib::Engine::ITMViewBuilder_CPU::Segment(const ITMFloat4Image *vertexMap,ITMFloatImage *propaMapOut, ITMBoolImage *edgeMapOut,  float threshold,
												 const ITMFloat4Image *depthNormal,const ITMFloatImage *depthUncertainty) {
	Vector2i imgDims = depthUncertainty->noDims;
	
	// create the 2 maps we will fusion 
	ITMBoolImage* normalEgdeMap = new ITMBoolImage(imgDims, true, false) ;
	ITMBoolImage* geoEgdeMap = new ITMBoolImage(imgDims, true, false) ;
	ITMFloatImage* normalProbaMap = new ITMFloatImage(imgDims, true, false) ;
	ITMFloatImage* geoProbaMap = new ITMFloatImage(imgDims, true, false) ;

	// get back their pointors 
	bool *normalEgdeData_Out = normalEgdeMap->GetData(MEMORYDEVICE_CPU);
	bool *geoEgdeData_Out = geoEgdeMap->GetData(MEMORYDEVICE_CPU);
	float *normProbaData_Out = normalProbaMap->GetData(MEMORYDEVICE_CPU);
	float *geoProbaData_Out = geoProbaMap->GetData(MEMORYDEVICE_CPU);

	// pointors for the result
	const Vector4f *vertexData_in = vertexMap->GetData(MEMORYDEVICE_CPU);
	const Vector4f *depthNormal_in = depthNormal->GetData(MEMORYDEVICE_CPU);
	bool *edgeMap_out = edgeMapOut->GetData(MEMORYDEVICE_CPU);
	float *probaMap_out = propaMapOut->GetData(MEMORYDEVICE_CPU);
	const float *depthUncernData_in = depthUncertainty->GetData(MEMORYDEVICE_CPU);

	for (int y = 3; y < imgDims.y - 3; y++) for (int x = 3; x < imgDims.x - 3; x++) {			
		goeEdgeSegmentation(geoProbaData_Out, geoEgdeData_Out, depthNormal_in, vertexData_in, depthUncernData_in, x, y, imgDims) ;
		normEdgeSegmentation(normProbaData_Out, normalEgdeData_Out, threshold, depthNormal_in, vertexData_in, x, y, imgDims) ;

		// fuse both map 
		mapFusion(probaMap_out, normProbaData_Out, geoProbaData_Out, edgeMap_out, normalEgdeData_Out, geoEgdeData_Out, x, y, imgDims) ;
	}

	// delete the inter images 
	delete normalEgdeMap ; delete geoEgdeMap ; 	delete normalProbaMap ; delete geoProbaMap ;
}

void ITMLib::Engine::ITMViewBuilder_CPU::Erosion(ITMBoolImage *edgeMapOut, ITMBoolImage *edgeMapIn, Vector2i imgDims) {
	bool *edgeMapData_Out = edgeMapOut->GetData(MEMORYDEVICE_CPU);
	bool *edgeMapData_In = edgeMapIn->GetData(MEMORYDEVICE_CPU);

	for (int y = 2; y < imgDims.y - 2; y++) for (int x = 2; x < imgDims.x - 2; x++) 
			erosion(edgeMapData_Out, edgeMapData_In, x, y, imgDims) ;
}

void ITMLib::Engine::ITMViewBuilder_CPU::Dilate(ITMBoolImage *edgeMapOut, ITMBoolImage *edgeMapIn, Vector2i imgDims) {
	bool *edgeMapData_Out = edgeMapOut->GetData(MEMORYDEVICE_CPU);
	bool *edgeMapData_In = edgeMapIn->GetData(MEMORYDEVICE_CPU);

	for (int y = 2; y < imgDims.y - 2; y++) for (int x = 2; x < imgDims.x - 2; x++) 
			dilate(edgeMapData_Out, edgeMapData_In, x, y, imgDims) ;
}

void ITMLib::Engine::ITMViewBuilder_CPU::Label(ITMBoolImage *edgeMapIn, ITMIntImage *labelMapOut, Vector2i imgDims) {
	bool *edgeMapData_In = edgeMapIn->GetData(MEMORYDEVICE_CPU);
	int *labelMapData_Out = labelMapOut->GetData(MEMORYDEVICE_CPU);

    // creation of visited 
	std::vector<int> visited ;
	visited.resize(imgDims.x*imgDims.y, 0) ;

    // creation of the stack of pair 
    std::stack<std::pair<int,int> > neighbors;
                
    // initialization 
	int current_label = 1 ; int connectivity = 4;
	bool newGroup = false ;

	for (int y = 2; y < imgDims.y - 2; y++) for (int x = 2; x < imgDims.x - 2; x++) 
            label(edgeMapData_In,labelMapData_Out, imgDims, newGroup, visited, neighbors, connectivity, current_label, x, y) ;
}

void ITMLib::Engine::ITMViewBuilder_CPU::MergeSemantic(ITMIntImage *labelMapOut, const ITMShortImage* sem_sementation, Vector2i imgDims) {
	int *labelMapData_Out = labelMapOut->GetData(MEMORYDEVICE_CPU);
	const short *semMapData_out = sem_sementation->GetData(MEMORYDEVICE_CPU) ;

	// create the map to perform the merging if needed 
	std::map<int, int> merging ; 
	findSemanticMerging(merging, labelMapData_Out, semMapData_out, imgDims) ;

	// apply the right merging 
	for (int y = 2; y < imgDims.y - 2; y++) for (int x = 2; x < imgDims.x - 2; x++) 
		mergingSemantic(merging, labelMapData_Out, semMapData_out, x, y, imgDims) ;
}
