// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Utils/ITMLibDefines.h"
#include "../Utils/ITMLibSettings.h"

#include "../Objects/ITMView.h"
#include "../Objects/ITMViewIMU.h"
#include "../Objects/ITMRGBDCalib.h"

using namespace ITMLib::Objects;

namespace ITMLib
{
	namespace Engine
	{
		/** \brief
		*/
		class ITMViewBuilder
		{
		protected:
			const ITMRGBDCalib *calib;
			ITMShortImage *shortImage;
			ITMFloatImage *floatImage;
			ITMFloatImage *probaMap;

			ITMFloat4Image *depthFloat4;
			ITMFloat4Image *vertexFloat4;
			ITMFloatImage *depthUncertaintyFloat;
			ITMBoolImage *depthSegmentedBool;
			ITMIntImage *depthLabeledInt ;

		public:
			virtual void ConvertDisparityToDepth(ITMFloatImage *depth_out, const ITMShortImage *disp_in, const ITMIntrinsics *depthIntrinsics,
				Vector2f disparityCalibParams) = 0;
			virtual void ConvertDepthAffineToFloat(ITMFloatImage *depth_out, const ITMShortImage *depth_in, Vector2f depthCalibParams) = 0;

			virtual void DepthFiltering(ITMFloatImage *image_out, const ITMFloatImage *image_in) = 0;
			virtual void DepthFilteringRGB(ITMFloatImage *image_out, const ITMFloatImage *image_in, const ITMUChar4Image *rgb_in) = 0 ;
			virtual void DepthNormalFiltering(ITMFloat4Image *image_out, const ITMFloat4Image *image_in, const ITMFloatImage *depth_in) = 0 ;
			virtual void VertexFiltering(ITMFloat4Image *image_out, const ITMFloat4Image *image_in, const ITMFloatImage *depth_in) = 0 ;
			virtual void VertexFilteringRGB(ITMFloat4Image *image_out, const ITMFloat4Image *image_in,  const ITMFloatImage *depth_in, const ITMUChar4Image *rgb_in) = 0 ;
			virtual void DepthNormalFilteringRGB(ITMFloat4Image *image_out, const ITMFloat4Image *image_in, const ITMFloatImage *depth_in, const ITMUChar4Image *rgb_in) = 0 ;
			
			virtual void ComputeNormalAndWeights(ITMFloat4Image *normal_out, ITMFloat4Image *vertex_out, ITMFloatImage *sigmaZ_out, const ITMFloatImage *depth_in, Vector4f intrinsic) = 0;
			virtual void ComputeVertices(ITMFloat4Image *vertex_out, const ITMFloatImage *depth_in, Vector4f intrinsic) = 0 ;
			virtual void ComputeNormalSigma(ITMFloat4Image *normal_out, ITMFloatImage *sigmaZ_out, ITMFloat4Image *vertex_in, ITMFloatImage *depth_in, Vector4f intrinsic) = 0 ;

			// to perform the segmentation of the depth map 
			virtual void Segmentation(ITMFloatImage *propaMapOut, ITMBoolImage *edgeMapOut, ITMIntImage *labelMapOut, float threshold, const ITMFloat4Image *depthNormal,const ITMFloat4Image *vertexMap,const ITMFloatImage *depthUncertainty) = 0 ;
			
			// the segmentation with the semantic information 
			virtual void SegmentationWithSemantique(ITMFloatImage *propaMapOut, ITMBoolImage *edgeMapOut, ITMIntImage *labelMapOut, float threshold, const ITMFloat4Image *depthNormal,const ITMFloat4Image *vertexMap,const ITMFloatImage *depthUncertainty, const ITMShortImage* sem_sementation) = 0 ;

			// side function for the segmentation 
			virtual void Segment(const ITMFloat4Image *vertexMap,ITMFloatImage *propaMapOut, ITMBoolImage *edgeMapOut,  float threshold,
								 const ITMFloat4Image *depthNormal,const ITMFloatImage *depthUncertainty) = 0 ; 	
			virtual void Erosion(ITMBoolImage *edgeMapOut, ITMBoolImage *edgeMapIn, Vector2i imgDims) = 0 ;
			virtual void Dilate(ITMBoolImage *edgeMapOut, ITMBoolImage *edgeMapIn, Vector2i imgDims) = 0 ;
			virtual void Label(ITMBoolImage *edgeMapOut, ITMIntImage *labelMapOut, Vector2i imgDims) =0 ; 
			virtual void MergeSemantic(ITMIntImage *labelMapOut, const ITMShortImage* sem_sementation, Vector2i imgDims) = 0;

			// update the view in function of the entry 
			virtual void UpdateView(ITMView **view, ITMUChar4Image *rgbImage, ITMShortImage *rawDepthImage, bool useBilateralFilter, bool modelSensorNoise = false, bool segmentation= true) = 0;
			virtual void UpdateView(ITMView **view, ITMUChar4Image *rgbImage, ITMFloatImage *depthImage) = 0;
			virtual void UpdateView(ITMView **view, ITMUChar4Image *rgbImage, ITMShortImage *depthImage, bool useBilateralFilter,
				ITMIMUMeasurement *imuMeasurement) = 0;
		    virtual void UpdateAllView(ITMView **view, ITMUChar4Image *rgbImage, ITMShortImage *rawDepthImage, ITMShortImage *semImage, bool useBilateralFilter, bool modelSensorNoise = false, bool segmentation= true) = 0;

			ITMViewBuilder(const ITMRGBDCalib *calib)
			{
				this->calib = calib;
				this->shortImage = NULL;
				this->floatImage = NULL;

				this->probaMap = NULL ;
				this->depthFloat4 = NULL;
				this->vertexFloat4 = NULL;
				this->depthUncertaintyFloat = NULL;
				this->depthSegmentedBool = NULL;
				this->depthLabeledInt = NULL;
			}

			virtual ~ITMViewBuilder()
			{
				if (this->shortImage != NULL) delete this->shortImage;
				if (this->floatImage != NULL) delete this->floatImage;
			}
		};
	}
}

