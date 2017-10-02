// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../ITMViewBuilder.h"
#include "../../DeviceAgnostic/ITMViewBuilder.h"
#include "../../../../ORUtils/MetalContext.h"

namespace ITMLib
{
	namespace Engine
	{
		class ITMViewBuilder_CPU : public ITMViewBuilder
		{
		public:
			void ConvertDisparityToDepth(ITMFloatImage *depth_out, const ITMShortImage *disp_in, const ITMIntrinsics *depthIntrinsics, 
				Vector2f disparityCalibParams);
			void ConvertDepthAffineToFloat(ITMFloatImage *depth_out, const ITMShortImage *depth_in, Vector2f depthCalibParams);

			// filter only with depth information 
			void DepthFiltering(ITMFloatImage *image_out, const ITMFloatImage *image_in);
			void DepthNormalFiltering(ITMFloat4Image *image_out, const ITMFloat4Image *image_in, const ITMFloatImage *depth_in) ;
			void VertexFiltering(ITMFloat4Image *image_out, const ITMFloat4Image *image_in, const ITMFloatImage *depth_in) ;
			
			// filter with RGB information 
			void DepthFilteringRGB(ITMFloatImage *image_out, const ITMFloatImage *image_in, const ITMUChar4Image *rgb_in) ;
			void VertexFilteringRGB(ITMFloat4Image *image_out, const ITMFloat4Image *image_in,  const ITMFloatImage *depth_in, const ITMUChar4Image *rgb_in) ;
			void DepthNormalFilteringRGB(ITMFloat4Image *image_out, const ITMFloat4Image *image_in, const ITMFloatImage *depth_in, const ITMUChar4Image *rgb_in) ;
			
			// add function to compute Vertex map and normal map separatly 
			void ComputeVertices(ITMFloat4Image *vertex_out, const ITMFloatImage *depth_in, Vector4f intrinsic) ;
			void ComputeNormalAndWeights(ITMFloat4Image *normal_out, ITMFloat4Image *vertex_out, ITMFloatImage *sigmaZ_out, const ITMFloatImage *depth_in, Vector4f intrinsic);
			void ComputeNormalSigma(ITMFloat4Image *normal_out, ITMFloatImage *sigmaZ_out, ITMFloat4Image *vertex_in, ITMFloatImage *depth_in, Vector4f intrinsic) ;

			// add the segmentation part 
			void Segmentation(ITMFloatImage *propaMapOut, ITMBoolImage *edgeMapOut, ITMIntImage *labelMapOut, float threshold, const ITMFloat4Image *depthNormal,const ITMFloat4Image *vertexMap,const ITMFloatImage *depthUncertainty) ;
			void SegmentationWithSemantique(ITMFloatImage *propaMapOut, ITMBoolImage *edgeMapOut, ITMIntImage *labelMapOut, float threshold, const ITMFloat4Image *depthNormal,const ITMFloat4Image *vertexMap,const ITMFloatImage *depthUncertainty, const ITMShortImage* sem_sementation) ;
			
			// helper function 
			void Segment(const ITMFloat4Image *vertexMap,ITMFloatImage *propaMapOut, ITMBoolImage *edgeMapOut,  float threshold,
						 const ITMFloat4Image *depthNormal,const ITMFloatImage *depthUncertainty) ;
			void Erosion(ITMBoolImage *edgeMapOut, ITMBoolImage *edgeMapIn, Vector2i imgDims) ;
			void Dilate(ITMBoolImage *edgeMapOut, ITMBoolImage *edgeMapIn, Vector2i imgDims);
			void Label(ITMBoolImage *edgeMapOut, ITMIntImage *labelMapOut, Vector2i imgDims); 
			void MergeSemantic(ITMIntImage *labelMapOut, const ITMShortImage* sem_sementation, Vector2i imgDims) ;

			void UpdateView(ITMView **view, ITMUChar4Image *rgbImage, ITMShortImage *rawDepthImage, bool useBilateralFilter, bool modelSensorNoise = false, bool segmentation= true);
			void UpdateView(ITMView **view, ITMUChar4Image *rgbImage, ITMFloatImage *depthImage);
			void UpdateView(ITMView **view, ITMUChar4Image *rgbImage, ITMShortImage *depthImage, bool useBilateralFilter, ITMIMUMeasurement *imuMeasurement);
		    void UpdateAllView(ITMView **view, ITMUChar4Image *rgbImage, ITMShortImage *rawDepthImage, ITMShortImage *semImage, bool useBilateralFilter, bool modelSensorNoise = false, bool segmentation= true) ;

			ITMViewBuilder_CPU(const ITMRGBDCalib *calib);
			~ITMViewBuilder_CPU(void);
		};
	}
}
