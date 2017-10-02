// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Objects/ITMRGBDCalib.h"
#include "../Utils/ITMCalibIO.h"

namespace ITMLib
{
	namespace Objects
	{
		/** \brief
		    Represents a single "view", i.e. RGB and depth images along
		    with all intrinsic and relative calibration information
		*/
		class ITMView
		{
		public:
			/// Intrinsic calibration information for the view.
			ITMRGBDCalib *calib;

			/// RGB colour image.
			ITMUChar4Image *rgb; 

			/// Float valued depth image, if available according to @ref inputImageType.
			ITMFloatImage *depth;

			// add the segmentation segmentation 
			ITMShortImage *sem_seg ;

			/// surface normal of depth image
			// allocated when needed
			ITMFloat4Image *depthNormal;

			// add vertex position to the map 
			ITMFloat4Image *vertex;

			// add the segmentation of the depth map 
			ITMBoolImage *depthSegmented;

			// add the segmentation of the depth map 
			ITMFloatImage *depthSegFloat;

			// add the segmentation of the depth map 
			ITMIntImage *depthLabeled;

			// add the projected labeled map  
			ITMIntImage *modelLabeled;

			// add the projected labeled map  
			ITMIntImage *resultPropag;

			/// uncertainty (std) in each pixel of depth value based on sensor noise model
			/// allocated when needed
			ITMFloatImage *depthUncertainty;

			ITMView(const ITMRGBDCalib *calibration, Vector2i imgSize_rgb, Vector2i imgSize_d, bool useGPU)
			{
				this->calib = new ITMRGBDCalib(*calibration);
				this->rgb = new ITMUChar4Image(imgSize_rgb, true, useGPU);
				this->depth = new ITMFloatImage(imgSize_d, true, useGPU);

				// add the depth normal computation and allocation  //****** CHANGED *******
				this->sem_seg = new ITMShortImage(imgSize_rgb, true, useGPU);
				this->depthNormal = new ITMFloat4Image(imgSize_d, true, useGPU) ;
				this->vertex = new ITMFloat4Image(imgSize_d, true, useGPU) ;
				this->depthSegmented = new ITMBoolImage(imgSize_d, true, useGPU) ;
				this->depthSegFloat = new ITMFloatImage(imgSize_d, true, useGPU); // the propability in float 
				this->depthLabeled = new ITMIntImage(imgSize_d, true, useGPU) ;
				this->modelLabeled = new ITMIntImage(imgSize_d, true, useGPU) ;
				this->resultPropag = new ITMIntImage(imgSize_d, true, useGPU) ;
				this->depthUncertainty = new ITMFloatImage(imgSize_d, true, useGPU) ;
			}

			virtual ~ITMView(void)
			{
				delete calib;
				delete rgb;
				delete depth;

				if (depthNormal != NULL) delete depthNormal;
				if (sem_seg != NULL) delete sem_seg;
				if (vertex != NULL) delete vertex;
				if (depthSegmented != NULL) delete depthSegmented;
				if (depthSegFloat != NULL) delete depthSegFloat;
				if (depthLabeled != NULL) delete depthLabeled;
				if (modelLabeled != NULL) delete modelLabeled;
				if (resultPropag != NULL) delete resultPropag;
				if (depthUncertainty != NULL) delete depthUncertainty;
			}

			// Suppress the default copy constructor and assignment operator
			ITMView(const ITMView&);
			ITMView& operator=(const ITMView&);
		};
	}
}

