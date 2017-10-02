// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#include "ITMVisualisationEngine.h"

using namespace ITMLib::Engine;

inline float interpolate(float val, float y0, float x0, float y1, float x1) {
	return (val - x0)*(y1 - y0) / (x1 - x0) + y0;
}

inline float base(float val) {
	if (val <= -0.75f) return 0.0f;
	else if (val <= -0.25f) return interpolate(val, 0.0f, -0.75f, 1.0f, -0.25f);
	else if (val <= 0.25f) return 1.0f;
	else if (val <= 0.75f) return interpolate(val, 1.0f, 0.25f, 0.0f, 0.75f);
	else return 0.0;
}

void IITMVisualisationEngine::DepthToUchar4(ITMUChar4Image *dst, ITMFloatImage *src)
{
	Vector4u *dest = dst->GetData(MEMORYDEVICE_CPU);
	float *source = src->GetData(MEMORYDEVICE_CPU);
	int dataSize = static_cast<int>(dst->dataSize);

	memset(dst->GetData(MEMORYDEVICE_CPU), 0, dataSize * 4);

	Vector4u *destUC4;
	float lims[2], scale;

	destUC4 = (Vector4u*)dest;
	lims[0] = 100000.0f; lims[1] = -100000.0f;

	for (int idx = 0; idx < dataSize; idx++)
	{
		float sourceVal = source[idx];
		if (sourceVal > 0.0f) { lims[0] = MIN(lims[0], sourceVal); lims[1] = MAX(lims[1], sourceVal); }
	}

	scale = ((lims[1] - lims[0]) != 0) ? 1.0f / (lims[1] - lims[0]) : 1.0f / lims[1];

	if (lims[0] == lims[1]) return;

	for (int idx = 0; idx < dataSize; idx++)
	{
		float sourceVal = source[idx];

		if (sourceVal > 0.0f)
		{
			sourceVal = (sourceVal - lims[0]) * scale;

			destUC4[idx].r = (uchar)(base(sourceVal - 0.5f) * 255.0f);
			destUC4[idx].g = (uchar)(base(sourceVal) * 255.0f);
			destUC4[idx].b = (uchar)(base(sourceVal + 0.5f) * 255.0f);
			destUC4[idx].a = 255;
		}
	}
}

void IITMVisualisationEngine::NormalToUchar4(ITMUChar4Image *dst, ITMFloat4Image *src)
{
	Vector4u *dest = dst->GetData(MEMORYDEVICE_CPU);
	Vector4f *source = src->GetData(MEMORYDEVICE_CPU);
	int dataSize = static_cast<int>(dst->dataSize);

	memset(dst->GetData(MEMORYDEVICE_CPU), 0, dataSize * 4);
	{
		for (int idx = 0; idx < dataSize; idx++)
		{
			Vector4f sourceVal = source[idx];
			if (sourceVal.w >= 0.0f)
			{
				dest[idx].r = (uchar)((0.3f + (sourceVal.r + 1.0f)*0.35f)*255.0f);
				dest[idx].g = (uchar)((0.3f + (sourceVal.g + 1.0f)*0.35f)*255.0f);
				dest[idx].b = (uchar)((0.3f + (sourceVal.b + 1.0f)*0.35f)*255.0f);
			}
			else {
				dest[idx].r = (uchar)(0);
				dest[idx].g = (uchar)(0);
				dest[idx].b = (uchar)(0);
			}
		}
	}
}

void IITMVisualisationEngine::WeightToUchar4(ITMUChar4Image *dst, ITMFloatImage *src)
{
	Vector4u *dest = dst->GetData(MEMORYDEVICE_CPU);
	float *source = src->GetData(MEMORYDEVICE_CPU);
	int dataSize = static_cast<int>(dst->dataSize);
	
	float mindepth = 1000;
	for (size_t i = 0; i < src->dataSize; i++)
		if (source[i]>0) mindepth = MIN(mindepth, source[i]);

	memset(dst->GetData(MEMORYDEVICE_CPU), 0, dataSize * 4);
	{
		for (int idx = 0; idx < dataSize; idx++)
		{
			float sourceVal = source[idx];
			if (sourceVal>0)
			{
				sourceVal = mindepth / sourceVal * 0.8f + 0.2f;
				dest[idx].r = (uchar)((1 - sourceVal)*255.0f);
				dest[idx].b = 0;
				dest[idx].g = (uchar)(sourceVal*255.0f);
			}

		}
	}
}


// store the proba image 
/*void IITMVisualisationEngine::WeightToUchar4(ITMUChar4Image *dst, ITMFloatImage *src)
{
	Vector4u *dest = dst->GetData(MEMORYDEVICE_CPU);
	float *source = src->GetData(MEMORYDEVICE_CPU);
	int dataSize = static_cast<int>(dst->dataSize);
	
	float mindepth = 1000;
	for (size_t i = 0; i < src->dataSize; i++)
		if (source[i]>0) mindepth = MIN(mindepth, source[i]);

	memset(dst->GetData(MEMORYDEVICE_CPU), 0, dataSize * 4);
	{
		for (int idx = 0; idx < dataSize; idx++)
		{
			float sourceVal = source[idx];
			if (sourceVal>0)
			{
				sourceVal = mindepth / sourceVal * 0.8f + 0.2f;
				dest[idx].r = (uchar)((1 - sourceVal)*255.0f);
				dest[idx].b = 0;
				dest[idx].g = (uchar)(sourceVal*255.0f);
			}

		}
	}
}*/

void IITMVisualisationEngine::BinaryToUchar4(ITMUChar4Image *dst, ITMBoolImage *src) {
	Vector4u *dest = dst->GetData(MEMORYDEVICE_CPU);
	bool *source = src->GetData(MEMORYDEVICE_CPU);
	int dataSize = static_cast<int>(dst->dataSize);
	
	for (int idx = 0; idx < dataSize; idx++) {
		bool sourceVal = source[idx];
		if (sourceVal){
			dest[idx].r = (uchar)(255.0f);
			dest[idx].b = (uchar)(255.0f);
			dest[idx].g = (uchar)(255.0f);
		}
		else {
			dest[idx].r = (uchar)(0.0f);
			dest[idx].b = (uchar)(0.0f);
			dest[idx].g = (uchar)(0.0f);
		}
	}
}

void IITMVisualisationEngine::IntToUchar4(ITMUChar4Image *dst, ITMIntImage *src) {
	Vector4u *dest = dst->GetData(MEMORYDEVICE_CPU);
	int *source = src->GetData(MEMORYDEVICE_CPU);
	int dataSize = static_cast<int>(dst->dataSize);

	memset(dst->GetData(MEMORYDEVICE_CPU), 0, dataSize * 4);

	Vector4u *destUC4;
	int lims[2] ;
	float scale, sourceVal2;

	destUC4 = (Vector4u*)dest;
	/*lims[0] = 100000; lims[1] = -100000;

	for (int idx = 0; idx < dataSize; idx++)
	{
		int sourceVal = source[idx];
		if (sourceVal > 0.0f) { lims[0] = MIN(lims[0], sourceVal); lims[1] = MAX(lims[1], sourceVal); }
	}

	scale = ((lims[1] - lims[0]) != 0) ? 1.0f / float(lims[1] - lims[0]) : float(1.0f / lims[1]);*/

	//scale = 200.0f ;

	// if (lims[0] == lims[1]) return;
    int test1, test2, test3 ;
	for (int idx = 0; idx < dataSize; idx++)
	{
		int sourceVal = source[idx];

		if (sourceVal > 0)
		{
			//sourceVal2 = double(sourceVal) * scale;

            test1 = ((sourceVal & 0xff) << 5) % 256 ; //& 0xff;
            test2 = ((sourceVal & 0xff) << 1) % 256 ; //& 0xff;
            test3 = ((sourceVal & 0xff) << 3) % 256 ; // 0xff;

            destUC4[idx].r =  (uchar)test1 ;
            destUC4[idx].g =  (uchar)test2 ;
            destUC4[idx].b =  (uchar)test3 ;
			destUC4[idx].a = 255 ;
		}
		else if (sourceVal == -1) {
			destUC4[idx].r =  (uchar)128 ;
			destUC4[idx].g =  (uchar)128 ;
			destUC4[idx].b =  (uchar)128 ;
			destUC4[idx].a = 255 ;
		}
	}
}

void IITMVisualisationEngine::ShortToUchar4(ITMUChar4Image *dst, ITMShortImage *src) {
	Vector4u *dest = dst->GetData(MEMORYDEVICE_CPU);
	short *source = src->GetData(MEMORYDEVICE_CPU);
	int dataSize = static_cast<int>(dst->dataSize);

	memset(dst->GetData(MEMORYDEVICE_CPU), 0, dataSize * 4);

	Vector4u *destUC4;
	int lims[2] ;
	float scale, sourceVal2;

	destUC4 = (Vector4u*)dest;
	
    short test1, test2, test3 ;
	for (int idx = 0; idx < dataSize; idx++)
	{
		short sourceVal = source[idx];
//std::cout << "souce vql : " << sourceVal << std::endl ;
		if (sourceVal > 0)
		{
			//sourceVal2 = double(sourceVal) * scale;

            test1 = ((sourceVal & 0xff) << 5) % 256 ; //& 0xff;
            test2 = ((sourceVal & 0xff) << 1) % 256 ; //& 0xff;
            test3 = ((sourceVal & 0xff) << 3) % 256 ; // 0xff;

            destUC4[idx].r =  (uchar)test1 ;
            destUC4[idx].g =  (uchar)test2 ;
            destUC4[idx].b =  (uchar)test3 ;
			destUC4[idx].a = 255 ;

           }
	}
}

template class ITMLib::Engine::ITMVisualisationEngine<ITMVoxel, ITMVoxelIndex>;
