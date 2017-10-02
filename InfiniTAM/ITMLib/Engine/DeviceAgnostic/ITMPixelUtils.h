// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#pragma once
#ifndef __METALC__
#include <math.h>
#include <ostream>
#endif

#include "../../Utils/ITMLibDefines.h"
#include <map>
#include <utility>
#include <iostream>


template<typename T> _CPU_AND_GPU_CODE_ inline Vector4f interpolateBilinear(const CONSTPTR(T) *source,
	const THREADPTR(Vector2f) & position, const CONSTPTR(Vector2i) & imgSize)
{
	T a, b, c, d; Vector4f result;
	Vector2i p; Vector2f delta;

	p.x = (int)floor(position.x); p.y = (int)floor(position.y);
	delta.x = position.x - (float)p.x; delta.y = position.y - (float)p.y;

	b.x = 0; b.y = 0; b.z = 0; b.w = 0;
	c.x = 0; c.y = 0; c.z = 0; c.w = 0;
	d.x = 0; d.y = 0; d.z = 0; d.w = 0;

	a = source[p.x + p.y * imgSize.x];
	if (delta.x != 0) b = source[(p.x + 1) + p.y * imgSize.x];
	if (delta.y != 0) c = source[p.x + (p.y + 1) * imgSize.x];
	if (delta.x != 0 && delta.y != 0) d = source[(p.x + 1) + (p.y + 1) * imgSize.x];

	result.x = ((float)a.x * (1.0f - delta.x) * (1.0f - delta.y) + (float)b.x * delta.x * (1.0f - delta.y) +
		(float)c.x * (1.0f - delta.x) * delta.y + (float)d.x * delta.x * delta.y);
	result.y = ((float)a.y * (1.0f - delta.x) * (1.0f - delta.y) + (float)b.y * delta.x * (1.0f - delta.y) +
		(float)c.y * (1.0f - delta.x) * delta.y + (float)d.y * delta.x * delta.y);
	result.z = ((float)a.z * (1.0f - delta.x) * (1.0f - delta.y) + (float)b.z * delta.x * (1.0f - delta.y) +
		(float)c.z * (1.0f - delta.x) * delta.y + (float)d.z * delta.x * delta.y);
	result.w = ((float)a.w * (1.0f - delta.x) * (1.0f - delta.y) + (float)b.w * delta.x * (1.0f - delta.y) +
		(float)c.w * (1.0f - delta.x) * delta.y + (float)d.w * delta.x * delta.y);

	return result;
}

_CPU_AND_GPU_CODE_ inline int neighborLabel(const CONSTPTR(int) *source,
	const THREADPTR(Vector2f) & position, const CONSTPTR(Vector2i) & imgSize)
{
	int x = (int)round(position.x); int y = (int)round(position.y); int current_lab ;
	std::map<int, int> lab_occu ;

	// find the label around and their occurance -> with a window of 5 
	int windowSize = 3 ;
	for(int i=x-windowSize; i<=x+windowSize; i++) {
		for(int j=y-windowSize; j<=y+windowSize; j++) {
			if (i != x && j !=y ) {

				if (i<0 || j<0 || i>imgSize.x || j>imgSize.y) continue ; // sanity check 

				current_lab = source[i + j*imgSize.x] ;
				if (current_lab != 0) {
					lab_occu[current_lab] += 1 ; 
				}
			}
		}
	}

	// find the maximal label 
	int lab = 0 ; int max_occu = 0 ; 

	for(std::map<int, int >::iterator it = lab_occu.begin() ; it != lab_occu.end() ; ++it) {
		if (it->second > max_occu) {
			lab = it->first ; 
			max_occu = it->second ;
		}
	}
// std::cout << "lab : " << lab << "max_occu : " << max_occu << std::endl ;
	if (max_occu >= 10) return lab ;
	else return 0 ;
}

template<typename T> _CPU_AND_GPU_CODE_ inline Vector4f interpolateBilinear_withHoles(const CONSTPTR(T) *source,
	const THREADPTR(Vector2f) & position, const CONSTPTR(Vector2i) & imgSize)
{
	T a, b, c, d; Vector4f result;
	Vector2s p; Vector2f delta;

	p.x = (short)floor(position.x); p.y = (short)floor(position.y);
	delta.x = position.x - (float)p.x; delta.y = position.y - (float)p.y;

	a = source[p.x + p.y * imgSize.x];
	b = source[(p.x + 1) + p.y * imgSize.x];
	c = source[p.x + (p.y + 1) * imgSize.x];
	d = source[(p.x + 1) + (p.y + 1) * imgSize.x];

	if (a.w < 0 || b.w < 0 || c.w < 0 || d.w < 0)
	{
		result.x = 0; result.y = 0; result.z = 0; result.w = -1.0f;
		return result;
	}

	result.x = ((float)a.x * (1.0f - delta.x) * (1.0f - delta.y) + (float)b.x * delta.x * (1.0f - delta.y) +
		(float)c.x * (1.0f - delta.x) * delta.y + (float)d.x * delta.x * delta.y);
	result.y = ((float)a.y * (1.0f - delta.x) * (1.0f - delta.y) + (float)b.y * delta.x * (1.0f - delta.y) +
		(float)c.y * (1.0f - delta.x) * delta.y + (float)d.y * delta.x * delta.y);
	result.z = ((float)a.z * (1.0f - delta.x) * (1.0f - delta.y) + (float)b.z * delta.x * (1.0f - delta.y) +
		(float)c.z * (1.0f - delta.x) * delta.y + (float)d.z * delta.x * delta.y);
	result.w = ((float)a.w * (1.0f - delta.x) * (1.0f - delta.y) + (float)b.w * delta.x * (1.0f - delta.y) +
		(float)c.w * (1.0f - delta.x) * delta.y + (float)d.w * delta.x * delta.y);

	return result;
}

template<typename T> _CPU_AND_GPU_CODE_ inline float interpolateBilinear_withHoles_single(const CONSTPTR(T) *source,
	const THREADPTR(Vector2f) & position, const CONSTPTR(Vector2i) & imgSize)
{
	T a = 0, b = 0, c = 0, d = 0; float result;
	Vector2i p; Vector2f delta;

	p.x = (int)floor(position.x); p.y = (int)floor(position.y);
	delta.x = position.x - (float)p.x; delta.y = position.y - (float)p.y;

	a = source[p.x + p.y * imgSize.x];
	if (delta.x != 0) b = source[(p.x + 1) + p.y * imgSize.x];
	if (delta.y != 0) c = source[p.x + (p.y + 1) * imgSize.x];
	if (delta.x != 0 && delta.y != 0) d = source[(p.x + 1) + (p.y + 1) * imgSize.x];

	if (a < 0 || b < 0 || c < 0 || d < 0) return -1;

	result = ((float)a * (1.0f - delta.x) * (1.0f - delta.y) + (float)b * delta.x * (1.0f - delta.y) +
		(float)c * (1.0f - delta.x) * delta.y + (float)d * delta.x * delta.y);

	return result;
}
