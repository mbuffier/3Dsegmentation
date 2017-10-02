// created by Maud Buffier 


#ifndef NORMALSEGMENTATION_H
#define NORMALSEGMENTATION_H

#pragma once

//#include "../ITMLib/Utils/ITMLibDefines.h"
#include "../Utils/ITMMath.h"
#include "../Utils/ITMLibDefines.h"
#include "../../ORUtils/Vector.h"
#include <vector>
#include <stdlib.h>     
#include <cmath>
#include <iostream>
#include <stack>  
#include <utility>    
#include <tuple>
#include <functional>   


// segment in fonction of the angle between normals (look for convexity)
_CPU_AND_GPU_CODE_ void normEdgeSegmentation(DEVICEPTR(float) *normProbaMapOut, DEVICEPTR(bool) *normEdgeMapOut, float threshold, const CONSTPTR(Vector4f) *depthNormal, const CONSTPTR(Vector4f) *vertexMap, int x, int y, Vector2i imgDims) {
	float minPhi = 1 ; float thisPhi, test ;
	int idx_current = x + y*imgDims.x; int thisIndex ;
	Vector4f diff4D; Vector3f diff3D ; Vector3f normal3D ; Vector3f neigh3D ;
	
	// store the index of the 3 neighboors in the 8 directions 
	std::vector<int> neighbors ;

	neighbors.push_back((x-1) + (y-1)*imgDims.x) ; neighbors.push_back((x) + (y-1)*imgDims.x) ; neighbors.push_back((x+1) + (y-1)*imgDims.x) ;
	neighbors.push_back((x-1) + y*imgDims.x) ; neighbors.push_back((x+1) + y*imgDims.x) ;
	neighbors.push_back((x-1) + (y+1)*imgDims.x) ; neighbors.push_back((x) + (y+1)*imgDims.x) ; neighbors.push_back((x+1) + (y+1)*imgDims.x) ;

	// if uncertain, treated as an edge 
	if (depthNormal[idx_current].w == -1.0f) {
		normEdgeMapOut[idx_current] = false ;
		return ;
	}

	// get back the 3D normal 
	normal3D.x = depthNormal[idx_current].x ; normal3D.y = depthNormal[idx_current].y ;normal3D.z = depthNormal[idx_current].z ;
	normal3D /= sqrt(normal3D.x*normal3D.x + normal3D.y*normal3D.y + normal3D.z*normal3D.z) ;

	//std::cout << "normal3D : " << normal3D << std::endl ;
	// for all the direction 
	for (int i=0; i< 8; i++) {
		thisIndex = neighbors[i] ;

		// compute the normalized difference vector  
		diff4D = vertexMap[thisIndex] - vertexMap[idx_current] ;
		diff3D.x = diff4D.x ; diff3D.y = diff4D.y ;diff3D.z = diff4D.z ;
		// diff3D /= sqrt(diff3D.x*diff3D.x + diff3D.y*diff3D.y + diff3D.z*diff3D.z) ;

		// compute the 3D normal for this neighboor
		if (depthNormal[thisIndex].w == -1.0f) continue ;
		else {
			neigh3D.x = depthNormal[thisIndex].x ; neigh3D.y = depthNormal[thisIndex].y ;neigh3D.z = depthNormal[thisIndex].z ;
			neigh3D /= sqrt(neigh3D.x*neigh3D.x + neigh3D.y*neigh3D.y + neigh3D.z*neigh3D.z) ;
		}

		// compute the angle 
		test = dot(diff3D, normal3D) ;
		
		// the more concave, the lower thisPhi
		if (test < 0) thisPhi=1 ;
		else thisPhi = dot(normal3D, neigh3D) ;

		// store if it's the min 
		if (thisPhi < minPhi) minPhi=thisPhi ;
	}
	//std::cout << "minPhi : " << minPhi << std::endl ;
	//threshold = 0.95 ;
	// test for the threshold 
	if (minPhi < threshold) normEdgeMapOut[idx_current] = false ; // 1 == edge
	else normEdgeMapOut[idx_current] = true ;
}

// segment in fonction of the distance between neighboring vertices 
_CPU_AND_GPU_CODE_ inline void goeEdgeSegmentation(DEVICEPTR(float) *geoProbaMapOut, DEVICEPTR(bool) *geoEdgeMapOut,const CONSTPTR(Vector4f) *depthNormal, const CONSTPTR(Vector4f) *vertexMap, const CONSTPTR(float) *depthUncertainty, int x, int y, Vector2i imgDims) {
	float maxGamma = 0 ; float thisGamma ;
	int idx_current = x + y*imgDims.x; int thisIndex ;
	Vector4f diff4D; Vector3f diff3D ; Vector3f normal3D ;

	// if uncertain, treated as an edge 
	if (vertexMap[idx_current].w == -1.0f) {
		geoEdgeMapOut[idx_current] = false ;
		geoProbaMapOut[idx_current] = 0.0f ;
		return ;
	}
	else geoEdgeMapOut[idx_current] = true ;

	// store the index of the neighboors 
	std::vector<int> neighbors ;
	neighbors.push_back((x-1) + (y-1)*imgDims.x) ; neighbors.push_back((x) + (y-1)*imgDims.x) ; neighbors.push_back((x+1) + (y-1)*imgDims.x) ;
	neighbors.push_back((x-1) + y*imgDims.x) ; neighbors.push_back((x+1) + y*imgDims.x) ;
	neighbors.push_back((x-1) + (y+1)*imgDims.x) ; neighbors.push_back((x) + (y+1)*imgDims.x) ; neighbors.push_back((x+1) + (y+1)*imgDims.x) ;

	// get back the 3D normal 
	normal3D.x = depthNormal[idx_current].x ; normal3D.y = depthNormal[idx_current].y ;normal3D.z = depthNormal[idx_current].z ;
	//normal3D /= sqrt(normal3D.x*normal3D.x + normal3D.y*normal3D.y + normal3D.z*normal3D.z) ;

	for (int i=0; i< 8; i++) {
		thisIndex = neighbors[i] ;
		// compute the normalized difference vector  

		if (vertexMap[thisIndex].w == -1.0f) continue ;

		diff4D = vertexMap[thisIndex] - vertexMap[idx_current] ;
		diff3D.x = diff4D.x ; diff3D.y = diff4D.y ;diff3D.z = diff4D.z ;

		thisGamma = std::abs(dot(diff3D, normal3D)) ;
	
		// look for the max distance 
		if (thisGamma > maxGamma) maxGamma = thisGamma ;
	}

	// test for the threshold 	
	if (maxGamma > depthUncertainty[idx_current]) {
		geoEdgeMapOut[idx_current] = 0 ; // 0 == edge
		geoProbaMapOut[idx_current] = depthUncertainty[idx_current] - maxGamma ; }
	else {
		geoEdgeMapOut[idx_current] = 1 ;
		geoProbaMapOut[idx_current] = 0.0f ; }
}

//  create the final map by fusing the 2 segmentation 
_CPU_AND_GPU_CODE_ inline void mapFusion(DEVICEPTR(float) *finalProbaMapOut, const CONSTPTR(float) *normProbaMapOut, const CONSTPTR(float) *geoProbaMapOut, 
			DEVICEPTR(bool) *finalEdgeMapOut, const CONSTPTR(bool) *normEdgeMapOut, const CONSTPTR(bool) *geoEdgeMapOut, int x, int y, Vector2i imgDims) {
	int idx_current = x + y*imgDims.x ;

	if ( !geoEdgeMapOut[idx_current] || !normEdgeMapOut[idx_current]) {
		finalEdgeMapOut[idx_current] = false ;
		finalProbaMapOut[idx_current] = std::max(normProbaMapOut[idx_current], geoProbaMapOut[idx_current]) ; }
	else {
		finalEdgeMapOut[idx_current] = true ;
		finalProbaMapOut[idx_current] = 0.0f ; }

	//finalEdgeMapOut[idx_current] = normEdgeMapOut[idx_current] ;
}

_CPU_AND_GPU_CODE_ inline void erosion(DEVICEPTR(bool) *edgeMap_out, CONSTPTR(bool) * edgeMap_in, int x, int y, Vector2i imgDims) {
	bool ind1, ind2, ind3, ind4, ind5, ind6, ind7, ind8, ind9 ;

	ind5 = edgeMap_in[x+y*imgDims.x] ; ind1 = edgeMap_in[x-1+(y-1)*imgDims.x] ; ind2 = edgeMap_in[x+(y-1)*imgDims.x] ;  
	ind3 = edgeMap_in[x+1+(y-1)*imgDims.x] ;
	ind4 = edgeMap_in[(x-1)+y*imgDims.x] ; ind6 = edgeMap_in[(x+1)+y*imgDims.x] ; 
	ind7 = edgeMap_in[(x-1)+(y+1)*imgDims.x] ;  ind8 = edgeMap_in[x+(y+1)*imgDims.x] ; ind9 = edgeMap_in[x+1+(y+1)*imgDims.x] ;

	if (ind1 || ind2 || ind3 || ind4 || ind5 || ind6 || ind7|| ind8) edgeMap_out[x+y*imgDims.x] = true ;
		else edgeMap_out[x+y*imgDims.x] = false ;
}

_CPU_AND_GPU_CODE_ inline void dilate(DEVICEPTR(bool) *edgeMap_out, CONSTPTR(bool) * edgeMap_in, int x, int y, Vector2i imgDims) {
	bool ind1, ind2, ind3, ind4, ind5, ind6, ind7, ind8, ind9 ;

	ind5 = edgeMap_in[x+y*imgDims.x] ; ind1 = edgeMap_in[x-1+(y-1)*imgDims.x] ; ind2 = edgeMap_in[x+(y-1)*imgDims.x] ;  
	ind3 = edgeMap_in[x+1+(y-1)*imgDims.x] ;
	ind4 = edgeMap_in[(x-1)+y*imgDims.x] ; ind6 = edgeMap_in[(x+1)+y*imgDims.x] ; 
	ind7 = edgeMap_in[(x-1)+(y+1)*imgDims.x] ;  ind8 = edgeMap_in[x+(y+1)*imgDims.x] ; ind9 = edgeMap_in[x+1+(y+1)*imgDims.x] ;

	if (ind1 && ind2 && ind3 && ind4 && ind5 && ind6 && ind7&& ind8) edgeMap_out[x+y*imgDims.x] = true ;
	else edgeMap_out[x+y*imgDims.x] = false ;
}

// label the map using connected component algorithm 
_CPU_AND_GPU_CODE_ inline void label(const CONSTPTR(bool) * edgeMapOut,DEVICEPTR(int) * labelMapOut, Vector2i imgDims, bool& newGroup, 
									std::vector<int>& visited, std::stack<std::pair<int,int> >& neighbors, int connectivity, int& current_label, 
									int x, int y) {
    
	int thisIdx = x + y*imgDims.x, thisIdx2, x_current, y_current ;
    std::pair<int,int> thisPair ; 

    if (visited.at(thisIdx) == 0) {
       	neighbors.push(std::make_pair(x,y)) ;
				
        while (!neighbors.empty()) {
            thisPair = neighbors.top() ;
            neighbors.pop() ;
            x_current = thisPair.first  ; y_current = thisPair.second ;

			if (x_current < 2 || y_current < 2 || x_current > imgDims.x - 2 || y_current > imgDims.y - 2) continue ;

            thisIdx2 = x_current + y_current*imgDims.x ;
            if (visited.at(thisIdx2) == 0) {
                visited.at(thisIdx2) = 1 ;

                // check the condition 
                if (edgeMapOut[thisIdx2] == true) {
					newGroup = true ;
                    labelMapOut[thisIdx2] = current_label ;

                    // fill the stack with the neigboors 
                    if (connectivity==8) {	// 8 connected components 
						neighbors.push(std::make_pair(x_current-1,y_current-1)) ; neighbors.push( std::make_pair(x_current,y_current-1)) ; neighbors.push( std::make_pair(x_current+1,y_current-1)) ;
		                neighbors.push(std::make_pair(x_current-1,y_current)) ; neighbors.push( std::make_pair(x_current+1,y_current)) ;
		                neighbors.push(std::make_pair(x_current-1,y_current+1)) ; neighbors.push( std::make_pair(x_current,y_current+1)) ; neighbors.push( std::make_pair(x_current+1,y_current+1)) ;
                    }
	                else if (connectivity==4) {	// 4 connected components 
	                    neighbors.push(std::make_pair(x_current,y_current-1)) ;	
						neighbors.push(std::make_pair(x_current-1,y_current)) ; neighbors.push(std::make_pair(x_current+1,y_current)) ;
		                neighbors.push(std::make_pair(x_current,y_current+1)) ; 
	                }   
                }
                else {
                    labelMapOut[thisIdx2] = 0 ;  
                }
            }
        }
        if (newGroup) {
			current_label +=1 ;
			newGroup = false ;
		}
    }
}

#endif //NORMALSEGMENTATION_H
