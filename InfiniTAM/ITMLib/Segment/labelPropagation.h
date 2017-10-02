// created by Maud Buffier 

#ifndef LABELPROPAGATION_H
#define LABELPROPAGATION_H

#include "../Utils/ITMLibDefines.h"
#include "../Engine/DeviceAgnostic/ITMVisualisationEngine.h"
#include "../Objects/ITMScene.h"
#include "../Engine/DeviceAgnostic/ITMRepresentationAccess.h"

#include <vector>
#include <cmath>
#include <algorithm>
#include <iostream>
#include <map>
#include <iterator>
#include <utility>

#pragma once

using key_type = std::pair<int,int>;
using map_type = std::map<key_type,float>;

// compute the projection of the label map into the camera frame -> V1 forward mapping 
/*template<class TVoxel> 
_CPU_AND_GPU_CODE_ inline void computeProjectionMap(CONSTPTR(TVoxel) &voxel, const CONSTPTR(Vector4f) & pt_model, 
	const CONSTPTR(Matrix4f) & M_d, const CONSTPTR(Vector4f) & projParams_d,
	DEVICEPTR(int) *labelMap, const CONSTPTR(Vector2i) & imgSize)
{
	Vector4f pt_camera; Vector2f pt_image;
	int thisLabel;

	// project point into camera frame
	pt_camera = M_d * pt_model;

	// project the point into image frame 
	pt_image.x = projParams_d.x * pt_camera.x / pt_camera.z + projParams_d.z;
	pt_image.y = projParams_d.y * pt_camera.y / pt_camera.z + projParams_d.w;
	if ((pt_image.x < 1) || (pt_image.x > imgSize.x - 2) || (pt_image.y < 1) || (pt_image.y > imgSize.y - 2)) 
		return ;
    else if (pt_camera.z <= 0)
        labelMap[(int)(pt_image.x + 0.5f) + (int)(pt_image.y + 0.5f) * imgSize.x] = 0 ;

	// get the label for this voxel 
	thisLabel = voxel.label ;
	
    // project the label into image frame 
	labelMap[(int)(pt_image.x + 0.5f) + (int)(pt_image.y + 0.5f) * imgSize.x] = thisLabel ;
}*/

// compute the projection of the label map into the camera frame -> V2 backward mapping using raycasting 
template<class TVoxel, class TIndex>
_CPU_AND_GPU_CODE_ inline void computeProjectionMap_backward(const ITMScene<TVoxel, TIndex> *scene, const CONSTPTR(Vector2i)& imgSize, 
    const CONSTPTR(Matrix4f)& invM, Vector4f projParams, const CONSTPTR(ITMRenderState) *renderState, DEVICEPTR(int) *labelMap)
{
	projParams.x = 1.0f / projParams.x;
	projParams.y = 1.0f / projParams.y;

	const Vector2f *minmaximg = renderState->renderingRangeImage->GetData(MEMORYDEVICE_CPU);
	float mu = scene->sceneParams->mu;
	float oneOverVoxelSize = 1.0f / scene->sceneParams->voxelSize;
	Vector4f *pointsRay = renderState->raycastResult->GetData(MEMORYDEVICE_CPU);	
    
    const TVoxel *voxelData = scene->localVBA.GetVoxelBlocks();
	const typename TIndex::IndexData *voxelIndex = scene->index.getIndexData();

    // want to get the label all the time 
    int conf = 5 ;

#ifdef WITH_OPENMP
	#pragma omp parallel for
#endif
	for (int locId = 0; locId < imgSize.x*imgSize.y; ++locId)
	{
		int y = locId/imgSize.x;
		int x = locId - y*imgSize.x;
		int locId2 = (int)floor((float)x / minmaximg_subsample) + (int)floor((float)y / minmaximg_subsample) * imgSize.x;

		// modification to be able to find the label of the voxel 
		castRayColor<TVoxel, TIndex>(
			pointsRay[locId],
            labelMap[locId], // the one we want to fill from the model 
            conf,
			x, y,
			voxelData,
			voxelIndex,
			invM,
			projParams,
			oneOverVoxelSize,
			mu,
            minmaximg[locId2], 
            true
		);
	}
}

// compare labels for each positions an increment the map 
_CPU_AND_GPU_CODE_ inline bool propagate_label(const CONSTPTR(Vector4f)* depthN_mod,const CONSTPTR(Vector4f)* vertex_mod, const CONSTPTR(float)* depth_un, 
    const CONSTPTR(Vector4f)* depthN_cur, const CONSTPTR(Vector4f)* vertex_cur, const CONSTPTR(int)* pro_label_mod, 
    const CONSTPTR(int)* labelMap, DEVICEPTR(int)* pro_label_cur, const CONSTPTR(Vector2i) & imgSize, 
    std::map<std::pair<int, int>, int>& mergingRecord, std::vector<int>& labelToMerge) {

    int size_li, size_lj, li, lj, thisPos ;
    
    // find the sizes 
    size_li = *std::max_element(pro_label_mod,pro_label_mod+imgSize.x*imgSize.y) ;
    size_lj = *std::max_element(labelMap,labelMap+imgSize.x*imgSize.y) ;

    // no label on the voxel -> the label are exactly 
    if (size_li==0) {
        for (int y = 2; y < imgSize.y - 2; y++)
            for (int x = 2; x < imgSize.x - 2; x++) {
                pro_label_cur[x + y*imgSize.x] = labelMap[x + y*imgSize.x] ;
            }
	    return false ;
    }

    std::vector<float> numb_lj(size_lj+1) ;
    float vertex_dist, angle ; 
    Vector3f norm_mod, norm_cur, vertexM3D, vertexS3D ;

    // fill the map 
    map_type pi_matrix ;

    for (int y = 2; y < imgSize.y - 2; y++){
		for (int x = 2; x < imgSize.x - 2; x++) {
	        thisPos = x + y*imgSize.x ;
            // this is uncertained
            if (vertex_mod[thisPos].w == -1.0f || vertex_cur[thisPos].w == -1.0f ) continue ; 

			// compute the distance between the vertices and the angle between normals
            norm_mod.x = depthN_mod[thisPos].x ; norm_mod.y= depthN_mod[thisPos].y ; norm_mod.z = depthN_mod[thisPos].z ;
            norm_cur.x = depthN_cur[thisPos].x ; norm_cur.y= depthN_cur[thisPos].y ; norm_cur.z = depthN_cur[thisPos].z ;
            angle = std::abs(dot(norm_cur, norm_mod)) ; 
            
            vertexM3D.x = vertex_mod[thisPos].x ;vertexM3D.y = vertex_mod[thisPos].y ; vertexM3D.z = vertex_mod[thisPos].z ;
            vertexS3D.x = vertex_cur[thisPos].x ;vertexS3D.y = vertex_cur[thisPos].y ; vertexS3D.z = vertex_cur[thisPos].z ;
            vertex_dist = dot(vertexM3D - vertexS3D, normalize(vertexS3D)) ;

            lj = labelMap[thisPos] ;

            // increment if conditions are respected 
            if (angle > 0.6f) { // && std::abs(vertex_dist) < std::abs(2.0*depth_un[thisPos])) {
                li = pro_label_mod[thisPos] ; 
                if (li == 0 || lj == 0) continue ; // if one is zero, out
                pi_matrix[std::pair<int,int>(lj, li)] += 1.f;
            }
            numb_lj[lj] += 1.0f ;
		}
	}

    std::vector<std::pair<int,float> > max_pi(size_lj+1, std::make_pair(-1, -1)) ;
    std::vector<std::vector<int> > other_pi(size_lj+1) ;

    // compute for each lj the li with the maximum overlapping 
    float thisValue ;
    for(auto it = pi_matrix.begin(); it!=pi_matrix.end(); ++it) {

        thisValue = it->second / numb_lj[it->first.first] ;
        // max overlapping 
        if ( thisValue > max_pi[it->first.first].second ) {
            max_pi[it->first.first].first = it->first.second ;
            max_pi[it->first.first].second = thisValue ;    
        }
        // every overlapping with more than 23% 
        if (thisValue > 0.23) {
            other_pi[it->first.first].push_back(it->first.second) ;   // keep trace also with the ones above 0.2
        }
    }

    // compute the propagated image 
    int thisPos2 ; 
    for (int y1 = 2; y1 < imgSize.y - 2; y1++){
		for (int x1 = 2; x1 < imgSize.x - 2; x1++) {
            // to fill the map 
	        thisPos2 = x1 + y1*imgSize.x ;
            if (max_pi[labelMap[thisPos2]].first > -1 && max_pi[labelMap[thisPos2]].second > 0.3)
                    pro_label_cur[thisPos2] = max_pi[labelMap[thisPos2]].first ;
            else if(numb_lj[labelMap[thisPos2]] > 50.0f ) // propagate from the current depth map 
                    pro_label_cur[thisPos2] = labelMap[thisPos2] ;                  
            else pro_label_cur[thisPos2] = 0 ; // default 
		}
	}

    // create a vector of pair for merging 
    std::vector<std::pair<int, int> > pair2merged ; 
    // fill the merging record 
    for (int i=0; i<size_lj+1 ; i++) {
        for(int j=0; j < other_pi[i].size() ; j++) {
            if (other_pi[i][j] != max_pi[i].first) {
                pair2merged.push_back(std::minmax(max_pi[i].first, other_pi[i][j])) ;
            }
        } 
    }

    bool mergingNeeded = false ;
    // update merging and store the pair to merge if they exist 
    for(auto it = mergingRecord.begin(); it != mergingRecord.end(); ++it) {
        auto iter = find(pair2merged.begin(), pair2merged.end(), it->first) ;
        // if we find it in pair to merged  
        if (iter != pair2merged.end()) {
            it->second += 1 ;
            if (it->second >= 5) { // if it's seen more than 5 times (in a row)
                mergingNeeded = true ;
                labelToMerge.push_back(it->first.first) ; labelToMerge.push_back(it->first.second) ;
                mergingRecord.erase(it) ; // erasing by key 
            }
            pair2merged.erase(iter) ;
        }
        else it->second = std::max(0, it->second-1) ;
    }

    // add the new pairs 
    for(auto it = pair2merged.begin(); it != pair2merged.end(); it++) mergingRecord[(*it)] += 1 ;

    return mergingNeeded ; // 1 if a merging is needed
}

// go through all the current gsm to merge detected labels 
template<class TVoxel>
_CPU_AND_GPU_CODE_ inline void merging(std::vector<int> labelToMerge, ITMScene<TVoxel, ITMVoxelBlockHash> *scene) {
    // get back the datas to perform merging 
    TVoxel *localVBA = scene->localVBA.GetVoxelBlocks();
	ITMHashEntry *hashTable = scene->index.GetEntries(); // -> structure stored 
    int newLabel ;

#ifdef WITH_OPENMP
	#pragma omp parallel for
#endif

	for (int i = 0 ; i< ITMLib::Objects::ITMVoxelBlockHash::noTotalEntries; i++)
	{
		Vector3i globalPos;
		const ITMHashEntry &currentHashEntry = hashTable[i];

		if (currentHashEntry.ptr < 0) continue;

		TVoxel *localVoxelBlock = &(localVBA[currentHashEntry.ptr * (SDF_BLOCK_SIZE3)]);

		for (int z = 0; z < SDF_BLOCK_SIZE; z++) for (int y = 0; y < SDF_BLOCK_SIZE; y++) for (int x = 0; x < SDF_BLOCK_SIZE; x++)
		{
			int locId = x + y * SDF_BLOCK_SIZE + z * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE;

            if (localVoxelBlock[locId].label == 0) continue ;

            auto it = std::find(labelToMerge.begin(), labelToMerge.end(), localVoxelBlock[locId].label) ;
            if (it == labelToMerge.end()) continue ;
            else {
                if ((it - labelToMerge.begin()) % 2) newLabel = labelToMerge[it - labelToMerge.begin()-1] ;
                else newLabel = labelToMerge[it - labelToMerge.begin()] ;

                Vector3u color ; 
                color.x =  (uchar)(((newLabel & 0xff) << 5) % 256) ;
                color.y =  (uchar)(((newLabel & 0xff) << 1) % 256 ) ;
                color.z =  (uchar)(((newLabel & 0xff) << 3) % 256) ;

                localVoxelBlock[locId].label = newLabel ;
                localVoxelBlock[locId].confidence = 7 ;
                localVoxelBlock[locId].clr = color ;
            }
		}
	}

}

// test to improve the map 
_CPU_AND_GPU_CODE_ inline void erosion(DEVICEPTR(int) *label_in, Vector2i imgDims) {
	int count ,thisLabel ;

	for (int y = 2; y < imgDims.y - 2; y++){
		for (int x = 2; x < imgDims.x - 2; x++) {
			if (label_in[x+y*imgDims.x] == 0) { // if the label is zero 
                count =0; 

                // find the first non-zero label 
                if (label_in[x-1+(y-1)*imgDims.x] != 0) thisLabel = label_in[x-1+(y-1)*imgDims.x] ;// label 1
                else if (label_in[x+(y-1)*imgDims.x] != 0) thisLabel = label_in[x+(y-1)*imgDims.x]; // 2
                else if (label_in[x+1+(y-1)*imgDims.x] != 0) thisLabel = label_in[x+1+(y-1)*imgDims.x]; //3
                else if (label_in[(x-1)+y*imgDims.x] != 0) thisLabel = label_in[(x-1)+y*imgDims.x] ;//4
                else if (label_in[(x+1)+y*imgDims.x] != 0) thisLabel = label_in[(x+1)+y*imgDims.x] ;// 6
                else if (label_in[(x-1)+(y+1)*imgDims.x] != 0) thisLabel = label_in[(x-1)+(y+1)*imgDims.x]; // 7
                else continue ;

                // count the occurence of this one 
                if (label_in[x-1+(y-1)*imgDims.x] == thisLabel) count++ ; //label 1
                else continue ;
                if (label_in[x+(y-1)*imgDims.x] == thisLabel) count++ ; //label 2
                else continue ;
                if (label_in[x+1+(y-1)*imgDims.x] == thisLabel) count++ ; //label 3
                else continue ;
                if (label_in[(x-1)+y*imgDims.x] == thisLabel) count++ ; //label 4
                else continue ;
                if (label_in[(x+1)+y*imgDims.x] == thisLabel) count++ ; //label 6
                else continue ;  
                if (label_in[(x-1)+(y+1)*imgDims.x] == thisLabel) count++ ; //label 7
                else continue ;
                if (label_in[x+(y+1)*imgDims.x] == thisLabel) count++ ; //label 8
                else continue ;
                if (label_in[x+1+(y+1)*imgDims.x] == thisLabel) count++ ; //label 9
                else continue ;      
                if (count >= 3) label_in[x+y*imgDims.x] = thisLabel ;
            }
		}
	}
}



#endif //LABELPROPAGATION_H
