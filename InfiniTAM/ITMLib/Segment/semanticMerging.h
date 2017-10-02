#ifndef SEMANTICMERGING_H
#define SEMANTICMERGING_H

#pragma once

#include <vector>
#include <cmath>
#include <algorithm>
#include <iostream>
#include <map>
#include <iterator>
#include <utility>

using key_type = std::pair<int,int>;
using map_type = std::map<key_type,float>;

// find the possible merging 
_CPU_AND_GPU_CODE_ void findSemanticMerging(std::map<int, int>& merging, DEVICEPTR(int)* label_map, const CONSTPTR(short) *semantic_map, Vector2i imgDims) {
    int l_sem, l_geo, size_l_geo, thisPos ; 
    
    // find the sizes for the geometric label 
    size_l_geo = *std::max_element(label_map,label_map+imgDims.x*imgDims.y) ;

    // to divide after 
    std::vector<float> numb_l_geo(size_l_geo+1) ;
    map_type pi_map ;

    for (int y = 2; y < imgDims.y - 2; y++){
		for (int x = 2; x < imgDims.x - 2; x++) {
	        thisPos = x + y*imgDims.x ;
 
            // find the label associated in the geometric and semantic segmentation              
            l_sem = int(semantic_map[thisPos]) ;
            l_geo = label_map[thisPos] ;

            if (l_geo!= 0) {  
                numb_l_geo[l_geo] += 1.0f ;
                if (l_sem != 0) pi_map[std::pair<int,int>(l_geo, l_sem)] += 1.f;
            }
		}
    }
    
    // will store the sementic label and the first geometric label associated with it 
    std::map<int, int> store_l_seg ;
    float thisValue ;
    std::pair<std::map<int,int>::iterator,bool> find_value ;

    for(auto it = pi_map.begin(); it!=pi_map.end(); ++it) {
        thisValue = it->second / numb_l_geo[it->first.first] ;

        if ( thisValue > 0.8f && numb_l_geo[it->first.first] > 10) { // pass the first test : need to be associated 
            find_value = store_l_seg.insert(std::pair<int,int>(it->first.second,it->first.first)) ;

            // change the label in merging 
            if (find_value.second == false ) {
                merging[it->first.first] = store_l_seg[it->first.second] ;
            }
        }
    }
}

// go through the map and change the label if needed 
_CPU_AND_GPU_CODE_ void mergingSemantic(std::map<int, int> merging , DEVICEPTR(int)* label_map, const CONSTPTR(short) *semantic_map, 
                                        int x, int y, Vector2i imgDims) {
    
    // go throught the merging map and give each label in the vector the value of the first one 
    int idx_current = x + y*imgDims.x;
    int l_geo = label_map[idx_current] ;

    if (l_geo != 0 && merging.find(l_geo) != merging.end()) { // if their is some label to merge 
        label_map[idx_current] = merging[l_geo] ;
    }
}

#endif //SEMANTICMERGING_H
