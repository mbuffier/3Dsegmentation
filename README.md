# Incremental instance-level 3D segmentation performed by fusing geometric and deep-learning cues

This repository contains code from my master project at University College London. It aims at incrementally achieving an object-aware 3D segmentation by fusing geometrical and deep-learning frame-wise information.
It's build on InfiniTam (https://github.com/victorprad/InfiniTAM). 

## Getting Started

### Prerequisites

Everything InfiniTam needs

OpenCV 

### Build Process

```
  $ mkdir build
  $ cd build
  $ mkdir normals
  $ cmake ../InfiniTAM/ 
  $ make
```

## Run the code

### Run with own images

To test with images, the image folder needs to be in build and use the same format as InfiniTam (calib file and a 'frames' folder)

To test without semantic segmentation use : 
```
./InfiniTAM folderName/calib.txt folderName/Frames/%04i.ppm folderName/Frames/%04i.pgm
```
and with semantic segmentation : 
```
./InfiniTAM folderName/calib.txt folderName/Frames/%04i.ppm folderName/Frames/%04i.pgm folderName/Frames/sem_seg%04i.pgm
```

### Saving results 
The code to save result is in UIEngine.cpp, result folders need to be added to the sequence folder. 
For example : 
```
build
  sequence
    frames
    calib.txt
    out
```

## Thanks !



