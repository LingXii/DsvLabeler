# Introduction
This is a rule-based classifier for 3D Lidar data semantic segmentation. The origin point cloud is transformed into a range image, and then we use a region grow method to do segmentation. Then, every region in the range image will get a label by the classifier, which consider the geometric feature of the region.  
![fig0](https://github.com/LingXii/DsvLabeler/blob/master/res/fig0.png)  

# Algorithm Detail
## Step1: Smooth Data
For some nonvalid Lidar points, use linear interpolation to make them valid, and the remains will be labeled with NONVALID.  
Function `SmoothingData()` is used to do this step.  

## Step2: Extract Ground
Find the ground and too high points by height information, these points will be labeled with ROAD or HIGH.   
Function `EstimateGround()` and `LabelBackground()` is used to do this step.  

## Step3: Extract Edge Points
If a point **p**, at least one point **q** which is one of the four neighbours of **p**, is far away from **p**, or **q** is NONVALID, ROAD, HIGH, then **p** will be labeled with EDGEPT.  
Function `ContourSegger()` is used to do this step.  
Now, we get a segmentation like the following figure, where white denotes NONVALID, blue denotes ROAD, black denotes HIGH, shallow grey denotes EDGEPT, deep grey denotes UNKNOWN.  
![fig1](https://github.com/LingXii/DsvLabeler/blob/master/res/fig1.png)  

## Step4: Finish segmentation
Let some edge points join in the neighbour region, and record the geometric feature of the region in the `SEGBUF` structure.  
```
typedef	struct {
	unsigned short	lab;// label
	int		prid;
	point2i		dmin;
	point2i		dmax;   // a bounding box which contain the region in range image
	point3d		maxp;
	point3d		minp;   // a 3D bounding box which contain the region in space
	int		ptnum;  // number of points
	point3d		maxpH;
	point3d		minpH;  // a 3D bounding box which contain the high part of region in space
	int		ptnumH; // number of points of the high part
	point3d		cp;     // center of the region
	double		wi;     // length of diagonal of the cross section
	double		minDisH;// minimum space distance of horizontal neighbour pixels
	double		minDisV;// minimum space distance of vertical neighbour pixels
	double		maxDisH;
	double		maxDisV;
} SEGBUF;
```
Function `EstimateSeg ()` is used to do this step. And we get a final segmentation like the following figure:  
![fig2](https://github.com/LingXii/DsvLabeler/blob/master/res/fig2.png)  

## Step5: Classify Region
Classify every region by height and thickness of the region. The rule of classification is like:  
![fig3](https://github.com/LingXii/DsvLabeler/blob/master/res/fig3.png)  
Function `ClassiSeg()` is used to do this step. Finally, we get a semantic segmentation result.  

# Using PCL - LCCPSegmentation Algorithm
Function `LCCP_seg(string s_time)` in `pclmethod.cpp` is used to do segmentation using LCCP algorithm, which input the timestamp and return the labeled point cloud pointer. This function need point cloud data, you can transform the dsvl data to point cloud format by `transformToPclFormat()` function.  
Then The labeled point cloud will be projected to the range image, and the rule-based classifier mentioned before will help to classify every region.  
Now, two mode is defined in line 12,13 in `main.cpp`. If you're in mode `RG`, you will use region growing method to do segmentation and get the point cloud data transformed by dsvl data. If you're in mode `LCCP`, you will use LCCP method to do segmentation.  

# Evaluating result
File `acc.py` is used to evaluate the segmentation and classification result. Before you run the code, ground truth images and result images should be in the `IMAGE_PATH`.
