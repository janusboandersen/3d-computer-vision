# 3D Computer Vision :rocket:

This project exploratively develops a pipeline for sparse 3D reconstruction and depth-based segmentation from two views. 

The [report](https://github.com/janusboandersen/3d-computer-vision/blob/main/3D%20Vision%2C%20Two-view%20Correspondence%2C%203D%20Reconstruction%20and%20Depth-based%20Segmentation.pdf) covers design and development, theory and algorithm review, and exposition of developed Matlab code. The [slides](https://github.com/janusboandersen/3d-computer-vision/blob/main/slides-3D%20Vision.pdf) introduce the project and give an overview of parts of the pipeline.

The code is fused with LaTeX into a single [matlab file](https://github.com/janusboandersen/3d-computer-vision/blob/main/3dcv_janus.m) that is either run directly or used to generate a report. If making a report, export to a `.tex` file using the `.xsl`-stylesheet, and compile with LuaLatex.

__Project summary__:
- Images are acquired using a calibrated camera (intrinsics).
- Correspondences between two views are established from Harris corners and BRIEF feature descriptors, with subsequent brute-force search using the Hamming distance and Lowe's Ratio Test.
- Relative camera pose (extrinsics - translation and rotation) is estimated using epipolar geometry.
- 3D reconstruction is done by triangulation after removal of epipolar outliers, and results in generation of a world point cloud.
- Segmentation and selection is enabled by morphological operations and region selection, using distance information from the point cloud.
- Other uses are to develop into visual SLAM or use segmented images as input to machine learning applications (e.g. an image recognition system).

![image](https://github.com/janusboandersen/3d-computer-vision/blob/main/img/cover.png)
