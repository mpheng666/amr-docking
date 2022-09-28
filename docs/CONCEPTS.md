# concepts for amr-docking
# LiDAR approach
## Scan preprocess by a passthrough filter (define it a rectangular or triangular region)
## Points Clustering Method
1. Density-based (DBSCAN, DENCLUE)
2. Ordering points (OPTICS)
3. Fuzzy C-means algorithm (FCM)
4. Self-organizing maps (SOM)
5. K nearest neighbours (KNN)
- Radius parameter ε is diffuclt to tune
- Parameter can be auto-tuned

## Model Fitting-Based Method
1. Hough Transform (HT)
2. Random Sample Consensus (RANSAC)
- Robust method for point cloud segmentation
- RANSAC is good with dealing noisy data
- Not suitable for complex geometries

## Region Growing-Based Method
1. Variable-order high-order polynomials as surface fitting
2. Octree-based region (coarse-to-fine)
- Seed points and parameters are required human intervention

# Camera approach
1. Double target tag alignment
2. Target recognition with CV
3. Target recognition with CNN

# Chosen approach
## Scan clustering/ransac with PCL
### Method steps
1. Receive input cloud from merged_cloud
2. Convert ros_pcl to pcl
3. Set param for clustering (For example cart's leg)
4. Cluster with Euclidean Cluster Extraction
5. RANSAC plane/circular object
6. Use the returned clustered list to form rectangle
7. Draw rectangle lines
8. Find the centroid of rectangle
9. Send the location of centroid

## Controller method
# ROS navigation planner
1. Send the position of the object to movebase
2. Wait for the magic to happen

# Pure pursuit
1. Align orientation
2. Move straight with PID
3. Loop to minimize location error
- This method assumes the robot is located somewhat in front of the object

# More custom planner and controller

# Pseudocode
1. Receive scan data (from merger)
2. Convert scan data to cloud (from merger)
3. Trim cloud with passthrough filter (preprocess launch)
4. Voxel grid filter cloud (preprocess launch)
5. Outlier removal filter cloud (preprocess launch)
6. Cluster cloud based on the clustering params


7. LOOP through clustered cloud:
   1. CALCULATE clustered centroids
8. LOOP through clustered centroids:
   1. CALCULATE the distance with each others
      1. CHECK distance is diagonal+-tolerance:
         1. CALCULATE the middle point
         2. STORE three points into a (pointA, pointB, middlePointAB) list
9. LOOP through (pointA, pointB, middlePointAB) list:
   2. CHECK two middlePointAB that fall within 0+-tolerance.
   3. STORE 4 points into (pointA1, pointB1, pointA2, pointB2)
   4. SORT 4 points into (pointA1, pointA2, pointB1, pointB2)
   5. CHECK IF (distance pointA1 and pointA2 is width):
      1. CHECK IF (distance pointB1 and pointB2 is length)
         1. IF reach here, target found
   6. CHECK IF (distance pointA1 and pointA2 is length):
      1. CHECK IF (distance pointB1 and pointB2 is width)
         1. IF reach here, target found
   7. PUBLISH target point (middlePointAB)



10. Find clustered centroid that forms a rectangle (target_loc)
11. Find rectangle centroid (target_loc)
12. Publish rectangle centroid with marker
13. Choose the centroid nearest to the robot (euclidean distance)
14. Generate a valid path from the robot to the rectangle
15. Follow the generated path

