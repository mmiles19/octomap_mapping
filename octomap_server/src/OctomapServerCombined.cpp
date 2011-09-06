/**
* octomap_server: A Tool to serve 3D OctoMaps in ROS (binary and as visualization)
* (inspired by the ROS map_saver)
* @author A. Hornung, University of Freiburg, Copyright (C) 2010-2011.
* @see http://octomap.sourceforge.net/
* License: BSD
*/

/*
 * Copyright (c) 2010-2011, A. Hornung, University of Freiburg
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of Freiburg nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <octomap_server/OctomapServerCombined.h>

namespace octomap{

	OctomapServerCombined::OctomapServerCombined(const std::string& filename)
	  : m_nh(),
	    m_pointCloudSub(NULL), m_tfPointCloudSub(NULL),
	    m_octoMap(0.05),
	    m_maxRange(-1.0),
	    m_worldFrameId("/map"), m_baseFrameId("base_footprint"),
	    m_useHeightMap(true),
		m_colorFactor(0.8),
		m_pointcloudMinZ(-std::numeric_limits<double>::max()),
		m_pointcloudMaxZ(std::numeric_limits<double>::max()),
		m_occupancyMinZ(-std::numeric_limits<double>::max()),
		m_occupancyMaxZ(std::numeric_limits<double>::max()),
		m_minSizeX(0.0), m_minSizeY(0.0),
		m_filterSpeckles(false), m_filterGroundPlane(true),
		m_groundFilterDistance(0.04), m_groundFilterAngle(0.15), m_groundFilterPlaneDistance(0.07)
	{
		ros::NodeHandle private_nh("~");
		private_nh.param("frame_id", m_worldFrameId, m_worldFrameId);
		private_nh.param("base_frame_id", m_baseFrameId, m_baseFrameId);
		private_nh.param("height_map", m_useHeightMap, m_useHeightMap);
		private_nh.param("color_factor", m_colorFactor, m_colorFactor);

		private_nh.param("pointcloud_min_z", m_pointcloudMinZ,m_pointcloudMinZ);
		private_nh.param("pointcloud_max_z", m_pointcloudMaxZ,m_pointcloudMaxZ);
		private_nh.param("occupancy_min_z", m_occupancyMinZ,m_occupancyMinZ);
		private_nh.param("occupancy_max_z", m_occupancyMaxZ,m_occupancyMaxZ);
		private_nh.param("min_x_size", m_minSizeX,m_minSizeX);
		private_nh.param("min_y_size", m_minSizeY,m_minSizeY);

		private_nh.param("filter_speckles", m_filterSpeckles, m_filterSpeckles);
		private_nh.param("filter_ground", m_filterGroundPlane, m_filterGroundPlane);
		// distance of points from plane for RANSAC
		private_nh.param("ground_filter/distance", m_groundFilterDistance, m_groundFilterDistance);
		// angular derivation of found plane:
		private_nh.param("ground_filter/angle", m_groundFilterAngle, m_groundFilterAngle);
		// distance of found plane from z=0 to be detected as ground (e.g. to exclude tables)
		private_nh.param("ground_filter/plane_distance", m_groundFilterPlaneDistance, m_groundFilterPlaneDistance);

		double res = 0.05;
		private_nh.param("resolution", res, res);
		m_octoMap.octree.setResolution(res);

		private_nh.param("sensor_model/max_range", m_maxRange, m_maxRange);

		double probHit = 0.7;
		double probMiss = 0.4;
		double thresMin = 0.12;
		double thresMax = 0.97;
		private_nh.param("sensor_model/hit", probHit, probHit);
		private_nh.param("sensor_model/miss", probMiss, probMiss);
		private_nh.param("sensor_model/min", thresMin, thresMin);
		private_nh.param("sensor_model/max", thresMax, thresMax);
		m_octoMap.octree.setProbHit(probHit);
		m_octoMap.octree.setProbMiss(probMiss);
		m_octoMap.octree.setClampingThresMin(thresMin);
		m_octoMap.octree.setClampingThresMax(thresMax);

		double r, g, b, a;
		private_nh.param("color/r", r, 0.0);
		private_nh.param("color/g", g, 0.0);
		private_nh.param("color/b", b, 1.0);
		private_nh.param("color/a", a, 1.0);
		m_color.r = r;
		m_color.g = g;
		m_color.b = b;
		m_color.a = a;

		bool staticMap = false;
		if (filename != "")
			staticMap = true;

		m_markerPub = m_nh.advertise<visualization_msgs::MarkerArray>("occupied_cells_vis_array", 1, staticMap);
		m_binaryMapPub = m_nh.advertise<octomap_ros::OctomapBinary>("octomap_binary", 1, staticMap);
		m_pointCloudPub = m_nh.advertise<sensor_msgs::PointCloud2>("octomap_point_cloud_centers", 1, staticMap);
		m_collisionObjectPub = m_nh.advertise<mapping_msgs::CollisionObject>("octomap_collision_object", 1, staticMap);
		m_mapPub = m_nh.advertise<nav_msgs::OccupancyGrid>("map", 5, staticMap);

		m_octomapService = m_nh.advertiseService("octomap_binary", &OctomapServerCombined::serviceCallback, this);
		m_clearBBXService = private_nh.advertiseService("clear_bbx", &OctomapServerCombined::clearBBXSrv, this);

		// a filename to load is set => distribute a static map latched
		if (staticMap){
			if (m_octoMap.octree.readBinary(filename)){
				ROS_INFO("Octomap file %s loaded (%zu nodes).", filename.c_str(),m_octoMap.octree.size());

				publishAll();
			} else{
				ROS_ERROR("Could not open requested file %s, exiting.", filename.c_str());
				exit(-1);
			}
		} else { // otherwise: do scan integration



			m_pointCloudSub = new message_filters::Subscriber<sensor_msgs::PointCloud2> (m_nh, "cloud_in", 5);
			m_tfPointCloudSub = new tf::MessageFilter<sensor_msgs::PointCloud2> (*m_pointCloudSub, m_tfListener, m_worldFrameId, 5);
			m_tfPointCloudSub->registerCallback(boost::bind(&OctomapServerCombined::insertCloudCallback, this, _1));
		}



	}

	OctomapServerCombined::~OctomapServerCombined(){
		if (m_tfPointCloudSub)
			delete m_tfPointCloudSub;
		if (m_pointCloudSub)
			delete m_pointCloudSub;

	}

	void OctomapServerCombined::insertCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud){
		ros::WallTime startTime = ros::WallTime::now();


		//
		// ground filtering in base frame
		//
		pcl::PointCloud<pcl::PointXYZ> pc; // input cloud for filtering and ground-detection
		pcl::fromROSMsg(*cloud, pc);

		tf::StampedTransform sensorToWorldTf, sensorToBaseTf, baseToWorldTf;
		try {
			m_tfListener.waitForTransform(m_worldFrameId, cloud->header.frame_id, cloud->header.stamp, ros::Duration(0.2));

			m_tfListener.lookupTransform(m_worldFrameId, cloud->header.frame_id, cloud->header.stamp, sensorToWorldTf);
			m_tfListener.lookupTransform(m_baseFrameId, cloud->header.frame_id, cloud->header.stamp, sensorToBaseTf);
			m_tfListener.lookupTransform(m_worldFrameId, m_baseFrameId, cloud->header.stamp, baseToWorldTf);
		} catch(tf::TransformException& ex){
			ROS_ERROR_STREAM( "Transform error: " << ex.what() << ", quitting callback");
			return;
		}
		point3d sensorOrigin = pointTfToOctomap(sensorToWorldTf.getOrigin());
		Eigen::Matrix4f sensorToBase, baseToWorld;
		pcl_ros::transformAsMatrix(sensorToBaseTf, sensorToBase);
		pcl_ros::transformAsMatrix(baseToWorldTf, baseToWorld);

		// transform pointcloud from sensor frame to fixed robot frame
		pcl::transformPointCloud(pc, pc, sensorToBase);

		// filter height and range, also removes NANs:
		pcl::PassThrough<pcl::PointXYZ> pass;
		pass.setFilterFieldName("z");
		pass.setFilterLimits(m_pointcloudMinZ, m_pointcloudMaxZ);
		pass.setInputCloud(pc.makeShared());
		pass.filter(pc);

		pcl::PointCloud<pcl::PointXYZ> pc_ground; // segmented ground plane
		pcl::PointCloud<pcl::PointXYZ> pc_nonground; // everything else
		pc_ground.header = pc.header;
		pc_nonground.header = pc.header;

		if (m_filterGroundPlane){
			if (pc.size() < 50){
				ROS_WARN("Pointcloud in OctomapServerCombined too small, skipping ground plane extraction");
				pc_nonground = pc;
			} else {
				// plane detection for ground plane removal:
				pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
				pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

				// Create the segmentation object and set up:
				pcl::SACSegmentation<pcl::PointXYZ> seg;
				seg.setOptimizeCoefficients (true);
				// TODO: maybe a filtering based on the surface normals might be more robust / accurate?
				seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
				seg.setMethodType(pcl::SAC_RANSAC);
				seg.setMaxIterations(200);
				seg.setDistanceThreshold (m_groundFilterDistance);
				seg.setAxis(Eigen::Vector3f(0,0,1));
				seg.setEpsAngle(m_groundFilterAngle);


				pcl::PointCloud<pcl::PointXYZ> cloud_filtered(pc);
				// Create the filtering object
				pcl::ExtractIndices<pcl::PointXYZ> extract;
				bool groundPlaneFound = false;

				while(cloud_filtered.size() > 10 && !groundPlaneFound){
					seg.setInputCloud(cloud_filtered.makeShared());
					seg.segment (*inliers, *coefficients);
					if (inliers->indices.size () == 0){
						ROS_WARN("No plane found in cloud.");

						break;
					}

					extract.setInputCloud(cloud_filtered.makeShared());
					extract.setIndices(inliers);

					if (std::abs(coefficients->values.at(3)) < m_groundFilterPlaneDistance){
						ROS_DEBUG("Ground plane found: %zu/%zu inliers. Coeff: %f %f %f %f", inliers->indices.size(), cloud_filtered.size(),
								coefficients->values.at(0), coefficients->values.at(1), coefficients->values.at(2), coefficients->values.at(3));
						extract.setNegative (false);
						extract.filter (pc_ground);

						// remove ground points from full pointcloud:
						// workaround for PCL bug:
						if(inliers->indices.size() != cloud_filtered.size()){
							extract.setNegative(true);
							pcl::PointCloud<pcl::PointXYZ> cloud_out;
							extract.filter(cloud_out);
							pc_nonground += cloud_out;
							cloud_filtered = cloud_out;
						}

						groundPlaneFound = true;
					} else{
						ROS_DEBUG("Horizontal plane (not ground) found: %zu/%zu inliers. Coeff: %f %f %f %f", inliers->indices.size(), cloud_filtered.size(),
								coefficients->values.at(0), coefficients->values.at(1), coefficients->values.at(2), coefficients->values.at(3));
						pcl::PointCloud<pcl::PointXYZ> cloud_out;
						extract.setNegative (false);
						extract.filter(cloud_out);
						pc_nonground +=cloud_out;
						// debug
//						pcl::PCDWriter writer;
//						writer.write<pcl::PointXYZ>("nonground_plane.pcd",cloud_out, false);

						// remove current plane from scan for next iteration:
						// workaround for PCL bug:
						if(inliers->indices.size() != cloud_filtered.size()){
							extract.setNegative(true);
							cloud_out.points.clear();
							extract.filter(cloud_out);
							cloud_filtered = cloud_out;
						} else{
							cloud_filtered.points.clear();
						}
					}

				}
				if (!groundPlaneFound){ // no plane found or remaining points too small
					ROS_WARN("No ground plane found in scan");
					pc_nonground += cloud_filtered;
				}

				// debug:
//				pcl::PCDWriter writer;
//				if (pc_ground.size() > 0)
//					writer.write<pcl::PointXYZ>("ground.pcd",pc_ground, false);
//				if (pc_nonground.size() > 0)
//					writer.write<pcl::PointXYZ>("nonground.pcd",pc_nonground, false);

			}
		} else {
			pc_nonground = pc;
		}

		// transform clouds to world frame for insertion
		pcl::transformPointCloud(pc_ground, pc_ground, baseToWorld);
		pcl::transformPointCloud(pc_nonground, pc_nonground, baseToWorld);


//		// insert without pruning and 'dirty'
//		geometry_msgs::Point sensorOrigin;
//		tf::pointTFToMsg(trans.getOrigin(), sensorOrigin);
//		m_octoMap.insertScan(transformed_cloud, sensorOrigin, m_maxRange, false, true);
//		// TODO: eval which faster: "dirty" with updateInner?
//		m_octoMap.octree.updateInnerOccupancy();
//		m_octoMap.octree.prune();
//

		// instead of direct scan insertion, compute update to filter ground:
		KeySet free_cells, occupied_cells;
		// insert ground points only as free:
		for (pcl::PointCloud<pcl::PointXYZ>::const_iterator it = pc_ground.begin(); it != pc_ground.end(); ++it){
			point3d point(it->x, it->y, it->z);
			// maxrange check
			if ((m_maxRange > 0.0) && ((point - sensorOrigin).norm() > m_maxRange) ) {
				point = sensorOrigin + (point - sensorOrigin).normalized() * m_maxRange;
			}

			// only clear space (ground points)
			if (m_octoMap.octree.computeRayKeys(sensorOrigin, point, m_keyRay)){
				free_cells.insert(m_keyRay.begin(), m_keyRay.end());
			}
		}

		// all other points: free on ray, occupied on endpoint:
	    for (pcl::PointCloud<pcl::PointXYZ>::const_iterator it = pc_nonground.begin(); it != pc_nonground.end(); ++it){
	    	point3d point(it->x, it->y, it->z);
	    	// maxrange check
	    	if ((m_maxRange < 0.0) || ((point - sensorOrigin).norm() <= m_maxRange) ) {

				// free cells
				if (m_octoMap.octree.computeRayKeys(sensorOrigin, point, m_keyRay)){
					free_cells.insert(m_keyRay.begin(), m_keyRay.end());
				}
				// occupied endpoint
				OcTreeKey key;
				if (m_octoMap.octree.genKey(point, key)){
					occupied_cells.insert(key);
				}
	    	} else {// ray longer than maxrange:;
	    		point3d new_end = sensorOrigin + (point - sensorOrigin).normalized() * m_maxRange;
	    		if (m_octoMap.octree.computeRayKeys(sensorOrigin, new_end, m_keyRay)){
	    			free_cells.insert(m_keyRay.begin(), m_keyRay.end());
	    		}
	    	}
	    }

	    // mark free cells only if not seen occupied in this cloud
	    for(KeySet::iterator it = free_cells.begin(), end=free_cells.end(); it!= end; ++it){
	      if (occupied_cells.find(*it) == occupied_cells.end()){
	    	m_octoMap.octree.updateNode(*it, false, true);
	      }
	    }

	    // now mark all occupied cells:
		for (KeySet::iterator it = occupied_cells.begin(), end=free_cells.end(); it!= end; it++) {
			m_octoMap.octree.updateNode(*it, true, true);
		}
		m_octoMap.octree.updateInnerOccupancy();
		m_octoMap.octree.prune();

		double total_elapsed = (ros::WallTime::now() - startTime).toSec();
		ROS_DEBUG("Pointcloud insertion in OctomapServer done (%d+%d pts (ground/nonground), %f sec)", int(pc_ground.size()), int(pc_nonground.size()), total_elapsed);

		publishAll(cloud->header.stamp);
	}


	void OctomapServerCombined::publishAll(const ros::Time& rostime){
		ros::WallTime startTime = ros::WallTime::now();

		if (m_octoMap.octree.size() <= 1){
			ROS_WARN("Nothing to publish, octree is empty");
			return;
		}
		// configuration, right now only for testing:
		bool publishCollisionObject = true;
		bool publishMarkerArray = true;
		bool publishPointCloud = true;
		bool publish2DMap = true;

		// init collision object:
		mapping_msgs::CollisionObject collisionObject;
		collisionObject.header.frame_id = m_worldFrameId;
		collisionObject.header.stamp = rostime;
		collisionObject.id = "map";

		geometric_shapes_msgs::Shape shape;
		shape.type = geometric_shapes_msgs::Shape::BOX;
		shape.dimensions.resize(3);

		geometry_msgs::Pose pose;
		pose.orientation = tf::createQuaternionMsgFromYaw(0.0);

		// init markers:
		double minX, minY, minZ, maxX, maxY, maxZ;
		m_octoMap.octree.getMetricMin(minX, minY, minZ);
		m_octoMap.octree.getMetricMax(maxX, maxY, maxZ);

		visualization_msgs::MarkerArray occupiedNodesVis;
		// each array stores all cubes of a different size, one for each depth level:
		occupiedNodesVis.markers.resize(m_octoMap.octree.getTreeDepth());
		double lowestRes = m_octoMap.octree.getResolution();

		// init pointcloud:
		pcl::PointCloud<pcl::PointXYZ> pclCloud;

		// init projected 2D map:
		nav_msgs::OccupancyGrid map;
		map.info.resolution = m_octoMap.octree.getResolution();
	    map.header.frame_id = m_worldFrameId;
	    map.header.stamp = rostime;

		octomap::point3d minPt(minX, minY, minZ);
		octomap::point3d maxPt(maxX, maxY, maxZ);
		octomap::OcTreeKey minKey, maxKey, curKey;
		if (!m_octoMap.octree.genKey(minPt, minKey)){
			ROS_ERROR("Could not create min OcTree key at %f %f %f", minPt.x(), minPt.y(), minPt.z());
			return;
		}
		if (!m_octoMap.octree.genKey(maxPt, maxKey)){
			ROS_ERROR("Could not create max OcTree key at %f %f %f", maxPt.x(), maxPt.y(), maxPt.z());
			return;
		}

		ROS_DEBUG("MinKey: %d %d %d / MaxKey: %d %d %d", minKey[0], minKey[1], minKey[2], maxKey[0], maxKey[1], maxKey[2]);

		// add padding if requested (= new min/maxPts in x&y):
		double halfPaddedX = 0.5*m_minSizeX;
		double halfPaddedY = 0.5*m_minSizeY;
		minX = std::min(minX, -halfPaddedX);
		maxX = std::max(maxX, halfPaddedX);
		minY = std::min(minY, -halfPaddedY);
		maxY = std::max(maxY, halfPaddedY);
		minPt = octomap::point3d(minX, minY, minZ);
		maxPt = octomap::point3d(maxX, maxY, maxZ);

		octomap::OcTreeKey paddedMinKey, paddedMaxKey;
		if (!m_octoMap.octree.genKey(minPt, paddedMinKey)){
			ROS_ERROR("Could not create padded min OcTree key at %f %f %f", minPt.x(), minPt.y(), minPt.z());
			return;
		}
		if (!m_octoMap.octree.genKey(maxPt, paddedMaxKey)){
			ROS_ERROR("Could not create padded max OcTree key at %f %f %f", maxPt.x(), maxPt.y(), maxPt.z());
			return;
		}

		ROS_DEBUG("Padded MinKey: %d %d %d / padded MaxKey: %d %d %d", paddedMinKey[0], paddedMinKey[1], paddedMinKey[2], paddedMaxKey[0], paddedMaxKey[1], paddedMaxKey[2]);
		assert(paddedMaxKey[0] >= maxKey[0] && paddedMaxKey[1] >= maxKey[1]);

		map.info.width = paddedMaxKey[0] - paddedMinKey[0] +1;
		map.info.height = paddedMaxKey[1] - paddedMinKey[1] +1;
		int mapOriginX = minKey[0] - paddedMinKey[0];
		int mapOriginY = minKey[1] - paddedMinKey[1];
		assert(mapOriginX >= 0 && mapOriginY >= 0);

		// might not exactly be min / max of octree:
		octomap::point3d origin;
		m_octoMap.octree.genCoords(paddedMinKey, m_octoMap.octree.getTreeDepth(), origin);
		map.info.origin.position.x = origin.x() - m_octoMap.octree.getResolution()*0.5;
		map.info.origin.position.y = origin.y() - m_octoMap.octree.getResolution()*0.5;

		// Allocate space to hold the data (init to unknown)
		map.data.resize(map.info.width * map.info.height, -1);


		for (OcTreeROS::OcTreeType::iterator it = m_octoMap.octree.begin(),
				end = m_octoMap.octree.end(); it != end; ++it)
		{
			if (m_octoMap.octree.isNodeOccupied(*it)){
				double z = it.getZ();
				if (z > m_occupancyMinZ && z < m_occupancyMaxZ)
				{
					octomap::OcTreeKey nKey = it.getKey();
					octomap::OcTreeKey key;
					// Ignore speckles in the map:
					// TODO: only at lowest res!
					if (m_filterSpeckles){
						bool neighborFound = false;
						for (key[2] = nKey[2] - 1; !neighborFound && key[2] <= nKey[2] + 1; ++key[2]){
							for (key[1] = nKey[1] - 1; !neighborFound && key[1] <= nKey[1] + 1; ++key[1]){
								for (key[0] = nKey[0] - 1; !neighborFound && key[0] <= nKey[0] + 1; ++key[0]){
									if (key != nKey){
										OcTreeNode* node = m_octoMap.octree.search(key);
										if (node && m_octoMap.octree.isNodeOccupied(node)){
											// we have a neighbor => break!
											neighborFound = true;
										}
									}
								}
							}
						}
						// done with search, see if found and skip otherwise:
						if (!neighborFound){
							ROS_DEBUG("Ignoring single speckle at (%f,%f,%f)", it.getX(), it.getY(), it.getZ());
							continue;
						}
					} // else: current octree node is no speckle, send it out

					double size = it.getSize();
					double x = it.getX();
					double y = it.getY();

					// create collision object:
					if (publishCollisionObject){
						shape.dimensions[0] = shape.dimensions[1] = shape.dimensions[2] = size;
						collisionObject.shapes.push_back(shape);
						pose.position.x = x;
						pose.position.y = y;
						pose.position.z = z;
						collisionObject.poses.push_back(pose);
					}

					//create marker:
					if (publishMarkerArray){
						int idx = int(log2(it.getSize() / lowestRes) +0.5);
						assert (idx >= 0 && unsigned(idx) < occupiedNodesVis.markers.size());
						geometry_msgs::Point cubeCenter;
						cubeCenter.x = x;
						cubeCenter.y = y;
						cubeCenter.z = z;

						occupiedNodesVis.markers[idx].points.push_back(cubeCenter);
						if (m_useHeightMap){
							double h = (1.0 - std::min(std::max((cubeCenter.z-minZ)/ (maxZ - minZ), 0.0), 1.0)) *m_colorFactor;
							occupiedNodesVis.markers[idx].colors.push_back(heightMapColor(h));
						}
					}

					// insert into pointcloud:
					if (publishPointCloud)
						pclCloud.push_back(pcl::PointXYZ(x, y, z));

					// update 2D map (occupied always overrides):
					if (publish2DMap){
						if (it.getDepth() == m_octoMap.octree.getTreeDepth()){
							int i = nKey[0] - paddedMinKey[0];
							int j = nKey[1] - paddedMinKey[1];
							map.data[map.info.width*j + i] = 100;
						} else{
							int intSize = 1 << (m_octoMap.octree.getTreeDepth() - it.getDepth());
							octomap::OcTreeKey minKey=it.getIndexKey();
							for(int dx=0; dx < intSize; dx++){
								int i = minKey[0]+dx - paddedMinKey[0];
								for(int dy=0; dy < intSize; dy++){
									int j = minKey[1]+dy - paddedMinKey[1];
									map.data[map.info.width*j + i] = 100;
								}
							}
						}
					}
				}
			} else{ // node not occupied => mark as free in 2D map if unknown so far
				if (publish2DMap){
					if (it.getDepth() == m_octoMap.octree.getTreeDepth()){
						octomap::OcTreeKey nKey = it.getKey();
						int i = nKey[0] - paddedMinKey[0];
						int j = nKey[1] - paddedMinKey[1];
						if (map.data[map.info.width*j + i] == -1){
							map.data[map.info.width*j + i] = 0;
						}
					} else{
						int intSize = 1 << (m_octoMap.octree.getTreeDepth() - it.getDepth());
						octomap::OcTreeKey minKey=it.getIndexKey();
						for(int dx=0; dx < intSize; dx++){
							int i = minKey[0]+dx - paddedMinKey[0];
							for(int dy=0; dy < intSize; dy++){
								int j = minKey[1]+dy - paddedMinKey[1];
								if (map.data[map.info.width*j + i] == -1){
									map.data[map.info.width*j + i] = 0;
								}
							}
						}
					}
				}
			}
		}

		// finish MarkerArray:
		if (publishMarkerArray){
			for (unsigned i= 0; i < occupiedNodesVis.markers.size(); ++i){
				double size = lowestRes * pow(2,i);

				occupiedNodesVis.markers[i].header.frame_id = m_worldFrameId;
				occupiedNodesVis.markers[i].header.stamp = rostime;
				occupiedNodesVis.markers[i].ns = "map";
				occupiedNodesVis.markers[i].id = i;
				occupiedNodesVis.markers[i].type = visualization_msgs::Marker::CUBE_LIST;
				occupiedNodesVis.markers[i].scale.x = size;
				occupiedNodesVis.markers[i].scale.y = size;
				occupiedNodesVis.markers[i].scale.z = size;
				occupiedNodesVis.markers[i].color = m_color;


				if (occupiedNodesVis.markers[i].points.size() > 0)
					occupiedNodesVis.markers[i].action = visualization_msgs::Marker::ADD;
				else
					occupiedNodesVis.markers[i].action = visualization_msgs::Marker::DELETE;
			}


			m_markerPub.publish(occupiedNodesVis);
		}

		// finish pointcloud:
		if (publishPointCloud){
			sensor_msgs::PointCloud2 cloud;
			pcl::toROSMsg (pclCloud, cloud);
			cloud.header.frame_id = m_worldFrameId;
			cloud.header.stamp = rostime;
			m_pointCloudPub.publish(cloud);
		}

		if (publishCollisionObject)
			m_collisionObjectPub.publish(collisionObject);

		if (publish2DMap)
			m_mapPub.publish(map);

		double total_elapsed = (ros::WallTime::now() - startTime).toSec();
		ROS_DEBUG("Map publishing in OctomapServer took %f sec", total_elapsed);

	}


	bool OctomapServerCombined::serviceCallback(octomap_ros::GetOctomap::Request  &req,
			octomap_ros::GetOctomap::Response &res)
	{
		ROS_INFO("Sending map data on service request");
		res.map.header.frame_id = m_worldFrameId;
		res.map.header.stamp = ros::Time::now();
		octomap::octomapMapToMsgData(m_octoMap.octree, res.map.data);

		return true;
	}

	bool OctomapServerCombined::clearBBXSrv(octomap_ros::ClearBBXRegionRequest& req, octomap_ros::ClearBBXRegionRequest& resp){
		OcTreeROS::OcTreeType::leaf_bbx_iterator it, end;
		point3d min = pointMsgToOctomap(req.min);
		point3d max = pointMsgToOctomap(req.max);

		for(OcTreeROS::OcTreeType::leaf_bbx_iterator it = m_octoMap.octree.begin_leafs_bbx(min,max),
		     end=m_octoMap.octree.end_leafs_bbx(); it!= end; ++it){
			it->setLogOdds(-2);
//			m_octoMap.octree.updateNode(it.getKey(), -6.0f);
		}
		m_octoMap.octree.updateInnerOccupancy();

		publishAll(ros::Time::now());

		return true;
	}

	void OctomapServerCombined::publishMap(const ros::Time& rostime){

		octomap_ros::OctomapBinary map;
		map.header.frame_id = m_worldFrameId;
		map.header.stamp = rostime;

		octomap::octomapMapToMsgData(m_octoMap.octree, map.data);

		m_binaryMapPub.publish(map);
	}

	std_msgs::ColorRGBA OctomapServerCombined::heightMapColor(double h) const {

		std_msgs::ColorRGBA color;
		color.a = 1.0;
		// blend over HSV-values (more colors)

		double s = 1.0;
		double v = 1.0;

		h -= floor(h);
		h *= 6;
		int i;
		double m, n, f;

		i = floor(h);
		f = h - i;
		if (!(i & 1))
			f = 1 - f; // if i is even
		m = v * (1 - s);
		n = v * (1 - s * f);

		switch (i) {
		case 6:
		case 0:
			color.r = v; color.g = n; color.b = m;
			break;
		case 1:
			color.r = n; color.g = v; color.b = m;
			break;
		case 2:
			color.r = m; color.g = v; color.b = n;
			break;
		case 3:
			color.r = m; color.g = n; color.b = v;
			break;
		case 4:
			color.r = n; color.g = m; color.b = v;
			break;
		case 5:
			color.r = v; color.g = m; color.b = n;
			break;
		default:
			color.r = 1; color.g = 0.5; color.b = 0.5;
			break;
		}

		return color;
	}
}



