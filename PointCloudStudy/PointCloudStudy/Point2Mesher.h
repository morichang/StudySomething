#pragma once

#include "stdafx.h"

#include "Delaunay3d.h"
#include "DxLib.h"

#include <pcl/octree/octree_impl.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/qhull.h>

class Point2Mesher{
private:
	VECTOR v1, v2, v3;
	std::set<Tercel::Vector> vertices;
	std::set<Tercel::Triangle> triangles;
public:
	pcl::PolygonMesh GreedyMesher(std::string name, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
	void DelaunayTriangulation(std::string name, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
};