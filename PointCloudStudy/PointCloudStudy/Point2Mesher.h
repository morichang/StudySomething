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

	inline void PointCloud2Vector3d(pcl::PointCloud<pcl::PointNormal>::Ptr &cloud, pcl::on_nurbs::vector_vec3d &data);

public:
	pcl::PolygonMesh GreedyMesher(pcl::PointCloud<pcl::PointNormal>::Ptr &cloud);
	//pcl::PolygonMesh OrganizedFastMesh(pcl::PointCloud<pcl::PointNormal>::Ptr &cloud);
	pcl::PolygonMesh GridProjection(pcl::PointCloud<pcl::PointNormal>::Ptr &cloud);
	pcl::PolygonMesh Poisson(pcl::PointCloud<pcl::PointNormal>::Ptr &cloud);
	pcl::PolygonMesh B_Splines(pcl::PointCloud<pcl::PointNormal>::Ptr &cloud);
	pcl::PolygonMesh MarchingCubeRBF(pcl::PointCloud<pcl::PointNormal>::Ptr &cloud);
	pcl::PolygonMesh DelaunayTriangulation(pcl::PointCloud<pcl::PointNormal>::Ptr &cloud);
};