#include "Point2Mesher.h"

pcl::PolygonMesh Point2Mesher::GreedyMesher(std::string name, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(cloud);
	n.setInputCloud(cloud);
	n.setSearchMethod(tree);
	n.setRadiusSearch(0.005);
	//n.setKSearch(20);
	n.compute(*normals);

	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normal(new pcl::PointCloud<pcl::PointNormal>);
	pcl::concatenateFields(*cloud, *normals, *cloud_with_normal);

	pcl::search::KdTree<pcl::PointNormal>::Ptr searchtree(new pcl::search::KdTree<pcl::PointNormal>);
	searchtree->setInputCloud(cloud_with_normal);

	pcl::GreedyProjectionTriangulation<pcl::PointNormal> gpt;
	pcl::PolygonMesh::Ptr triangles;

	gpt.setSearchRadius(0.25);

	gpt.setMu(2.5);
	gpt.setMaximumNearestNeighbors(100);
	gpt.setMaximumSurfaceAngle(M_PI / 4); //45 degrees
	gpt.setMaximumAngle(M_PI / 18); //10 degrees
	gpt.setMaximumAngle(2 * M_PI / 3); //120 degrees
	gpt.setNormalConsistency(false);

	gpt.setInputCloud(cloud_with_normal);
	gpt.setSearchMethod(searchtree);
	gpt.reconstruct(*triangles);

	return *triangles;
}

void Point2Mesher::DelaunayTriangulation(std::string name, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	std::ofstream ofs;

	ofs.open(name + ".txt", std::ios::out);

	if (ofs.fail())
	{
		std::cout << "ファイルオープン失敗" << std::endl;
		return ;
	}

	for (int i = 0; i < cloud->size(); i++)
	{
		Tercel::Vector v;

		v.x = cloud->points[i].x;
		v.y = cloud->points[i].y;
		v.z = cloud->points[i].z;

		vertices.insert(v);
	}

	Tercel::Delaunay3d::getDelaunayTriangles(vertices, &triangles);

	std::cout << "三角形取得完了!" << std::endl;

	for (std::set<Tercel::Triangle>::iterator it = triangles.begin(); it != triangles.end(); ++it)
	{
		Tercel::Triangle t = *it;  //三角形取得
		for (int i = 0; i < 3; i++)
		{
			v1.x = (float)t.p[i]->x;
			v1.y = (float)t.p[i]->y;
			v1.z = (float)t.p[i]->z;

			v2.x = (float)t.p[(i + 1) % 3]->x;
			v2.y = (float)t.p[(i + 1) % 3]->y;
			v2.z = (float)t.p[(i + 1) % 3]->z;

			v3.x = (float)t.p[(i + 2) % 3]->x;
			v3.y = (float)t.p[(i + 2) % 3]->y;
			v3.z = (float)t.p[(i + 2) % 3]->z;

			if (i == 0)
			{
				ofs << v1.x << ", " << v1.y << ", " << v1.z << ", "
				<< v2.x << ", " << v2.y << ", " << v2.z << ", "
				<< v3.x << ", " << v3.y << ", " << v3.z << std::endl;
			}
		}
	}

	std::cout << "書き終わり" << std::endl;

	return ;
}