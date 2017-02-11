#include "Point2Mesher.h"

inline void Point2Mesher::PointCloud2Vector3d(pcl::PointCloud<pcl::PointNormal>::Ptr &cloud, pcl::on_nurbs::vector_vec3d &data)
{
	for (unsigned i = 0; i < cloud->size(); ++i) {
		pcl::PointNormal &p = cloud->at(i);
		if (!pcl_isnan(p.x) && !pcl_isnan(p.y) && !pcl_isnan(p.z))
			data.push_back(Eigen::Vector3d(p.x, p.y, p.z));
	}
}

pcl::PolygonMesh Point2Mesher::GreedyMesher(pcl::PointCloud<pcl::PointNormal>::Ptr &cloud)
{
	pcl::search::KdTree<pcl::PointNormal>::Ptr kdTree(new pcl::search::KdTree<pcl::PointNormal>);
	kdTree->setInputCloud(cloud);

	// Initialize objects
	pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
	pcl::PolygonMesh triangles;

	// Set the maximum distance between connected points (maximum edge length)
	gp3.setSearchRadius(0.1);

	// Set typical values for the parameters
	gp3.setMu(2.5);
	gp3.setMaximumNearestNeighbors(10);
	gp3.setMaximumSurfaceAngle(M_PI / 4); // 45 degrees
	gp3.setMinimumAngle(M_PI / 18); // 10 degrees
	gp3.setMaximumAngle(2 * M_PI / 3); // 120 degrees
	gp3.setNormalConsistency(true);

	// Get result
	gp3.setInputCloud(cloud);
	gp3.setSearchMethod(kdTree);
	gp3.reconstruct(triangles);

	return triangles;
}

//pcl::PolygonMesh Point2Mesher::OrganizedFastMesh(pcl::PointCloud<pcl::PointNormal>::Ptr &cloud)
//{
//	boost::shared_ptr<std::vector<pcl::Vertices>> verts(new std::vector<pcl::Vertices>);
//	pcl::PolygonMesh triangles;
//	pcl::OrganizedFastMesh<pcl::PointNormal> fast;
//
//	fast.setTrianglePixelSize(1);
//	fast.setTriangulationType(pcl::OrganizedFastMesh<pcl::PointNormal>::TRIANGLE_ADAPTIVE_CUT);
//	
//	return triangles;
//}

pcl::PolygonMesh Point2Mesher::GridProjection(pcl::PointCloud<pcl::PointNormal>::Ptr &cloud)
{
	pcl::search::KdTree<pcl::PointNormal>::Ptr kdTree(new pcl::search::KdTree<pcl::PointNormal>);
	kdTree->setInputCloud(cloud);

	pcl::PolygonMesh triangles;
	pcl::GridProjection<pcl::PointNormal> grid;

	//grid.setResolution(1);
	//grid.setPaddingSize(3);
	grid.setNearestNeighborNum(100);

	grid.setInputCloud(cloud);
	grid.setSearchMethod(kdTree);
	grid.reconstruct(triangles);

	return triangles;
}

pcl::PolygonMesh Point2Mesher::Poisson(pcl::PointCloud<pcl::PointNormal>::Ptr &cloud)
{
	pcl::PolygonMesh triangles;
	pcl::Poisson<pcl::PointNormal> poisson;
	pcl::search::KdTree<pcl::PointNormal>::Ptr kdTree(new pcl::search::KdTree<pcl::PointNormal>);
	kdTree->setInputCloud(cloud);

	poisson.setDepth(10);
	poisson.setInputCloud(cloud);
	poisson.setPointWeight(100);
	poisson.setSearchMethod(kdTree);
	poisson.performReconstruction(triangles);
	//poisson.reconstruct(triangles);

	return triangles;
}

pcl::PolygonMesh Point2Mesher::B_Splines(pcl::PointCloud<pcl::PointNormal>::Ptr &cloud)
{
	pcl::PolygonMesh mesh;
	pcl::on_nurbs::NurbsDataSurface data;

	unsigned order(3);
	unsigned refinement(5);
	unsigned iterations(10);
	unsigned mesh_resolution(512);

	pcl::on_nurbs::FittingSurface::Parameter params;
	params.interior_smoothness = 0.2;
	params.interior_weight = 1.0;
	params.boundary_smoothness = 0.2;
	params.boundary_weight = 0.0;

	PointCloud2Vector3d(cloud, data.interior);

	ON_NurbsSurface nurbs = pcl::on_nurbs::FittingSurface::initNurbsPCABoundingBox(order, &data);
	pcl::on_nurbs::FittingSurface fit(&data, nurbs);

	pcl::PointCloud<pcl::PointXYZ>::Ptr mesh_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	std::vector<pcl::Vertices> mesh_vertices;
	std::string mesh_id = "mesh_nurbs";
	pcl::on_nurbs::Triangulation::convertSurface2PolygonMesh(fit.m_nurbs, mesh, mesh_resolution);

	for (unsigned i = 0; i < refinement; i++)
	{
		fit.refine(0);
		fit.refine(1);
		fit.assemble(params);
		fit.solve();
		pcl::on_nurbs::Triangulation::convertSurface2Vertices(fit.m_nurbs, mesh_cloud, mesh_vertices, mesh_resolution);
	}

	for (unsigned i = 0; i < iterations; i++)
	{
		fit.assemble(params);
		fit.solve();
		pcl::on_nurbs::Triangulation::convertSurface2Vertices(fit.m_nurbs, mesh_cloud, mesh_vertices, mesh_resolution);
	}

	pcl::on_nurbs::FittingCurve2dAPDM::FitParameter curve_params;
	curve_params.addCPsAccuracy = 5e-2;
	curve_params.addCPsIteration = 3;
	curve_params.maxCPs = 200;
	curve_params.accuracy = 1e-3;
	curve_params.iterations = 100;

	curve_params.param.closest_point_resolution = 0;
	curve_params.param.closest_point_weight = 1.0;
	curve_params.param.closest_point_sigma2 = 0.1;
	curve_params.param.interior_sigma2 = 0.00001;
	curve_params.param.smooth_concavity = 1.0;
	curve_params.param.smoothness = 1.0;

	pcl::on_nurbs::NurbsDataCurve2d curve_data;
	curve_data.interior = data.interior_param;
	curve_data.interior_weight_function.push_back(true);
	ON_NurbsCurve curve_nurbs = pcl::on_nurbs::FittingCurve2dAPDM::initNurbsCurve2D(order, curve_data.interior);

	// curve fitting
	pcl::on_nurbs::FittingCurve2dASDM curve_fit(&curve_data, curve_nurbs);
	// curve_fit.setQuiet (false); // enable/disable debug output
	curve_fit.fitting(curve_params);

	pcl::on_nurbs::Triangulation::convertTrimmedSurface2PolygonMesh(fit.m_nurbs, curve_fit.m_nurbs, mesh, mesh_resolution);

	return mesh;
}

pcl::PolygonMesh Point2Mesher::MarchingCubeRBF(pcl::PointCloud<pcl::PointNormal>::Ptr &cloud)
{
	std::vector<pcl::Vertices> vertices;
	pcl::PointCloud<pcl::PointNormal> points;
	pcl::PolygonMesh mesh;
	pcl::MarchingCubesRBF<pcl::PointNormal> rbf;
	rbf.setIsoLevel(0);
	rbf.setGridResolution(10, 10, 10);
	rbf.setPercentageExtendGrid(0.1f);
	rbf.setInputCloud(cloud);
	rbf.setOffSurfaceDisplacement(0.02f);
	rbf.reconstruct(points, vertices);

	return mesh;
}

pcl::PolygonMesh  Point2Mesher::DelaunayTriangulation(pcl::PointCloud<pcl::PointNormal>::Ptr &cloud)
{
	pcl::PointCloud<pcl::PointNormal>::Ptr src_cloud(new pcl::PointCloud<pcl::PointNormal>);
	pcl::PolygonMesh mesh;

	for (int i = 0; i < cloud->size(); ++i)
	{
		Tercel::Vector v;

		v.x = cloud->points[i].x;
		v.y = cloud->points[i].y;
		v.z = cloud->points[i].z;

		vertices.insert(v);
	}

	Tercel::Delaunay3d::getDelaunayTriangles(vertices, &triangles);

	std::cout << "三角形取得完了!" << std::endl;

	pcl::copyPointCloud(*cloud, *src_cloud);

	int element = 0;
	for (std::set<Tercel::Triangle>::iterator it = triangles.begin(); it != triangles.end(); ++it)
	{
		Tercel::Triangle t = *it;  //三角形取得
		for (int i = 0; i < 3; ++i)
		{
			mesh.polygons[element].vertices[i] = (float)t.p[i]->x;
			mesh.polygons[element].vertices[i] = (float)t.p[i]->y;
			mesh.polygons[element].vertices[i] = (float)t.p[i]->z;

			mesh.polygons[element].vertices[(i + 1) % 3] = (float)t.p[(i + 1) % 3]->x;
			mesh.polygons[element].vertices[(i + 1) % 3] = (float)t.p[(i + 1) % 3]->y;
			mesh.polygons[element].vertices[(i + 1) % 3] = (float)t.p[(i + 1) % 3]->z;
		}
		++element;
	}

	std::cout << "書き終わり" << std::endl;

	return mesh;
}