#include "EditingUtility.h"

EditingUtility::EditingUtility(const std::string &dind_id)
{
	char szUserName[256] = { '\0' };
	DWORD dwSize = sizeof(szUserName) / sizeof(szUserName[0]);
	GetUserName(szUserName, &dwSize);
	userName = std::string(szUserName);

	// 色んなpathの設定
	this->calib_path = "../../../../../CalibrationParameter/" + dind_id + "-20170109/" + dind_id + "-PC";
	this->load_path = "C:/Users/" + userName + "/Desktop/" + dind_id + "-PC/2017-01-31_21-20-00_" + dind_id + "-PC";
	this->pc_save_path = "C:/Users/" + userName + "/Desktop/" + dind_id + "-PC/2017-01-31_21-20-00_" + dind_id + "-PC-PointCloud/";

	this->focal_length.x = 1081.37;
	this->focal_length.y = 1081.37;
	this->principal_point.x = 769.5;
	this->principal_point.y = 539.5;
}

inline void EditingUtility::split(std::vector<std::string> &v, const std::string &input_string, const std::string &delimiter)
{
	std::string::size_type index = input_string.find_first_of(delimiter);

	if (index != std::string::npos) {
		v.push_back(input_string.substr(0, index));
		split(v, input_string.substr(index + 1), delimiter);
	}
	else {
		v.push_back(input_string);
	}
}

cv::Point3d EditingUtility::getCameraPos(std::string &dind)
{
	this->pos_save_path = "C:/Users/" + userName + "/Desktop/" + dind + "-PC/" + dind + "-PC";

	cv::Point3d cameraPoint;
	std::vector<std::string> camera_pos;

	std::ifstream ifs_pos(pos_save_path + "-CameraPos.txt");
	if (ifs_pos.fail())	{
		std::cerr << "失敗" << std::endl;
		cameraPoint.x = 0.0;
		cameraPoint.y = 0.0;
		cameraPoint.z = 0.0;
		return cameraPoint;
	}

	getline(ifs_pos, str);
	split(camera_pos, str, ",");
	cameraPoint.x = std::stod(camera_pos[0]);
	cameraPoint.y = std::stod(camera_pos[1]);
	cameraPoint.z = std::stod(camera_pos[2]);

	return cameraPoint;
}

inline pcl::PointCloud<pcl::PointNormal>::Ptr EditingUtility::computeNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, cv::Point3d view_pos)
{
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
	pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> normalEstimation;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);

	// VoxelGridダウンサンプリング
	downsampling.exec(cloud);

	// 外れ値除去
	conv.filter.Remove_outliers(cloud, cloud);

	movingLeastSquares(cloud);

	normalEstimation.setInputCloud(cloud);

	//カメラ(Kinect v2)の位置
	normalEstimation.setViewPoint(view_pos.x, view_pos.y, view_pos.z);

	normalEstimation.setRadiusSearch(0.3);  // 30cm
	normalEstimation.setSearchMethod(kdtree);

	normalEstimation.compute(*cloud_normals);

	pcl::concatenateFields(*cloud, *cloud_normals, *cloud_with_normals);

	return cloud_with_normals;
}

inline void EditingUtility::movingLeastSquares(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_smoothed(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> mls;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);

	mls.setComputeNormals(false);

	mls.setInputCloud(cloud);
	mls.setPolynomialFit(true);
	mls.setSearchMethod(kdtree);
	mls.setSearchRadius(0.1);

	mls.process(*cloud_smoothed);

	*cloud = *cloud_smoothed;

	return;
}

inline void EditingUtility::planeSegmentation(pcl::PointCloud<pcl::PointNormal>::Ptr &cloud, pcl::PointCloud<pcl::PointNormal>::Ptr &plane, pcl::PointCloud<pcl::PointNormal>::Ptr &without_plane)
{
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	pcl::ExtractIndices<pcl::PointNormal> extract;
	pcl::SACSegmentation<pcl::PointNormal> seg;

	// Optional
	seg.setOptimizeCoefficients(true);
	// Mandatory
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setMaxIterations(1000);
	seg.setDistanceThreshold(0.1);

	seg.setInputCloud(cloud);
	seg.segment(*inliers, *coefficients);

	extract.setInputCloud(cloud);
	extract.setIndices(inliers);
	extract.setNegative(false);
	extract.filter(*plane);

	extract.setNegative(true);
	extract.filter(*without_plane);

	return;
}

inline void EditingUtility::euclideanClusterExtraction(pcl::PointCloud<pcl::PointNormal>::Ptr &cloud, std::vector<pcl::PointIndices> &cluster)
{
	//std::vector<int> index;
	//pcl::removeNaNFromPointCloud(*cloud, *cloud, index);

	pcl::search::KdTree<pcl::PointNormal>::Ptr kdtree(new pcl::search::KdTree<pcl::PointNormal>);
	kdtree->setInputCloud(cloud);

	pcl::EuclideanClusterExtraction<pcl::PointNormal> ecextraction;

	ecextraction.setClusterTolerance(0.02);  // 2cm

	ecextraction.setMinClusterSize(100);
	ecextraction.setMaxClusterSize(25000);
	ecextraction.setSearchMethod(kdtree);
	ecextraction.setInputCloud(cloud);
	ecextraction.extract(cluster);

	return;
}

void EditingUtility::extractData()
{
	int index;
	std::string split;
	std::ostringstream oss;
	cv::Mat color, depth;
	boost::posix_time::ptime time;

	std::ifstream ifs(load_path + ".bin", std::ios::in | std::ios::binary);
	if (!ifs.is_open()) {
		std::cout << load_path + ".bin開けないゾ~!" << std::endl;
		return;
	}

	std::ofstream ofs(load_path + ".txt", std::ios::out);
	if (!ofs.is_open()) {
		std::cout << load_path + ".txt開けないゾ~!" << std::endl;
		return;
	}

	std::cout << "データ解凍開始" << std::endl;

	// Header読み込み
	int rows, cols, colorType;
	ifs.read((char *)(&rows), sizeof(int));
	if (rows == 0)	{
		return;
	}
	ifs.read((char *)(&cols), sizeof(int));
	ifs.read((char *)(&colorType), sizeof(int));

	int depthType;
	ifs.read((char *)(&depthType), sizeof(int));

	while (!ifs.eof()) {
		index = 0;

		auto facet = new boost::posix_time::time_facet("%Y-%m-%dT%H-%M-%S%F");  // 2017-01-09T03-24-00.000000
		oss.imbue(std::locale(oss.getloc(), facet));

		ifs.read((char *)(&time), sizeof(boost::posix_time::ptime));

		// facetで直された時間の秒以下の表現を "." から "_" に変換する(検索しやすくする)
		oss << time;
		split = oss.str();
		while ((index = split.find(".")) != -1) {
			split.replace(index, 1, "_");
		}

		//std::cout << split << std::endl;

		// Color書き出し
		color.release();
		color.create(rows, cols, colorType);
		ifs.read((char *)(color.data), color.elemSize()*color.total());

		// Depth書き出し
		depth.release();
		depth.create(rows, cols, depthType);
		ifs.read((char *)(depth.data), depth.elemSize()*depth.total());

		if (!(split == "not-a-date-time")) {
			ofs << split << std::endl;
			cv::imwrite(load_path + "-Color/" + split + "-color.png", color);
			cv::imwrite(load_path + "-Depth/" + split + "-depth.png", depth);
			//colorFrameList.push_back(color);
			//depthFrameList.push_back(depth);
			//Sleep(33);
		}

		// ストリングストリームのバッファの状態をクリアする
		oss.str("");
		oss.clear(std::stringstream::goodbit);
	}
	ifs.close();
	ofs.close();
	color.release();
	depth.release();

	std::cout << "解凍終了" << std::endl;

	return;
}

void EditingUtility::createPointCloud()
{
	cv::Mat trunc_color, trunc_depth;
	cv::Point2d focal_length, principal_point;

	pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PLYReader ply_read;
	pcl::PLYWriter ply_write;

	std::string file_name;

	std::cout << "確認" << "\n"
		<< "file path:" << calib_path << "\n"
		<< "load path:" << load_path << "\n"
		<< "save path:" << pc_save_path << std::endl;

	std::ifstream ifs_data(load_path + ".txt", std::ios::in);
	if (!ifs_data.is_open()) {
		std::cout << "おかしいぞ~!" << std::endl;
		return;
	}

	while (!ifs_data.eof()) {
		getline(ifs_data, file_name);
		cv::Mat color = cv::imread(load_path + "-Color/" + file_name + "-color.png", -1);
		cv::Mat depth = cv::imread(load_path + "-Depth/" + file_name + "-depth.png", -1);

		hf.imageTruncate(color, depth, trunc_color, trunc_depth);

		conv.convPNG2PointCloud(pc_save_path + file_name, trunc_depth, focal_length, principal_point);

		ply_read.read<pcl::PointXYZ>(pc_save_path + file_name + "-pointcloud.ply", *in_cloud);

		t3p.convLocalToWorld(calib_path, in_cloud, out_cloud);

		ply_write.write<pcl::PointXYZ>(pc_save_path + file_name + "-pointcloud.ply", *out_cloud, true, false);

		in_cloud->clear();
		out_cloud->clear();
		color.release();
		depth.release();
	}
	ifs_data.close();
	ply_read.~PLYReader();
	ply_write.~PLYWriter();

	return;
}

void EditingUtility::computeCameraPos()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointXYZ point;

	std::ofstream ofs(pos_save_path + "-CameraPos.txt", std::ios::out);
	if (!ofs.is_open()) {
		std::cout << pos_save_path + "-CameraPos.txt開けないゾ~!" << std::endl;
		return;
	}

	point.x = 0.0;
	point.y = 0.0;
	point.z = 0.0;
	in_cloud->push_back(point);

	t3p.convLocalToWorld(calib_path, in_cloud, out_cloud);

	ofs << out_cloud->points[0].x << "," << out_cloud->points[0].y << "," << out_cloud->points[0].z << std::endl;

	in_cloud->clear();
	out_cloud->clear();
	ofs.close();
	std::cout << "CameraPos計算終了!" << std::endl;

	return;
}

void EditingUtility::frameExtract()
{
	std::string file_name;
	//std::string time_of_day;
	std::vector<std::string> save_date;  // 記録した日付
	std::vector<std::string> save_TOD;  // 記録した時刻(Time Of Day)
	std::vector<std::string> extract =
	{
		"21", "21"
	};  // 21-21-xx_xxxxxxのフレームを抽出する

	std::ifstream ifs_data(load_path + ".txt", std::ios::in);
	if (ifs_data.fail()) {
		std::cerr << "失敗" << std::endl;
		return;
	}

	std::ofstream ofs_extract(load_path + "_extract.txt", std::ios::out);
	if (ofs_extract.fail()) {
		std::cerr << "失敗" << std::endl;
		return;
	}

	std::cout << "Frame抽出開始!" << std::endl;

	while (!ifs_data.eof()) {
		getline(ifs_data, file_name);
		split(save_date, file_name, "T");  // 20xx-xx-xx と xx-xx-xx_xxxxxx に分けられる
		//time_of_day = save_date[1];  // 時刻の文字列を入れる
		split(save_TOD, save_date[1], "-");  // xx(h) と xx(m) と xx_xxxxxx(s_μsec) に分けられる
		if ((save_TOD[0] == extract[0]) && (save_TOD[1] == extract[1])) {
			ofs_extract << file_name << std::endl;
		}
		save_date.clear();
		save_TOD.clear();
	}
	ofs_extract.close();
	ifs_data.close();

	std::cout << "Frame抽出終了!" << std::endl;

	return;
}

void EditingUtility::loadMatchingFrame()  //同一時刻のフレームをマッチングするぅ
{
	std::string str;
	std::vector<std::string> file_name;

	std::ifstream ifs_matching_list("DIND112_104_107_108_109_110_111_pair.txt", std::ios::in);
	if (ifs_matching_list.fail()) {
		std::cerr << "失敗" << std::endl;
		return;
	}

	while (!ifs_matching_list.eof()) {
		getline(ifs_matching_list, str);
		split(file_name, str, ",");
		dind104List.push_back(file_name[0]);
		dind107List.push_back(file_name[1]);
		dind108List.push_back(file_name[2]);
		dind109List.push_back(file_name[3]);
		dind110List.push_back(file_name[4]);
		dind111List.push_back(file_name[5]);
		dind112List.push_back(file_name[6]);
		file_name.clear();
	}

	return;
}

void EditingUtility::matchingFrameMarge()
{
	std::string before_name104 = { '\0' };
	std::string before_name107 = { '\0' };
	std::string before_name108 = { '\0' };
	std::string before_name109 = { '\0' };
	std::string before_name110 = { '\0' };
	std::string before_name111 = { '\0' };
	std::string before_name112 = { '\0' };
	std::string file_name;
	std::vector<std::string> file_path =
	{
		"C:/Users/" + userName + "/Desktop/DIND104-PC/2017-01-31_21-20-00_DIND104-PC-PointCloud/",
		"C:/Users/" + userName + "/Desktop/DIND107-PC/2017-01-31_21-20-00_DIND107-PC-PointCloud/",
		"C:/Users/" + userName + "/Desktop/DIND108-PC/2017-01-31_21-20-00_DIND108-PC-PointCloud/",
		"C:/Users/" + userName + "/Desktop/DIND109-PC/2017-01-31_21-20-00_DIND109-PC-PointCloud/",
		"C:/Users/" + userName + "/Desktop/DIND110-PC/2017-01-31_21-20-00_DIND110-PC-PointCloud/",
		"C:/Users/" + userName + "/Desktop/DIND111-PC/2017-01-31_21-20-00_DIND111-PC-PointCloud/",
		"C:/Users/" + userName + "/Desktop/DIND112-PC/2017-01-31_21-20-00_DIND112-PC-PointCloud/"
	};

	std::vector<std::string> dind_id =
	{
		"DIND104", "DIND107", "DIND108", "DIND109", "DIND110", "DIND111", "DIND112"
	};
	std::vector<int> index104, index107, index108, index109, index110, index111, index112;

	cv::Point3d view_pos104, view_pos107, view_pos108, view_pos109, view_pos110, view_pos111, view_pos112;

	pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud104(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud107(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud108(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud109(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud110(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud111(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud112(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals104(new pcl::PointCloud<pcl::PointNormal>);
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals107(new pcl::PointCloud<pcl::PointNormal>);
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals108(new pcl::PointCloud<pcl::PointNormal>);
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals109(new pcl::PointCloud<pcl::PointNormal>);
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals110(new pcl::PointCloud<pcl::PointNormal>);
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals111(new pcl::PointCloud<pcl::PointNormal>);
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals112(new pcl::PointCloud<pcl::PointNormal>);

	pcl::PointCloud<pcl::PointNormal> marge_cloud;
	//pcl::PointCloud<pcl::PointNormal>::Ptr plane(new pcl::PointCloud<pcl::PointNormal>);
	//pcl::PointCloud<pcl::PointNormal>::Ptr without_plane(new pcl::PointCloud<pcl::PointNormal>);

	pcl::PLYReader ply_reader;
	pcl::PLYWriter ply_writer;

	// Load view pos
	view_pos104 = getCameraPos(dind_id[0]);
	view_pos107 = getCameraPos(dind_id[1]);
	view_pos108 = getCameraPos(dind_id[2]);
	view_pos109 = getCameraPos(dind_id[3]);
	view_pos110 = getCameraPos(dind_id[4]);
	view_pos111 = getCameraPos(dind_id[5]);
	view_pos112 = getCameraPos(dind_id[6]);

	loadMatchingFrame();

	std::cout << "Marge開始" << std::endl;

	for (int i = 0; i < dind112List.size(); ++i) {
		marge_cloud.clear();

		if (dind104List[i] != before_name104) {
			before_name104 = dind104List[i];
			ply_reader.read<pcl::PointXYZ>(file_path[0] + dind104List[i] + "-pointcloud.ply", *in_cloud104);
			pcl::removeNaNFromPointCloud(*(computeNormals(in_cloud104, view_pos104)), *cloud_with_normals104, index104);
			pcl::removeNaNNormalsFromPointCloud(*cloud_with_normals104, *cloud_with_normals104, index104);
		}

		if (dind107List[i] != before_name107) {
			before_name107 = dind107List[i];
			ply_reader.read<pcl::PointXYZ>(file_path[1] + dind107List[i] + "-pointcloud.ply", *in_cloud107);
			pcl::removeNaNFromPointCloud(*(computeNormals(in_cloud107, view_pos107)), *cloud_with_normals107, index107);
			pcl::removeNaNNormalsFromPointCloud(*cloud_with_normals107, *cloud_with_normals107, index107);
		}

		if (dind108List[i] != before_name108) {
			before_name108 = dind108List[i];
			ply_reader.read<pcl::PointXYZ>(file_path[2] + dind108List[i] + "-pointcloud.ply", *in_cloud108);
			pcl::removeNaNFromPointCloud(*(computeNormals(in_cloud108, view_pos108)), *cloud_with_normals108, index108);
			pcl::removeNaNNormalsFromPointCloud(*cloud_with_normals108, *cloud_with_normals108, index108);
		}

		if (dind109List[i] != before_name109) {
			before_name109 = dind109List[i];
			ply_reader.read<pcl::PointXYZ>(file_path[3] + dind109List[i] + "-pointcloud.ply", *in_cloud109);
			pcl::removeNaNFromPointCloud(*(computeNormals(in_cloud109, view_pos109)), *cloud_with_normals109, index109);
			pcl::removeNaNNormalsFromPointCloud(*cloud_with_normals109, *cloud_with_normals109, index109);
		}

		if (dind110List[i] != before_name110) {
			before_name110 = dind110List[i];
			ply_reader.read<pcl::PointXYZ>(file_path[4] + dind110List[i] + "-pointcloud.ply", *in_cloud110);
			pcl::removeNaNFromPointCloud(*(computeNormals(in_cloud110, view_pos110)), *cloud_with_normals110, index110);
			pcl::removeNaNNormalsFromPointCloud(*cloud_with_normals110, *cloud_with_normals110, index110);
		}

		if (dind111List[i] != before_name111) {
			before_name111 = dind111List[i];
			ply_reader.read<pcl::PointXYZ>(file_path[5] + dind111List[i] + "-pointcloud.ply", *in_cloud111);
			pcl::removeNaNFromPointCloud(*(computeNormals(in_cloud111, view_pos111)), *cloud_with_normals111, index111);
			pcl::removeNaNNormalsFromPointCloud(*cloud_with_normals111, *cloud_with_normals104, index111);
		}

		if (dind112List[i] != before_name112) {
			before_name112 = dind112List[i];
			ply_reader.read<pcl::PointXYZ>(file_path[6] + dind112List[i] + "-pointcloud.ply", *in_cloud112);
			pcl::removeNaNFromPointCloud(*(computeNormals(in_cloud112, view_pos112)), *cloud_with_normals112, index112);
			pcl::removeNaNNormalsFromPointCloud(*cloud_with_normals112, *cloud_with_normals112, index112);
		}

		marge_cloud = *cloud_with_normals104;
		marge_cloud += *cloud_with_normals107;
		marge_cloud += *cloud_with_normals108;
		marge_cloud += *cloud_with_normals109;
		marge_cloud += *cloud_with_normals110;
		marge_cloud += *cloud_with_normals111;
		marge_cloud += *cloud_with_normals112;

		ply_writer.write<pcl::PointNormal>("C:/Users/" + userName + "/Desktop/marge/PointCloud/Frame_" + std::to_string(i + 1) + "-maregecloud.ply", marge_cloud, true, false);

		in_cloud104->clear();
		in_cloud107->clear();
		in_cloud108->clear();
		in_cloud109->clear();
		in_cloud110->clear();
		in_cloud111->clear();
		in_cloud112->clear();
		index104.clear();
		index107.clear();
		index108.clear();
		index109.clear();
		index110.clear();
		index111.clear();
		index112.clear();
	}

	cloud_with_normals104->clear();
	cloud_with_normals107->clear();
	cloud_with_normals108->clear();
	cloud_with_normals109->clear();
	cloud_with_normals110->clear();
	cloud_with_normals111->clear();
	cloud_with_normals112->clear();

	return;
}

void EditingUtility::computeMesh()
{
	std::vector<pcl::PointIndices> cluster_index;
	std::vector<pcl::PointIndices> plane_cluster;
	pcl::PointCloud<pcl::PointNormal>::Ptr in_cloud(new pcl::PointCloud<pcl::PointNormal>);
	pcl::PointCloud<pcl::PointNormal>::Ptr plane(new pcl::PointCloud<pcl::PointNormal>);
	pcl::PointCloud<pcl::PointNormal>::Ptr others(new pcl::PointCloud<pcl::PointNormal>);
	pcl::ExtractIndices<pcl::PointNormal> extract;

	pcl::PLYReader ply_reader;
	pcl::PLYWriter ply_writer;
	pcl::PolygonMesh mesh;
	pcl::PolygonMesh mesh2;

	for (int i = 0; i < 1; ++i) {
		ply_reader.read<pcl::PointNormal>("C:/Users/" + userName + "/Desktop/marge/PointCloud/Frame_" + std::to_string(i + 1) + "-maregecloud.ply", *in_cloud);
		planeSegmentation(in_cloud, plane, others);
		//mesh = p2m.Poisson(plane);
		//pcl::io::savePLYFileBinary("C:/Users/" + userName + "/Desktop/marge/Mesh/Frame_" + std::to_string(i + 1) + "_plane1-cluster.ply", *plane);
		//pcl::io::savePolygonFileSTL("C:/Users/" + userName + "/Desktop/marge/Mesh/Frame_" + std::to_string(i + 1) + "_plane1-mesh.stl", mesh);

		//pcl::copyPointCloud<pcl::PointNormal>(*others, *in_cloud);
		//plane->clear();
		//others->clear();
		//planeSegmentation(in_cloud, plane, others);
		//mesh2 = p2m.Poisson(plane);
		//pcl::io::savePLYFileBinary("C:/Users/" + userName + "/Desktop/marge/Mesh/Frame_" + std::to_string(i + 1) + "_plane2-cluster.ply", *plane);
		//pcl::io::savePolygonFileSTL("C:/Users/" + userName + "/Desktop/marge/Mesh/Frame_" + std::to_string(i + 1) + "_plane2-mesh.stl", mesh2);
		mesh2 = p2m.GridProjection(plane);
		//pcl::io::savePLYFileBinary("C:/Users/" + userName + "/Desktop/marge/Mesh/Frame_" + std::to_string(i + 1) + "_others-cluster.ply", *others);
		pcl::io::savePolygonFileSTL("C:/Users/" + userName + "/Desktop/marge/Mesh/Frame_" + std::to_string(i + 1) + "_others-mesh.stl", mesh2);
	}

	//planeSegmentation(cloud_with_normals, plane, without_plane);
	//mesh = p2m.GreedyMesher(plane);
	//mesh = p2m.Poisson(without_plane);
	//pcl::io::savePLYFile(load_path + "-Extract/" + file_name + "-mesh.ply", mesh);
}