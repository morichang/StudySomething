// stdafx.h : 標準のシステム インクルード ファイルのインクルード ファイル、または
// 参照回数が多く、かつあまり変更されない、プロジェクト専用のインクルード ファイル
// を記述します。
//

#pragma once

#include "targetver.h"

#include <stdio.h>
#include <tchar.h>



// TODO: プログラムに必要な追加ヘッダーをここで参照してください。
#include <opencv2/opencv.hpp>

// バージョン取得
#define CV_VERSION_STR CVAUX_STR(CV_MAJOR_VERSION) CVAUX_STR(CV_MINOR_VERSION) CVAUX_STR(CV_SUBMINOR_VERSION)

// ビルドモード
#ifdef _DEBUG
#define CV_EXT_STR "d.lib"
#else
#define CV_EXT_STR ".lib"
#endif
//ライブラリのリンク
#pragma comment(lib, "opencv_core"		CV_VERSION_STR CV_EXT_STR)
#pragma comment(lib, "opencv_highgui"	CV_VERSION_STR CV_EXT_STR)
#pragma comment(lib, "opencv_imgproc"	CV_VERSION_STR CV_EXT_STR)
#pragma comment(lib, "opencv_video"		CV_VERSION_STR CV_EXT_STR)


/*
PCL
*/

#define _CRT_SECURE_NO_WARNINGS
#pragma warning(disable : 4996)

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>

#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/passthrough.h>

//#include <pcl/ModelCoefficients.h>
//#include <pcl/sample_consensus/method_types.h>
//#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
//#include <pcl/filters/voxel_grid.h>

#ifdef _DEBUG
#pragma comment(lib, "pcl_common_debug.lib")
#pragma comment(lib, "pcl_io_debug.lib")
#pragma comment(lib, "pcl_visualization_debug.lib")

#pragma comment(lib, "pcl_kdtree_debug.lib")

#pragma comment(lib, "flann-gd.lib")
#else
#pragma comment(lib, "pcl_common_release.lib")
#pragma comment(lib, "pcl_io_release.lib")
#pragma comment(lib, "pcl_visualization_release.lib")

#pragma comment(lib, "pcl_common_release.lib")
#pragma comment(lib, "pcl_features_release.lib")
#pragma comment(lib, "pcl_filters_release.lib")
#pragma comment(lib, "pcl_io_ply_release.lib")
#pragma comment(lib, "pcl_io_release.lib")
#pragma comment(lib, "pcl_kdtree_release.lib")
#pragma comment(lib, "pcl_keypoints_release.lib")
#pragma comment(lib, "pcl_octree_release.lib")
#pragma comment(lib, "pcl_outofcore_release.lib")
#pragma comment(lib, "pcl_people_release.lib")
#pragma comment(lib, "pcl_recognition_release.lib")
#pragma comment(lib, "pcl_registration_release.lib")
#pragma comment(lib, "pcl_sample_consensus_release.lib")
#pragma comment(lib, "pcl_search_release.lib")
#pragma comment(lib, "pcl_segmentation_release.lib")
#pragma comment(lib, "pcl_surface_release.lib")
#pragma comment(lib, "pcl_tracking_release.lib")
#pragma comment(lib, "pcl_visualization_release.lib")
//FLANN
#pragma comment(lib, "flann.lib")
#pragma comment(lib, "flann_cpp_s.lib")
#pragma comment(lib, "flann_s.lib")
//VTK
#pragma comment(lib, "vtkChartsCore-6.2.lib")
#pragma comment(lib, "vtkCommonColor-6.2.lib")
#pragma comment(lib, "vtkCommonCore-6.2.lib")
#pragma comment(lib, "vtkCommonDataModel-6.2.lib")
#pragma comment(lib, "vtkCommonSystem-6.2.lib")
#pragma comment(lib, "vtkCommonMath-6.2.lib")
#pragma comment(lib, "vtkCommonMisc-6.2.lib")

#pragma comment(lib, "vtkFiltersCore-6.2.lib")
#pragma comment(lib, "vtkFiltersGeneral-6.2.lib")
#pragma comment(lib, "vtkftgl-6.2.lib")

#pragma comment(lib, "vtkImagingColor-6.2.lib")
#pragma comment(lib, "vtkImagingCore-6.2.lib")
#pragma comment(lib, "vtkImagingGeneral-6.2.lib")
#pragma comment(lib, "vtkImagingMath-6.2.lib")

#pragma comment(lib, "vtkViewsContext2D-6.2.lib")
#pragma comment(lib, "vtkViewsCore-6.2.lib")
#pragma comment(lib, "vtkImagingGeneral-6.2.lib")
#pragma comment(lib, "vtkImagingMath-6.2.lib")

#pragma comment(lib, "vtkRenderingAnnotation-6.2.lib")
#pragma comment(lib, "vtkRenderingContext2D-6.2.lib")
#pragma comment(lib, "vtkRenderingCore-6.2.lib")
#pragma comment(lib, "vtkRenderingFreeType-6.2.lib")
#pragma comment(lib, "vtkRenderingImage-6.2.lib")
#pragma comment(lib, "vtkRenderingLabel-6.2.lib")
#endif