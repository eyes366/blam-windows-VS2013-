#include <blam_slam/BlamSlam.h>
#include"common.h"
#include"hdevicePose.h"

#include <point_cloud_read/pcap_read.h>

// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>
// 类型定义
#include <geometry_utils/Transform3.h>
//#include <parameter_utils/ParameterUtils.h>
#include<sstream>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/random_sample.h>

#include "parameter_utils/slamBase.h"

//namespace pu = parameter_utils;
namespace gu = geometry_utils;

typedef pcl::POINT_TYPE PointT;
typedef pcl::PointCloud<PointT> PointCloud;

using namespace pcl::visualization;
BlamSlam blamSlam_;

std::vector<LaserLoopClosure::ConstraitLine> g_LoopLines;

PointCloud::Ptr pcap2PointCloud(std::vector<_array> &Points)
{
	PointCloud::Ptr cloud(new PointCloud);
	int cnt = 0;
	for (int i = 0; i < Points.size(); i++)
	{
		//if (Points[i].LaserId == 1)
		{
			PointT p;
			cnt++;
			p.z = Points[i].xyz[2];
			p.x = Points[i].xyz[0];
			p.y = Points[i].xyz[1];

			p.intensity = Points[i].xyz[2];
			cloud->points.push_back(p);
		}

	}
	// 设置并保存点云
	cloud->height = 1;
	cloud->width = cnt;
	cloud->is_dense = false;

	return cloud;
}
std::string itos(int i)   // 将int 转换成string 
{
	std::stringstream s;
	s << i;
	return s.str();
}

long long frameNumber = 0;
long long totalNumber = 0;
bool endFrame = 0;
pcl::PointXYZ pointCam_old, pointCam_new;
Eigen::Transform<double, 3, Eigen::Affine> ICP_Transform;
bool g_bIsDispCloud = false;
int g_nCameraType = 0;
int g_cnt = 0;
void viewerPsycho(pcl::visualization::PCLVisualizer& viewer)
{
	std::stringstream ss;
	ss << "Once per viewer loop: " << g_cnt/*frameNumber*/ << "/" << totalNumber << endl;
	ss << "Display cloud points(Press 'a' to display cloud points): " << g_bIsDispCloud << endl;
	ss << "Camera type(Press 'c' to follow lidar): " << g_nCameraType << endl;
	viewer.removeShape("text", 0);
	viewer.addText(ss.str(), 50, 50, 15.0, 1.0, 1.0, 1.0,"text", 0);

	if (g_cnt/*frameNumber*/ > 1)
	{
		//viewer.addLine(pointCam_old, pointCam_new, 1, 0, 0, itos(cnt++), 0);
	}
	if (g_cnt/*frameNumber*/ == 1)
	{
		viewer.addCoordinateSystem(1.0, (Eigen::Affine3f)ICP_Transform, "reference", 0);     //Add CoordinateSystem
		viewer.setCameraPosition(0, 0, 30,
			0, 0, 0,
			0, 0, 0);
	}
	viewer.updateCoordinateSystemPose("reference", (Eigen::Affine3f)ICP_Transform);

	for (unsigned int i = 0; i < g_LoopLines.size(); i++)
	{
		char szItem[1024] = { 0 };
		sprintf(szItem, "Line%d", i);
		if (viewer.contains(szItem))
		{
			viewer.removeShape(szItem);
		}
		viewer.addLine<pcl::PointXYZ, pcl::PointXYZ>(g_LoopLines[i].first, g_LoopLines[i].second,
			0.0, 255.0, 0.0, szItem);
	}

	if (endFrame || g_cnt%100 != 0)
	{
		return;
	}
	
	pcl::visualization::Camera cam;
	viewer.getCameraParameters(cam);
	cam.pos[0] = ICP_Transform.matrix()(0, 3);
	cam.pos[1] = ICP_Transform.matrix()(1, 3);
//	cam.pos[2] = ICP_Transform.matrix()(2, 3);
	if (g_nCameraType == 0)
	{
		viewer.setCameraPosition(cam.pos[0], cam.pos[1], cam.pos[2],
			cam.pos[0], cam.pos[1], ICP_Transform.matrix()(2, 3),
			cam.view[0], cam.view[1], cam.view[2]);
	}

	if (viewer.contains("m_closureCondidates"))
	{
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3.0, "m_closureCondidates");
	}
	
	
	//viewer.updateCoordinateSystemPose("reference", (Eigen::Affine3f)ICP_Transform); //update CoordinateSystem
	//FIXME: possible race condition here:  
}

//DWORD WINAPI VisualizationThread(LPVOID lpparentet)
//{
//	pcl::visualization::PCLVisualizer viewer("viewer1");
//	viewer.addCoordinateSystem(1.0, (Eigen::Affine3f)ICP_Transform, "reference", 0);     //Add CoordinateSystem
//	static long long cnt = 0;
//
//	while (!viewer.wasStopped())
//	{
//		viewer.addPointCloud<pcl::POINT_TYPE>(blamSlam_.mapper_.map_data_, itos(frameNumber++));
//		const gu::Transform3 estimate = blamSlam_.localization_.GetIntegratedEstimate();
//		const Eigen::Matrix<double, 3, 3> R = estimate.rotation.Eigen();
//		const Eigen::Matrix<double, 3, 1> T = estimate.translation.Eigen();
//		Eigen::Matrix4d tf = Eigen::Matrix4d::Identity();;
//		tf.block(0, 0, 3, 3) = R;
//		tf.block(0, 3, 3, 1) = T;
//		pcl::PointXYZ  mm;
//		mm.x = T(0);
//		mm.y = T(1);
//		mm.z = T(2);
//		pointCam_old = pointCam_new;
//		pointCam_new = mm;
//		ICP_Transform = Eigen::Transform<double, 3, Eigen::Affine>(tf);
//		std::stringstream ss;
//		ss << "Once per viewer loop: " << frameNumber;
//		viewer.removeShape("text", 0);
//		viewer.addText(ss.str(), 50, 50, "text", 0);
//		viewer.addLine(pointCam_old, pointCam_new, 1, 0, 0, itos(cnt++), 0);
//		viewer.updateCoordinateSystemPose("reference", (Eigen::Affine3f)ICP_Transform);
//		viewer.spinOnce(10);
//	}
//	
//	return 0;
//}

void key_callback(const pcl::visualization::KeyboardEvent& key_event, void* puser)
{
	if (key_event.keyDown() && (key_event.getKeyCode() == 'a' || key_event.getKeyCode() == 'A'))
	{
		g_bIsDispCloud = !g_bIsDispCloud;
		cout << "g_bIsDispCloud: " << g_bIsDispCloud << endl;
	}
	if (key_event.keyDown() && (key_event.getKeyCode() == 'c' || key_event.getKeyCode() == 'C'))
	{
		g_nCameraType++;
		if (g_nCameraType >= 2)
		{
			g_nCameraType = 0;
		}
		cout << "g_nCameraType: " << g_nCameraType << endl;
	}
}

int main(void)
{
	//	pcl::visualization::CloudViewer viewer("viewer2");
	//
	//	PointCloud::Ptr cloud_(new PointCloud);
	//	pcl::io::loadPCDFile("./2.pcd", *cloud_);
	//
	////	pcl::PassThrough<pcl::POINT_TYPE> pass;
	////	pcl::PointCloud<pcl::POINT_TYPE>::Ptr cloud_(new pcl::PointCloud<pcl::POINT_TYPE>);
	//	/*pass.setInputCloud(cloud_);
	//	pass.setFilterFieldName("z");
	//	pass.setFilterLimits(0.0, 1.0);
	//	pass.filter(*cloud_filtered);*/
	//	int maxDAT = 0;
	//	int minDAT = 10000;
	//
	//	for (int i = 0; i < cloud_->size(); i++)
	//	{
	//		if (maxDAT < cloud_->points[i].z)
	//		{
	//			maxDAT = (int)cloud_->points[i].z;
	//		}
	//		if (minDAT > cloud_->points[i].z)
	//		{
	//			minDAT = (int)cloud_->points[i].z;
	//		}
	//	}
	//	std::cout << "max: " << maxDAT << "min: " << minDAT << std::endl;
	//	float t = (maxDAT - minDAT + 0.00001) / 10.0;
	//	for (int i = 0; i < cloud_->size(); i++)
	//	{
	//		cloud_->points[i].intensity = ((int)((cloud_->points[i].z - minDAT) / t+0.5)) * 20;
	//	}
	//	viewer.showCloud(cloud_, "viewer2");
	//	while (!viewer.wasStopped()){}
	//
	//	const std::string fileNamePcap = "D:\\MyData\\0.pcap";
	//	const std::string fileNamePcap = "E:\\MyData\\BackPack20180321\\20180321111741\\Laser\\RawData\\2018-03-21_11-17-42-692.pcap";
	ParameterReader param_rd;
	std::string fileNamePcap = param_rd.getData("pcap_file_path");
	cout << "fileNamePcap:" << fileNamePcap << endl;
	const std::string calibrationPath = "./VLP-16.xml";
	MyHdlGrabber _myHdlGrabber(fileNamePcap, calibrationPath);
	totalNumber = _myHdlGrabber.GetNumberOfFrames();

	blamSlam_.Initialize();
	pcl::visualization::CloudViewer viewer2("viewer1");
	viewer2.runOnVisualizationThread(viewerPsycho);
	viewer2.registerKeyboardCallback(key_callback);

	//	CreateThread(NULL, 0, VisualizationThread, NULL, 0, NULL);
	devicePose devicePose_;
	FILE* file = fopen("corRT.txt", "w");

	pcl::PointCloud<pcl::PointXYZI>::Ptr track(new pcl::PointCloud<pcl::PointXYZI>);

//	static int cnt = 0;
	while (!viewer2.wasStopped())
	{
		if (endFrame)
		{
			continue;
		}
		if (g_cnt % 2000 == 0 && g_cnt != 0)
		{
			std::cout << "Save map: " << g_cnt << std::endl;
			pcl::PCDWriter writer;
			char szMapSaveName[1024] = { 0 };
			sprintf(szMapSaveName, "%06dMap.pcd", g_cnt);
			writer.write(blamSlam_.m_szLogPath + szMapSaveName, *blamSlam_.mapper_.map_data_);
		}
		// 		if (cnt > 1580)
		// 		{
		// 			std::cout << "No Frame!" << std::endl;
		// 			endFrame = 1;
		// 			pcl::PCDWriter writer;
		// 			writer.write("./1.pcd", *blamSlam_.mapper_.map_data_);
		// 			continue;
		// 		}
		if (_myHdlGrabber.GetFrame(g_cnt++)) {
			fclose(file);
			endFrame = 1;
			pcl::PCDWriter writer;
			writer.write(blamSlam_.m_szLogPath+"save_c_loop.pcd", *blamSlam_.mapper_.map_data_);
			std::cout << "No Frame!" << std::endl;
			continue;
		}
		//		cout << "Frame: " << g_cnt << endl;
		PointCloud::Ptr newCloud = pcap2PointCloud(_myHdlGrabber.Points);
		//		cout << "Point cont: %d" << newCloud->size() << endl;
		// 		char szLogName[1024] = { 0 };
		// 		sprintf(szLogName, "%06d.pcd", g_cnt);
		// 		pcl::io::savePCDFileBinary(szLogName, *newCloud);
		pcl::PCDWriter writer;
//		writer.write("./1.pcd", *newCloud);
		blamSlam_.ProcessPointCloudMessage(newCloud);
		const gu::Transform3 estimate = blamSlam_.localization_.GetIntegratedEstimate();
		const Eigen::Matrix<double, 3, 3> R = estimate.rotation.Eigen();
		const Eigen::Matrix<double, 3, 1> T = estimate.translation.Eigen();
		Eigen::Matrix4d tf = Eigen::Matrix4d::Identity();;
		tf.block(0, 0, 3, 3) = R;
		tf.block(0, 3, 3, 1) = T;
		pcl::PointXYZ  mm;
		mm.x = T(0);
		mm.y = T(1);
		mm.z = T(2);
		double szR[9];
		double szT[3];


		for (int x = 0; x < 3; x++)
		{
			szT[x] = T(x);
			for (int y = 0; y < 3; y++)
			{
				szR[x * 3 + y] = R(x * 3 + y);
			}
		}

		devicePose_.setR(szR);
		devicePose_.setT(szT);
		devicePose_.write(file);
		//fprintf(file, "\r\n");
		//fclose(file);

		pointCam_old = pointCam_new;
		pointCam_new = mm;
		ICP_Transform = Eigen::Transform<double, 3, Eigen::Affine>(tf);

// 		pcl::PointXYZI pt;
// 		pt.x = T(0);
// 		pt.y = T(1);
// 		pt.z = T(2);
// 		pt.intensity = 1.0;
// 		track->points.push_back(pt);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr pTrack = blamSlam_.loop_closure_.GetTrack();
		if (g_bIsDispCloud)
		{
			viewer2.showCloud(/*track*/blamSlam_.mapper_.map_data_, "viewer1");
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr pt_cur(new pcl::PointCloud<pcl::PointXYZRGB>);
			pcl::copyPointCloud<pcl::PointXYZI, pcl::PointXYZRGB>(*newCloud, *pt_cur);
			pcl::transformPointCloud(*pt_cur, *pt_cur, tf);
			for (unsigned int i = 0; i < pt_cur->size(); i++)
			{
				pt_cur->points[i].r = 255.0;
				pt_cur->points[i].g = 255.0;
				pt_cur->points[i].b = 255.0;
			}
			viewer2.showCloud(pt_cur, "cur_point");
//			viewer2.showCloud(blamSlam_.loop_closure_.m_closureCondidates.makeShared(), "m_closureCondidates");
//			viewer2.showCloud(blamSlam_.loop_closure_.m_closureValid.makeShared(), "m_closureValid");
		}
		else
		{
			pcl::PointCloud<pcl::PointXYZI> NoUsed;
			viewer2.showCloud(NoUsed.makeShared(), "viewer1");
			viewer2.showCloud(NoUsed.makeShared(), "cur_point");
//			viewer2.showCloud(NoUsed.makeShared(), "m_closureCondidates");
//			viewer2.showCloud(NoUsed.makeShared(), "m_closureValid");
		}
		viewer2.showCloud(blamSlam_.loop_closure_.m_closureCondidates.makeShared(), "m_closureCondidates");
		viewer2.showCloud(pTrack, "Track");
		g_LoopLines = blamSlam_.loop_closure_.m_LoopLines;
		frameNumber++;
		//showPointCloud(g_cnt - 1, newCloud);
	}
	return 0;
}
