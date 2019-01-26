#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/PCLPointCloud2.h>
#include <iostream>
#include <string>
#include <opencv2/opencv.hpp>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/io.h>
//#include <vtkAutoInit.h>

//using namespace pcl;
using namespace pcl::io;
using namespace cv;
using namespace std;

//VTK_MODULE_INIT(vtkRenderingOpenGL);
//VTK_MODULE_INIT(vtkInteractionStyle);
using namespace std;
int user_data;
//第一部分函数：
/*
pcl::PointCloud<pcl::PointXYZRGB>::Ptr depth2cloud(cv::Mat rgb_image, cv::Mat depth_image)
{
	float f = 570.3;
	float cx = 342.0, cy = 456.0;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
	cloud_ptr->width = rgb_image.cols;
	cloud_ptr->height = rgb_image.rows;
	cloud_ptr->is_dense = false;

	for (int y = 0; y < rgb_image.rows; ++y) {
		for (int x = 0; x < rgb_image.cols; ++x) {
			pcl::PointXYZRGB pt;
			//pt.r = rgb_image.at<cv::Vec3b>(y, x)[2];
			if (depth_image.at<unsigned short>(y, x) != 0)
			{
				pt.z = depth_image.at<unsigned short>(y, x) / 1000.0;
				//pt.z = depth_image.at<unsigned short>(y, x);
				pt.x = (x - cx)*pt.z / f;
				pt.y = (y - cy)*pt.z / f;
				pt.r = rgb_image.at<cv::Vec3b>(y, x)[2];
				pt.g = rgb_image.at<cv::Vec3b>(y, x)[1];
				pt.b = rgb_image.at<cv::Vec3b>(y, x)[0];
				cloud_ptr->points.push_back(pt);
			}
			else
			{
				pt.z = 0;
				pt.x = 0;
				pt.y = 0;
				pt.r = 0;
				pt.g = 0;
				pt.b = 0;
				cloud_ptr->points.push_back(pt);
			}
		}
	}
	return cloud_ptr;
}
*/
//第二部分函数：
void viewerOneOff(pcl::visualization::PCLVisualizer& viewer)
{
	viewer.setBackgroundColor(1.0, 0.5, 1.0);
	pcl::PointXYZ o;
	o.x = 1.0;
	o.y = 0;
	o.z = 0;
	viewer.addSphere(o, 0.25, "sphere", 0);
	std::cout << "i only run once" << std::endl;

}

void viewerPsycho(pcl::visualization::PCLVisualizer& viewer)
{
	static unsigned count = 0;
	std::stringstream ss;
	ss << "Once per viewer loop: " << count++;
	viewer.removeShape("text", 0);
	viewer.addText(ss.str(), 200, 300, "text", 0);

	//FIXME: possible race condition here:
	user_data++;
}
//第三部分函数：
int PCDtoPLYconvertor(string & input_filename, string& output_filename)
{
	pcl::PCLPointCloud2 cloud;
	if (loadPCDFile(input_filename, cloud) < 0)
	{
		cout << "Error: cannot load the PCD file!!!" << endl;
		return -1;
	}
	pcl::PLYWriter writer;
	writer.write(output_filename, cloud, Eigen::Vector4f::Zero(), Eigen::Quaternionf::Identity(), true, true);
	return 0;

}

int main(int argc, char* argv[])
{

	string str = "F:\\c++project\\DENSE";
	string className = "";
	string imgPath = str + className + "_img2.png";
	string depthPath = str + className + "_prediction.png";
	string pcdName2 = "F:\\c++project\\DENSE\\result.pcd";

	cv::Mat depth;
	cv::Mat image;

	//第一部分：生成点云pcd文件
	/*
	image = cv::imread(imgPath);
	//cv::Size t_s;
	//t_s.width = 320;
	//t_s.height = 240;
	//cv::resize(image, image, t_s);
	depth = cv::imread(depthPath, IMREAD_ANYDEPTH);
	//cv::resize(depth, depth, t_s);
	string pcdName(pcdName2);
	if (!image.data || !depth.data)        // 判断图片调入是否成功
	return -1;        // 调入图片失败则退出

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

	cloud = depth2cloud(image, depth);

	pcl::io::savePCDFileASCII(pcdName, *cloud);
	*/

	//第二部分：显示pcd文件
	//*
	cout << "line 136" << endl;
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
	cout << "line 138" << endl;
	pcl::io::loadPCDFile("/home/yixuan/DENSE/build/result.pcd", *cloud);
	cout << "line 140" << endl;
	/*
	pcl::visualization::CloudViewer viewer("Cloud Viewer");

	//blocks until the cloud is actually rendered
	viewer.showCloud(cloud);

	//use the following functions to get access to the underlying more advanced/powerful
	//PCLVisualizer

	//This will only get called once
	viewer.runOnVisualizationThreadOnce(viewerOneOff);
	cout << "line 150" << endl;
	//This will get called once per visualization iteration
	viewer.runOnVisualizationThread(viewerPsycho);
	while (!viewer.wasStopped())
	{
		user_data++;
	}
	*/

	//第三部分：将pcd转化为ply
	
	string input_filename = "/home/yixuan/DENSE/build/result.pcd";
	string output_filename = "/home/yixuan/pcd2ply/result.ply";
	PCDtoPLYconvertor(input_filename, output_filename);
	

	return 0;
}
