#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/common/centroid.h>

#include <opencv2/opencv.hpp>

#include"common.h"




int main(int argc, char** argv)
{
	std::string file_path = "D:\\Project\\平面度测量项目\\平面度数据\\小板C点云-0727-PCD";//自己设置目录  
	std::vector<std::string> files;
	//获取该路径下的所有文件  
	getFiles(file_path, files);

	
	std::string result_boot = "..\\result\\";

	std::ofstream write(result_boot + "平面度测试结果.csv");
	if (write.is_open())
	{
		std::cout << "打开结果文件成功!" << std::endl;
		write << "文件" << "," << "时间ms" << "," << "平面度mm" << std::endl;
	}
	else
	{
		std::cout << "打开结果文件失败!" << std::endl;
		return -1;
	}

	std::vector<float> flatness_vec;
	for (int i = 0; i < files.size(); i++)
	{
		std::cout << std::endl;
		std::cout << files[i].c_str() << std::endl;

		std::string::size_type iPos = files[i].find_last_of("\\") + 1;
		std::string name = files[i].substr(iPos, files[i].length() - iPos);//获取带后缀的文件名
		std::string path = files[i].substr(0, iPos);//获取文件路径
		std::string nameNoTag = name.substr(0, name.rfind(".")); //huoqu-不带后缀的文件名    //从字符串右侧开始匹配str，并返回在字符串中的位置（下标）

		//1.读取点云
		pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud(new pcl::PointCloud < pcl::PointXYZ>);

		/*pcl::PLYReader plyReader;
		plyReader.read<pcl::PointXYZ>("D:/Project/平面度数据/小板C点云-0727/32106B-OPLY15um-02.ply", *src_cloud);*/

		pcl::PCDReader pcdReader;
		pcdReader.read<pcl::PointXYZ>(files[i], *src_cloud);
		if (src_cloud->empty())
		{
			std::cout << "读取点云失败!" << std::endl;
			return -1;
		}
		else
		{
			std::cout << "读取点云成功!" << std::endl;
		}

		auto start = std::chrono::steady_clock::now();

		//获取有效点的索引
		std::vector<int> valid_point_indices;
		for (int i = 0; i < src_cloud->size(); i++)
		{
			if (src_cloud->points[i].z != INVALID_POINT_Z)
			{
				valid_point_indices.push_back(i);
			}
		}


		//2.生成高度图
		//cv::Mat height_img = generateHeightImage(src_cloud, 0.014, 0.015);
		cv::Mat height_img = generateHeightImage2(src_cloud, valid_point_indices, 7700, 2400);
		//cv::namedWindow("高度图", cv::WINDOW_NORMAL);//可以自由调节显示图像的尺寸
		//cv::imshow("高度图", height_img);
		//cv::imwrite(result_boot + nameNoTag + "  " + "高度图.jpg", height_img);


		//3.找最外围的边缘点
		std::vector<cv::Point> contours = findCounters(height_img);
		////绘制最外围边缘点
		//cv::Mat out_canny_img(height_img.size(), CV_8UC1);
		//for (int i = 0; i < contours.size(); i++)
		//{
		//	out_canny_img.at<uchar>(contours[i].y, contours[i].x) = 255;
		//}
		//cv::imwrite(result_boot + nameNoTag + "  " + "最外围边缘点.jpg", out_canny_img);


		//4.获取18个ROI区域
		std::vector<cv::RotatedRect> roi_rrect = generateROI(contours);
		////绘制旋转矩形与中心位置
		//for (int i = 0; i < roi_rrect.size(); i++)
		//{
		//	drawRotatedRect(height_img, roi_rrect[i]);
		//}
		//cv::imwrite(result_boot + nameNoTag + "  " + "18个ROI区域.jpg", height_img);


		//5.获取18个ROI内的像素点
		std::vector<std::vector<cv::Point2f>> roi_pts;
		for (int i = 0; i < roi_rrect.size(); i++)
		{
			auto pts = computePointInROI(roi_rrect[i]);
			roi_pts.push_back(pts);
		}
		//for (int i = 0; i < roi_pts.size(); i++)
		//{
		//	for (int j = 0; j < roi_pts.size(); j++)
		//	{
		//		height_img.at<cv::Vec3b>(roi_pts[i][j].y, roi_pts[i][j].x) = cv::Vec3b(0, 0, 255);
		//	}
		//}

		//6.获取18个区域内的点云
		pcl::PointCloud<pcl::PointXYZ> roi_pc_pts;
		
		for (int i = 0; i < roi_pts.size(); i++)
		{
			pcl::PointCloud<pcl::PointXYZ> each_roi_pc_pts;
			for (int j = 0; j < roi_pts[i].size(); j++)
			{
				int index = (int)roi_pts[i][j].x + (int)(roi_pts[i][j].y) * height_img.cols;
				if (src_cloud->points[index].z == INVALID_POINT_Z)
					continue;
				each_roi_pc_pts.push_back(src_cloud->points[index]);
			}
			
			roi_pc_pts.insert(roi_pc_pts.end(), each_roi_pc_pts.begin(), each_roi_pc_pts.end());
		}


		//7.计算平面度
		pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
		//pcl::PointCloud<pcl::PointXYZ> cloud_filtered;
		sor.setInputCloud(roi_pc_pts.makeShared());// 填入点云
		sor.setMeanK(30);        // 设置均值参数K
		sor.setStddevMulThresh(1.0);// 设置
		sor.filter(roi_pc_pts);

		float flatness = computeFlatness(roi_pc_pts, 0.1);
		std::cout << "平面度: " << flatness << std::endl;

		auto end = std::chrono::steady_clock::now();
		auto time = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
		std::cout << "计算总时间: " << time << "  ms." << std::endl;

		pcl::io::savePCDFile(result_boot + nameNoTag + "  " + "ROI区域内的点.pcd", roi_pc_pts);

		write << name << "," << time << "," << flatness << std::endl;
		flatness_vec.push_back(flatness);
	}
	//float max_flatness = *(std::max_element(flatness_vec.begin(), flatness_vec.end()));
	//float min_flatness = *(std::min_element(flatness_vec.begin(), flatness_vec.end()));
	std::sort(flatness_vec.rbegin(), flatness_vec.rend());
	float max_flatness = *flatness_vec.begin();
	float min_flatness = *(flatness_vec.end() - 1);
	float repeatability = max_flatness - min_flatness;

	std::cout << "最大值" << "," << max_flatness << std::endl;
	std::cout << "最小值" << "," << min_flatness << std::endl;
	std::cout << "\n重复精度: " << repeatability << "  mm." << std::endl;

	write << std::endl << std::endl;
	write << "最大值" << "," << max_flatness << std::endl;
	write << "最小值" << "," << min_flatness << std::endl;
	write << "重复精度" << "," << repeatability << std::endl;
	write.close();	
	
}