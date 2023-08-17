#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/common/centroid.h>

#include <opencv2/opencv.hpp>

#include"common.h"




int main(int argc, char** argv)
{
	std::string file_path = "D:\\Project\\平面度数据\\ROI";//自己设置目录  
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
		std::cout << files[i].c_str() << std::endl;

		std::string::size_type iPos = files[i].find_last_of("\\") + 1;
		std::string name = files[i].substr(iPos, files[i].length() - iPos);//获取带后缀的文件名
		std::string path = files[i].substr(0, iPos);//获取文件路径
		std::string nameNoTag = name.substr(0, name.rfind(".")); //huoqu-不带后缀的文件名    //从字符串右侧开始匹配str，并返回在字符串中的位置（下标）

		//1.读取点云
		pcl::PointCloud<pcl::PointXYZ> roi_pc_pts;
		readPointCloudTxt(files[i], roi_pc_pts);
		
		//7.计算平面度
		auto start = std::chrono::steady_clock::now();

		float flatness = computeFlatness(roi_pc_pts, 0.07);
		std::cout << "平面度: " << flatness << std::endl;

		auto end = std::chrono::steady_clock::now();
		auto time = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
		std::cout << "计算总时间: " << time << "  ms." << std::endl;


		write << name << "," << time << "," << flatness << std::endl;
		flatness_vec.push_back(flatness);
	}
	float max_flatness = *(std::max_element(flatness_vec.begin(), flatness_vec.end()));
	float min_flatness = *(std::min_element(flatness_vec.begin(), flatness_vec.end()));
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