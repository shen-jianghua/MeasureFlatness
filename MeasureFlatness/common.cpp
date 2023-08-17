#include"common.h"
#include<io.h>  
#include<iostream> 
#include<fstream>
#include<sstream>
#include<string>


void getFiles(std::string path, std::vector<std::string>& files)
{
	//文件句柄  
	intptr_t hFile = 0;
	//文件信息，声明一个存储文件信息的结构体  
	struct _finddata_t fileinfo;
	std::string p;  //字符串，存放路径
	if ((hFile = _findfirst(p.assign(path).append("\\*").c_str(), &fileinfo)) != -1)//若查找成功，则进入
	{
		do
		{
			//如果是目录,迭代之（即文件夹内还有文件夹）  
			if ((fileinfo.attrib & _A_SUBDIR))
			{
				//文件名不等于"."&&文件名不等于".."
				//.表示当前目录
				//..表示当前目录的父目录
				//判断时，两者都要忽略，不然就无限递归跳不出去了！
				if (strcmp(fileinfo.name, ".") != 0 && strcmp(fileinfo.name, "..") != 0)
					getFiles(p.assign(path).append("\\").append(fileinfo.name), files);
			}
			//如果不是,加入列表  
			else
			{
				files.push_back(p.assign(path).append("\\").append(fileinfo.name));
			}
		} while (_findnext(hFile, &fileinfo) == 0);
		//_findclose函数结束查找
		_findclose(hFile);
	}
}




void readPointCloudTxt(const std::string& file, pcl::PointCloud<pcl::PointXYZ>& pc)
{
	std::ifstream read_file(file.c_str());//打开文件
	std::string line;//定义行
	pcl::PointXYZ point;
	while (std::getline(read_file, line)) //按行读取文件
	{
		//std::string::iterator it;
		//for (it = line.begin(); it < line.end(); it++)
		//{
		//	if (*it == ',')//判断是否有,
		//	{
		//		line.erase(it);//删除，
		//		line.insert(it, ' ');//插入空格
		//		it--;
		//	}
		//}
		std::stringstream ss(line);//点云赋值
		ss >> point.x;
		ss >> point.y;
		ss >> point.z;
		pc.points.push_back(point);
	}
	read_file.close();

}




cv::Mat generateHeightImage(const pcl::PointCloud<pcl::PointXYZ>::Ptr pc, const float x_resolution, const float y_resolution)
{
	//获取最小最大值
	pcl::PointXYZ min_pt;
	pcl::PointXYZ max_pt;
	pcl::getMinMax3D(*pc, min_pt, max_pt);
	//std::cout << "最小Z值: " << min_pt.z << std::endl;
	//std::cout << "最大Z值: " << max_pt.z << std::endl;

	int height = (max_pt.y - min_pt.y) / y_resolution + 1;
	int width = (max_pt.x - min_pt.x) / x_resolution + 1;
	cv::Mat height_image(height, width, CV_8UC1);
	
	for (int i = 0; i < pc->size(); i++)
	{
		int value = (pc->points[i].z - min_pt.z) / (max_pt.z - min_pt.z) * 255;
		//int row = i / width;//点位于的行数
		//int col = i % width;//点位于的列数
		int row = height - 1 - (pc->points[i].y - min_pt.y) / y_resolution;//点位于的行数
		int col = (pc->points[i].x - min_pt.x) / x_resolution;//点位于的列数
		height_image.at<uchar>(row, col) = value;
	}
	return height_image;
}




cv::Mat generateHeightImage2(const pcl::PointCloud<pcl::PointXYZ>::Ptr pc, const std::vector<int>& indices, const int height, const int width)
{
	//获取最小最大值
	Eigen::Vector4f min_pt;
	Eigen::Vector4f max_pt;
	pcl::getMinMax3D(*pc, indices, min_pt, max_pt);
	//std::cout << "最小Z值: " << min_pt.z << std::endl;
	//std::cout << "最大Z值: " << max_pt.z << std::endl;

	cv::Mat height_image(height, width, CV_8UC3);

	auto start = std::chrono::steady_clock::now();

	for (int i = 0; i < indices.size(); i++)
	{
		int row = indices[i] / width;//点位于的行数
		int col = indices[i] % width;//点位于的列数

		int value = -1;
		if (fabs(pc->points[indices[i]].z - INVALID_POINT_Z) < 0.001)
		{
			value = 0;
		}
		else
		{
			value = (pc->points[indices[i]].z - min_pt[2]) / (max_pt[2] - min_pt[2]) * 255;

		}
		//row = height - row - 1;//决定了点云和高度图中的物体方向是否一致
		height_image.at<cv::Vec3b>(row, col) = cv::Vec3b(value, value, value);
	}
	auto end = std::chrono::steady_clock::now();
	//std::cout << "点云转高度图时间:" << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << std::endl;

	return height_image;
}




std::vector<cv::Point2i> findCounters(const cv::Mat& image)
{
	cv::Mat filter_img;
	//中值滤波
	//cv::blur(image, img, cv::Size(3, 3));
	auto start = std::chrono::steady_clock::now();
	cv::medianBlur(image, filter_img, 9);
	auto end = std::chrono::steady_clock::now();
	//std::cout << "高度图滤波时间:" << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << std::endl;

	//二值化
	cv::Mat threshold_img;
	cv::threshold(filter_img, threshold_img, 50, 255, cv::THRESH_BINARY);
	//cv::imwrite("二值化图.jpg", threshold_img);


	//边缘提取
	cv::Mat canny_img;
	cv::Canny(threshold_img, canny_img, 50, 100, 3);
	//cv::imwrite("边缘图.jpg", canny_img);


	//找最外围的边缘点
	std::vector<cv::Point2i> contours;
	for (int row = 0; row < canny_img.rows; row++)
	{
		int left_col = -1;
		int right_col = -1;
		for (int col = 0; col < canny_img.cols; col++)
		{

			if (canny_img.at<uchar>(row, col) == 255)
			{
				//contours.push_back(cv::Point(col, row));
				if (left_col == -1)
					left_col = col;
				right_col = col;
			}
		}
		if (left_col > -1)
			contours.push_back(cv::Point(left_col, row));
		if (right_col > -1)
			contours.push_back(cv::Point(right_col, row));
	}
	for (int col = 0; col < canny_img.cols; col++)
	{
		int top_row = -1;
		int bottom_row = -1;
		for (int row = 0; row < canny_img.rows; row++)
		{

			if (canny_img.at<uchar>(row, col) == 255)
			{
				//contours.push_back(cv::Point(col, row));
				if (top_row == -1)
					top_row = row;
				bottom_row = row;
			}
		}
		if (top_row > -1)
			contours.push_back(cv::Point(col, top_row));
		if (bottom_row > -1)
			contours.push_back(cv::Point(col, bottom_row));
	}

	return contours;
}



std::vector<std::vector<cv::Point2i>> find4SideCounters(const cv::Mat& image)
{
	cv::Mat filter_img;
	//中值滤波
	//cv::blur(image, img, cv::Size(3, 3));
	auto start = std::chrono::steady_clock::now();
	cv::medianBlur(image, filter_img, 9);
	auto end = std::chrono::steady_clock::now();
	std::cout << "高度图滤波时间:" << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << std::endl;

	//二值化
	cv::Mat threshold_img;
	cv::threshold(image, threshold_img, 50, 255, cv::THRESH_BINARY);
	//cv::imwrite("二值化图.jpg", threshold_img);


	//边缘提取
	cv::Mat canny_img;
	cv::Canny(threshold_img, canny_img, 50, 100, 3);
	//cv::imwrite("边缘图.jpg", canny_img);


	//找最外围的边缘点
	std::vector<std::vector<cv::Point2i>> contours;
	std::vector<cv::Point2i> top_contours;
	std::vector<cv::Point2i> bottom_contours;
	std::vector<cv::Point2i> left_contours;
	std::vector<cv::Point2i> right_contours;
	//从左往右遍历
	for (int row = 0; row < canny_img.rows; row++)
	{
		int left_col = -1;
		int right_col = -1;
		for (int col = 0; col < canny_img.cols; col++)
		{

			if (canny_img.at<uchar>(row, col) == 255)
			{
				//contours.push_back(cv::Point(col, row));
				if (left_col == -1)
					left_col = col;
				right_col = col;
			}
		}
		if (left_col > -1)
			left_contours.push_back(cv::Point(left_col, row));
		if (right_col > -1)
			right_contours.push_back(cv::Point(right_col, row));
	}
	//从上往下遍历
	for (int col = 0; col < canny_img.cols; col++)
	{
		int top_row = -1;
		int bottom_row = -1;
		for (int row = 0; row < canny_img.rows; row++)
		{

			if (canny_img.at<uchar>(row, col) == 255)
			{
				//contours.push_back(cv::Point(col, row));
				if (top_row == -1)
					top_row = row;
				bottom_row = row;
			}
		}
		if (top_row > -1)
			top_contours.push_back(cv::Point(col, top_row));
		if (bottom_row > -1)
			bottom_contours.push_back(cv::Point(col, bottom_row));
	}

	contours.push_back(top_contours);
	contours.push_back(right_contours);
	contours.push_back(bottom_contours);
	contours.push_back(left_contours);

	return contours;
}



std::vector<cv::RotatedRect> generateROI(const std::vector<cv::Point2i>& contours)
{
	//最小外接矩形
	cv::RotatedRect rrect = cv::minAreaRect(contours);
	cv::Point2f points[4];
	rrect.points(points);//最小外接矩形的四个顶点
	cv::Point2f center = rrect.center;//最小外接矩形的中心点


	//最小外接矩形内缩一定量的像素
	cv::RotatedRect resize_rrect(center, cv::Size(rrect.size.width - 50, rrect.size.height - 50), rrect.angle);
	cv::Point2f resize_points[4];
	resize_rrect.points(resize_points);//最小外接矩形的四个顶点
	cv::Point2f resize_center = resize_rrect.center;//最小外接矩形的中心点

	//std::cout << "外接矩形的Size: " << rrect.size << ",  角度：" << rrect.angle << std::endl;
	//std::cout << "内缩矩形的Size: " << resize_rrect.size << ",  角度：" << resize_rrect.angle << std::endl;

	//计算18个区域的中心点坐标
	std::vector<cv::Point2f> roi_center;
	if (distancePoint2Point(resize_points[0], resize_points[1]) > distancePoint2Point(resize_points[0], resize_points[3]))
	{
		std::vector<cv::Point2f> left_pts = getEquidistantPoints(resize_points[0], resize_points[1], 7);
		std::vector<cv::Point2f> right_pts = getEquidistantPoints(resize_points[2], resize_points[3], 7);
		//std::vector<cv::Point2f> top_pts = getEquidistantPoints(cv::Point2f(resize_points[1].x, resize_points[1].y + 10), cv::Point2f(resize_points[2].x, resize_points[2].y + 10), 2);
		std::vector<cv::Point2f> top_pts = getEquidistantPoints(resize_points[1], resize_points[2], 2);
		std::vector<cv::Point2f> bottom_pts = getEquidistantPoints(resize_points[0], resize_points[3], 2);
		roi_center.insert(roi_center.end(), left_pts.begin(), left_pts.end());
		roi_center.insert(roi_center.end(), top_pts.begin(), top_pts.end());
		roi_center.insert(roi_center.end(), right_pts.begin(), right_pts.end());
		roi_center.insert(roi_center.end(), bottom_pts.begin(), bottom_pts.end());
	}
	else
	{
		std::vector<cv::Point2f> left_pts = getEquidistantPoints(resize_points[0], resize_points[1], 2);
		std::vector<cv::Point2f> right_pts = getEquidistantPoints(resize_points[2], resize_points[3], 2);
		std::vector<cv::Point2f> top_pts = getEquidistantPoints(resize_points[1], resize_points[2], 7);
		std::vector<cv::Point2f> bottom_pts = getEquidistantPoints(resize_points[0], resize_points[3], 7);
		roi_center.insert(roi_center.end(), left_pts.begin(), left_pts.end());
		roi_center.insert(roi_center.end(), top_pts.begin(), top_pts.end());
		roi_center.insert(roi_center.end(), right_pts.begin(), right_pts.end());
		roi_center.insert(roi_center.end(), bottom_pts.begin(), bottom_pts.end());
	}

	//构建18个ROI
	std::vector<cv::RotatedRect> roi_rrect;
	for (int i = 0; i < roi_center.size(); i++)
	{
		auto rrect = getRotatedRectROI(roi_center[i], 10, 10, resize_rrect.angle);
		roi_rrect.push_back(rrect);
	}


	return roi_rrect;
}




std::vector< cv::Point2f> getEquidistantPoints(const cv::Point2f& pt1, const cv::Point2f& pt2, const int pt_num)
{
	float dx = (pt2.x - pt1.x) / (pt_num + 1);
	float dy = (pt2.y - pt1.y) / (pt_num + 1);

	std::vector<cv::Point2f> pts;
	for (int i = 1; i <= pt_num; i++)
	{
		cv::Point2f pt;
		pt.x = pt1.x + i * dx;
		pt.y = pt1.y + i * dy;
		pts.push_back(pt);
	}
	return pts;
}




std::vector<cv::Point2f> computePointInROI(const cv::RotatedRect& rrect)
{
	int width = rrect.size.width;
	int height = rrect.size.height;
	float angle = rrect.angle;
	cv::Point2f center = rrect.center;
	cv::Point2f tl = rrect.boundingRect2f().tl();

	cv::Mat matrix = cv::getRotationMatrix2D(center, angle, 1.0);

	std::vector<cv::Point2f> roi_pt;
	for (int row = 0; row < height; row++)
	{
		for (int col = 0; col < width; col++)
		{
			cv::Point2f pt;
			pt.x = tl.x + col;
			pt.y = tl.y + row;

			

			cv::Point2f rpt = rotatePoint(pt, center, angle);
			roi_pt.push_back(rpt);
		}
	}
	return roi_pt;
}




cv::Point2f rotatePoint(const cv::Point2f& pt, const cv::Point2f& rcenter, const float& rangle)
{
	float x1 = pt.x;
	float y1 = pt.y;

	float x2 = rcenter.x;
	float y2 = rcenter.y;

	float x = (x1 - x2) * cos(M_PI / 180.0 * rangle) - (y1 - y2) * sin(M_PI / 180.0 * rangle) + x2;
	float y = (x1 - x2) * sin(M_PI / 180.0 * rangle) + (y1 - y2) * cos(M_PI / 180.0 * rangle) + y2;

	return cv::Point2f(x, y);
}




float distancePoint2Point(const cv::Point2f& pt1, const cv::Point2f& pt2)
{
	return sqrt((pt1.x - pt2.x) * (pt1.x - pt2.x) + (pt1.y - pt2.y) * (pt1.y - pt2.y));
}




cv::RotatedRect getRotatedRectROI(const cv::Point2f& center, const float& width, const float& height, const float& angle)
{
	cv::RotatedRect rrect(center, cv::Size(width, height), angle);
	return rrect;
}




void drawRotatedRect(cv::Mat& img, const cv::RotatedRect& rrect)
{
	cv::Point2f points[4];
	rrect.points(points);//最小外接矩形的四个顶点
	cv::Point2f center = rrect.center;//最小外接矩形的中心点

	for (int i = 0; i < 4; i++)
	{
		if (i == 3)
		{
			cv::line(img, points[i], points[0], cv::Scalar(0, 255, 0), 2, 8, 0);
			break;
		}
		cv::line(img, points[i], points[i + 1], cv::Scalar(0, 255, 0), 2, 8, 0);
	}
	cv::circle(img, center, 4, cv::Scalar(0, 255, 0), -1, 8, 0);
}




float computeFlatness(const pcl::PointCloud<pcl::PointXYZ>& pc, const float& distance_threshold)
{

	//平面拟合
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	//seg.setOptimizeCoefficients(true);//设置可以选定结果平面展示的点是分割掉的点还是分割剩下的点
	seg.setModelType(pcl::SACMODEL_PLANE);//设置目标几何形状
	seg.setMethodType(pcl::SAC_RMSAC);//分割方法 随机采样法
	seg.setDistanceThreshold(distance_threshold);//设置误差容忍范围
	seg.setInputCloud(pc.makeShared());
	//seg.setMaxIterations(100);
	seg.segment(*inliers, *coefficients);

	//点到平面的距离
	float normal_x = coefficients->values[0];
	float normal_y = coefficients->values[1];
	float normal_z = coefficients->values[2];
	float d = coefficients->values[3];


	//std::cout << "平面方程系数(Hessian Normal Form): " << normal_x << " , " << normal_y << " , " << normal_z << " , " << d << std::endl;
	std::vector<float> distance;
	for (int i = 0; i < pc.size(); i++)
	{
		float x = pc.points[i].x;
		float y = pc.points[i].y;
		float z = pc.points[i].z;

		float dis = normal_x * x + normal_y * y + normal_z * z + d;
		distance.push_back(dis);
	}
	
	//float max_dis = *(std::max_element(distance.begin(), distance.end()));
	//float min_dis = *(std::min_element(distance.begin(), distance.end()));

	std::sort(distance.rbegin(), distance.rend());

	distance.assign(distance.begin() + distance.size() * 0.05, distance.end() - distance.size() * 0.05);
	float max_dis = *distance.begin();
	float min_dis = *(distance.end() - 1);

	std::cout << "平面距离最大值: " << max_dis << " , 最小值: " << min_dis << std::endl;
	float flatness = max_dis - min_dis;

	return flatness;
}




std::vector<float> fit3DPlaneRansac(const pcl::PointCloud<pcl::PointXYZ>& pc, const int& evaluation_time, const float& threshold)
{
	if (pc.size() < 3)
	{
		return std::vector<float>(4, 0);
	}

	float a0, b0, c0, d0, pnum;
	a0 = b0 = c0 = d0 = pnum = 0;
	int numpt = pc.size();
	int maxcounts = 0;

	int trialcount = 0;
	float h = 0;

	while (true)
	{
		trialcount++;
		int seed = GetTickCount();
		float a, b, c, d;
		a = b = c = d = 0;

		//随机生成三个不同的点
		srand(seed++);
		int p1 = rand() % numpt;
		int p2 = rand() % numpt;

		while (true)
		{
			if (p1 != p2)
			{
				break;
			}
			p2 = rand() % numpt;
		}

		int p3 = rand() % numpt;

		while (true)
		{
			if (p1 != p3 && p2 != p3)
			{
				break;
			}
			p3 = rand() % numpt;
		}

		float x1 = pc[p1].x;
		float y1 = pc[p1].y;
		float z1 = pc[p1].z;
		float x2 = pc[p2].x;
		float y2 = pc[p2].y;
		float z2 = pc[p2].z;
		float x3 = pc[p3].x;
		float y3 = pc[p3].y;
		float z3 = pc[p3].z;

		a = (y2 - y1) * (z3 - z1) - (y3 - y1) * (z2 - z1);
		b = (z2 - z1) * (x3 - x1) - (z3 - z1) * (x2 - x1);
		c = (x2 - x1) * (y3 - y1) - (x3 - x1) * (y2 - y1);
		d = -(a * x1 + b * y1 + c * z1);

		int count = 0;
		for (int j = 0; j < numpt; j++)
		{
			float distan = abs(a * pc[j].x + b * pc[j].y + c * pc[j].z + d) / sqrt((double)(a * a + b * b + c * c));

			if (distan <= threshold)
			{
				count++;
			}
		}

		if (maxcounts < count)
		{
			maxcounts = count;
			a0 = a;
			b0 = b;
			c0 = c;
			d0 = d;
			pnum = maxcounts;
			h = (float)pc.size() / (float)maxcounts;
			h = h * h * h;
		}
		/*std::cout << "h * Evaluation_Time:   " << h * evaluation_time << std::endl;
		std::cout << "mesh points number :" << pc.size() << std::endl;
		std::cout << "pnum:   " << pnum << std::endl;*/

		if (h * evaluation_time < trialcount)
		{
			break;
		}
	}
	std::vector<float> equation;
	equation.push_back(a0);
	equation.push_back(b0);
	equation.push_back(c0);
	equation.push_back(d0);

	return equation;
}
