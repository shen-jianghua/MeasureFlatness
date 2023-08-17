#pragma once
#include<pcl/point_cloud.h>
#include<pcl/point_types.h>
#include<pcl/common/common.h>
#include<pcl/common/centroid.h>
#include<pcl/sample_consensus/sac_model_plane.h>
#include<pcl/segmentation/sac_segmentation.h>
#include<pcl/filters/statistical_outlier_removal.h>

#include<opencv2/opencv.hpp>


//无效点的Z值
#define INVALID_POINT_Z 0


/// <summary>
/// 获取文件夹下所有文件
/// </summary>
/// <param name="path">					文件夹</param>
/// <param name="files">				文件名</param>
void getFiles(std::string path, std::vector<std::string>& files);


/// <summary>
/// 读取txt文件点云
/// </summary>
/// <param name="file">					文件名</param>
/// <param name="pc">					点云数据</param>
void readPointCloudTxt(const std::string& file, pcl::PointCloud<pcl::PointXYZ>& pc);


/// <summary>
/// 点云生成高度图
/// </summary>
/// <param name="pc">					点云</param>
/// <param name="x_resolution">			点云x方向分辨率</param>
/// <param name="y_resolution">			点云y方向分辨率</param>
/// <returns>							高度图</returns>
cv::Mat generateHeightImage(const pcl::PointCloud<pcl::PointXYZ>::Ptr pc, const float x_resolution, const float y_resolution);


/// <summary>
/// 点云生成高度图
/// </summary>
/// <param name="pc">					点云</param>
/// <param name="indices">				有效点的索引</param>
/// <param name="height">				y方向扫描行数</param>
/// <param name="width">				x方向的轮廓点数</param>
/// <returns>							高度图</returns>
cv::Mat generateHeightImage2(const pcl::PointCloud<pcl::PointXYZ>::Ptr pc, const std::vector<int>& indices, const int height, const int width);


/// <summary>
/// 检测角点
/// </summary>
/// <param name="image">				图像</param>
/// <returns>							角点</returns>
std::vector<cv::Point2i> findCounters(const cv::Mat& image);


/// <summary>
/// 分4条边检测角点
/// </summary>
/// <param name="image">				图像</param>
/// <returns>							4条边的角点</returns>
std::vector<std::vector<cv::Point2i>> find4SideCounters(const cv::Mat& image);


/// <summary>
/// 生成18个ROI区域
/// </summary>
/// <param name="contours">				角点</param>
/// <returns>							18个矩形ROI</returns>
std::vector<cv::RotatedRect> generateROI(const std::vector<cv::Point2i>& contours);


/// <summary>
/// 获取等间距的点
/// </summary>
/// <param name="pt1">					起始点</param>
/// <param name="pt2">					终止点</param>
/// <param name="pt_num">				点的个数</param>
/// <returns>							等间距点(不包含起始点和终止点)</returns>
std::vector< cv::Point2f> getEquidistantPoints(const cv::Point2f& pt1, const cv::Point2f& pt2, const int pt_num);


/// <summary>
/// 获取矩形ROI
/// </summary>
/// <param name="center">				中心点</param>
/// <param name="width">				宽度</param>
/// <param name="height">				长度</param>
/// <param name="angle">				旋转角度</param>
/// <returns>							矩形ROI</returns>
cv::RotatedRect getRotatedRectROI(const cv::Point2f& center, const float& width, const float& height, const float& angle);
 

/// <summary>
/// 获取矩形ROI区域内图像坐标
/// </summary>
/// <param name="rrect">				矩形框</param>
/// <returns>							矩形框内的图像坐标</returns>
std::vector<cv::Point2f> computePointInROI(const cv::RotatedRect& rrect);


/// <summary>
/// 旋转图像点
/// </summary>
/// <param name="pt">					图像点</param>
/// <param name="rcenter">				旋转中心</param>
/// <param name="rangle">				旋转角度</param>
/// <returns>							旋转后的点</returns>
cv::Point2f rotatePoint(const cv::Point2f& pt, const cv::Point2f& rcenter, const float& rangle);


/// <summary>
/// 点距离
/// </summary>
/// <param name="pt1">					图像点</param>
/// <param name="pt2">					图像点</param>
/// <returns>							欧式距离</returns>
float distancePoint2Point(const cv::Point2f& pt1, const cv::Point2f& pt2);


/// <summary>
/// 画旋转矩形
/// </summary>
/// <param name="img">					图像</param>
/// <param name="rrect">				旋转矩形</param>
void drawRotatedRect(cv::Mat& img, const cv::RotatedRect& rrect);


/// <summary>
/// 计算平面度
/// </summary>
/// <param name="pc">					点云</param>
/// <param name="distance_threshold">	距离阈值</param>
/// <returns>							平面度</returns>
float computeFlatness(const pcl::PointCloud<pcl::PointXYZ>& pc, const float& distance_threshold = 0.05);


/// <summary>
/// RANSAC拟合平面
/// </summary>
/// <param name="pc">					点云</param>
/// <param name="evaluation_time">		评估次数</param>
/// <param name="threshold">			距离阈值</param>
/// <returns>							平面方程系数</returns>
std::vector<float> fit3DPlaneRansac(const pcl::PointCloud<pcl::PointXYZ>& pc, const int& evaluation_time, const float& threshold);
