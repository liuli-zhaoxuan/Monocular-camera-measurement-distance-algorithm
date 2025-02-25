#include<iostream>
#include<string>
#include<opencv2/opencv.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
// Eigen 部分
#include <Eigen/Core>
// 稠密矩阵的代数运算（逆，特征值等）
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// 畸变校正、特征点检测、特征点匹配、位姿计算、三角测量
using namespace cv;
using namespace std;

Point2d pixel2cam(const Point2d& p, const Mat& K) {
    return Point2d
    (
        (p.x - K.at<double>(0, 2)) / K.at<double>(0, 0),
        (p.y - K.at<double>(1, 2)) / K.at<double>(1, 1)
    );
}

inline cv::Scalar get_color(float depth) {
    float up_th = 50, low_th = 10, th_range = up_th - low_th;
    if (depth > up_th) depth = up_th;
    if (depth < low_th) depth = low_th;
    return cv::Scalar(255 * depth / th_range, 0, 255 * (1 - depth / th_range));
}

int main() {
    string image1 = "D:\\Projects\\22.SLAM\\asserts\\1\\left.jpg";
    string image2 = "D:\\Projects\\22.SLAM\\asserts\\1\\right.jpg";
    Mat img_1 = imread(image1, cv::IMREAD_GRAYSCALE);
    Mat img_2 = imread(image2, cv::IMREAD_GRAYSCALE);
    assert(img_1.data && img_2.data && "Can not load images!");
    
    // 畸变校正
    double k1 = -0.373163034119991, k2 = 45.861527084349390, p1 = -0.002211977730186, p2 = -0.002049398767250;
    double fx = 1.222611907657529e+04, fy = 1.224148986736974e+04, cx = 7.895732090409291e+02, cy = 4.689106175920385e+02;

    Mat K = (Mat_<double>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);  // 内参矩阵
    Mat distCoeffs = (Mat_<double>(1, 5) << k1, k2, p1, p2, 0); // 畸变系数 (k1, k2, p1, p2, k3)

    Mat img_1_undistort;
    Mat img_2_undistort;
    undistort(img_1, img_1_undistort, K, distCoeffs);
    undistort(img_2, img_2_undistort, K, distCoeffs);

    cv::imshow("img_1_undistort", img_1_undistort);
    cv::imshow("img_2_undistort", img_2_undistort);

    // 创建 SIFT 对象
    cv::Ptr<cv::SIFT> sift = cv::SIFT::create();

    // 检测关键点
    std::vector<cv::KeyPoint> keypoints1, keypoints2;
    //sift->detect(img_1_undistort, keypoints1);
    //sift->detect(img_2_undistort, keypoints2);

    //// 计算描述符
    cv::Mat descriptors1, descriptors2;
    sift->detectAndCompute(img_1_undistort, noArray(), keypoints1, descriptors1);
    sift->detectAndCompute(img_2_undistort, noArray(), keypoints2, descriptors2);

    cout << "keypoints1的数量(原始图片)：" << keypoints1.size() << endl;
    cout << "keypoints2的数量(原始图片)：" << keypoints2.size() << endl;

    // 使用 FLANN 匹配器进行初步匹配
    cv::FlannBasedMatcher matcher;
    std::vector<std::vector<cv::DMatch>> knn_matches;
    matcher.knnMatch(descriptors1, descriptors2, knn_matches, 2);

    // 应用比率测试筛选匹配
    const float ratio_thresh = 0.7f;
    std::vector<cv::DMatch> good_matches;
    for (size_t i = 0; i < knn_matches.size(); ++i) {
        if (knn_matches[i][0].distance < ratio_thresh * knn_matches[i][1].distance) {
            good_matches.push_back(knn_matches[i][0]);
        }
    }
    cout << "FLANN(ratio = " << ratio_thresh << ")初步匹配结果数量：" << good_matches.size() << endl;
    
    // 提取匹配点对
    std::vector<cv::Point2f> points1, points2;
    for (const auto& match : good_matches) {
        points1.push_back(keypoints1[match.queryIdx].pt);
        points2.push_back(keypoints2[match.trainIdx].pt);
    }
    // 使用 RANSAC 计算单应性矩阵
    cv::Mat mask;
    cv::Mat H = cv::findHomography(points1, points2, cv::RANSAC, 3.0, mask);
    // 筛选内点
    std::vector<cv::DMatch> final_matches;
    for (size_t i = 0; i < good_matches.size(); ++i) {
        if (mask.at<uchar>(i)) {
            final_matches.push_back(good_matches[i]);
        }
    }
    cout << "RANSAC计算结果：" << final_matches.size() << endl;

    // 绘制匹配结果
    cv::Mat img_matches;
    cv::drawMatches(img_1_undistort, keypoints1, img_2_undistort, keypoints2, final_matches, img_matches);
    cv::imshow("Matches", img_matches);
    cv::waitKey(0);

    // 计算本质矩阵、旋转矩阵R、平移向量t
    Mat R, t;
    ////-- 计算基础矩阵
    //Mat fundamental_matrix;
    //fundamental_matrix = findFundamentalMat(points1, points2, FM_8POINT);
    ////fundamental_matrix = findFundamentalMat(points1, points2, FM_RANSAC);
    //cout << "fundamental_matrix is " << endl << fundamental_matrix << endl;

    //-- 计算本质矩阵
    Point2d principal_point(7.895732090409291e+02, 4.689106175920385e+02);
    double focal_length = (1.222611907657529e+04+ 1.224148986736974e+04)/2;      //相机焦距, MATLAB标定值

    Mat essential_matrix;
    essential_matrix = findEssentialMat(points1, points2, focal_length, principal_point);
    cout << "essential_matrix is " << endl << essential_matrix << endl;

    //-- 从本质矩阵中恢复旋转和平移信息.
    recoverPose(essential_matrix, points1, points2, R, t, focal_length, principal_point);
    cout << "R is " << endl << R << endl;
    cout << "t is " << endl << t << endl;

    // Eigen库求解欧拉角，作为参考（可选）
    //Eigen::Matrix3d R_eigen;
    //cv::cv2eigen(R, R_eigen);  // OpenCV 到 Eigen 的转换函数
    //// 计算欧拉角 (假设使用 ZYX 顺序)
    //Eigen::Vector3d euler_angles = R_eigen.eulerAngles(2, 1, 0);  // ZYX 顺序

    // 输出欧拉角
    //std::cout << "Euler Angles (radians): " << euler_angles.transpose() << std::endl;
    //std::cout << "Euler Angles (degrees): " << euler_angles.transpose() * 180.0 / M_PI << std::endl;

    // 三角测距
    Mat T1 = (Mat_<float>(3, 4) <<
        1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0);
    Mat T2 = (Mat_<float>(3, 4) <<
        R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2), t.at<double>(0, 0),
        R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2), t.at<double>(1, 0),
        R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2), t.at<double>(2, 0)
        );

    vector<Point2f> pts_1, pts_2;
    for (DMatch m : final_matches) {
        // 将像素坐标转换至相机坐标
        pts_1.push_back(pixel2cam(keypoints1[m.queryIdx].pt, K));
        pts_2.push_back(pixel2cam(keypoints2[m.trainIdx].pt, K));
    }

    Mat pts_4d;
    cv::triangulatePoints(T1, T2, pts_1, pts_2, pts_4d);

    // 转换成非齐次坐标
    vector<Point3d> points;
    for (int i = 0; i < pts_4d.cols; i++) {
        Mat x = pts_4d.col(i);
        x /= x.at<float>(3, 0); // 归一化
        Point3d p(
            x.at<float>(0, 0),
            x.at<float>(1, 0),
            x.at<float>(2, 0)
        );
        points.push_back(p);
    }
    
    //-- 验证三角化点与特征点的重投影关系
    Mat img1_plot = img_1.clone();
    Mat img2_plot = img_2.clone();
    for (int i = 0; i < final_matches.size(); i++) {
        // 第一个图
        float depth1 = points[i].z;
        cout << "depth: " << depth1 << endl;
        Point2d pt1_cam = pixel2cam(keypoints1[final_matches[i].queryIdx].pt, K);
        cv::circle(img1_plot, keypoints1[final_matches[i].queryIdx].pt, 2, get_color(depth1), 2);

        // 第二个图
        Mat pt2_trans = R * (Mat_<double>(3, 1) << points[i].x, points[i].y, points[i].z) + t;
        float depth2 = pt2_trans.at<double>(2, 0);
        cv::circle(img2_plot, keypoints2[final_matches[i].trainIdx].pt, 2, get_color(depth2), 2);
    }
    cv::imshow("img 1", img1_plot);
    cv::imshow("img 2", img2_plot);
    cv::waitKey(0);
	return 0;
}