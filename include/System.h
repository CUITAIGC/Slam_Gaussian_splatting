/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/


#ifndef SYSTEM_H
#define SYSTEM_H


#include <unistd.h>
#include<stdio.h>
#include<stdlib.h>
#include<string>
#include<thread>
#include<opencv2/core/core.hpp>
#include <iostream>  
#include <dirent.h>  
#include <unistd.h>
#include <fstream>  
#include <cstdlib> // 为了使用std::system调用外部命令（可选）  
#include <cstdio>   // 为了使用FILE* 和相关函数 

#include "Tracking.h"
#include "FrameDrawer.h"
#include "MapDrawer.h"
#include "Atlas.h"
#include "LocalMapping.h"
#include "LoopClosing.h"
#include "KeyFrameDatabase.h"
#include "ORBVocabulary.h"
#include "Viewer.h"
#include "ImuTypes.h"
#include "Settings.h"

// for point cloud viewing
#include "pointcloudmapping.h"


static const string OUTPUT_TXT_PATH = "orbData/sparse/0/"; 
static const string OUTPUT_IMAGE_PATH = "orbData/images/"; 

namespace ORB_SLAM3
{

class Verbose
{
public:
    enum eLevel
    {
        VERBOSITY_QUIET=0,
        VERBOSITY_NORMAL=1,
        VERBOSITY_VERBOSE=2,
        VERBOSITY_VERY_VERBOSE=3,
        VERBOSITY_DEBUG=4
    };

    static eLevel th;

public:
    static void PrintMess(std::string str, eLevel lev)
    {
        if(lev <= th)
            cout << str << endl;
    }

    static void SetTh(eLevel _th)
    {
        th = _th;
    }
};

class Viewer;
class FrameDrawer;
class MapDrawer;
class Atlas;
class PointCloudMapping;
// class ImageColmap;
// class Points3DColmap;
class Tracking;
class LocalMapping;
class LoopClosing;
class Settings;

class System
{
public:
    // Input sensor
    enum eSensor{
        MONOCULAR=0,
        STEREO=1,
        RGBD=2,
        IMU_MONOCULAR=3,
        IMU_STEREO=4,
        IMU_RGBD=5,
    };

    // File type
    enum FileType{
        TEXT_FILE=0,
        BINARY_FILE=1,
    };

    string input_file_path;
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    // 构造函数. 启动 Local Mapg, Loop Closing and Viewer 线程.
    System(const string &strVocFile, const string &strSettingsFile, const eSensor sensor, const bool bUseViewer = false, const int initFr = 0, const string &strSequence = std::string());

    // 处理双目相机. 图像必须同步和矫正.
    // Input images: RGB (CV_8UC3) or grayscale (CV_8U). RGB 转成 灰度
    // Returns the camera pose (empty if tracking fails).
    Sophus::SE3f TrackStereo(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timestamp, const vector<IMU::Point>& vImuMeas = vector<IMU::Point>(), string filename="");

    // 处理RGBD, 深度图必须注册到RGB帧.
    // Input image: RGB (CV_8UC3) or grayscale (CV_8U). RGB 转成 灰度
    // Input depthmap: Float (CV_32F).
    // Returns the camera pose (empty if tracking fails).
    Sophus::SE3f TrackRGBD(const cv::Mat &im, const cv::Mat &depthmap, const double &timestamp, const vector<IMU::Point>& vImuMeas = vector<IMU::Point>(), string filename="");

    // 处理单目帧和可选的imu数据
    // Input images: RGB (CV_8UC3) or grayscale (CV_8U). RGB 转成 灰度
    // Returns the camera pose (empty if tracking fails).
    Sophus::SE3f TrackMonocular(const cv::Mat &im, const double &timestamp, const vector<IMU::Point>& vImuMeas = vector<IMU::Point>(), string filename="");


    // 停止局部建图线程，并仅执行相机跟踪.
    void ActivateLocalizationMode();
    // 恢复局部建图线程，并再次执行SLAM
    void DeactivateLocalizationMode();

    //自上次调用此函数以来发生了较大的地图改变（循环闭合，全局BA）则返回true
    bool MapChanged();

    // 重置地图 (clear Atlas or the active map)
    void Reset();
    void ResetActiveMap();

    // 请求完成所有线程.
    // 将等待所有线程都完成.
    // 在保存轨迹之前必须调用此函数
    void Shutdown();
    bool isShutDown();

    // 保存相机轨迹在TUM数据集.
    // 仅适用于双目和RGB_D，不适应单目.
    // 必须先调用shutdown
    // See format details at: http://vision.in.tum.de/data/datasets/rgbd-dataset
    void SaveTrajectoryTUM(const string &filename);

    // 存储关键帧的位姿在TUM数据集.
    // 使用于所有sener.
    // Call first Shutdown()
    // See format details at: http://vision.in.tum.de/data/datasets/rgbd-dataset
    void SaveKeyFrameTrajectoryTUM(const string &filename);

    void SaveTrajectoryEuRoC(const string &filename);
    void SaveKeyFrameTrajectoryEuRoC(const string &filename);

    void SaveTrajectoryEuRoC(const string &filename, Map* pMap);
    void SaveKeyFrameTrajectoryEuRoC(const string &filename, Map* pMap);

    // 保存用于初始化调试的数据
    void SaveDebugData(const int &iniIdx);

    // Save camera trajectory in the KITTI dataset format.
    // Only for stereo and RGB-D. This method does not work for monocular.
    // Call first Shutdown()
    // See format details at: http://www.cvlibs.net/datasets/kitti/eval_odometry.php
    void SaveTrajectoryKITTI(const string &filename);

    // TODO: Save/Load functions
    // SaveMap(const string &filename);
    // LoadMap(const string &filename);

    // Information from most recent processed frame
    // You can call this right after TrackMonocular (or stereo or RGBD)
    int GetTrackingState();
    std::vector<MapPoint*> GetTrackedMapPoints();
    std::vector<cv::KeyPoint> GetTrackedKeyPointsUn();

    // For debugging
    double GetTimeFromIMUInit();
    bool isLost();
    bool isFinished();

    void ChangeDataset();

    float GetImageScale(); 

#ifdef REGISTER_TIMES
    void InsertRectTime(double& time);
    void InsertResizeTime(double& time);
    void InsertTrackTime(double& time);
#endif

private:

    void SaveAtlas(int type);
    bool LoadAtlas(int type);
    
    // point cloud mapping
    shared_ptr<PointCloudMapping> mpPointCloudMapping;

    string CalculateCheckSum(string filename, int type);

    // 传感器类型 MONOCULAR，STEREO，RGBD
    eSensor mSensor;

    // ORB字典，保存ORB描述子聚类结果.
    ORBVocabulary* mpVocabulary;

    // 关键帧数据库，保存ORB描述子倒排索引
    // 用于位置识别（重新定位和循环检测）的KeyFrame数据库。
    KeyFrameDatabase* mpKeyFrameDatabase;
     
    //升级的地图关系
    // 存储指向所有关键帧和映射点的指针的映射结构。
    //Map* mpMap;
    Atlas* mpAtlas;

   //跟踪器。它接收一个帧并计算相关的相机位姿。
   //它还决定何时插入新的关键帧、创建一些新的MapPoints
   //如果跟踪失败，则执行重新定位。
    Tracking* mpTracker;

    // 局部建图器。它管理局部建图并执行局部BA调整。
    LocalMapping* mpLocalMapper;

    // 回环检测器. 负责检测每一个新加入的关键帧. 如果发现存在循环，则在新线程中进行位姿优化并进行全局BA调整.
    LoopClosing* mpLoopCloser;

    // 查看器，用于绘制地图和当前相机位姿.
    Viewer* mpViewer;
     //帧绘制器
    FrameDrawer* mpFrameDrawer;
    //地图绘制器
    MapDrawer* mpMapDrawer;

    // 系统线程：局部建图，回环检测，查看器.
    // 追踪线程在创建系统对象的主线程中执行
    std::thread* mptLocalMapping;
    std::thread* mptLoopClosing;
    std::thread* mptViewer;

    // Reset flag
    std::mutex mMutexReset;
    bool mbReset;
    bool mbResetActiveMap;

    // Change mode flags
    std::mutex mMutexMode;
    bool mbActivateLocalizationMode;
    bool mbDeactivateLocalizationMode;

    // Shutdown flag
    bool mbShutDown;

    // Tracking state
    int mTrackingState;
    std::vector<MapPoint*> mTrackedMapPoints;
    std::vector<cv::KeyPoint> mTrackedKeyPointsUn;
    std::mutex mMutexState;

    //
    string mStrLoadAtlasFromFile;
    string mStrSaveAtlasToFile;

    string mStrVocabularyFilePath;

    Settings* settings_;
};

}// namespace ORB_SLAM

#endif // SYSTEM_H
