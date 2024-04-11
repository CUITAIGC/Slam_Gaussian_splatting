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

#include "System.h"
#include "Converter.h"
#include <thread>
#include <pangolin/pangolin.h>
#include <iomanip>
#include <openssl/md5.h>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/string.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>
#include <boost/archive/xml_oarchive.hpp>



#include <map>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include "ImageColmap.h"
#include "Points3DColmap.h"
namespace ORB_SLAM3
{


    void makeDirectory(const std::string &directory)
    {
	 cout<<directory<<" make path"<<endl;
         mode_t mode = S_IRWXU | S_IROTH | S_IRWXG;
        if(chmod(directory.c_str(),mode)==-1){
	  cout<<"open permission fialed "<<strerror(errno)<<endl;
	  return;
	}		
        int status = mkdir(directory.c_str(), 0777); // 创建文件夹，并设置权
        if (status == -1)
        {
            // 创建失败
            perror("mkdir error");
            return;
        }
    }
    bool directoryExists(const std::string &directory)
    {
        return (access(directory.c_str(), F_OK) == 0);
    }
    std::string directoryFromPath(const std::string &path)
    {
        if (path.size() == 0)
            return path;
        if (path.back() == '\\' || path.back() == '/')
            return path;
        size_t c = path.size();
        while (c)
        {
            c--;
            if (path[c] == '\\' || path[c] == '/')
            {
                if (c == 0)
                    return "";
                else
                    return path.substr(0, c) + '/';
            }
        }
        return "";
    }
   
    void writeColmapDate(const std::string &filename, std::vector<ImageColmap> imageData,std::vector<Points3DColmap> pointsData)
    {
        if (imageData.size() > 0)
        {
            std::string images_path = filename + "images.txt";
            std::ofstream image_file(images_path, std::ios::out);
            if (!image_file.is_open())
                return;
            int total_tranks = 0;
            for (int i = 0; i < imageData.size(); i++)
            {
                total_tranks += imageData[i].readImagePoints2D().size();
            }
            image_file << "# Image list with two lines of data per image:\r#   IMAGE_ID, QW, QX, QY, QZ, TX, TY, TZ, CAMERA_ID, NAME\r#   POINTS2D[] as (X, Y, POINT3D_ID)\r# Number of images: " + std::to_string(imageData.size()) + ", mean observations per image: " + std::to_string(total_tranks/imageData.size()) + "\r";
            for (int i = 0; i < imageData.size(); i++)
            {
                image_file << imageData[i].readImageOneLine();
                std::vector<points_2D> data = imageData[i].readImagePoints2D();
                for (int j = 0; j < data.size(); j++)
                {
                    image_file << data[j].x_2D << " " << data[j].y_2D << " " << data[j].corresponding_point_3D << " ";
                }
                image_file << "\r";
            }
            image_file.close();
        }
        if (pointsData.size() > 0)
        {
            std::string points3D_path = filename + "points3D.txt";

            std::ofstream point_3d_file(points3D_path, std::ios::out);
            if (!point_3d_file.is_open())
                return;
            int total_tranks = 0;
            for (int i = 0; i < pointsData.size(); i++)
            {
                total_tranks += pointsData[i].getTranksCount();
            }
            point_3d_file << "# 3D point list with one line of data per point:\r#   POINT3D_ID, X, Y, Z, R, G, B, ERROR, TRACK[] as (IMAGE_ID, POINT2D_IDX)\r# Number of points: " + std::to_string(pointsData.size()) + ", mean track length: " + std::to_string(total_tranks/pointsData.size()) + "\r";
            for (int i = 0; i < pointsData.size(); i++)
            {
                point_3d_file << pointsData[i].readOneLine();
            }
            point_3d_file.close();
        }
    }
    void saveCameraPonit(Settings* settings2 )
    {
        cout << "Starting to write the save saveCameraPonit "<< endl;
        // 写出数据
        std::string filename = OUTPUT_TXT_PATH;
        std::string camera_path = filename + "cameras.txt";
        std::string folder = directoryFromPath(filename);
        if (!directoryExists(folder))
        {
            makeDirectory(folder);
        }
        std::ostringstream oss;
         for(size_t i = 0; i < settings2->originalCalib1_->size(); i++){
            oss << settings2->originalCalib1_->getParameter(i)<<" ";
        }
        // Camera list with one line of data per camera:
        // CAMERA_ID, MODEL, WIDTH, HEIGHT, PARAMS[]
        std::string cameraData[] = {"1", "PINHOLE", to_string(settings2->originalImSize_.width),to_string(settings2->originalImSize_.height), oss.str()};
        std::ofstream camera_file(camera_path, std::ios::out);
        if (!camera_file.is_open())return;
        camera_file << "# Camera list with one line of data per camera:\r#   CAMERA_ID, MODEL, WIDTH, HEIGHT, PARAMS[]\r# Number of cameras: 1\r";
        for (int i = 0; i < 5; i++)
        {
            camera_file << cameraData[i] << " ";
        }
        camera_file << "\r";
        camera_file.close();
        cout << "end to write the save saveCameraPonit " << endl;
    }

    bool DeleteDirectory(const std::string& path) {  
        DIR* dir;  
        struct dirent* ent;  
        std::string fullPath;  
        bool result = true;  
    
        dir = opendir(path.c_str());  
        if (dir != nullptr) {  
            while ((ent = readdir(dir)) != nullptr) {  
                if (strcmp(ent->d_name, ".") == 0 || strcmp(ent->d_name, "..") == 0) {  
                    continue;  
                }  
    
                fullPath = path + "/" + ent->d_name;  
                if (ent->d_type == DT_DIR) {  
                    // 递归删除子目录  
                    result = DeleteDirectory(fullPath);  
                } else {  
                    // 删除文件  
                    result = (unlink(fullPath.c_str()) == 0);  
                }  
    
                if (!result) {  
                    break;  
                }  
            }  
            closedir(dir);  
        }  
        return result;  
    }  

    bool copyFileToFolder(const char* sourceFile, const char* destinationFolder) {  
        // 构造目标文件的完整路径  
        //cout << sourceFile << "  path " << destinationFolder << endl;  
    
        // 打开源文件和目标文件  
        std::ifstream srcFile(sourceFile, std::ios::binary);  
        if (!srcFile) {  
            std::cerr << "无法打开源文件: " << sourceFile << std::endl;  
            return false;  
        }  
    
        std::ofstream dstFile(destinationFolder, std::ios::out | std::ios::binary);  
        if (!dstFile) {  
            std::cerr << "无法创建目标文件: " << destinationFolder <<" error "<<strerror(errno) <<std::endl;  
            srcFile.close();  
            return false;  
        }  
    
        // 使用C++的文件流复制文件内容  
        dstFile << srcFile.rdbuf();  
    
        // 关闭文件  
        srcFile.close();  
        dstFile.close();  
    
       // std::cout << "文件已复制到: " <<destinationFolder << std::endl;  
        return true;  
    }  
  

    void saveImageColmapDate(const vector<KeyFrame *> vpKFs,string input_file_path)
    {
        cout << "Starting to write the save saveImageColmapDate " << endl;
        std::vector<Points3DColmap> pointsData;
        //存储获得关键帧图片
        DeleteDirectory(OUTPUT_IMAGE_PATH);
        ///* Image list with two lines of data per image :
        //   IMAGE_ID, QW, QX, QY, QZ, TX, TY, TZ, CAMERA_ID, NAME
        //   POINTS2D[] as(X, Y, POINT3D_ID)
        // */
        std::vector<ImageColmap> imageData;
        for (size_t i = 0; i < vpKFs.size(); i++)
        {
            KeyFrame *pKF = vpKFs[i];
            ImageColmap image1;
            Sophus::SE3f postPose = pKF->GetPose();
            copyFileToFolder((input_file_path+"/"+pKF->mNameFile).c_str(),(OUTPUT_IMAGE_PATH+pKF->mNameFile).c_str());
            // std::cout << "w:" << q1.w << ", x:" << q1.x << ", y:" << q1.y << ", z:" << q1.z << "\n";
            float camera_pr[] = {postPose.unit_quaternion().w(), postPose.unit_quaternion().x(), postPose.unit_quaternion().y(), postPose.unit_quaternion().z(),
                                 postPose.translation()[0], postPose.translation()[1], postPose.translation()[2]};
            std::vector<points_2D> data1;
            std::vector<cv::KeyPoint> key_ponits = pKF->mvKeys;
            map<size_t, MapPoint *> allMapKey = pKF->GetMapHashPoints();
            for (int i = 0; i < key_ponits.size(); i++)
            {
                cv::KeyPoint element = key_ponits[i];
                points_2D po;
                po.corresponding_point_3D = -1;
                auto it = allMapKey.find(i);
                if (it != allMapKey.end() && it->second->mReprojectionError > ERROR_MAX)
                {
                    po.corresponding_point_3D = allMapKey[i]->mnId;
                }
                po.x_2D = element.pt.x;
                po.y_2D = element.pt.y;
                data1.push_back(po);
            }
            image1.writeImageDate(pKF->mNameFile, pKF->mnId, 1, camera_pr, data1);
            imageData.push_back(image1);
        }
        writeColmapDate(OUTPUT_TXT_PATH, imageData, pointsData);
        cout << "End to write the save saveImageColmapDate " << endl;
    }

    void savePoint3DColmapDate(const vector<MapPoint *> &vpMPs, const set<MapPoint *> spRefMPs)
    {
        cout << "Starting to write the save savePoint3DColmapDate " << endl;
        // 3D point list with one line of data per point :
        // POINT3D_ID, X, Y, Z, R, G, B, ERROR, TRACK[] as(IMAGE_ID, POINT2D_IDX)
        std::vector<Points3DColmap> pointsData;
        cout <<"point all  count  = "<< vpMPs.size()<< endl;
        for (size_t i = 0, iend = vpMPs.size(); i < iend; i++)
        {
            if (vpMPs[i]->isBad() || spRefMPs.count(vpMPs[i]))
                 continue;
            if(vpMPs[i]->mReprojectionError>ERROR_MAX)continue;
            Eigen::Matrix<float, 3, 1> pos = vpMPs[i]->GetWorldPos();
            glVertex3f(pos(0), pos(1), pos(2));
            Points3DColmap point_3d;
            std::vector<Tracks> tracks;
            std::map<KeyFrame *, std::tuple<int, int>> pobs = vpMPs[i]->GetObservations();
            int R = 0,G = 0,B = 0;
            int sizel = pobs.size();
            for (const auto &element : pobs)
            {
                KeyFrame * pKF = element.first;
                Tracks pos1;
                pos1.image_id = pKF->mnId;
                pos1.image_point2D_Index = get<0>(element.second);
                if(pos1.image_point2D_Index >= 0 && pos1.image_point2D_Index < pKF->mvKeys.size()){
                    float x =  pKF->mvKeys[pos1.image_point2D_Index].pt.x;
                    float y =  pKF->mvKeys[pos1.image_point2D_Index].pt.y;
                    if (x >= 0 && x < pKF->RGBM.cols && y >= 0 && y < pKF->RGBM.rows) {  
                        cv::Vec3b bgr_pixel =pKF->RGBM.at<cv::Vec3b>(y, x);
                        // 分别获取B、G、R值 
                        B += (int)bgr_pixel[0] / sizel;
                        G += (int)bgr_pixel[1] / sizel;  
                        R += (int)bgr_pixel[2] / sizel; 
                    } else {
                        // 索引无效，处理错误  
                        //std::cout<<" wx = "<<pKF->RGBM.cols<<" wy = "<<pKF->RGBM.rows<<std::endl;
                       // std::cout << "Invalid index! x = " << x<< " y = "<<y<<std::endl;  
                    }
                }
                tracks.push_back(pos1);
            }
            //cout<<"out R "<<R<<" G "<<G<<" B " <<B<<endl;
            //cout<<"out error " <<error<<endl;
            point_3d.writePointDate(vpMPs[i]->mnId, pos(0), pos(1), pos(2), R, G, B, vpMPs[i]->mReprojectionError/10000.0, tracks);
            pointsData.push_back(point_3d);
        }
        std::vector<ImageColmap> imageData;
        writeColmapDate(OUTPUT_TXT_PATH, imageData, pointsData);
        cout << "End to write the save savePoint3DColmapDate " << endl;
    }

    Verbose::eLevel Verbose::th = Verbose::VERBOSITY_NORMAL;

    System::System(const string &strVocFile, const string &strSettingsFile, const eSensor sensor,
                   const bool bUseViewer, const int initFr, const string &strSequence) : mSensor(sensor), mpViewer(static_cast<Viewer *>(NULL)), mbReset(false), mbResetActiveMap(false),
                                                                                         mbActivateLocalizationMode(false), mbDeactivateLocalizationMode(false), mbShutDown(false)
    {
        // Output welcome message
        cout << endl
             << "ORB-SLAM3 Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, Juan J. Gómez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza." << endl
             << "ORB-SLAM2 Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza." << endl
             << "This program comes with ABSOLUTELY NO WARRANTY;" << endl
             << "This is free software, and you are welcome to redistribute it" << endl
             << "under certain conditions. See LICENSE.txt." << endl
             << endl;

        cout << "Input sensor was set to: ";

        if (mSensor == MONOCULAR)
            cout << "Monocular" << endl;
        else if (mSensor == STEREO)
            cout << "Stereo" << endl;
        else if (mSensor == RGBD)
            cout << "RGB-D" << endl;
        else if (mSensor == IMU_MONOCULAR)
            cout << "Monocular-Inertial" << endl;
        else if (mSensor == IMU_STEREO)
            cout << "Stereo-Inertial" << endl;
        else if (mSensor == IMU_RGBD)
            cout << "RGB-D-Inertial" << endl;

        // 读取配置文件信息
        cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);
        if (!fsSettings.isOpened())
        {
            cerr << "Failed to open settings file at: " << strSettingsFile << endl;
            exit(-1);
        }
        // for point cloud resolution
        float resolution = 0.4;
        cv::FileNode node = fsSettings["File.version"];
        if (!node.empty() && node.isString() && node.string() == "1.0")
        {
            settings_ = new Settings(strSettingsFile, mSensor);

            mStrLoadAtlasFromFile = settings_->atlasLoadFile();
            mStrSaveAtlasToFile = settings_->atlasSaveFile();

            cout << (*settings_) << endl;
        }
        else
        {
            settings_ = nullptr;
            cv::FileNode node = fsSettings["System.LoadAtlasFromFile"];
            if (!node.empty() && node.isString())
            {
                mStrLoadAtlasFromFile = (string)node;
            }

            node = fsSettings["System.SaveAtlasToFile"];
            if (!node.empty() && node.isString())
            {
                mStrSaveAtlasToFile = (string)node;
            }
        }

        node = fsSettings["loopClosing"];
        bool activeLC = true;
        if (!node.empty())
        {
            activeLC = static_cast<int>(fsSettings["loopClosing"]) != 0;
        }

        mStrVocabularyFilePath = strVocFile;

        bool loadedAtlas = false;

        if (mStrLoadAtlasFromFile.empty())
        {
            // Load ORB Vocabulary
            cout << endl
                 << "Loading ORB Vocabulary. This could take a while..." << endl;
            //创建ORB词袋
            mpVocabulary = new ORBVocabulary();
            bool bVocLoad = mpVocabulary->loadFromTextFile(strVocFile);
            if (!bVocLoad)
            {
                cerr << "Wrong path to vocabulary. " << endl;
                cerr << "Falied to open at: " << strVocFile << endl;
                exit(-1);
            }
            cout << "Vocabulary loaded!" << endl
                 << endl;

            // 创建关键帧数据库
            mpKeyFrameDatabase = new KeyFrameDatabase(*mpVocabulary);

            // 创建地图关系
            cout << "Initialization of Atlas from scratch " << endl;
            mpAtlas = new Atlas(0);
        }
        else
        {
            // Load ORB Vocabulary
            cout << endl
                 << "Loading ORB Vocabulary. This could take a while..." << endl;

            mpVocabulary = new ORBVocabulary();
            bool bVocLoad = mpVocabulary->loadFromTextFile(strVocFile);
            if (!bVocLoad)
            {
                cerr << "Wrong path to vocabulary. " << endl;
                cerr << "Falied to open at: " << strVocFile << endl;
                exit(-1);
            }
            cout << "Vocabulary loaded!" << endl
                 << endl;

            // Create KeyFrame Database
            mpKeyFrameDatabase = new KeyFrameDatabase(*mpVocabulary);

            cout << "Load File" << endl;

            // Load the file with an earlier session
            // clock_t start = clock();
            cout << "Initialization of Atlas from file: " << mStrLoadAtlasFromFile << endl;
            bool isRead = LoadAtlas(FileType::BINARY_FILE);

            if (!isRead)
            {
                cout << "Error to load the file, please try with other session file or vocabulary file" << endl;
                exit(-1);
            }
            // mpKeyFrameDatabase = new KeyFrameDatabase(*mpVocabulary);

            // cout << "KF in DB: " << mpKeyFrameDatabase->mnNumKFs << "; words: " << mpKeyFrameDatabase->mnNumWords << endl;

            loadedAtlas = true;

            mpAtlas->CreateNewMap();

            // clock_t timeElapsed = clock() - start;
            // unsigned msElapsed = timeElapsed / (CLOCKS_PER_SEC / 1000);
            // cout << "Binary file read in " << msElapsed << " ms" << endl;

            // usleep(10*1000*1000);
        }

        if (mSensor == IMU_STEREO || mSensor == IMU_MONOCULAR || mSensor == IMU_RGBD)
            mpAtlas->SetInertialSensor();

        // Create Drawers. These are used by the Viewer
        mpFrameDrawer = new FrameDrawer(mpAtlas);
        mpMapDrawer = new MapDrawer(mpAtlas, strSettingsFile, settings_);
         // Initialize pointcloud mapping
        mpPointCloudMapping = make_shared<PointCloudMapping>( resolution );

        // Initialize the Tracking thread
        //(it will live in the main thread of execution, the one that called this constructor)
        cout << "Seq. Name: " << strSequence << endl;
        mpTracker = new Tracking(this, mpVocabulary, mpFrameDrawer, mpMapDrawer,
                                 mpAtlas, mpKeyFrameDatabase, strSettingsFile, mSensor, settings_,mpPointCloudMapping, strSequence);

        // Initialize the Local Mapping thread and launch
        mpLocalMapper = new LocalMapping(this, mpAtlas, mSensor == MONOCULAR || mSensor == IMU_MONOCULAR,
                                         mSensor == IMU_MONOCULAR || mSensor == IMU_STEREO || mSensor == IMU_RGBD, strSequence);
        mptLocalMapping = new thread(&ORB_SLAM3::LocalMapping::Run, mpLocalMapper);
        mpLocalMapper->mInitFr = initFr;
        if (settings_)
            mpLocalMapper->mThFarPoints = settings_->thFarPoints();
        else
            mpLocalMapper->mThFarPoints = fsSettings["thFarPoints"];
        if (mpLocalMapper->mThFarPoints != 0)
        {
            cout << "Discard points further than " << mpLocalMapper->mThFarPoints << " m from current camera" << endl;
            mpLocalMapper->mbFarPoints = true;
        }
        else
            mpLocalMapper->mbFarPoints = false;

        // Initialize the Loop Closing thread and launch
        //  mSensor!=MONOCULAR && mSensor!=IMU_MONOCULAR
        mpLoopCloser = new LoopClosing(mpAtlas, mpKeyFrameDatabase, mpVocabulary, mSensor != MONOCULAR, activeLC); // mSensor!=MONOCULAR);
        mptLoopClosing = new thread(&ORB_SLAM3::LoopClosing::Run, mpLoopCloser);

        // 设置线程间通信
        mpTracker->SetLocalMapper(mpLocalMapper);
        mpTracker->SetLoopClosing(mpLoopCloser);

        mpLocalMapper->SetTracker(mpTracker);
        mpLocalMapper->SetLoopCloser(mpLoopCloser);

        mpLoopCloser->SetTracker(mpTracker);
        mpLoopCloser->SetLocalMapper(mpLocalMapper);

        // usleep(10*1000*1000);

        // Initialize the Viewer thread and launch
        //if (bUseViewer)
        if(false) // TODO
        {
            mpViewer = new Viewer(this, mpFrameDrawer, mpMapDrawer, mpTracker, strSettingsFile, settings_);
            mptViewer = new thread(&Viewer::Run, mpViewer);
            mpTracker->SetViewer(mpViewer);
            mpLoopCloser->mpViewer = mpViewer;
            mpViewer->both = mpFrameDrawer->both;
        }

        // Fix verbosity
        Verbose::SetTh(Verbose::VERBOSITY_QUIET);
    }

    Sophus::SE3f System::TrackStereo(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timestamp, const vector<IMU::Point> &vImuMeas, string filename)
    {
        if (mSensor != STEREO && mSensor != IMU_STEREO)
        {
            cerr << "ERROR: you called TrackStereo but input sensor was not set to Stereo nor Stereo-Inertial." << endl;
            exit(-1);
        }

        cv::Mat imLeftToFeed, imRightToFeed;
        if (settings_ && settings_->needToRectify())
        {
            cv::Mat M1l = settings_->M1l();
            cv::Mat M2l = settings_->M2l();
            cv::Mat M1r = settings_->M1r();
            cv::Mat M2r = settings_->M2r();

            cv::remap(imLeft, imLeftToFeed, M1l, M2l, cv::INTER_LINEAR);
            cv::remap(imRight, imRightToFeed, M1r, M2r, cv::INTER_LINEAR);
        }
        else if (settings_ && settings_->needToResize())
        {
            cv::resize(imLeft, imLeftToFeed, settings_->newImSize());
            cv::resize(imRight, imRightToFeed, settings_->newImSize());
        }
        else
        {
            imLeftToFeed = imLeft.clone();
            imRightToFeed = imRight.clone();
        }

        // Check mode change
        {
            unique_lock<mutex> lock(mMutexMode);
            if (mbActivateLocalizationMode)
            {
                mpLocalMapper->RequestStop();

                // Wait until Local Mapping has effectively stopped
                while (!mpLocalMapper->isStopped())
                {
                    usleep(1000);
                }

                mpTracker->InformOnlyTracking(true);
                mbActivateLocalizationMode = false;
            }
            if (mbDeactivateLocalizationMode)
            {
                mpTracker->InformOnlyTracking(false);
                mpLocalMapper->Release();
                mbDeactivateLocalizationMode = false;
            }
        }

        // Check reset
        {
            unique_lock<mutex> lock(mMutexReset);
            if (mbReset)
            {
                mpTracker->Reset();
                mbReset = false;
                mbResetActiveMap = false;
            }
            else if (mbResetActiveMap)
            {
                mpTracker->ResetActiveMap();
                mbResetActiveMap = false;
            }
        }

        if (mSensor == System::IMU_STEREO)
            for (size_t i_imu = 0; i_imu < vImuMeas.size(); i_imu++)
                mpTracker->GrabImuData(vImuMeas[i_imu]);

        // std::cout << "start GrabImageStereo" << std::endl;
        Sophus::SE3f Tcw = mpTracker->GrabImageStereo(imLeftToFeed, imRightToFeed, timestamp, filename);

        // std::cout << "out grabber" << std::endl;

        unique_lock<mutex> lock2(mMutexState);
        mTrackingState = mpTracker->mState;
        mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
        mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;

        return Tcw;
    }

    Sophus::SE3f System::TrackRGBD(const cv::Mat &im, const cv::Mat &depthmap, const double &timestamp, const vector<IMU::Point> &vImuMeas, string filename)
    {
        if (mSensor != RGBD && mSensor != IMU_RGBD)
        {
            cerr << "ERROR: you called TrackRGBD but input sensor was not set to RGBD." << endl;
            exit(-1);
        }

        cv::Mat imToFeed = im.clone();
        cv::Mat imDepthToFeed = depthmap.clone();
        if (settings_ && settings_->needToResize())
        {
            cv::Mat resizedIm;
            cv::resize(im, resizedIm, settings_->newImSize());
            imToFeed = resizedIm;

            cv::resize(depthmap, imDepthToFeed, settings_->newImSize());
        }

        // Check mode change
        {
            unique_lock<mutex> lock(mMutexMode);
            if (mbActivateLocalizationMode)
            {
                mpLocalMapper->RequestStop();

                // Wait until Local Mapping has effectively stopped
                while (!mpLocalMapper->isStopped())
                {
                    usleep(1000);
                }

                mpTracker->InformOnlyTracking(true);
                mbActivateLocalizationMode = false;
            }
            if (mbDeactivateLocalizationMode)
            {
                mpTracker->InformOnlyTracking(false);
                mpLocalMapper->Release();
                mbDeactivateLocalizationMode = false;
            }
        }

        // Check reset
        {
            unique_lock<mutex> lock(mMutexReset);
            if (mbReset)
            {
                mpTracker->Reset();
                mbReset = false;
                mbResetActiveMap = false;
            }
            else if (mbResetActiveMap)
            {
                mpTracker->ResetActiveMap();
                mbResetActiveMap = false;
            }
        }

        if (mSensor == System::IMU_RGBD)
            for (size_t i_imu = 0; i_imu < vImuMeas.size(); i_imu++)
                mpTracker->GrabImuData(vImuMeas[i_imu]);

        Sophus::SE3f Tcw = mpTracker->GrabImageRGBD(imToFeed, imDepthToFeed, timestamp, filename);

        unique_lock<mutex> lock2(mMutexState);
        mTrackingState = mpTracker->mState;
        mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
        mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;
        return Tcw;
    }

    Sophus::SE3f System::TrackMonocular(const cv::Mat &im, const double &timestamp, const vector<IMU::Point> &vImuMeas, string filename)
    {

        {
            unique_lock<mutex> lock(mMutexReset);
            if (mbShutDown)
                return Sophus::SE3f();
        }

        if (mSensor != MONOCULAR && mSensor != IMU_MONOCULAR)
        {
            cerr << "ERROR: you called TrackMonocular but input sensor was not set to Monocular nor Monocular-Inertial." << endl;
            exit(-1);
        }

        cv::Mat imToFeed = im.clone();
        if (settings_ && settings_->needToResize())
        {
            cv::Mat resizedIm;
            cv::resize(im, resizedIm, settings_->newImSize());
            imToFeed = resizedIm;
        }

        // Check mode change
        {
            unique_lock<mutex> lock(mMutexMode);
            if (mbActivateLocalizationMode)
            {
                mpLocalMapper->RequestStop();

                // Wait until Local Mapping has effectively stopped
                while (!mpLocalMapper->isStopped())
                {
                    usleep(1000);
                }

                mpTracker->InformOnlyTracking(true);
                mbActivateLocalizationMode = false;
            }
            if (mbDeactivateLocalizationMode)
            {
                mpTracker->InformOnlyTracking(false);
                mpLocalMapper->Release();
                mbDeactivateLocalizationMode = false;
            }
        }

        // Check reset
        {
            unique_lock<mutex> lock(mMutexReset);
            if (mbReset)
            {
                mpTracker->Reset();
                mbReset = false;
                mbResetActiveMap = false;
            }
            else if (mbResetActiveMap)
            {
                cout << "SYSTEM-> Reseting active map in monocular case" << endl;
                mpTracker->ResetActiveMap();
                mbResetActiveMap = false;
            }
        }

        if (mSensor == System::IMU_MONOCULAR)
            for (size_t i_imu = 0; i_imu < vImuMeas.size(); i_imu++)
                mpTracker->GrabImuData(vImuMeas[i_imu]);

        Sophus::SE3f Tcw = mpTracker->GrabImageMonocular(imToFeed, timestamp, filename);

        unique_lock<mutex> lock2(mMutexState);
        mTrackingState = mpTracker->mState;
        mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
        mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;

        return Tcw;
    }

    void System::ActivateLocalizationMode()
    {
        unique_lock<mutex> lock(mMutexMode);
        mbActivateLocalizationMode = true;
    }

    void System::DeactivateLocalizationMode()
    {
        unique_lock<mutex> lock(mMutexMode);
        mbDeactivateLocalizationMode = true;
    }

    bool System::MapChanged()
    {
        static int n = 0;
        int curn = mpAtlas->GetLastBigChangeIdx();
        if (n < curn)
        {
            n = curn;
            return true;
        }
        else
            return false;
    }

    void System::Reset()
    {
        unique_lock<mutex> lock(mMutexReset);
        mbReset = true;
    }

    void System::ResetActiveMap()
    {
        unique_lock<mutex> lock(mMutexReset);
        mbResetActiveMap = true;
    }

    void System::Shutdown()
    {
        {
            unique_lock<mutex> lock(mMutexReset);
            mbShutDown = true;
        }
        Map *pActiveMap = mpAtlas->GetCurrentMap();
        if (!pActiveMap)
            return;
        const vector<MapPoint *> &vpMPs = pActiveMap->GetAllMapPoints();
        const vector<MapPoint *> &vpRefMPs = pActiveMap->GetReferenceMapPoints();
        cout <<"point all  count MapPointsInMap = "<< pActiveMap->MapPointsInMap()<< endl;
        const vector<KeyFrame *> vpKFs = pActiveMap->GetAllKeyFrames();
        set<MapPoint *> spRefMPs(vpRefMPs.begin(), vpRefMPs.end());
        saveCameraPonit(settings_);
        saveImageColmapDate(vpKFs,input_file_path);
        savePoint3DColmapDate(vpMPs,spRefMPs);


        cout << "Shutdown" << endl;
        mpLocalMapper->RequestFinish();
        mpLoopCloser->RequestFinish();
        mpPointCloudMapping->shutdown();
       

      
        /*if(mpViewer)
        {
            mpViewer->RequestFinish();
            while(!mpViewer->isFinished())
                usleep(5000);
        }*/

        // Wait until all thread have effectively stopped
        /*while(!mpLocalMapper->isFinished() || !mpLoopCloser->isFinished() || mpLoopCloser->isRunningGBA())
        {
            if(!mpLocalMapper->isFinished())
                cout << "mpLocalMapper is not finished" << endl;*/
        /*if(!mpLoopCloser->isFinished())
            cout << "mpLoopCloser is not finished" << endl;
        if(mpLoopCloser->isRunningGBA()){
            cout << "mpLoopCloser is running GBA" << endl;
            cout << "break anyway..." << endl;
            break;
        }*/
        /*usleep(5000);
    }*/

        if (!mStrSaveAtlasToFile.empty())
        {
            Verbose::PrintMess("Atlas saving to file " + mStrSaveAtlasToFile, Verbose::VERBOSITY_NORMAL);
            SaveAtlas(FileType::BINARY_FILE);
        }

        /*if(mpViewer)
            pangolin::BindToContext("ORB-SLAM2: Map Viewer");*/

#ifdef REGISTER_TIMES
        mpTracker->PrintTimeStats();
#endif
    }

    bool System::isShutDown()
    {
        unique_lock<mutex> lock(mMutexReset);
        return mbShutDown;
    }

    void System::SaveTrajectoryTUM(const string &filename)
    {
        cout << endl
             << "Saving camera trajectory to " << filename << " ..." << endl;
        if (mSensor == MONOCULAR)
        {
            cerr << "ERROR: SaveTrajectoryTUM cannot be used for monocular." << endl;
            return;
        }

        vector<KeyFrame *> vpKFs = mpAtlas->GetAllKeyFrames();
        sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

        // Transform all keyframes so that the first keyframe is at the origin.
        // After a loop closure the first keyframe might not be at the origin.
        Sophus::SE3f Two = vpKFs[0]->GetPoseInverse();

        ofstream f;
        f.open(filename.c_str());
        f << fixed;

        // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
        // We need to get first the keyframe pose and then concatenate the relative transformation.
        // Frames not localized (tracking failure) are not saved.

        // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
        // which is true when tracking failed (lbL).
        list<ORB_SLAM3::KeyFrame *>::iterator lRit = mpTracker->mlpReferences.begin();
        list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
        list<bool>::iterator lbL = mpTracker->mlbLost.begin();
        for (list<Sophus::SE3f>::iterator lit = mpTracker->mlRelativeFramePoses.begin(),
                                          lend = mpTracker->mlRelativeFramePoses.end();
             lit != lend; lit++, lRit++, lT++, lbL++)
        {
            if (*lbL)
                continue;

            KeyFrame *pKF = *lRit;

            Sophus::SE3f Trw;

            // If the reference keyframe was culled, traverse the spanning tree to get a suitable keyframe.
            while (pKF->isBad())
            {
                Trw = Trw * pKF->mTcp;
                pKF = pKF->GetParent();
            }

            Trw = Trw * pKF->GetPose() * Two;

            Sophus::SE3f Tcw = (*lit) * Trw;
            Sophus::SE3f Twc = Tcw.inverse();

            Eigen::Vector3f twc = Twc.translation();
            Eigen::Quaternionf q = Twc.unit_quaternion();

            f << setprecision(6) << *lT << " " << setprecision(9) << twc(0) << " " << twc(1) << " " << twc(2) << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;
        }
        f.close();
        // cout << endl << "trajectory saved!" << endl;
    }

    void System::SaveKeyFrameTrajectoryTUM(const string &filename)
    {
        cout << endl
             << "Saving keyframe trajectory to " << filename << " ..." << endl;

        vector<KeyFrame *> vpKFs = mpAtlas->GetAllKeyFrames();
        sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

        // Transform all keyframes so that the first keyframe is at the origin.
        // After a loop closure the first keyframe might not be at the origin.
        ofstream f;
        f.open(filename.c_str());
        f << fixed;

        for (size_t i = 0; i < vpKFs.size(); i++)
        {
            KeyFrame *pKF = vpKFs[i];

            // pKF->SetPose(pKF->GetPose()*Two);

            if (pKF->isBad())
                continue;

            Sophus::SE3f Twc = pKF->GetPoseInverse();
            Eigen::Quaternionf q = Twc.unit_quaternion();
            Eigen::Vector3f t = Twc.translation();
            f << setprecision(6) << pKF->mTimeStamp << setprecision(7) << " " << t(0) << " " << t(1) << " " << t(2)
              << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;
        }

        f.close();
    }

    void System::SaveTrajectoryEuRoC(const string &filename)
    {

        cout << endl
             << "Saving trajectory to " << filename << " ..." << endl;
        /*if(mSensor==MONOCULAR)
        {
            cerr << "ERROR: SaveTrajectoryEuRoC cannot be used for monocular." << endl;
            return;
        }*/

        vector<Map *> vpMaps = mpAtlas->GetAllMaps();
        int numMaxKFs = 0;
        Map *pBiggerMap;
        std::cout << "There are " << std::to_string(vpMaps.size()) << " maps in the atlas" << std::endl;
        for (Map *pMap : vpMaps)
        {
            std::cout << "  Map " << std::to_string(pMap->GetId()) << " has " << std::to_string(pMap->GetAllKeyFrames().size()) << " KFs" << std::endl;
            if (pMap->GetAllKeyFrames().size() > numMaxKFs)
            {
                numMaxKFs = pMap->GetAllKeyFrames().size();
                pBiggerMap = pMap;
            }
        }

        vector<KeyFrame *> vpKFs = pBiggerMap->GetAllKeyFrames();
        sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

        // Transform all keyframes so that the first keyframe is at the origin.
        // After a loop closure the first keyframe might not be at the origin.
        Sophus::SE3f Twb; // Can be word to cam0 or world to b depending on IMU or not.
        if (mSensor == IMU_MONOCULAR || mSensor == IMU_STEREO || mSensor == IMU_RGBD)
            Twb = vpKFs[0]->GetImuPose();
        else
            Twb = vpKFs[0]->GetPoseInverse();

        ofstream f;
        f.open(filename.c_str());
        // cout << "file open" << endl;
        f << fixed;

        // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
        // We need to get first the keyframe pose and then concatenate the relative transformation.
        // Frames not localized (tracking failure) are not saved.

        // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
        // which is true when tracking failed (lbL).
        list<ORB_SLAM3::KeyFrame *>::iterator lRit = mpTracker->mlpReferences.begin();
        list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
        list<bool>::iterator lbL = mpTracker->mlbLost.begin();

        // cout << "size mlpReferences: " << mpTracker->mlpReferences.size() << endl;
        // cout << "size mlRelativeFramePoses: " << mpTracker->mlRelativeFramePoses.size() << endl;
        // cout << "size mpTracker->mlFrameTimes: " << mpTracker->mlFrameTimes.size() << endl;
        // cout << "size mpTracker->mlbLost: " << mpTracker->mlbLost.size() << endl;

        for (auto lit = mpTracker->mlRelativeFramePoses.begin(),
                  lend = mpTracker->mlRelativeFramePoses.end();
             lit != lend; lit++, lRit++, lT++, lbL++)
        {
            // cout << "1" << endl;
            if (*lbL)
                continue;

            KeyFrame *pKF = *lRit;
            // cout << "KF: " << pKF->mnId << endl;

            Sophus::SE3f Trw;

            // If the reference keyframe was culled, traverse the spanning tree to get a suitable keyframe.
            if (!pKF)
                continue;

            // cout << "2.5" << endl;

            while (pKF->isBad())
            {
                // cout << " 2.bad" << endl;
                Trw = Trw * pKF->mTcp;
                pKF = pKF->GetParent();
                // cout << "--Parent KF: " << pKF->mnId << endl;
            }

            if (!pKF || pKF->GetMap() != pBiggerMap)
            {
                // cout << "--Parent KF is from another map" << endl;
                continue;
            }

            // cout << "3" << endl;

            Trw = Trw * pKF->GetPose() * Twb; // Tcp*Tpw*Twb0=Tcb0 where b0 is the new world reference

            // cout << "4" << endl;

            if (mSensor == IMU_MONOCULAR || mSensor == IMU_STEREO || mSensor == IMU_RGBD)
            {
                Sophus::SE3f Twb = (pKF->mImuCalib.mTbc * (*lit) * Trw).inverse();
                Eigen::Quaternionf q = Twb.unit_quaternion();
                Eigen::Vector3f twb = Twb.translation();
                f << setprecision(6) << 1e9 * (*lT) << " " << setprecision(9) << twb(0) << " " << twb(1) << " " << twb(2) << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;
            }
            else
            {
                Sophus::SE3f Twc = ((*lit) * Trw).inverse();
                Eigen::Quaternionf q = Twc.unit_quaternion();
                Eigen::Vector3f twc = Twc.translation();
                f << setprecision(6) << 1e9 * (*lT) << " " << setprecision(9) << twc(0) << " " << twc(1) << " " << twc(2) << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;
            }

            // cout << "5" << endl;
        }
        // cout << "end saving trajectory" << endl;
        f.close();
        cout << endl
             << "End of saving trajectory to " << filename << " ..." << endl;
    }

    void System::SaveTrajectoryEuRoC(const string &filename, Map *pMap)
    {

        cout << endl
             << "Saving trajectory of map " << pMap->GetId() << " to " << filename << " ..." << endl;
        /*if(mSensor==MONOCULAR)
        {
            cerr << "ERROR: SaveTrajectoryEuRoC cannot be used for monocular." << endl;
            return;
        }*/

        int numMaxKFs = 0;

        vector<KeyFrame *> vpKFs = pMap->GetAllKeyFrames();
        sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

        // Transform all keyframes so that the first keyframe is at the origin.
        // After a loop closure the first keyframe might not be at the origin.
        Sophus::SE3f Twb; // Can be word to cam0 or world to b dependingo on IMU or not.
        if (mSensor == IMU_MONOCULAR || mSensor == IMU_STEREO || mSensor == IMU_RGBD)
            Twb = vpKFs[0]->GetImuPose();
        else
            Twb = vpKFs[0]->GetPoseInverse();

        ofstream f;
        f.open(filename.c_str());
        // cout << "file open" << endl;
        f << fixed;

        // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
        // We need to get first the keyframe pose and then concatenate the relative transformation.
        // Frames not localized (tracking failure) are not saved.

        // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
        // which is true when tracking failed (lbL).
        list<ORB_SLAM3::KeyFrame *>::iterator lRit = mpTracker->mlpReferences.begin();
        list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
        list<bool>::iterator lbL = mpTracker->mlbLost.begin();

        // cout << "size mlpReferences: " << mpTracker->mlpReferences.size() << endl;
        // cout << "size mlRelativeFramePoses: " << mpTracker->mlRelativeFramePoses.size() << endl;
        // cout << "size mpTracker->mlFrameTimes: " << mpTracker->mlFrameTimes.size() << endl;
        // cout << "size mpTracker->mlbLost: " << mpTracker->mlbLost.size() << endl;

        for (auto lit = mpTracker->mlRelativeFramePoses.begin(),
                  lend = mpTracker->mlRelativeFramePoses.end();
             lit != lend; lit++, lRit++, lT++, lbL++)
        {
            // cout << "1" << endl;
            if (*lbL)
                continue;

            KeyFrame *pKF = *lRit;
            // cout << "KF: " << pKF->mnId << endl;

            Sophus::SE3f Trw;

            // If the reference keyframe was culled, traverse the spanning tree to get a suitable keyframe.
            if (!pKF)
                continue;

            // cout << "2.5" << endl;

            while (pKF->isBad())
            {
                // cout << " 2.bad" << endl;
                Trw = Trw * pKF->mTcp;
                pKF = pKF->GetParent();
                // cout << "--Parent KF: " << pKF->mnId << endl;
            }

            if (!pKF || pKF->GetMap() != pMap)
            {
                // cout << "--Parent KF is from another map" << endl;
                continue;
            }

            // cout << "3" << endl;

            Trw = Trw * pKF->GetPose() * Twb; // Tcp*Tpw*Twb0=Tcb0 where b0 is the new world reference

            // cout << "4" << endl;

            if (mSensor == IMU_MONOCULAR || mSensor == IMU_STEREO || mSensor == IMU_RGBD)
            {
                Sophus::SE3f Twb = (pKF->mImuCalib.mTbc * (*lit) * Trw).inverse();
                Eigen::Quaternionf q = Twb.unit_quaternion();
                Eigen::Vector3f twb = Twb.translation();
                f << setprecision(6) << 1e9 * (*lT) << " " << setprecision(9) << twb(0) << " " << twb(1) << " " << twb(2) << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;
            }
            else
            {
                Sophus::SE3f Twc = ((*lit) * Trw).inverse();
                Eigen::Quaternionf q = Twc.unit_quaternion();
                Eigen::Vector3f twc = Twc.translation();
                f << setprecision(6) << 1e9 * (*lT) << " " << setprecision(9) << twc(0) << " " << twc(1) << " " << twc(2) << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;
            }

            // cout << "5" << endl;
        }
        // cout << "end saving trajectory" << endl;
        f.close();
        cout << endl
             << "End of saving trajectory to " << filename << " ..." << endl;
    }

    /*void System::SaveTrajectoryEuRoC(const string &filename)
    {

        cout << endl << "Saving trajectory to " << filename << " ..." << endl;
        if(mSensor==MONOCULAR)
        {
            cerr << "ERROR: SaveTrajectoryEuRoC cannot be used for monocular." << endl;
            return;
        }

        vector<Map*> vpMaps = mpAtlas->GetAllMaps();
        Map* pBiggerMap;
        int numMaxKFs = 0;
        for(Map* pMap :vpMaps)
        {
            if(pMap->GetAllKeyFrames().size() > numMaxKFs)
            {
                numMaxKFs = pMap->GetAllKeyFrames().size();
                pBiggerMap = pMap;
            }
        }

        vector<KeyFrame*> vpKFs = pBiggerMap->GetAllKeyFrames();
        sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

        // Transform all keyframes so that the first keyframe is at the origin.
        // After a loop closure the first keyframe might not be at the origin.
        Sophus::SE3f Twb; // Can be word to cam0 or world to b dependingo on IMU or not.
        if (mSensor==IMU_MONOCULAR || mSensor==IMU_STEREO || mSensor==IMU_RGBD)
            Twb = vpKFs[0]->GetImuPose_();
        else
            Twb = vpKFs[0]->GetPoseInverse_();

        ofstream f;
        f.open(filename.c_str());
        // cout << "file open" << endl;
        f << fixed;

        // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
        // We need to get first the keyframe pose and then concatenate the relative transformation.
        // Frames not localized (tracking failure) are not saved.

        // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
        // which is true when tracking failed (lbL).
        list<ORB_SLAM3::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
        list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
        list<bool>::iterator lbL = mpTracker->mlbLost.begin();

        //cout << "size mlpReferences: " << mpTracker->mlpReferences.size() << endl;
        //cout << "size mlRelativeFramePoses: " << mpTracker->mlRelativeFramePoses.size() << endl;
        //cout << "size mpTracker->mlFrameTimes: " << mpTracker->mlFrameTimes.size() << endl;
        //cout << "size mpTracker->mlbLost: " << mpTracker->mlbLost.size() << endl;


        for(list<Sophus::SE3f>::iterator lit=mpTracker->mlRelativeFramePoses.begin(),
            lend=mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++, lbL++)
        {
            //cout << "1" << endl;
            if(*lbL)
                continue;


            KeyFrame* pKF = *lRit;
            //cout << "KF: " << pKF->mnId << endl;

            Sophus::SE3f Trw;

            // If the reference keyframe was culled, traverse the spanning tree to get a suitable keyframe.
            if (!pKF)
                continue;

            //cout << "2.5" << endl;

            while(pKF->isBad())
            {
                //cout << " 2.bad" << endl;
                Trw = Trw * pKF->mTcp;
                pKF = pKF->GetParent();
                //cout << "--Parent KF: " << pKF->mnId << endl;
            }

            if(!pKF || pKF->GetMap() != pBiggerMap)
            {
                //cout << "--Parent KF is from another map" << endl;
                continue;
            }

            //cout << "3" << endl;

            Trw = Trw * pKF->GetPose()*Twb; // Tcp*Tpw*Twb0=Tcb0 where b0 is the new world reference

            // cout << "4" << endl;


            if (mSensor == IMU_MONOCULAR || mSensor == IMU_STEREO || mSensor==IMU_RGBD)
            {
                Sophus::SE3f Tbw = pKF->mImuCalib.Tbc_ * (*lit) * Trw;
                Sophus::SE3f Twb = Tbw.inverse();

                Eigen::Vector3f twb = Twb.translation();
                Eigen::Quaternionf q = Twb.unit_quaternion();
                f << setprecision(6) << 1e9*(*lT) << " " <<  setprecision(9) << twb(0) << " " << twb(1) << " " << twb(2) << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;
            }
            else
            {
                Sophus::SE3f Tcw = (*lit) * Trw;
                Sophus::SE3f Twc = Tcw.inverse();

                Eigen::Vector3f twc = Twc.translation();
                Eigen::Quaternionf q = Twc.unit_quaternion();
                f << setprecision(6) << 1e9*(*lT) << " " <<  setprecision(9) << twc(0) << " " << twc(1) << " " << twc(2) << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;
            }

            // cout << "5" << endl;
        }
        //cout << "end saving trajectory" << endl;
        f.close();
        cout << endl << "End of saving trajectory to " << filename << " ..." << endl;
    }*/

    /*void System::SaveKeyFrameTrajectoryEuRoC_old(const string &filename)
    {
        cout << endl << "Saving keyframe trajectory to " << filename << " ..." << endl;

        vector<Map*> vpMaps = mpAtlas->GetAllMaps();
        Map* pBiggerMap;
        int numMaxKFs = 0;
        for(Map* pMap :vpMaps)
        {
            if(pMap->GetAllKeyFrames().size() > numMaxKFs)
            {
                numMaxKFs = pMap->GetAllKeyFrames().size();
                pBiggerMap = pMap;
            }
        }

        vector<KeyFrame*> vpKFs = pBiggerMap->GetAllKeyFrames();
        sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

        // Transform all keyframes so that the first keyframe is at the origin.
        // After a loop closure the first keyframe might not be at the origin.
        ofstream f;
        f.open(filename.c_str());
        f << fixed;

        for(size_t i=0; i<vpKFs.size(); i++)
        {
            KeyFrame* pKF = vpKFs[i];

           // pKF->SetPose(pKF->GetPose()*Two);

            if(pKF->isBad())
                continue;
            if (mSensor == IMU_MONOCULAR || mSensor == IMU_STEREO || mSensor==IMU_RGBD)
            {
                cv::Mat R = pKF->GetImuRotation().t();
                vector<float> q = Converter::toQuaternion(R);
                cv::Mat twb = pKF->GetImuPosition();
                f << setprecision(6) << 1e9*pKF->mTimeStamp  << " " <<  setprecision(9) << twb.at<float>(0) << " " << twb.at<float>(1) << " " << twb.at<float>(2) << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;

            }
            else
            {
                cv::Mat R = pKF->GetRotation();
                vector<float> q = Converter::toQuaternion(R);
                cv::Mat t = pKF->GetCameraCenter();
                f << setprecision(6) << 1e9*pKF->mTimeStamp << " " <<  setprecision(9) << t.at<float>(0) << " " << t.at<float>(1) << " " << t.at<float>(2) << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;
            }
        }
        f.close();
    }*/

    void System::SaveKeyFrameTrajectoryEuRoC(const string &filename)
    {
        cout << endl
             << "Saving keyframe trajectory to " << filename << " ..." << endl;

        vector<Map *> vpMaps = mpAtlas->GetAllMaps();
        Map *pBiggerMap;
        int numMaxKFs = 0;
        for (Map *pMap : vpMaps)
        {
            if (pMap && pMap->GetAllKeyFrames().size() > numMaxKFs)
            {
                numMaxKFs = pMap->GetAllKeyFrames().size();
                pBiggerMap = pMap;
            }
        }

        if (!pBiggerMap)
        {
            std::cout << "There is not a map!!" << std::endl;
            return;
        }

        vector<KeyFrame *> vpKFs = pBiggerMap->GetAllKeyFrames();
        sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

        // Transform all keyframes so that the first keyframe is at the origin.
        // After a loop closure the first keyframe might not be at the origin.
        ofstream f;
        f.open(filename.c_str());
        f << fixed;

        for (size_t i = 0; i < vpKFs.size(); i++)
        {
            KeyFrame *pKF = vpKFs[i];

            // pKF->SetPose(pKF->GetPose()*Two);

            if (!pKF || pKF->isBad())
                continue;
            if (mSensor == IMU_MONOCULAR || mSensor == IMU_STEREO || mSensor == IMU_RGBD)
            {
                Sophus::SE3f Twb = pKF->GetImuPose();
                Eigen::Quaternionf q = Twb.unit_quaternion();
                Eigen::Vector3f twb = Twb.translation();
                f << setprecision(6) << 1e9 * pKF->mTimeStamp << " " << setprecision(9) << twb(0) << " " << twb(1) << " " << twb(2) << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;
            }
            else
            {
                Sophus::SE3f Twc = pKF->GetPoseInverse();
                Eigen::Quaternionf q = Twc.unit_quaternion();
                Eigen::Vector3f t = Twc.translation();
                f << setprecision(6) << 1e9 * pKF->mTimeStamp << " " << setprecision(9) << t(0) << " " << t(1) << " " << t(2) << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;
            }
        }
        f.close();
    }

    void System::SaveKeyFrameTrajectoryEuRoC(const string &filename, Map *pMap)
    {
        cout << endl
             << "Saving keyframe trajectory of map " << pMap->GetId() << " to " << filename << " ..." << endl;

        vector<KeyFrame *> vpKFs = pMap->GetAllKeyFrames();
        sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

        // Transform all keyframes so that the first keyframe is at the origin.
        // After a loop closure the first keyframe might not be at the origin.
        ofstream f;
        f.open(filename.c_str());
        f << fixed;

        for (size_t i = 0; i < vpKFs.size(); i++)
        {
            KeyFrame *pKF = vpKFs[i];

            if (!pKF || pKF->isBad())
                continue;
            if (mSensor == IMU_MONOCULAR || mSensor == IMU_STEREO || mSensor == IMU_RGBD)
            {
                Sophus::SE3f Twb = pKF->GetImuPose();
                Eigen::Quaternionf q = Twb.unit_quaternion();
                Eigen::Vector3f twb = Twb.translation();
                f << setprecision(6) << 1e9 * pKF->mTimeStamp << " " << setprecision(9) << twb(0) << " " << twb(1) << " " << twb(2) << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;
            }
            else
            {
                Sophus::SE3f Twc = pKF->GetPoseInverse();
                Eigen::Quaternionf q = Twc.unit_quaternion();
                Eigen::Vector3f t = Twc.translation();
                f << setprecision(6) << 1e9 * pKF->mTimeStamp << " " << setprecision(9) << t(0) << " " << t(1) << " " << t(2) << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;
            }
        }
        f.close();
    }

    /*void System::SaveTrajectoryKITTI(const string &filename)
    {
        cout << endl << "Saving camera trajectory to " << filename << " ..." << endl;
        if(mSensor==MONOCULAR)
        {
            cerr << "ERROR: SaveTrajectoryKITTI cannot be used for monocular." << endl;
            return;
        }

        vector<KeyFrame*> vpKFs = mpAtlas->GetAllKeyFrames();
        sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

        // Transform all keyframes so that the first keyframe is at the origin.
        // After a loop closure the first keyframe might not be at the origin.
        cv::Mat Two = vpKFs[0]->GetPoseInverse();

        ofstream f;
        f.open(filename.c_str());
        f << fixed;

        // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
        // We need to get first the keyframe pose and then concatenate the relative transformation.
        // Frames not localized (tracking failure) are not saved.

        // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
        // which is true when tracking failed (lbL).
        list<ORB_SLAM3::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
        list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
        for(list<cv::Mat>::iterator lit=mpTracker->mlRelativeFramePoses.begin(), lend=mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++)
        {
            ORB_SLAM3::KeyFrame* pKF = *lRit;

            cv::Mat Trw = cv::Mat::eye(4,4,CV_32F);

            while(pKF->isBad())
            {
                Trw = Trw * Converter::toCvMat(pKF->mTcp.matrix());
                pKF = pKF->GetParent();
            }

            Trw = Trw * pKF->GetPoseCv() * Two;

            cv::Mat Tcw = (*lit)*Trw;
            cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
            cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);

            f << setprecision(9) << Rwc.at<float>(0,0) << " " << Rwc.at<float>(0,1)  << " " << Rwc.at<float>(0,2) << " "  << twc.at<float>(0) << " " <<
                 Rwc.at<float>(1,0) << " " << Rwc.at<float>(1,1)  << " " << Rwc.at<float>(1,2) << " "  << twc.at<float>(1) << " " <<
                 Rwc.at<float>(2,0) << " " << Rwc.at<float>(2,1)  << " " << Rwc.at<float>(2,2) << " "  << twc.at<float>(2) << endl;
        }
        f.close();
    }*/

    void System::SaveTrajectoryKITTI(const string &filename)
    {
        cout << endl
             << "Saving camera trajectory to " << filename << " ..." << endl;
        if (mSensor == MONOCULAR)
        {
            cerr << "ERROR: SaveTrajectoryKITTI cannot be used for monocular." << endl;
            return;
        }

        vector<KeyFrame *> vpKFs = mpAtlas->GetAllKeyFrames();
        sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

        // Transform all keyframes so that the first keyframe is at the origin.
        // After a loop closure the first keyframe might not be at the origin.
        Sophus::SE3f Tow = vpKFs[0]->GetPoseInverse();

        ofstream f;
        f.open(filename.c_str());
        f << fixed;

        // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
        // We need to get first the keyframe pose and then concatenate the relative transformation.
        // Frames not localized (tracking failure) are not saved.

        // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
        // which is true when tracking failed (lbL).
        list<ORB_SLAM3::KeyFrame *>::iterator lRit = mpTracker->mlpReferences.begin();
        list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
        for (list<Sophus::SE3f>::iterator lit = mpTracker->mlRelativeFramePoses.begin(),
                                          lend = mpTracker->mlRelativeFramePoses.end();
             lit != lend; lit++, lRit++, lT++)
        {
            ORB_SLAM3::KeyFrame *pKF = *lRit;

            Sophus::SE3f Trw;

            if (!pKF)
                continue;

            while (pKF->isBad())
            {
                Trw = Trw * pKF->mTcp;
                pKF = pKF->GetParent();
            }

            Trw = Trw * pKF->GetPose() * Tow;

            Sophus::SE3f Tcw = (*lit) * Trw;
            Sophus::SE3f Twc = Tcw.inverse();
            Eigen::Matrix3f Rwc = Twc.rotationMatrix();
            Eigen::Vector3f twc = Twc.translation();

            f << setprecision(9) << Rwc(0, 0) << " " << Rwc(0, 1) << " " << Rwc(0, 2) << " " << twc(0) << " " << Rwc(1, 0) << " " << Rwc(1, 1) << " " << Rwc(1, 2) << " " << twc(1) << " " << Rwc(2, 0) << " " << Rwc(2, 1) << " " << Rwc(2, 2) << " " << twc(2) << endl;
        }
        f.close();
    }

    void System::SaveDebugData(const int &initIdx)
    {
        // 0. Save initialization trajectory
        SaveTrajectoryEuRoC("init_FrameTrajectoy_" + to_string(mpLocalMapper->mInitSect) + "_" + to_string(initIdx) + ".txt");

        // 1. Save scale
        ofstream f;
        f.open("init_Scale_" + to_string(mpLocalMapper->mInitSect) + ".txt", ios_base::app);
        f << fixed;
        f << mpLocalMapper->mScale << endl;
        f.close();

        // 2. Save gravity direction
        f.open("init_GDir_" + to_string(mpLocalMapper->mInitSect) + ".txt", ios_base::app);
        f << fixed;
        f << mpLocalMapper->mRwg(0, 0) << "," << mpLocalMapper->mRwg(0, 1) << "," << mpLocalMapper->mRwg(0, 2) << endl;
        f << mpLocalMapper->mRwg(1, 0) << "," << mpLocalMapper->mRwg(1, 1) << "," << mpLocalMapper->mRwg(1, 2) << endl;
        f << mpLocalMapper->mRwg(2, 0) << "," << mpLocalMapper->mRwg(2, 1) << "," << mpLocalMapper->mRwg(2, 2) << endl;
        f.close();

        // 3. Save computational cost
        f.open("init_CompCost_" + to_string(mpLocalMapper->mInitSect) + ".txt", ios_base::app);
        f << fixed;
        f << mpLocalMapper->mCostTime << endl;
        f.close();

        // 4. Save biases
        f.open("init_Biases_" + to_string(mpLocalMapper->mInitSect) + ".txt", ios_base::app);
        f << fixed;
        f << mpLocalMapper->mbg(0) << "," << mpLocalMapper->mbg(1) << "," << mpLocalMapper->mbg(2) << endl;
        f << mpLocalMapper->mba(0) << "," << mpLocalMapper->mba(1) << "," << mpLocalMapper->mba(2) << endl;
        f.close();

        // 5. Save covariance matrix
        f.open("init_CovMatrix_" + to_string(mpLocalMapper->mInitSect) + "_" + to_string(initIdx) + ".txt", ios_base::app);
        f << fixed;
        for (int i = 0; i < mpLocalMapper->mcovInertial.rows(); i++)
        {
            for (int j = 0; j < mpLocalMapper->mcovInertial.cols(); j++)
            {
                if (j != 0)
                    f << ",";
                f << setprecision(15) << mpLocalMapper->mcovInertial(i, j);
            }
            f << endl;
        }
        f.close();

        // 6. Save initialization time
        f.open("init_Time_" + to_string(mpLocalMapper->mInitSect) + ".txt", ios_base::app);
        f << fixed;
        f << mpLocalMapper->mInitTime << endl;
        f.close();
    }

    int System::GetTrackingState()
    {
        unique_lock<mutex> lock(mMutexState);
        return mTrackingState;
    }

    vector<MapPoint *> System::GetTrackedMapPoints()
    {
        unique_lock<mutex> lock(mMutexState);
        return mTrackedMapPoints;
    }

    vector<cv::KeyPoint> System::GetTrackedKeyPointsUn()
    {
        unique_lock<mutex> lock(mMutexState);
        return mTrackedKeyPointsUn;
    }

    double System::GetTimeFromIMUInit()
    {
        double aux = mpLocalMapper->GetCurrKFTime() - mpLocalMapper->mFirstTs;
        if ((aux > 0.) && mpAtlas->isImuInitialized())
            return mpLocalMapper->GetCurrKFTime() - mpLocalMapper->mFirstTs;
        else
            return 0.f;
    }

    bool System::isLost()
    {
        if (!mpAtlas->isImuInitialized())
            return false;
        else
        {
            if ((mpTracker->mState == Tracking::LOST)) //||(mpTracker->mState==Tracking::RECENTLY_LOST))
                return true;
            else
                return false;
        }
    }

    bool System::isFinished()
    {
        return (GetTimeFromIMUInit() > 0.1);
    }

    void System::ChangeDataset()
    {
        if (mpAtlas->GetCurrentMap()->KeyFramesInMap() < 12)
        {
            mpTracker->ResetActiveMap();
        }
        else
        {
            mpTracker->CreateMapInAtlas();
        }

        mpTracker->NewDataset();
    }

    float System::GetImageScale()
    {
        return mpTracker->GetImageScale();
    }

#ifdef REGISTER_TIMES
    void System::InsertRectTime(double &time)
    {
        mpTracker->vdRectStereo_ms.push_back(time);
    }

    void System::InsertResizeTime(double &time)
    {
        mpTracker->vdResizeImage_ms.push_back(time);
    }

    void System::InsertTrackTime(double &time)
    {
        mpTracker->vdTrackTotal_ms.push_back(time);
    }
#endif

    void System::SaveAtlas(int type)
    {
        if (!mStrSaveAtlasToFile.empty())
        {
            // clock_t start = clock();

            // Save the current session
            mpAtlas->PreSave();

            string pathSaveFileName = "./";
            pathSaveFileName = pathSaveFileName.append(mStrSaveAtlasToFile);
            pathSaveFileName = pathSaveFileName.append(".osa");

            string strVocabularyChecksum = CalculateCheckSum(mStrVocabularyFilePath, TEXT_FILE);
            std::size_t found = mStrVocabularyFilePath.find_last_of("/\\");
            string strVocabularyName = mStrVocabularyFilePath.substr(found + 1);

            if (type == TEXT_FILE) // File text
            {
                cout << "Starting to write the save text file " << endl;
                std::remove(pathSaveFileName.c_str());
                std::ofstream ofs(pathSaveFileName, std::ios::binary);
                boost::archive::text_oarchive oa(ofs);

                oa << strVocabularyName;
                oa << strVocabularyChecksum;
                oa << mpAtlas;
                cout << "End to write the save text file" << endl;
            }
            else if (type == BINARY_FILE) // File binary
            {
                cout << "Starting to write the save binary file" << endl;
                std::remove(pathSaveFileName.c_str());
                std::ofstream ofs(pathSaveFileName, std::ios::binary);
                boost::archive::binary_oarchive oa(ofs);
                oa << strVocabularyName;
                oa << strVocabularyChecksum;
                oa << mpAtlas;
                cout << "End to write save binary file" << endl;
            }

        }
    }

    bool System::LoadAtlas(int type)
    {
        string strFileVoc, strVocChecksum;
        bool isRead = false;

        string pathLoadFileName = "./";
        pathLoadFileName = pathLoadFileName.append(mStrLoadAtlasFromFile);
        pathLoadFileName = pathLoadFileName.append(".osa");

        if (type == TEXT_FILE) // File text
        {
            cout << "Starting to read the save text file " << endl;
            std::ifstream ifs(pathLoadFileName, std::ios::binary);
            if (!ifs.good())
            {
                cout << "Load file not found" << endl;
                return false;
            }
            boost::archive::text_iarchive ia(ifs);
            ia >> strFileVoc;
            ia >> strVocChecksum;
            ia >> mpAtlas;
            cout << "End to load the save text file " << endl;
            isRead = true;
        }
        else if (type == BINARY_FILE) // File binary
        {
            cout << "Starting to read the save binary file" << endl;
            std::ifstream ifs(pathLoadFileName, std::ios::binary);
            if (!ifs.good())
            {
                cout << "Load file not found" << endl;
                return false;
            }
            boost::archive::binary_iarchive ia(ifs);
            ia >> strFileVoc;
            ia >> strVocChecksum;
            ia >> mpAtlas;
            cout << "End to load the save binary file" << endl;
            isRead = true;
        }

        if (isRead)
        {
            // Check if the vocabulary is the same
            string strInputVocabularyChecksum = CalculateCheckSum(mStrVocabularyFilePath, TEXT_FILE);

            if (strInputVocabularyChecksum.compare(strVocChecksum) != 0)
            {
                cout << "The vocabulary load isn't the same which the load session was created " << endl;
                cout << "-Vocabulary name: " << strFileVoc << endl;
                return false; // Both are differents
            }

            mpAtlas->SetKeyFrameDababase(mpKeyFrameDatabase);
            mpAtlas->SetORBVocabulary(mpVocabulary);
            mpAtlas->PostLoad();

            return true;
        }
        return false;
    }

    string System::CalculateCheckSum(string filename, int type)
    {
        string checksum = "";

        unsigned char c[MD5_DIGEST_LENGTH];

        std::ios_base::openmode flags = std::ios::in;
        if (type == BINARY_FILE) // Binary file
            flags = std::ios::in | std::ios::binary;

        ifstream f(filename.c_str(), flags);
        if (!f.is_open())
        {
            cout << "[E] Unable to open the in file " << filename << " for Md5 hash." << endl;
            return checksum;
        }

        MD5_CTX md5Context;
        char buffer[1024];

        MD5_Init(&md5Context);
        while (int count = f.readsome(buffer, sizeof(buffer)))
        {
            MD5_Update(&md5Context, buffer, count);
        }

        f.close();

        MD5_Final(c, &md5Context);

        for (int i = 0; i < MD5_DIGEST_LENGTH; i++)
        {
            char aux[10];
            sprintf(aux, "%02x", c[i]);
            checksum = checksum + aux;
        }

        return checksum;
    }

} // namespace ORB_SLAM
