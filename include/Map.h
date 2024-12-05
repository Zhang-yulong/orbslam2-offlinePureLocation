/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef MAP_H
#define MAP_H

#include "MapPoint.h"
#include "KeyFrame.h"
#include <set>

#include <mutex>

#include "Converter.h"

#include "SystemSetting.h"

#include "KeyFrameDatabase.h"

namespace ORB_SLAM2
{

class MapPoint;
class KeyFrame;

class SystemSetting;
class KeyFrameDatabase;

class Map
{
public:
    Map();

    //保存地图信息
    void Save(const string &filename);
    //加载地图信息
    void Load(const string &filename, SystemSetting* mySystemSetting );
    //https://blog.csdn.net/qq_40216084/article/details/115565435
    void Load(const string &filename, SystemSetting* mySystemSetting, KeyFrameDatabase *mpKeyFrameDatabase );

    //获取离线地图中的所有地图点的最大ID值是多少
    int FindOffLineMapPointMaxIdx(std::set<MapPoint*> spMapPoints);


    void AddKeyFrame(KeyFrame* pKF);
    void AddMapPoint(MapPoint* pMP);
    void EraseMapPoint(MapPoint* pMP);
    void EraseKeyFrame(KeyFrame* pKF);
    void SetReferenceMapPoints(const std::vector<MapPoint*> &vpMPs);
    void InformNewBigChange();
    int GetLastBigChangeIdx();

    std::vector<KeyFrame*> GetAllKeyFrames();
    std::vector<MapPoint*> GetAllMapPoints();
    std::vector<MapPoint*> GetReferenceMapPoints();

    long unsigned int MapPointsInMap();
    long unsigned  KeyFramesInMap();

    long unsigned int GetMaxKFid();

    void clear();

    vector<KeyFrame*> mvpKeyFrameOrigins;

    std::mutex mMutexMapUpdate;

    // This avoid that two points are created simultaneously in separate threads (id conflict)
    std::mutex mMutexPointCreation;

protected:
    std::set<MapPoint*> mspMapPoints;
    std::set<KeyFrame*> mspKeyFrames;

    std::vector<MapPoint*> mvpReferenceMapPoints;

    long unsigned int mnMaxKFid;

    // Index related to a big change in the map (loop closure, global BA)
    int mnBigChangeIdx;

    std::mutex mMutexMap;
    
    //保存地图点和关键帧
    void SaveMapPoint( ofstream &f, MapPoint* mp );
    void SaveKeyFrame( ofstream &f, KeyFrame* kf );
    //获取地图点ID
    void GetMapPointsIdx(); 
    //存储的是特征点对应的地图点的索引值 (没看原代码，暂时)
    std::map<MapPoint*, unsigned long int> mmpnMapPointsIdx;
    
    MapPoint* LoadMapPoint( ifstream &f );

    //正常的关键帧初始化涉及太多变量，读取bin文件的关键帧初始化不需要那么多，在里面会重新定义一个初始化关键帧的类
    KeyFrame* LoadKeyFrame( ifstream &f, SystemSetting* mySystemSetting );
    
    

};

} //namespace ORB_SLAM

#endif // MAP_H
