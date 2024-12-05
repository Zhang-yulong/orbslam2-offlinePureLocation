/*
 * <one line to give the program's name and a brief idea of what it does.>
 * Copyright (C) 2016  <copyright holder> <email>
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * 
 */


/*
1.生成关键帧在相机坐标系下的点云，再将此点云变换到世界坐标系保存为全局点云地图。因为涉及关键帧，故此部分与Tracking相关。 //跟踪线程

2.全局BA后用更新的位姿调整全局点云地图。此部分与LoopClosing相关。 //闭环线程

3.显示全局点云地图 。创建一个pointcloudmapping类，在该类中实现点云地图显示。//显示线程

4.全局点云地图的关闭和保存。//主线程

原文链接：https://blog.csdn.net/luoyihao123456/article/details/124963453
*/

#include "pointcloudmapping.h"
#include <KeyFrame.h>
#include <opencv2/highgui/highgui.hpp>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include "Converter.h"
#include "PointCloude.h"
#include "System.h"

int currentloopcount = 0;
PointCloudMapping::PointCloudMapping(double resolution_,double meank_,double thresh_)
{
    this->resolution = resolution_;
    this->meank = thresh_;
    this->thresh = thresh_;
    statistical_filter.setMeanK(meank);
    statistical_filter.setStddevMulThresh(thresh);
    voxel.setLeafSize( resolution, resolution, resolution); //设置体素滤波中每个体素的大小
    globalMap = boost::make_shared< PointCloud >( );//创建一个智能指针globalMap，记录全局地图
    loopbusy=false;

    //PointCloudMapping初始化时就开启viewerThread线程
    viewerThread = make_shared<thread>( bind(&PointCloudMapping::viewer, this ) );  //（调试暂时关闭）

/*
bind是一个函数适配器，接受一个可调用对象，生成一个新的可调用对象来适应原对象的参数列表
auto newCallable = bind(callable, arg_list);
该形式表达的意思是：当调用newCallable时，会调用callable，并传给它arg_list中的参数
https://blog.csdn.net/qq_38410730/article/details/103637778
*/
}

void PointCloudMapping::shutdown()
{
    {
        unique_lock<mutex> lck(shutDownMutex); //有一个互斥对象，给这个对象加把锁
        shutDownFlag = true;

        /*//keyFrameUpdated是condition_variable 条件变量对象，
        当调用 wait 函数时，使用 std::unique_lock(mutex) 锁住当前线程。
        当调用notify_one时，线程被唤醒。*/
        keyFrameUpdated.notify_one();
    }

    //调用join()，等viewerThread线程执行完后才返回，继续执行后面的程序。
    viewerThread->join();
    

}

//在Tracking.cc中初始化Tracking::Tracking时创建了对象，在Tracking::CreateNewKeyFrame创建新关键帧时，传进来
void PointCloudMapping::insertKeyFrame(KeyFrame* kf, cv::Mat& color, cv::Mat& depth,int idk,vector<KeyFrame*> vpKFs)
{
    cerr<<"receive a keyframe, id = "<<idk<<" 第"<<kf->mnId<<"个关键帧"<<endl;
    //cout<<"vpKFs数量"<<vpKFs.size()<<endl;
    
    // unique_lock尝试加锁，如果没有锁定成功，会立即返回
    unique_lock<mutex> lck(keyframeMutex); 
    
    keyframes.push_back( kf );
    
    //用 currentvpKFs保存地图中现有的所有关键帧
    currentvpKFs = vpKFs; 
    
    //colorImgs.push_back( color.clone() );
    //depthImgs.push_back( depth.clone() );

    //创建一个 PointCloude实例，目的是获取本帧图像关联的点云id, 相对于世界坐标系的变换Twc
    PointCloude pointcloude;  //PointCloude.h的
    pointcloude.pcID = idk; //记录关键帧索引
    pointcloude.T = ORB_SLAM2::Converter::toSE3Quat( kf->GetPose() ); //返回Tcw，并将关键帧位姿Tcw转化为符合PCL格式
    pointcloude.pcE = generatePointCloud(kf,color,depth);   //联合深度图和RGB图像,找到对应点的空间位置,并且构造空间点云.
    pointcloud.push_back(pointcloude);
    keyFrameUpdated.notify_one();//点云更新通知

    

    //输出当前关键帧对应的地图点，看他们的z
    // int mapSize = kf->GetMapPointMatches().size();
    // std::vector<MapPoint*> vMapPoint = kf->GetMapPointMatches();
    // // printf("当前关键帧的地图点数：%d\n",mapSize );
    // for(int i=0;i<mapSize;i++)
    // {
    //     MapPoint* it  = vMapPoint[i];
    //     it->GetWorldPos();
    //     std::cout<<"第 "<<i<<"个地图点的坐标： "<<it->GetWorldPos()<<std::endl;
    
    // }



/*
    keyFrameUpdated是一个条件变量condition_variable

    当 std::condition_variable 对象的 wait 函数被调用的时候，
    它使用 std::unique_lock(通过 std::mutex) 来锁住当前线程，
    直到另外一个线程在相同的 std::condition_variable 对象上调用了 notification 函数来唤醒当前线程。
*/
}


/*
    m指图像的行，n是图像的列。它和空间点的坐标系关系是这样的：
    !!!!!!!!!!!!!!!按照“先列后行”的顺序，遍历了整张深度图!!!!!!!!!!
    ——————————————————————————————> x
    |
    |     —————————————————> n
    |     |    —————————————————————————
    |     |    |                       |
    |     |    |                       |
    |     |    |                       |
    |  m  |    |                       |
    |     v    |                       |
 y  |          |                       |
    |          |                       |
    v          |                       |
               —————————————————————————    
深度图则是单通道的图像，每个像素由16个bit组成（也就是C++里的unsigned short），
像素的值代表该点离传感器的距离。通常1000的值代表1米，所以我们把camera_factor设置成1000. 
这样，深度图里每个像素点的读数除以1000，就是它离你的真实距离了。
*/

//Ptr是一个指向 PointCloud类的指针，PointCloud里面装的数据类型是 PointT
pcl::PointCloud< PointCloudMapping::PointT >::Ptr PointCloudMapping::generatePointCloud(KeyFrame* kf, cv::Mat& color, cv::Mat& depth)//,Eigen::Isometry3d T
{
    PointCloud::Ptr tmp( new PointCloud() );
    // point cloud is null ptr
    for ( int m=0; m<depth.rows; m+=3 )
    {
        for ( int n=0; n<depth.cols; n+=3 )
        {
            float d = depth.ptr<float>(m)[n];
            if (d < 0.01 || d>5)
                continue;//排除掉深度值过大或过小的点
            PointT p;
            p.z = d;
            p.x = ( n - kf->cx) * p.z / kf->fx;
            p.y = ( m - kf->cy) * p.z / kf->fy; //相当于把像素坐标转化为相机坐标
            
            p.b = color.ptr<uchar>(m)[n*3];
            p.g = color.ptr<uchar>(m)[n*3+1];
            p.r = color.ptr<uchar>(m)[n*3+2];
                
            tmp->points.push_back(p);
        }
    }
    
    //Eigen::Isometry3d T = ORB_SLAM2::Converter::toSE3Quat( kf->GetPose() );
    //PointCloud::Ptr cloud(new PointCloud);
    //pcl::transformPointCloud( *tmp, *cloud, T.inverse().matrix());
    //cloud->is_dense = false;
    
    //cout<<"generate point cloud for kf "<<kf->mnId<<", size="<<cloud->points.size()<<endl;
    return tmp;
}


//自己想加点东西
// pcl::PointCloud< PointCloudMapping::PointT >::Ptr PointCloudMapping::generatePointCloud(KeyFrame* kf, cv::Mat& color){


// }





void PointCloudMapping::viewer()
{
    //调用pcl下命名空间 visualization下的 CloudViewer类，创建 viewer对象
    pcl::visualization::CloudViewer viewer("viewer");

   //点云视窗类CloudViewer是简单的可视化点云工具类，但不能用于多线程程序中。
   
    while(1)
    {
        
        {
            unique_lock<mutex> lck_shutdown( shutDownMutex );
            if (shutDownFlag)
            {
                break;
            }
        }
        {
            unique_lock<mutex> lck_keyframeUpdated( keyFrameUpdateMutex );
            keyFrameUpdated.wait( lck_keyframeUpdated ); //等待关键帧更新，更新完后释放锁。注意 keyFrameUpdated是一个条件变量

        }
        
        // keyframe is updated 
        size_t N=0;
        {
            unique_lock<mutex> lck( keyframeMutex );
            N = keyframes.size();
        }
        if(loopbusy || bStop) //跳过闭环忙和停止命令
        {
          //cout<<"loopbusy || bStop"<<endl;
            continue;
        }
        //cout<<lastKeyframeSize<<"    "<<N<<endl;

        //如果已经处理过的点云最后一帧的序号等于所有帧中最后一帧的序号，说明当前点云已经全部处理完了
        if(lastKeyframeSize == N) 
            cloudbusy = false;
        //cout<<"待处理点云个数 = "<<N<<endl;
          cloudbusy = true;
        for ( size_t i=lastKeyframeSize; i<N ; i++ )
        {

            
            PointCloud::Ptr p (new PointCloud);
            
            //将每一个关键帧中的点云通过刚体变换变换到世界坐标系下;
            pcl::transformPointCloud( *(pointcloud[i].pcE), *p, pointcloud[i].T.inverse().matrix());
            //cout<<"处理好第i个点云"<<i<<endl;
            *globalMap += *p;
            //PointCloud::Ptr tmp(new PointCloud());
            //voxel.setInputCloud( globalMap );
           // voxel.filter( *tmp );
            //globalMap->swap( *tmp );
           
 
        }
      
        // depth filter and statistical removal 
        PointCloud::Ptr tmp1 ( new PointCloud );
        
        //全局点云作为输入,通过统计滤波器
        statistical_filter.setInputCloud(globalMap);
        statistical_filter.filter( *tmp1 ); //tmp1用来保存滤波后的全局点云

        PointCloud::Ptr tmp(new PointCloud());
        voxel.setInputCloud( tmp1 );
        voxel.filter( *globalMap );
        //globalMap->swap( *tmp );
        viewer.showCloud( globalMap );
        cerr<<"show global map, size="<<N<<"   "<<globalMap->points.size()<<endl;
        lastKeyframeSize = N;
        cloudbusy = false;
        //*globalMap = *tmp1;
        
        //if()
        //{
	    
	//}
    }
}
void PointCloudMapping::save()
{
	pcl::io::savePCDFile( "result.pcd", *globalMap );
	cout<<"globalMap save finished"<<endl;
}

//更新点云信息
void PointCloudMapping::updatecloud()
{
	if(!cloudbusy) //如果点云不忙
	{
		loopbusy = true;  //将回环设置为忙
		cout<<"startloopmappoint"<<endl;

        //提取每一个关键帧,提取每一帧上面的点云,用BA后的位姿转换到世界坐标系下
        //新建临时点云tmp1，tmp1存储所有的变换后的点云
        PointCloud::Ptr tmp1(new PointCloud);
		for (int i=0;i<currentvpKFs.size();i++) //遍历所有关键帧
		{
		    for (int j=0;j<pointcloud.size();j++) //遍历每一个点云，pointcloud存储的是每个关键帧的点云、位姿、id信息
		    {   
				if(pointcloud[j].pcID==currentvpKFs[i]->mnFrameId) //找到具有相同id的关键帧对应的点云，用全局BA后的位姿来替换旧的位姿，计算新的点云 
				{   
					Eigen::Isometry3d T = ORB_SLAM2::Converter::toSE3Quat(currentvpKFs[i]->GetPose() );
					PointCloud::Ptr cloud(new PointCloud); //创建一个临时的cloud指针，用来存放变换后的世界坐标系
					
                    //将点云*pcE转换到世界坐标系下*cloud
                    pcl::transformPointCloud( *pointcloud[j].pcE, *cloud, T.inverse().matrix());
					*tmp1 +=*cloud;

					//cout<<"第pointcloud"<<j<<"与第vpKFs"<<i<<"匹配"<<endl;
					continue;
				}
			}
		}
        cout<<"finishloopmap"<<endl;
        PointCloud::Ptr tmp2(new PointCloud());
        
        //点云比较稠密,体素滤波来降采样
        voxel.setInputCloud( tmp1 );
        voxel.filter( *tmp2 );
        globalMap->swap( *tmp2 );//交换内容
        //viewer.showCloud( globalMap );
        
        //释放闭环
        loopbusy = false; 
        //cloudbusy = true;
        loopcount++;    //更新完之后,闭环次数加一

        //*globalMap = *tmp1;
	}
}
