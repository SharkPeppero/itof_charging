#include <fstream>
#include <iostream>                                                
#include <math.h>
#include  <algorithm>
#include <vector>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>//标准头消息以及64位的点
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>//pcl_ros时间戳转换；pcl消息头与ros消息头转换；pcl::PointCloud2与sensor_msgs::PointCloud2转换；pcl::PCLPointField 与 sensor_msgs::PointField ；pcl::PointCloud 与 sensor_msgs::PointCloud2 间的转换
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/common.h>

#include <Eigen/Core>
#include <Eigen/Geometry>//提供Eigen各种旋转和平移的矩阵表示；角轴、四元素、旋转矩阵、欧拉角之间的相互转换
#include <Eigen/Dense>//稠密的矩阵代数运算

#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <time.h>
#include "utility.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2/LinearMath/Quaternion.h"
#include <boost/thread/thread.hpp>

class iToF_base:public ParamServer{
    private:
        ros::NodeHandle nh;
        double x1,x2,y1,y2,z1,z2;//墙面滤波范围

        Eigen::Matrix<double,3,1> Reference_wall;
        Eigen::Matrix<double,3,1> Reference_ground;
        Eigen::Matrix<double,3,3> Rotation_matrix_base2iToF;

        std::string Frame_id;
        std::string IToF_Topic;
        double t_x, t_y, t_z;

        std::string SavePath;

//订阅点云消息
        ros::Subscriber cloud_sub;

//平面约束，发布对象
        ros::Publisher pub_wall_plane;
        ros::Publisher pub_wall_normal;

//地面约束，发布对象
        ros::Publisher pub_ground_plane;
        ros::Publisher pub_ground_normal;

//原始点云发布
        ros::Publisher ori_pointcloud_pub;

        tf2_ros::StaticTransformBroadcaster broadcaster;

        std::vector<Eigen::Vector3d> eulerAngle_list;
        double err;
        double threshold=0.01;//欧拉角方差阈值

        bool is_ground_height;
        double height;

        bool is_ground_reference;

    public: 
    //TF坐标变换发布
        void TF_(const Eigen::Vector3d& eulerAngle_tmp){
            geometry_msgs::TransformStamped ts;

            ts.header.stamp=ros::Time::now();
            ts.header.frame_id="base_link";
            ts.child_frame_id=Frame_id;
            ts.transform.translation.x=t_x;
            ts.transform.translation.y=t_y;
            ts.transform.translation.z=t_z;
            
            tf2::Quaternion qtn;
            qtn.setRPY(eulerAngle_tmp[2],eulerAngle_tmp[1],eulerAngle_tmp[0]);//roll pitch yaw
            ts.transform.rotation.x = qtn.getX();
            ts.transform.rotation.y = qtn.getY();
            ts.transform.rotation.z = qtn.getZ();
            ts.transform.rotation.w = qtn.getW();
            broadcaster.sendTransform(ts);
         }
       
//点云剔除无效点
        void  Clerar_Nan (pcl::PointCloud<pcl::PointXYZ>::Ptr&  cloud_in){
            std::vector<int> indices_src; 
            pcl::removeNaNFromPointCloud(*cloud_in, *cloud_in, indices_src);

            return;
        }

//三轴直通滤波 
        void  Pass_fielt (pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in,
        double X1,double X2,
        double Y1,double Y2,
        double Z1,double Z2 ){
            pcl::PassThrough<pcl::PointXYZ> pass(true);
            pass.setInputCloud (cloud_in);
            pass.setFilterFieldName ("x");
            pass.setFilterLimits (X1,X2);
            pass.setNegative(false);
            pass.filter(*cloud_in);
            // ROS_INFO_STREAM("x滤波后点数:"<<cloud_in->size());

            pass.setInputCloud(cloud_in);
            pass.setFilterFieldName("y");
            pass.setFilterLimits(Y1, Y2);pass.setNegative(false);
            pass.filter(*cloud_in);
            // ROS_INFO_STREAM("xy滤波后点数:"<<cloud_in->size());

            pass.setInputCloud(cloud_in);
            pass.setFilterFieldName("z");
            pass.setFilterLimits(Z1, Z2);pass.setNegative(false);
            pass.filter(*cloud_in);
            // ROS_INFO_STREAM("xyz滤波后点数:" << cloud_in->size());

            return ;
        }

//体素下采样+统计滤波剔除李群点
        void down_Statistical_fielts(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in){
            pcl::VoxelGrid<pcl::PointXYZ> voxel;
            voxel.setInputCloud(cloud_in);
            voxel.setLeafSize(0.01, 0.01, 0.01);
            voxel.filter(*cloud_in);
            // ROS_INFO_STREAM("体素网格将采样点数:"<<cloud_in->size());

            //统计滤波剔除离群点
            pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
            sor.setInputCloud(cloud_in);
            sor.setMeanK(50);
            sor.setStddevMulThresh(1.0);
            sor.filter(*cloud_in);
            // ROS_INFO_STREAM("统计滤波消除离群点:"<<cloud_in->size());

            return;
        }

//彩色点云发（载入待发布的点云，发布的对象）
        void Color_Cloud_pub(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, const ros::Publisher& pub){
            sensor_msgs::PointCloud2 cloud_ros;
            toROSMsg(*cloud, cloud_ros);
            cloud_ros.header.stamp = ros::Time::now();
            cloud_ros.header.frame_id = Frame_id;
            pub.publish(cloud_ros);
            return;
        }

//普通点云发布（载入待发布的点云，发布的对象）
        void Cloud_pub(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const ros::Publisher& pub){
            sensor_msgs::PointCloud2 cloud_ros;
            toROSMsg(*cloud, cloud_ros);
            cloud_ros.header.stamp = ros::Time::now();
            cloud_ros.header.frame_id = Frame_id;
            pub.publish(cloud_ros);
            return;
        }

//法向量可视化(输入点云、输入法向量、输入彩色法向量的发布对象)
        void Normal_pub (const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in, 
        Eigen::Vector3d normal_vector, ros::Publisher pub){
            Eigen::Vector4f centroid;	
            pcl::compute3DCentroid(*cloud_in, centroid);	 
            // ROS_INFO_STREAM("质心:"<<centroid.transpose());

            pcl::PointCloud<pcl::PointXYZRGB>::Ptr normal_cloud (new pcl::PointCloud<pcl::PointXYZRGB>());
            for(int i=0; i<100; i++){
                pcl::PointXYZRGB p_tmp;
                p_tmp.x=centroid[0]+normal_vector[0]*i/100;
                p_tmp.y=centroid[1]+normal_vector[1]*i/100;
                p_tmp.z=centroid[2]+normal_vector[2]*i/100;
                p_tmp.r=0;
                p_tmp.g=0;
                p_tmp.b=255;
                normal_cloud->push_back(p_tmp);
            }
            Color_Cloud_pub(normal_cloud, pub);

            return;
        }

//法平面着色，发布
        void Normal_plane_pub(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in, ros::Publisher pub){
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
            for(const auto& p:cloud_in->points){
                pcl::PointXYZRGB pp;
                pp.x=p.x;
                pp.y=p.y;
                pp.z=p.z;
                pp.r=255;
                pp.g=0;
                pp.b=0;
                plane_cloud->points.push_back(pp);
            } 
            Color_Cloud_pub(plane_cloud, pub);
            return;
        }

//提取平面、发布平面、发布法向量
        Eigen::Vector4d extract_plane(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in, ros::Publisher pub_plane, ros::Publisher pub_normal){
            //RANSAC寻找平面
            pcl::SACSegmentation <pcl::PointXYZ> seg;
            pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
            pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
            seg.setOptimizeCoefficients (true); 
            seg.setModelType (pcl::SACMODEL_PLANE);
            seg.setMethodType (pcl::SAC_RANSAC);
            seg.setMaxIterations (300);
            seg.setDistanceThreshold (0.001);
            seg.setInputCloud (cloud_in);
            seg.segment (*inliers, *coefficients);

            //提取平面点云
            pcl::ExtractIndices<pcl::PointXYZ> extract;
            pcl::PointCloud<pcl::PointXYZ>::Ptr Plane_Cloud(new pcl::PointCloud<pcl::PointXYZ>());
            extract.setInputCloud(cloud_in);
            extract.setIndices(inliers);
            extract.setNegative(false);//提取内部点
            extract.filter(*Plane_Cloud);

            //发布法平面
            Normal_plane_pub(Plane_Cloud, pub_plane);

            //法向量定义
            double n1, n2, n3, n4;              //get the normal of the plane;Ax+By+Cz+D=0;
            n1 = coefficients->values[0];//A
            n2 = coefficients->values[1];//B
            n3 = coefficients->values[2];//C
            n4 = coefficients->values[3];//D

            //发布法向量+返回法向量
            Eigen::Vector3d normal_vector(n1,n2,n3);
            Normal_pub(Plane_Cloud, normal_vector, pub_normal);

            Eigen::Vector4d normal_vector_ (n1,n2,n3,n4);
            return normal_vector_;
        }

//输入角轴，输出旋转矩阵
        Eigen::Matrix3d transform( Eigen::Vector3d v, Eigen::Vector3d reference){
            //计算转轴
            v.normalize();
            Eigen::Vector3d n = reference.cross(v); 
            n.normalize();
            //计算转角
            double theta = acos(reference.dot(v));  
            // ROS_WARN_STREAM("角轴旋转角度 : "<<theta/M_PI*180<<"度.");
            //计算角轴->旋转矩阵
            Eigen::AngleAxisd Angle_axis(theta,-n);
            Eigen::Matrix3d Rotation_Matrix = Eigen::Matrix3d::Identity();
            Rotation_Matrix = Angle_axis.toRotationMatrix();

            return Rotation_Matrix;

            // ROS_INFO_STREAM("单位平面法向量 : "<<v.transpose());
            // ROS_INFO_STREAM("参考向量 : "<<reference.transpose());
            // ROS_INFO_STREAM("角轴旋转角度 : "<<theta/M_PI*180<<"度.");
            // std::cout<<"角轴到旋转矩阵:\n"<<rotation_matrix.matrix()<<std::endl;
        }

//构造函数，变量初始化、启动订阅
        iToF_base(const std::string& name){
            if(name=="up")  {
                //墙面滤波范围
                x1=x1_up;                x2=x2_up;
                y1=y1_up;                y2=y2_up;
                z1=z1_up;                z2=z2_up;
                //墙面参考向量||地面参考向量||Axis旋转变换矩阵
                Reference_wall = reference_wal_up;
                Reference_ground = reference_ground_up;
                Rotation_matrix_base2iToF = rotation_matrix_base2iToF_up; 
                //不变的参数
                Frame_id=frame_id_up;
                IToF_Topic=iToF_topic_up;
                t_x=t_x_up;                t_y=t_y_up;               t_z=t_z_up;
                SavePath=save_dir_up;
                is_ground_height=false; 
            }else if(name=="right"){
                //墙面滤波法范围
                x1=x1_right;                x2=x2_right;
                y1=y1_right;                y2=y2_right;
                z1=z1_right;                z2=z2_right;
                //墙面参考向量||地面参考向量||Axis旋转变换矩阵
                Reference_wall = reference_wal_right;
                Reference_ground = reference_ground_right;
                Rotation_matrix_base2iToF = rotation_matrix_base2iToF_right; 
                //不变的参数
                Frame_id=frame_id_right;
                IToF_Topic=iToF_topic_right;
                t_x=t_x_right;                t_y=t_y_right;                t_z=t_z_right;
                SavePath=save_dir_right;
                is_ground_height=true;
            }else if (name == "left"){
                //墙面滤波法范围
                x1=x1_left;                x2=x2_left;
                y1=y1_left;                y2=y2_left;
                z1=z1_left;                z2=z2_left;
                //墙面参考向量||地面参考向量||Axis旋转变换矩阵
                Reference_wall = reference_wal_left;
                Reference_ground = reference_ground_left;
                Rotation_matrix_base2iToF = rotation_matrix_base2iToF_left; 
                //不变的参数
                Frame_id=frame_id_left;
                IToF_Topic=iToF_topic_left;
                t_x=t_x_left;                t_y=t_y_left;                t_z=t_z_left;
                SavePath=save_dir_left;
                is_ground_height=true;
            }else{
                ROS_INFO_STREAM("invaile param!");
                nh.shutdown();
            }

            nh.getParam("is_ground_reference", is_ground_reference );
            
            // //初始化发送和订阅对象
            //平面约束发布对象
            pub_wall_plane=nh.advertise<sensor_msgs::PointCloud2>(Frame_id+"wall_plane",1);
            pub_wall_normal=nh.advertise<sensor_msgs::PointCloud2>(Frame_id+"wall_normal",1);

            //地面约束发布对象
            pub_ground_plane = nh.advertise<sensor_msgs::PointCloud2>(Frame_id+"ground_plane",1);
            pub_ground_normal = nh.advertise<sensor_msgs::PointCloud2>(Frame_id+"ground_normal",1);

            //原始点云
            ori_pointcloud_pub=nh.advertise<sensor_msgs::PointCloud2>(Frame_id+"original_cloud",1);

            cloud_sub=nh.subscribe<sensor_msgs::PointCloud2>(IToF_Topic, 20,&iToF_base::Main_Process,this,ros::TransportHints().tcpNoDelay());

        }

//回调函数
        void Main_Process(const sensor_msgs::PointCloud2::ConstPtr& cloud_in){
            //订阅初始点云
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ori(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::fromROSMsg(*cloud_in, *cloud_ori);
            // ROS_INFO_STREAM("订阅原始点云的数目:"<<cloud_ori->size());

            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ori_2(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::copyPointCloud(*cloud_ori, *cloud_ori_2);

//发布原始点云
            Cloud_pub(cloud_ori, ori_pointcloud_pub);

//----------------------------提取墙面点云--------------------------------------//
            //点云预处理
            Clerar_Nan(cloud_ori);
            //三轴直通滤波
            Pass_fielt(cloud_ori,x1,x2,y1,y2,z1,z2);
            //降采样+统计离群滤波
            down_Statistical_fielts(cloud_ori);
            //提取法平面，发布法平面+法向量
            Eigen::Vector4d normal_vector_4d = Eigen::Vector4d::Identity();
            normal_vector_4d = extract_plane(cloud_ori, pub_wall_plane, pub_wall_normal);
            Eigen::Vector3d normal_vector (normal_vector_4d[0],normal_vector_4d[1],normal_vector_4d[2]);
            //计算旋转矩阵1
            Eigen::Matrix3d rotation_matrix_wall = Eigen::Matrix3d::Identity();
            rotation_matrix_wall=transform(normal_vector, Reference_wall);

            //原始点云 旋转第一次
            Eigen::Affine3d T = Eigen::Affine3d::Identity();
            T.rotate(rotation_matrix_wall);
            Eigen::Vector3d t(0,0,0);            T.pretranslate(t);

            
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_transformed (new pcl::PointCloud<pcl::PointXYZ>);
            pcl::transformPointCloud(*cloud_ori_2, *cloud_transformed, T);


//---------------------------提取地面点云--------------------------------//
            Eigen::Vector3d eulerAngle_sum=Eigen::Vector3d::Identity();
            Eigen::Matrix3d Rotation_sum=Eigen::Matrix3d::Identity();

            //判断是否需要借助地面法向量约束
            if (Frame_id == "itof_left" || Frame_id == "itof_right")//对up 和left right分别处理
            {
                //点云预处理
                Clerar_Nan(cloud_transformed);
                //三轴直通滤波
                Pass_fielt(cloud_transformed, -1, 1, -1, 1, 0.2, 0.4);
                //降采样+统计离群滤波
                down_Statistical_fielts(cloud_transformed);
                //提取法平面，发布法平面+法向量
                Eigen::Vector4d normal_vector_ground_4d = Eigen::Vector4d::Identity();
                normal_vector_ground_4d = extract_plane(cloud_transformed, pub_ground_plane, pub_ground_normal);

                Eigen::Vector3d normal_vector_ground(normal_vector_ground_4d[0], normal_vector_ground_4d[1], normal_vector_ground_4d[2]);
                height = abs(normal_vector_ground_4d[3]);

                if (is_ground_reference)
                {
                    //计算旋转矩阵2
                    Eigen::Matrix3d rotation_matrix_ground = Eigen::Matrix3d::Identity();
                    Eigen::Vector3d Reference_ground(1, 0, 0);
                    rotation_matrix_ground = transform(normal_vector_ground, Reference_ground);
                    //发布经过两次约束后的点云
                    Eigen::Affine3d T_ = Eigen::Affine3d::Identity();
                    T_.rotate(rotation_matrix_ground);
                    Eigen::Vector3d t_(0, 0, 0);
                    T_.pretranslate(t_);

                    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_transformed_(new pcl::PointCloud<pcl::PointXYZ>);
                    pcl::transformPointCloud(*cloud_transformed, *cloud_transformed_, T_);

                    ROS_WARN_STREAM("墙面+地面参考！");

                    //发布TF关系
                    Rotation_sum = Rotation_matrix_base2iToF * rotation_matrix_ground * rotation_matrix_wall;
                    eulerAngle_sum = Rotation_sum.eulerAngles(2, 1, 0); // roll pitch yaw
                    TF_(eulerAngle_sum);
                }else{
                     ROS_WARN_STREAM("仅仅墙面参考！");
                    //发布TF关系
                    // Eigen::Matrix3d R_tmp = Eigen::Matrix3d::Zero();
                    // R_tmp<<0, -1,  0, 1, 0, 0, 0, 0, 1;
                    Rotation_sum = Rotation_matrix_base2iToF * rotation_matrix_wall;
                    eulerAngle_sum = Rotation_sum.eulerAngles(2, 1, 0); // roll pitch yaw
                    TF_(eulerAngle_sum);
                }
            } else
                {
                    ROS_WARN_STREAM("仅仅墙面参考！");
                    //发布TF关系
                    Rotation_sum = Rotation_matrix_base2iToF * rotation_matrix_wall;
                    eulerAngle_sum = Rotation_sum.eulerAngles(2, 1, 0); // roll pitch yaw
                    TF_(eulerAngle_sum);
                }

                //判断欧拉角是否满足要求
                eulerAngle_list.push_back(eulerAngle_sum);
                if (eulerAngle_list.size() < 15)
                {
                    return;
                }
                int num1 = rand() % 10;
                int num2 = num1 + 5;
                Eigen::Vector3d eulerAngle1 = eulerAngle_list[num1];
                Eigen::Vector3d eulerAngle2 = eulerAngle_list[num2];
                err = sqrt((eulerAngle1[0] - eulerAngle2[0]) * (eulerAngle1[0] - eulerAngle2[0]) + (eulerAngle1[1] - eulerAngle2[1]) * (eulerAngle1[1] - eulerAngle2[1]) + (eulerAngle1[2] - eulerAngle2[2]) * (eulerAngle1[2] - eulerAngle2[2]));
                Eigen::Vector3d eulerAngle = Eigen::Vector3d::Identity();
                eulerAngle = eulerAngle2;


                //写入yaml文件，写入欧拉角
                if (err < threshold)
                {   

                    ROS_WARN_STREAM("==标定成功，参数已写入到："<<SavePath<<"==");
                    ROS_WARN_STREAM("========================================================================");
                    ROS_WARN_STREAM("===============【！！请结合rviz再次确认墙面是否与地面垂直！！】===============");
                    ROS_WARN_STREAM("========================================================================");
                    ROS_INFO("\n");

                    cv::FileStorage fs_write(SavePath, cv::FileStorage::WRITE);
                    time_t rawtime;
                    time(&rawtime);
                    fs_write << "Time" << asctime(localtime(&rawtime));
                    cv::Mat iToF2base_link_roll = (cv::Mat_<double>(1, 1) << eulerAngle[2]);
                    cv::Mat iToF2base_link_pitch = (cv::Mat_<double>(1, 1) << eulerAngle[1]);
                    cv::Mat iToF2base_link_yaw = (cv::Mat_<double>(1, 1) << eulerAngle[0]);
                    fs_write << "iToF2base_link_yaw" << iToF2base_link_yaw
                             << "iToF2base_link_pitch" << iToF2base_link_pitch
                             << "iToF2base_link_roll" << iToF2base_link_roll;
                    if (is_ground_height == true)
                    {
                        fs_write << "the height of the sensor" << height;
                    }
                    fs_write.release();
                }

                return;
            }
};

int main(int argc, char  *argv[]){
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "iToF_base");
    iToF_base p1("up");
    iToF_base p2("right");
    iToF_base p3("left");
    ros::spin();
    return 0;
}
