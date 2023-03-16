#pragma once
#include <iostream>
#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/Geometry>//提供Eigen各种旋转和平移的矩阵表示；角轴、四元素、旋转矩阵、欧拉角之间的相互转换
#include <Eigen/Dense>//稠密的矩阵代数运算

#include <ctime>
#include <cstdlib>
#include <chrono>

class ParamServer{

    public:
        ros::NodeHandle nh_p;

//iToF_up参数配置
        //滤波参数 
        double x1_up, x2_up, y1_up, y2_up, z1_up, z2_up;

        //参考向量
        //reference_wall
        std::vector<double> _v_reference_wall_up;    
        Eigen::Matrix<double,3,1> reference_wal_up;

        //reference_ground
        std::vector<double> _v_reference_ground_up;    
        Eigen::Matrix<double,3,1> reference_ground_up;
        
        //坐标系之间的相对变换矩阵
        std::vector<double> _v_rotation_matrix_base2iToF_up;
        Eigen::Matrix<double,3,3> rotation_matrix_base2iToF_up;

        std::string frame_id_up;                           //frame_id
        std::string iToF_topic_up;                        // iToF_Topic
        double t_x_up, t_y_up, t_z_up;            //TF 关系
        std::string save_dir_up;                            //SavePath——来自launch文件

//iToF_right配置
        //滤波参数
        double x1_right, x2_right, y1_right, y2_right, z1_right, z2_right;

        //参考向量
        //reference_wall
        std::vector<double> _v_reference_wall_right;    
        Eigen::Matrix<double,3,1> reference_wal_right;

        //reference_ground
        std::vector<double> _v_reference_ground_right;    
        Eigen::Matrix<double,3,1> reference_ground_right;

        //坐标系之间的相对变换矩阵
        std::vector<double> _v_rotation_matrix_base2iToF_right;
        Eigen::Matrix<double,3,3> rotation_matrix_base2iToF_right;

        std::string frame_id_right;         //Frame_id
        std::string iToF_topic_right;                  //iToF话题
        double t_x_right, t_y_right, t_z_right;        //TF 关系
        std::string save_dir_right;       //SavePath(来自launch)       

        
//iToF_left配置
        //滤波参数
        double x1_left, x2_left, y1_left, y2_left, z1_left, z2_left;

        //参考向量
        //reference_wall
        std::vector<double> _v_reference_wall_left;    
        Eigen::Matrix<double,3,1> reference_wal_left;

        //reference_ground
        std::vector<double> _v_reference_ground_left;    
        Eigen::Matrix<double,3,1> reference_ground_left;

        //坐标系之间的相对变换矩阵
        std::vector<double> _v_rotation_matrix_base2iToF_left;
        Eigen::Matrix<double,3,3> rotation_matrix_base2iToF_left;

        //iToF话题
        std::string iToF_topic_left;                  
        //Frame_id
        std::string frame_id_left;      
        //SavePath(来自launch)          
        std::string save_dir_left;     
        //TF关系
        double t_x_left, t_y_left, t_z_left;

    ParamServer(){
//iTOF_up
        nh_p.getParam("x1_up",x1_up);        nh_p.getParam("x2_up",x2_up);
        nh_p.getParam("y1_up",y1_up);        nh_p.getParam("y2_up",y2_up);
        nh_p.getParam("z1_up",z1_up);        nh_p.getParam("z2_up",z2_up);

        //wall参考
        nh_p.getParam("wall_reference_up",_v_reference_wall_up);
        reference_wal_up=Eigen::Map<Eigen::Matrix<double,3,1,Eigen::ColMajor>>(_v_reference_wall_up.data());
        //ground参考
        nh_p.getParam("ground_reference_up",_v_reference_ground_up);
        reference_ground_up=Eigen::Map<Eigen::Matrix<double,3,1,Eigen::ColMajor>>(_v_reference_ground_up.data());
        //Axis transform
        nh_p.getParam("rotation_matrix_base2iToF_up",_v_rotation_matrix_base2iToF_up);
        rotation_matrix_base2iToF_up=Eigen::Map<Eigen::Matrix<double,3,3,Eigen::ColMajor>>(_v_rotation_matrix_base2iToF_up.data());

        //frame_id
        nh_p.getParam("frame_id_up",frame_id_up); 
        // iToF_Topic
        nh_p.getParam("iToF_topic_up",iToF_topic_up); 
        //虚拟TF关系
        nh_p.getParam("t_x_up",t_x_up);  nh_p.getParam("t_y_up",t_y_up);nh_p.getParam("t_z_up",t_z_up);
        //参数保存路径
        nh_p.getParam("save_dir_up",save_dir_up);  //SavePath
    

//iToF_right
        nh_p.getParam("x1_right",x1_right);        nh_p.getParam("x2_right",x2_right);
        nh_p.getParam("y1_right",y1_right);        nh_p.getParam("y2_right",y2_right);
        nh_p.getParam("z1_right",z1_right);        nh_p.getParam("z2_right",z2_right);
        //wall参考
        nh_p.getParam("wall_reference_right",_v_reference_wall_right);
        reference_wal_right=Eigen::Map<Eigen::Matrix<double,3,1,Eigen::ColMajor>>(_v_reference_wall_right.data());
        //ground参考
        nh_p.getParam("ground_reference_right",_v_reference_ground_right);
        reference_ground_right=Eigen::Map<Eigen::Matrix<double,3,1,Eigen::ColMajor>>(_v_reference_ground_right.data());
        //Axis旋转矩阵
        nh_p.getParam("rotation_matrix_base2iToF_right",_v_rotation_matrix_base2iToF_right);
        rotation_matrix_base2iToF_right=Eigen::Map<Eigen::Matrix<double,3,3,Eigen::ColMajor>>(_v_rotation_matrix_base2iToF_right.data());

        nh_p.getParam("frame_id_right",frame_id_right); //frame_id
        nh_p.getParam("iToF_topic_right",iToF_topic_right); // iToF_Topic
        nh_p.getParam("t_x_right",t_x_right);       nh_p.getParam("t_y_right",t_y_right);       nh_p.getParam("t_z_right",t_z_right);//TF 关系
        nh_p.getParam("save_dir_right",save_dir_right);  //SavePath


//iToF_left
        nh_p.getParam("x1_left",x1_left);        nh_p.getParam("x2_left",x2_left);
        nh_p.getParam("y1_left",y1_left);        nh_p.getParam("y2_left",y2_left);
        nh_p.getParam("z1_left",z1_left);        nh_p.getParam("z2_left",z2_left);

        //wall参考
        nh_p.getParam("wall_reference_left",_v_reference_wall_left);
        reference_wal_left=Eigen::Map<Eigen::Matrix<double,3,1,Eigen::ColMajor>>(_v_reference_wall_left.data());
        //ground参考
        nh_p.getParam("ground_reference_left",_v_reference_ground_left);
        reference_ground_left=Eigen::Map<Eigen::Matrix<double,3,1,Eigen::ColMajor>>(_v_reference_ground_left.data());
        //Axis旋转矩阵
        nh_p.getParam("rotation_matrix_base2iToF_left",_v_rotation_matrix_base2iToF_left);
        rotation_matrix_base2iToF_left=Eigen::Map<Eigen::Matrix<double,3,3,Eigen::ColMajor>>(_v_rotation_matrix_base2iToF_left.data());

        nh_p.getParam("frame_id_left",frame_id_left); //frame_id
        nh_p.getParam("iToF_topic_left",iToF_topic_left); // iToF_Topic
        nh_p.getParam("t_x_left",t_x_left);     nh_p.getParam("t_y_left",t_y_left);     nh_p.getParam("t_z_left",t_z_left);//TF
        nh_p.getParam("save_dir_left",save_dir_left);  //SavePath
    }
};
