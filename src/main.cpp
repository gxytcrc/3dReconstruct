#include "pointCloudMapping.hpp"
#include "vo.hpp"
#include <string.h>
using namespace cv;
using namespace std;
using namespace Eigen;


#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/surface/mls.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/poisson.h>

typedef pcl::PointXYZ PointT;

int main(int argc, const char * argv[])
{
    char file_name[1024];
    char fullpath[1024];
    sprintf(file_name, "%s/", argv[1]);
    sprintf(fullpath,"../Dataset/%s",file_name);
    //initialize-----------------------------------------------------------------------------------------------------------------
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    string dir = fullpath;
    //generate the camera value and scale
    vector<float> camera(6);
    float scale = 1;
    camera[0] = 540; //baseline
    camera[1] = 718.856; //fx
    camera[2] = 718.856; //fy
    camera[3] = 607.1928; //cx1
    camera[4] = 607.1928; // cx2
    camera[5] = 185.2157;//cy
    stereoDepth stereoDepth;
    pointCloudMapping pointCloudMapping;
    vo vo;

    //save the point cloud to ply file
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    //initialize the parameter of visual odometry
    Mat frame_pose = cv::Mat::eye(4, 4, CV_64F);
    Mat trajectory = cv::Mat::zeros(600, 1200, CV_8UC3);
    Mat previousImg0;
    Mat previousImg1;
    FeatureSet currentVOFeatures;
    bool keyframe = true;
    int keyframeID = 0;
    Mat Pose = frame_pose.col(3).clone();
    Mat LastKeyframePose = frame_pose.col(3).clone();
    Matrix3f R = Matrix3f::Identity();
    Matrix3f R_keyframe = Matrix3f::Identity();
    //pcl::visualization::CloudViewer viewer("viewer");
    //begain loop------------------------------------------------------------------------------------------------------------
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    for(int i =0; i<150 ; i++){
        char base_name[256];
        sprintf(base_name,"%06d.png",i);
        string left_img_file_name  = dir+"image_0/" + base_name;
        string right_img_file_name =dir+"image_1/" + base_name;

        Mat image0= imread(left_img_file_name,0);
        Mat image1= imread(right_img_file_name,0);
        Mat image_rgb = imread(right_img_file_name);
        cout<<" "<<endl;
        cout<<left_img_file_name<<endl;
        cout<<"Load the 2 correspondent images: "<<endl;
        // cout<<"the first image weight: "<<image0.cols<<endl;
        // cout<<"the first image height: "<<image0.rows<<endl;
        // cout<<"the second image weight: "<<image1.cols<<endl;
        // cout<<"the second image height: "<<image1.rows<<endl;
        cout<<"----------------------------------------"<<endl;
        //Tracking--------------------------------------------------------------------------------------------------------
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////
        if(i ==0){
            previousImg0 = image0;
            previousImg1 = image1;
        }
        else{
            vo.insertValue(previousImg0, previousImg1, image0, image1, frame_pose, trajectory, currentVOFeatures, i, camera, scale);
            vo.visualOdometry();
            vo.displayTrajectory();
            previousImg0 = image0;
            previousImg1 = image1;
            frame_pose = vo.frame_pose;
            currentVOFeatures = vo.currentVOFeatures;
            trajectory = vo.trajectory;
            //calculate the translation vector
            Pose = frame_pose.col(3).clone();
            //calculate the rotation matrix
            for(int row = 0; row<3; row++){
                for(int col = 0; col<3; col++){
                    R(row, col) = frame_pose.at<double>(row, col);
                }
            }
            //find the keyframe----------------------------------------------------------------------------------------------
            /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            Matrix3f R_mid = R*R_keyframe.inverse();
            vector<float> angle = vo.matrix2angle(R_mid);
            // cout<<"angleX: "<<angle[0]<<endl;
            // cout<<"angleY: "<<angle[1]<<endl;
            // cout<<"angleZ: "<<angle[2]<<endl;
            float sumAngle = abs(angle[0]) + abs(angle[0]) + abs(angle[0]);
            double distance = sqrt(pow(Pose.at<double>(0) - LastKeyframePose.at<double>(0),2) + pow(Pose.at<double>(2) - LastKeyframePose.at<double>(2),2));
            cout<<"distance: "<<distance<<endl;
            keyframeID = keyframeID + 1;
            if(distance>8 || keyframeID >20|| sumAngle > 20){
                keyframe = true;
                LastKeyframePose = Pose;
                R_keyframe = R;
                cout<<"----------------//////////////Found a new keyframe!//////////---------------------"<<endl;
            }
        }
        //Mapping--------------------------------------------------------------------------------------------------------
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////    }

        if(keyframe == true){
            //obtain the Dsiparity value
            stereoDepth.insertValue(image0, image1, camera, scale);
            stereoDepth.computeDisparity();
            stereoDepth.computeDepth();
            //calculate the point cloud
            pointCloudMapping.insertValue(cloud, stereoDepth.depth, image_rgb, frame_pose, camera, scale);
            pointCloudMapping.initialize3Dmap();
            //pointCloudMapping.pointCloudFilter();
            cloud = pointCloudMapping.outputPointCloud();
            keyframe = false;
            keyframeID = 0;
            //viewer.showCloud(cloud);
        }
        if(i ==0){
            waitKey(10000);
        }
        waitKey(1);
    }
    //pointCloudMapping.pointVisuallize();
    //pcl::io::savePLYFile ("../Result/pointcloud.ply", *cloud); 










    // Load input file
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_downSampled(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_smoothed(new pcl::PointCloud<pcl::PointXYZRGB>);

    // // 下采样
	// pcl::VoxelGrid<pcl::PointXYZRGB> downSampled;	//创建滤波对象
	// downSampled.setInputCloud (cloud);	//设置需要过滤的点云给滤波对象
	// downSampled.setLeafSize (0.01f,0.01f,0.01f); 	//设置滤波时创建的体素体积为1cm的立方体
	// downSampled.filter (*cloud_downSampled);	//执行滤波处理，存储输出

	// pcl::io::savePCDFile ("./downsampledPC.pcd", *cloud_downSampled);

    // // 统计滤波
	// pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> statisOutlierRemoval;       //创建滤波器对象
    // statisOutlierRemoval.setInputCloud (cloud_downSampled);            //设置待滤波的点云
    // statisOutlierRemoval.setMeanK (50);                                //设置在进行统计时考虑查询点临近点数
    // statisOutlierRemoval.setStddevMulThresh (3.0);                     //设置判断是否为离群点的阀值:均值+1.0*标准差
    // statisOutlierRemoval.filter (*cloud_filtered);                     //滤波结果存储到cloud_filtered

    // pcl::io::savePCDFile ("./filteredPC.pcd", *cloud_filtered);



    cout << "hehe 1 "<< endl;
    // 对点云重采样 
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr treeSampling (new pcl::search::KdTree<pcl::PointXYZRGB>); // 创建用于最近邻搜索的KD-Tree
    pcl::MovingLeastSquares<pcl::PointXYZRGB, pcl::PointXYZRGB> mls;  // 定义最小二乘实现的对象mls
    mls.setComputeNormals (false);  //设置在最小二乘计算中是否需要存储计算的法线
    mls.setInputCloud (cloud);        //设置待处理点云
    mls.setPolynomialOrder(2);             // 拟合2阶多项式拟合
    mls.setPolynomialFit (false);  // 设置为false可以 加速 smooth
    mls.setSearchMethod (treeSampling);    // 设置KD-Tree作为搜索方法
    mls.setSearchRadius (100.0); // 单位m.设置用于拟合的K近邻半径
    mls.process (*cloud_smoothed);        //输出
    cout << "hehe 2 "<< endl;

    pcl::io::savePCDFile ("./smoothedPC.pcd", *cloud_smoothed);

    cout << "hehe 3 "<< endl;
    //去除无效点
    PointCloud<pcl::PointXYZRGB>::iterator it = cloud_smoothed->points.begin();
    while (it != cloud_smoothed->points.end())
    {
        float x, y, z, rgb;
        x = it->x;
        y = it->y;
        z = it->z;
        rgb =it->rgb;  
        //cout << "x: " << x << "  y: " << y << "  z: " << z << "  rgb: " << rgb << endl;
        if (!pcl_isfinite(x) || !pcl_isfinite(y) || !pcl_isfinite(z) || !pcl_isfinite(rgb))
        {
            it = cloud_smoothed->points.erase(it);
        }
        else
            ++it;
    }
    cloud_smoothed->width = 1;
    cloud_smoothed->height = cloud_smoothed->points.size();

    pcl::io::savePCDFile ("./smoothedPC11.pcd", *cloud_smoothed);
    cout << "hehe 4 "<< endl;
	// 法线估计
	pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normalEstimation;                    //创建法线估计的对象
    normalEstimation.setInputCloud(cloud_smoothed);                                    //输入点云
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);          // 创建用于最近邻搜索的KD-Tree
    normalEstimation.setSearchMethod(tree);
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);     // 定义输出的点云法线
    // K近邻确定方法，使用k个最近点，或者确定一个以r为半径的圆内的点集来确定都可以，两者选1即可
    normalEstimation.setKSearch(100);                    // 使用当前点周围最近的10个点
    //normalEstimation.setRadiusSearch(0.03);            //对于每一个点都用半径为3cm的近邻搜索方式
    normalEstimation.compute(*normals);                 //计算法线
    cout << "hehe 5 "<< endl;
	
	// 将点云位姿、颜色、法线信息连接到一起
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	pcl::concatenateFields(*cloud_smoothed, *normals, *cloud_with_normals);
	// 贪心投影三角化
    cout << "hehe 6 "<< endl;

	//定义搜索树对象
	pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointXYZRGBNormal>);
	tree2->setInputCloud(cloud_with_normals);
    
	// 三角化
	pcl::GreedyProjectionTriangulation<pcl::PointXYZRGBNormal> gp3;   // 定义三角化对象
	pcl::PolygonMesh mesh; //存储最终三角化的网络模型
	cout << "hehe 7 "<< endl;

	// 设置三角化参数
	gp3.setSearchRadius(50);  //设置搜索时的半径，也就是KNN的球半径
	gp3.setMu (3.0);  //设置样本点搜索其近邻点的最远距离为2.5倍（典型值2.5-3），这样使得算法自适应点云密度的变化
	gp3.setMaximumNearestNeighbors (100);    //设置样本点最多可搜索的邻域个数，典型值是50-100

	gp3.setMinimumAngle(M_PI/18); // 设置三角化后得到的三角形内角的最小的角度为10°
	gp3.setMaximumAngle(2*M_PI/3); // 设置三角化后得到的三角形内角的最大角度为120°

	gp3.setMaximumSurfaceAngle(M_PI/4); // 设置某点法线方向偏离样本点法线的最大角度45°，如果超过，连接时不考虑该点
	gp3.setNormalConsistency(false);  //设置该参数为true保证法线朝向一致，设置为false的话不会进行法线一致性检查

	gp3.setInputCloud (cloud_with_normals);     //设置输入点云为有向点云
	gp3.setSearchMethod (tree2);   //设置搜索方式
    cout << "hehe 7.5 "<< endl;
	gp3.reconstruct (mesh);  //重建提取三角化
    cout << "hehe 8 "<< endl;


	// ----------------------结束你的代码--------------------------//

    // 显示网格化结果
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer_->setBackgroundColor(0, 0, 0);  //
    viewer_->addPolygonMesh(mesh, "mesh");  //
    while (!viewer_->wasStopped())
    {
    	viewer_->spinOnce(100);
    	boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }

    
    // //创建搜索树
	// pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointXYZRGBNormal>);
	// tree2->setInputCloud(cloud_with_normals);
	// //创建Poisson对象，并设置参数
	// pcl::Poisson<pcl::PointXYZRGBNormal> pn;
	// pn.setConfidence(false); //是否使用法向量的大小作为置信信息。如果false，所有法向量均归一化。
	// pn.setDegree(2); //设置参数degree[1,5],值越大越精细，耗时越久。
	// pn.setDepth(8); //树的最大深度，求解2^d x 2^d x 2^d立方体元。由于八叉树自适应采样密度，指定值仅为最大深度。
	// pn.setIsoDivide(8); //用于提取ISO等值面的算法的深度
	// pn.setManifold(false); //是否添加多边形的重心，当多边形三角化时。 设置流行标志，如果设置为true，则对多边形进行细分三角话时添加重心，设置false则不添加
	// pn.setOutputPolygons(false); //是否输出多边形网格（而不是三角化移动立方体的结果）
	// pn.setSamplesPerNode(3.0); //设置落入一个八叉树结点中的样本点的最小数量。无噪声，[1.0-5.0],有噪声[15.-20.]平滑
	// pn.setScale(1.25); //设置用于重构的立方体直径和样本边界立方体直径的比率。
	// pn.setSolverDivide(8); //设置求解线性方程组的Gauss-Seidel迭代方法的深度
	// //pn.setIndices();
 
	// //设置搜索方法和输入点云
	// pn.setSearchMethod(tree2);
	// pn.setInputCloud(cloud_with_normals);
	// //创建多变形网格，用于存储结果
	// pcl::PolygonMesh mesh;
	// //执行重构
 
	// pn.performReconstruction(mesh);
	// //保存网格图
	// pcl::io::savePLYFile("result.ply", mesh);
	// // 显示结果图
	// boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D viewer"));
	// viewer->setBackgroundColor(0, 0, 0);
	// viewer->addPolygonMesh(mesh, "my");
	// //viewer->addCoordinateSystem(1.0);//set coordinateSystem 
	// viewer->initCameraParameters();
 
	// //viewer->spin();
	// while (!viewer->wasStopped()){
	// 	viewer->spinOnce(100);
	// 	boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	// }
 
	return 0;

}
