#include <iostream>
#include "inc/teb_config.h"
#include "inc/pose_se2.h"
#include "inc/robot_footprint_model.h"
#include "inc/obstacles.h"
#include "inc/optimal_planner.h"
#include<boost/smart_ptr.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui_c.h>
//#include "/home/juchunyu/20231013/tebNoRos/teb_local_planner/src/matplotlibcpp.h"
#include "/home/juchunyu/20231013/tebNoRos/teb_local_planner/matplotlib-cpp/matplotlibcpp.h"
#include <vector>
#include <Eigen/Dense>


namespace plt = matplotlibcpp;//可视化

using namespace Eigen;

using namespace teb_local_planner;
using namespace std;

int main()
{
   
   /*
   cv::Mat m4 = cv::imread("/home/juchunyu/20231013/tebNoRos/teb_local_planner/PM.pgm",cv::IMREAD_GRAYSCALE);
   //cv::Mat m5(m4.size(), m4.type());
   //m4.convertTo(m5, -1, 0.3, 50);
   //cout<<mat<<endl;
   cout << "图像宽为：" << m4.cols << "\t高度为：" << m4.rows << "\t通道数为：" << m4.channels() << endl;
   for (int r = 0; r < m4.rows; ++r) {
        for (int c = 0; c < m4.cols; ++c) {
            int data = m4.at<unsigned char>(r, c);
            //std::cout << data << std::endl;
        }
    }
    for(int i = 0;i<000;i++){
        for(int j = 0;j<500;j++){
             m4.at<unsigned char>(i, j) = 255;
        }
    }
	cv::imshow("res_mat", m4);
	cv::waitKey(0);
    */
    // 参数配置
    TebConfig config;

    ViaPointContainer via_points;
    ViaPointContainer via_points_temp;
    vector<double> via_points_x;
    vector<double> via_points_y;

    double max_global_plan_lookahead_dist = 1;
   

    //add wayPoints
    double w_sigma =0.02;                 // 噪声Sigma值

    double w_sigma1 = 180/3.14;           // 噪声Sigma值
    cv::RNG rng;                         // OpenCV随机数产生器

   // via_points.push_back(Eigen::Vector2d(startPoint_x,startPOint_y));
   // via_points_x.push_back(startPoint_x);
   // via_points_y.push_back(startPOint_y);


    //生成隨幾
    /*
    double i = -2;
    while(i< 11){
        via_points.push_back( Eigen::Vector2d( i+rng.gaussian ( w_sigma ),rng.gaussian ( w_sigma1 ) ) );
        via_points_x.push_back(i+rng.gaussian ( w_sigma ));
        via_points_y.push_back(rng.gaussian ( w_sigma1 ));
       // cout<<"viaPoints("<<i<<','<<1<<")"<<endl;
       i = i + 0.05;
    }
   */
   //s曲
   /*
   double i = -2;
   while(i < 11){
   
    via_points.push_back( Eigen::Vector2d( i,0.2*sin(i)) );
    via_points_x.push_back(i);
    via_points_y.push_back(0.2*sin(i));
    i = i + 0.05;
}*/
/*
    //u 曲
    double i = -2;
    while(i < 3){
        via_points.push_back( Eigen::Vector2d( i,i*i+7 ));
        via_points_x.push_back(i);
        via_points_y.push_back(i*i+7);
        i = i + 0.05;
   }
*/
  // via_points.clear();
   /*
    double i = -2;
    while(i < 3){
            via_points.push_back( Eigen::Vector2d( i,5));
            via_points_x.push_back(i);
            via_points_y.push_back(5);
            i = i + 0.05;
   }
   cout<<"templeng"<<via_points_x.size()<<endl;
   double j = 5;
   while(j>3){
    via_points.push_back(Eigen::Vector2d(i,j));
    via_points_x.push_back(i);
    via_points_y.push_back(j);
    j = j - 0.05;
    cout<<"j"<<j<<endl;
   }
  */


   double i = -2;
    while(i < 2){
         if(i<0){
            via_points.push_back( Eigen::Vector2d( i,-6*i+3));
            via_points_x.push_back(i);
            via_points_y.push_back(-6*i+3);
            i = i + 0.05;
         } else
         {
            via_points.push_back( Eigen::Vector2d( i,6*i+3));
            via_points_x.push_back(i);
            via_points_y.push_back(6*i+3);
             i = i + 0.01;
         }
         
         
   }
  for(int i = 0;i<via_points_x.size();i++){
      cout<<"index ="<<i<<" "<<"("<<via_points_x[i]<<","<<via_points_y[i]<<")"<<endl;
  }

    //startPOint
    int size_ = via_points_x.size();
    double startPoint_x = via_points_x[0];
    double startPOint_y = via_points_y[0];
    double endPoint_x   = via_points_x[size_-1];
    double endPOint_y   = via_points_y[size_-1];
    PoseSE2 start(startPoint_x,startPOint_y,0);
    PoseSE2 end(endPoint_x ,endPOint_y,0);
  

    double r = 0.4;
    
    //Obstacles
    std::vector<ObstaclePtr> obst_vector;
    //obst_vector.emplace_back(boost::make_shared<PointObstacle>(5,0.1));　//add Point障礙物
    //obst_vector.emplace_back(boost::make_shared<LineObstacle>(5.8,-5,5.8,5));　//add Point障礙物

    //add Line Obstacle
    vector<double> lineObs_x = {5,5};
    vector<double> lineObs_y = {1.0,1.2};
    obst_vector.emplace_back(boost::make_shared<LineObstacle>(lineObs_x[0],lineObs_y[0],lineObs_x[1],lineObs_y[1]));  

    //add PolyObstacle
    vector<double> PolyOb_x = {1,0,1,2};
    vector<double> PolyOb_y = {-1.4,-1.3,-1.5,-1.2};
    PolygonObstacle* polyobst2 = new PolygonObstacle;
    
    polyobst2->pushBackVertex(PolyOb_x[0], PolyOb_y[0]);
    polyobst2->pushBackVertex(PolyOb_x[1], PolyOb_y[1]);
    polyobst2->pushBackVertex(PolyOb_x[2], PolyOb_y[2]);
    polyobst2->pushBackVertex(PolyOb_x[3], PolyOb_y[3]);
    polyobst2->finalizePolygon();
    obst_vector.emplace_back(polyobst2);
    

    vector<double> PolyOb_x_1 = {0,1,2,3};
    vector<double> PolyOb_y_1 = {1.1,1.2,1.3,1.5};
    PolygonObstacle* polyobst = new PolygonObstacle;

    polyobst->pushBackVertex(PolyOb_x_1[0], PolyOb_y_1[0]);
    polyobst->pushBackVertex(PolyOb_x_1[1], PolyOb_y_1[1]);
    polyobst->pushBackVertex(PolyOb_x_1[2], PolyOb_y_1[2]);
    polyobst->pushBackVertex(PolyOb_x_1[3], PolyOb_y_1[3]);
    polyobst->finalizePolygon();
    obst_vector.emplace_back(polyobst);



    // Setup robot shape model
    RobotFootprintModelPtr robot_model = boost::make_shared<CircularRobotFootprint>(r);
    auto visual = TebVisualizationPtr(new TebVisualization(config));
    //auto planner = new TebOptimalPlanner(config, &obst_vector, robot_model, visual, &via_points);
    //cv::Mat show_map = cv::Mat::zeros(cv::Size(500,500),CV_8UC3);

    // param
    int start_theta = 0;
    int end_theta = 0;
    float v_x = 0;
    float v_y = 0;
    float w   = 0;
    int look_ahead_poses = 2;
    vector<double> path_x;
    vector<double> path_y;

    vector<double> circle_x;
    vector<double> circle_y;

    double Robot_x = startPoint_x;
    double Robot_y = startPOint_y;

    double Robot_theta = atan2f( via_points_y[1]- via_points_y[0], via_points_x[1]- via_points_x[0]);//start_theta;

    double global_x = 0;
    double global_y = 0;
    double global_theta = 0;

    vector<double> global_x_arr;
    vector<double> global_y_arr;

    double ts = 0.1;
    double distToGoal =1000000000;
    int indexPrevious = 0;
   // cout<<"atan2f(1,2)="<<atan2(1,1)<<endl;
    int show_step = 0;
    while (distToGoal > 0.1){ 
           via_points_temp.clear();
           plt::clf();
           show_step ++;
           // start.theta() = start_theta * 0.1;
           // end.theta() = end_theta * 0.1;
           // start.theta() = start_theta;
           //end.theta()   = end_theta;
            //via_points.push_back( Eigen::Vector2d( i,i*i+7 ));
            //via_points_x.push_back(i);
            //via_points_y.push_back(i*i+7);
            double min_dist = 10000000000;
            int min_idx = 0;
            cout<<" via_points_x.size()="<< via_points_x.size()<<endl;
            for(int i = 0;i < via_points_x.size();i++){
                double dist = sqrt((via_points_x[i] - Robot_x)*(via_points_x[i] - Robot_x)+ (via_points_y[i] - Robot_y)*(via_points_y[i] - Robot_y));
                cout<<"idx = "<<i<<"dist = "<<dist<<endl;
                if(dist<min_dist){
                    min_dist = dist;
                    min_idx = i;
                 }
                /*
                if(dist < max_global_plan_lookahead_dist){
                    min_idx = i;
                }*/
            }
            //Control Robot Only towards
            //if(indexPrevious > min_idx){
            //    min_idx = indexPrevious;
            //}
            indexPrevious = min_idx;
            cout<<"min_idx="<<min_idx<<endl;
            double distNum = 0;
            double goal_x_temp;
            double goal_y_temp;
            double global_theta_temp;
            int times= 0;
            for(int i = min_idx;i<via_points_x.size();i++){
                cout<<"times="<<times<<endl;
                times++;
                via_points_temp.push_back(Eigen::Vector2d( via_points_x[i],via_points_y[i]));
                distNum = sqrt((Robot_x -via_points_x[i])*(Robot_x - via_points_x[i])+ (Robot_y - via_points_y[i])*(Robot_y - via_points_y[i]));
                if(distNum > max_global_plan_lookahead_dist){//||i >= via_points_x.size()){
                    //cout<<"index ="<<i<<"goal_x_temp=:"<<goal_x_temp<<"goal_y_temp"<<goal_y_temp<<""
                    goal_x_temp = via_points_x[i];
                    goal_y_temp = via_points_y[i];
                    if((i+1)>via_points_x.size()){
                    global_theta_temp = atan2l( via_points_y[i]- via_points_y[i-1],via_points_x[i]-via_points_x[i-1]);
                    }else{
                         global_theta_temp = atan2l( via_points_y[i+1]- via_points_y[i],via_points_x[i+1]-via_points_x[i]);
                    }
                    break;
                }
            }

            cout<<"goal_x_temp"<<goal_x_temp<<" "<<"goal_y:"<<goal_y_temp<<"global_theta_temp="<<global_theta_temp<<endl; 
            cout<<"vRobot_x="<<Robot_x<<"Robot_y="<<Robot_y<<endl;
            cout<<"endPoint_x = "<<endPoint_x<<"endPOint_y = "<<endPOint_y<<endl;
            cout<<"v_x="<<v_x<<" "<<"w="<<w<<endl;
            auto planner = new TebOptimalPlanner(config, &obst_vector, robot_model, visual, &via_points_temp);
            PoseSE2 start_(Robot_x,Robot_y,Robot_theta);
            PoseSE2 end_(goal_x_temp ,goal_y_temp,global_theta_temp);
            planner->plan(start_,end_);
            std::vector<Eigen::Vector3f> path;
            planner->getFullTrajectory(path);
            planner->getVelocityCommand(v_x,v_y,w,look_ahead_poses);

            //Robot Motion model 
            Robot_theta   = Robot_theta + w * ts; //+  rng.gaussian ( w_sigma1);;
            Robot_x       = Robot_x + v_x * cos(Robot_theta) * ts;// + rng.gaussian ( w_sigma );
            Robot_y       = Robot_y + v_x * sin(Robot_theta) * ts;// + rng.gaussian ( w_sigma );
            
            global_x_arr.push_back(Robot_x);
            global_y_arr.push_back(Robot_y);
            //cout<<"global_x_arr:"<<global_x_arr.size()<<endl;

            //cout<<"Robot_x:"<<Robot_x<<endl;
            //cout<<"Robot_y:"<<Robot_y<<endl;
            
            
          
           // cout<<"速度指令v_ｘ:"<<v_x<<" "<<"速度指令ｖ_y:"<<v_y<<" " <<"w指令"<<w<<endl;
           // cout<<"路徑長度："<<path.size()<<endl;
            for(int i = 0;i < path.size() - 1;i ++)
            {
                path_x.push_back(path.at(i)[0]);
                path_y.push_back(path.at(i)[1]);
                //cout<<"("<<path.at(i)[0]<<","<<path.at(i)[1]<<")"<<endl;
            }
          
           
            vector<double> path_x_show_step_x;
            vector<double> path_y_show_step_y;
            plt::clf();
            //plt::hold();
            std::map<std::string, std::string> keywords1;
            keywords1.insert(std::pair<std::string, std::string>("label", "TEB_Plan_Traj") );
            keywords1.insert(std::pair<std::string, std::string>("linewidth", "2.5") );
            plt::plot(path_x,path_y,keywords1);
               // path_x_show_step_x.push_back(path_x[i]);
               // path_y_show_step_y.push_back(path_y[i]);
            plt::plot(path_x,path_y,keywords1);
                
            std::map<std::string, std::string> keywords;
            keywords.insert(std::pair<std::string, std::string>("label", "global_Plan_traj") );
            keywords.insert(std::pair<std::string, std::string>("linewidth", "1.8") );
            plt::plot(via_points_x,via_points_y,keywords);


            std::map<std::string, std::string> keywords2;
            keywords2.insert(std::pair<std::string, std::string>("label", "PolyObstacle") );
            keywords2.insert(std::pair<std::string, std::string>("linewidth", "2.5") );
            plt::plot(PolyOb_x,PolyOb_y);
            plt::fill(PolyOb_x,PolyOb_y,keywords2);

            plt::plot(PolyOb_x_1,PolyOb_y_1);
            plt::fill(PolyOb_x_1,PolyOb_y_1,keywords2);

            std::map<std::string, std::string> keywords3;
            keywords3.insert(std::pair<std::string, std::string>("label", "LIneObstacle") );
            keywords3.insert(std::pair<std::string, std::string>("linewidth", "3") );
                
            plt::plot(lineObs_x,lineObs_y,keywords3);
                //plt::scatter(ob_x,ob_y,50);
                
            //Plot Robot
            double pi = 3.14;
            double theta = 0;
            while(theta < 2*pi){
                //double x_temp = start.x ＋r*cos(theta);
                double x_temp = Robot_x + r * cos(theta);
                double y_temp = Robot_y + r * sin(theta);
                circle_x.push_back(x_temp);
                circle_y.push_back(y_temp);
                theta = theta + 0.01;
            }

            std::map<std::string, std::string> keywords4;
            keywords4.insert(std::pair<std::string, std::string>("label", "RobotModel") );
            keywords4.insert(std::pair<std::string, std::string>("linewidth", "3") );

            plt::plot(circle_x,circle_y,keywords4);


             if(show_step%2==0){

                std::map<std::string, std::string> keywords5;
                keywords5.insert(std::pair<std::string, std::string>("label", "Acutal Robot Traj") );
                keywords5.insert(std::pair<std::string, std::string>("linewidth", "3") );  
                plt::plot(global_x_arr,global_y_arr,keywords5);
            }
     
            circle_x.clear();
            circle_y.clear();

            path_x.clear();
            path_y.clear();

            plt::xlabel("x");
            plt::ylabel("y");

            plt::xlim(-10, 18);
            plt::ylim(-10, 16);
            plt::title("Teb Plan Traj");
            plt::legend();
            plt::pause(0.0001);
               // plt::show();

            distToGoal = sqrt((Robot_x - endPoint_x) * (Robot_x - endPoint_x) + ((Robot_y - endPOint_y)*(Robot_y - endPOint_y)));
            //cout<<"RObot_x:"<<Robot_x<<" "<<"Robot_y:"<<Robot_y<<endl;
            //cout<<"endPoint_x:"<<endPoint_x<<" "<<"endPoint_y:"<<endPOint_y<<endl;



               
            if(distToGoal < 0.10){
                plt::show();
            }
           // plt::pause(0.0001);
          // cout<<"distTOGoal:"<<distToGoal<<endl;
          
            

    }
    //plt::pause(0);
    return 0;
}
