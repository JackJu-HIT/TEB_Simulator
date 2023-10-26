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

namespace plt = matplotlibcpp;//可视化

using namespace teb_local_planner;
using namespace std;

int main()
{
    // 参数配置
    TebConfig config;

    //startPOint
    double startPoint_x = -2;
    double startPOint_y = 1;
    double endPoint_x   = 10;
    double endPOint_y   = 1;
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



    ViaPointContainer via_points;
    vector<double> via_points_x;
    vector<double> via_points_y;
   

    //add wayPoints
    double w_sigma = 0.5;                 // 噪声Sigma值

    double w_sigma1 = 1;                 // 噪声Sigma值
    cv::RNG rng;                        // OpenCV随机数产生器

    via_points.push_back(Eigen::Vector2d(startPoint_x,startPOint_y));
    via_points_x.push_back(startPoint_x);
    via_points_y.push_back(startPOint_y);
    for(int i = -2;i < 11;i++){
        via_points.push_back( Eigen::Vector2d( i+rng.gaussian ( w_sigma ),rng.gaussian ( w_sigma1 ) ) );
        via_points_x.push_back(i+rng.gaussian ( w_sigma ));
        via_points_y.push_back(rng.gaussian ( w_sigma1 ));
       // cout<<"viaPoints("<<i<<','<<1<<")"<<endl;
    }
    via_points.push_back(Eigen::Vector2d(endPoint_x,endPOint_y));
    via_points_x.push_back(endPoint_x);
    via_points_y.push_back(endPOint_y);
    // Setup robot shape model
    RobotFootprintModelPtr robot_model = boost::make_shared<CircularRobotFootprint>(r);
    auto visual = TebVisualizationPtr(new TebVisualization(config));
    auto planner = new TebOptimalPlanner(config, &obst_vector, robot_model, visual, &via_points);
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
    double Robot_theta = start_theta;

    double global_x = 0;
    double global_y = 0;
    double global_theta = 0;

    vector<double> global_x_arr;
    vector<double> global_y_arr;

    double ts = 0.1;
    double distToGoal =1000000000;

 
    int show_step = 0;
    while (distToGoal > 0.01){ 
           plt::clf();
           show_step ++;
           // start.theta() = start_theta * 0.1;
           // end.theta() = end_theta * 0.1;
           // start.theta() = start_theta;
           //end.theta()   = end_theta;
            PoseSE2 start_(Robot_x,Robot_y,Robot_theta);
            planner->plan(start_,end);
            std::vector<Eigen::Vector3f> path;
            planner->getFullTrajectory(path);
            planner->getVelocityCommand(v_x,v_y,w,look_ahead_poses);

            //Robot Motion model 
            Robot_theta   = Robot_theta + w * ts;
            Robot_x       = Robot_x + v_x * cos(Robot_theta) * ts;
            Robot_y       = Robot_y + v_x * sin(Robot_theta) * ts;
            
            global_x_arr.push_back(Robot_x);
            global_y_arr.push_back(Robot_y);
            cout<<"global_x_arr:"<<global_x_arr.size()<<endl;

            cout<<"Robot_x:"<<Robot_x<<endl;
            cout<<"Robot_y:"<<Robot_y<<endl;
            
            
          
            cout<<"速度指令v_ｘ:"<<v_x<<" "<<"速度指令ｖ_y:"<<v_y<<" " <<"w指令"<<w<<endl;
            cout<<"路徑長度："<<path.size()<<endl;
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

            plt::xlim(-1, 13);
            plt::xlim(-5, 13);
            plt::title("Teb Plan Traj");
            plt::legend();
            plt::pause(0.0001);
               // plt::show();

            distToGoal = sqrt((Robot_x - endPoint_x) * (Robot_x - endPoint_x) + ((Robot_y - endPOint_y)*(Robot_y - endPOint_y)));
               
            if(distToGoal < 0.01){
                plt::show();
            }
           // plt::pause(0.0001);
            

    }
    //plt::pause(0);
    return 0;
}
