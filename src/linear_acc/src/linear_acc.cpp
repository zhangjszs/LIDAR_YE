#include <linear_acc.hpp>
#include <cmath>

using namespace linear_acc;
Linear_Acc::Linear_Acc(ros::NodeHandle node,
                 ros::NodeHandle private_nh){
	nh_ = node;
	private_nh_ = private_nh;
    init();
	cone_points_set_ = nh_.subscribe("/cone_position",1, &Linear_Acc::cone_position_callback, this);	
	// ros_pub_vehicle_info = nh.subscribe("vehicleStatusMsg", 1, &Linear_Acc::vehicleStatus_callback, this);
	pub_vehicle_cmd = nh_.advertise < common_msgs::vehicle_cmd> ("vehcileCMDMsg", 1);
    linear_acc_path = nh_.advertise<nav_msgs::Path>("/linear_acc_path",10);
	ROS_INFO_STREAM("Linear_Acc INIT DONE!");
}
void Linear_Acc::init(){
    if(!private_nh_.param<int>("vis", vis_,0)){
        ROS_WARN_STREAM("Did not load linear_acc_visualization. Standard value is: " << vis_);
    }
    if(!private_nh_.param<int>("forward_distance", forward_distance_,0)){
        ROS_WARN_STREAM("Did not load forward_distance. Standard value is: " << forward_distance_);
    }
    if(!private_nh_.param<int>("inverse_flag", inverse_flag_,0)){
        ROS_WARN_STREAM("Did not load inverse_flag. Standard value is: " << inverse_flag_);
    }
    if(!private_nh_.param<int>("end_x", end_x_,10)){
        ROS_WARN_STREAM("Did not load end_x. Standard value is: " << end_x_);
    }
    
    // forward_distance_
    // ROS_INFO("vis: %d", vis_);
}
void Linear_Acc::visualizeLinearPath(double at2_angle_mid){
    if(forward_distance_==0){
        ROS_WARN_STREAM("forward_distance is zero, linear_acc_path visualization failed.");

    }
    ros_path_.header.frame_id = "velodyne";
    pose.header = ros_path_.header;
    Eigen::AngleAxisf t_V(at2_angle_mid,Eigen::Vector3f::UnitZ());
    Eigen::Matrix3f rotate_matrix = t_V.matrix();
    Eigen::Matrix4f transform_matrix = Eigen::Matrix4f::Identity();
    if(inverse_flag_)
        transform_matrix.block<3,3>(0,0) = rotate_matrix.inverse();
    else
        transform_matrix.block<3,3>(0,0) = rotate_matrix;
    double interval = 0.05;
    for(double i = 0; i < forward_distance_; i += interval) {
        pose.pose.position.x = i;
        pose.pose.position.y = 0;
        ros_path_.poses.push_back(pose);
    }
    for(size_t i = 0; i < ros_path_.poses.size(); i++) {
    double temp_x, temp_y;
    temp_x = ros_path_.poses[i].pose.position.x;
    temp_y = ros_path_.poses[i].pose.position.y;
    Eigen::Vector4f temp(temp_x, temp_y, 0, 1);
    Eigen::Vector4f result;
    // result = m * temp;
    result = transform_matrix * temp;
    ros_path_.poses[i].pose.position.y = result[1] / result[3];
    ros_path_.poses[i].pose.position.x= result[0] / result[3];
  }
    std::cout<<ros_path_<<std::endl;
    linear_acc_path.publish(ros_path_);
    return ;
}
void Linear_Acc::createPath() {
    if(cluster.points.size() == 0)
        return;
    int accumulator[180][201]={0};
    double p,p1,p2,Y_right,Y_left;
    int theta1,theta2;
    for(int i=0; i<cluster.points.size();i++)
    {
        if(cluster.points[i].y > 2 || cluster.points[i].y < -2)
            continue;
        for (int j=0; j<180; j++)
        {
            p=(cluster.points[i].x * cos(j * M_PI / 180)+cluster.points[i].y*sin(j * M_PI / 180))*5;
            if(p > 100)
                p = 100;
            accumulator[j][(int)p+100]+=1;            
       }
    }

    int max1 = 0;
    int max2 = 0;

    for(int i = 90 - allow_angle_error; i < 90 + allow_angle_error; i++)//linear 89 ~ 91
    {
        for(int j = 0; j < 100; j++)
        {
            if(accumulator[i][j] >= max1)
            {
                max1 = accumulator[i][j];
                p1=((float)j-100)/5;
                theta1=i;
            }
        }
    }
   
    for(int i = 90 - allow_angle_error; i < 90 + allow_angle_error; i++)
    {
        for(int j = 100; j < 200; j++)
        {
            if(accumulator[i][j] >= max2)
            {
                max2 = accumulator[i][j];
                p2=((float)j-100)/5;
                theta2=i;
            }
        }
    }

    if (theta1==theta2)
	{
		if  (fabs(p1)<3 && fabs(p2)<3 )
        {
            getPath=true;
            std::cout<<"find ideal path"<<std::endl;
        }
	}

    else
    {
        double check_x=(p1*cos((float)theta2*M_PI/180.0)-p2*cos((float)theta1*M_PI/180.0))/(sin((float)theta1*M_PI/180.0)*cos((float)theta2*M_PI/180.0)-sin((float)theta2*M_PI/180.0)*cos((float)theta1*M_PI/180.0));
        if ((check_x > 200 || check_x < -200)&&(fabs(p1)<3 && fabs(p2)<3))//直线距离车小鱼于3米
        {
            getPath=true;
			std::cout<<"find path"<<std::endl;
        }
        else
        {
            getPath=false;
            return;
        }
    }

    Y_right = (p1-path_length*cos((float)theta1*M_PI/180.0))/sin((float)theta1*M_PI/180.0);
    Y_left = (p2-path_length*cos((float)theta2*M_PI/180.0))/sin((float)theta2*M_PI/180.0);

    end_point.x = path_length;
    end_point.y = (Y_left + Y_right)/2;
}
void Linear_Acc::cone_position_callback(const common_msgs::ConeConstPtr& cone_position){
	cluster = *cone_position;
	// std::vector<pcl::PointXYZ> order;
	// pcl::PointXYZ pc_trans;
	// 	for(int i = 0;i<cone_position->points.size();i++){		
	// 		pc_trans.x = cone_position->points[i].x;
	// 		pc_trans.y = cone_position->points[i].y;
	// 		pc_trans.z = cone_position->points[i].z;	
	// 		orderCloud.insert(pc_trans);
	// 	}
	createPath();
    if(vis_ && !vised){
        if(getPath){
            double angleRotation = atan2(end_point.y, end_x_);
            visualizeLinearPath(angleRotation);
            vised = 1;
            ROS_INFO_STREAM("Linear_Acc_Path Visualize Successfully.");
        }
    }
	
	double at2_angle_mid = atan2(end_point.y , 0.5);
    // double at2_angle_mid_80 = atan2(end_point.y , 80.0);
    // ROS_INFO_STREAM("at2_80:"<<at2_angle_mid_80<<"   " << "at2_3"<<at2_angle_mid);
	int steering = -at2_angle_mid * 5 + 110;
	v_cmd.steering = steering;
	v_cmd.pedal_ratio = 20;
    v_cmd.working_mode = 0x01;
    v_cmd.gear_position = 0x01;
    pub_vehicle_cmd.publish(v_cmd);
	// ROS_INFO_STREAM("end_points"<<end_point);
	// ROS_INFO_STREAM("steering:"<<steering);
}

// void Linear_Acc::vehicleStatus_callback(const common_msgs::vehicle_statusConstPtr& vehicle_status){
// 	v_status = *vehicle_status;
// }

// double Linear_Acc::UpdateSpeed(double currentSpeed) {

// 	longError = 7 - currentSpeed;

// 	longCurrent = 0.5*longError + longLast;


// 	longLast = longCurrent;


// 	if(currentSpeed >10){
// 		longCurrent = 15;
// 	}

// 	if(longCurrent > 25){
// 		longCurrent = 25;
// 	}
		
//     return  longCurrent;
// }