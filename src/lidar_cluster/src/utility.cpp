#include <utility.h>

struct comp{
  bool operator()(pcl::PointXYZ a,pcl::PointXYZ b){
    if (a.x<b.x) return true;
    else if (a.x>b.x) return false;
    else if (a.y<b.y) return true;
    else if (a.y>b.y) return false;
    else if (a.z>b.z) return true;
    else if (a.z>b.z) return false;
  }
};
set<pcl::PointXYZ,comp> orderCloud;


float euc_distance(pcl::PointXYZ a){
    return float(sqrt(a.x*a.x+a.y*a.y));
}

void point_clip(const pcl::PointCloud<pcl::PointXYZ>::Ptr in){
    pcl::ExtractIndices<pcl::PointXYZ> cliper;
    cliper.setInputCloud(in);
    pcl::PointIndices indices;
    float tmp_euc;
#pragma omp for
    for (size_t i = 0; i < in->points.size(); i++)
    {
        tmp_euc = euc_distance(in->points[i]);
        if ( tmp_euc<= 30 && in->points[i].x>0 && tmp_euc>=0.5)
        {
            indices.indices.push_back(i);
        }
    }
    cliper.setIndices(boost::make_shared<pcl::PointIndices>(indices));
    cliper.setNegative(false); //ture to remove the indices false to retain the indices
    cliper.filter(*in);
}


void lidar_cluster::init(){
        // m<< 0.976219,-0.216786,0,   1.19277,
        //     0.216786, 0.976219,0,   -0.328103,
        //     0,        0,       1,   0,
        //     0,        0,       0,   1;
    // m<< 0.976219,-0.216786,0,   0,
    //         0.216786, 0.976219,0,   0,
    //         0,        0,       1,   0,
    //         0,        0,       0,   1;
    if(!private_nh_.param<std::string>("input_topic", input_topic,"/velodyne_points")){
        ROS_WARN_STREAM("Did not load input_topic topic name. Standard value is: " << input_topic);
    }
    if(!private_nh_.param<std::string>("no_ground_point_topic", no_ground_topic,"/points_no_ground")){
        ROS_WARN_STREAM("Did not load points_no_ground topic name. Standard value is: " << no_ground_topic);
    }
    if(!private_nh_.param<std::string>("ground_point_topic", ground_topic,"/points_ground")){
        ROS_WARN_STREAM("Did not load points_ground topic name. Standard value is: " << ground_topic);
    }
    if(!private_nh_.param<std::string>("all_points_topic", all_points_topic,"/all_points")){
        ROS_WARN_STREAM("Did not load points_ground topic name. Standard value is: " << all_points_topic);
    }
    if(!private_nh_.param<double>("clip_height", clip_height_,2.0)){
        ROS_WARN_STREAM("Did not load clip_height. Standard value is: " << clip_height_);
    }
    if(!private_nh_.param<double>("sensor_height", sensor_height_,0.135)){
        ROS_WARN_STREAM("Did not load sensor_height. Standard value is: " << sensor_height_);
    }
    if(!private_nh_.param<double>("min_distance", min_distance_,0.1)){
        ROS_WARN_STREAM("Did not load min_distance. Standard value is: " << min_distance_);
    }
    if(!private_nh_.param<double>("max_distance", max_distance_,100.0)){
        ROS_WARN_STREAM("Did not load sensor_height. Standard value is: " << max_distance_);
    }
    if(!private_nh_.param<double>("sensor_height", sensor_height_,100.0)){
        ROS_WARN_STREAM("Did not load sensor_height. Standard value is: " << sensor_height_);
    }
    if(!private_nh_.param<int>("sensor_model", sensor_model_,16)){
        ROS_WARN_STREAM("Did not load sensor_model. Standard value is: " << sensor_model_);
    }
    if(!private_nh_.param<int>("num_iter", num_iter_,3)){
        ROS_WARN_STREAM("Did not load num_iter. Standard value is: " << num_iter_);
    }
    if(!private_nh_.param<int>("num_lpr", num_lpr_,5)){
        ROS_WARN_STREAM("Did not load num_lpr. Standard value is: " << num_lpr_);
    }
    if(!private_nh_.param<double>("th_seeds", th_seeds_,0.03)){
        ROS_WARN_STREAM("Did not load th_seed. Standard value is: " << th_seeds_);
    }
    if(!private_nh_.param<double>("th_dist", th_dist_,0.03)){
        ROS_WARN_STREAM("Did not load th_seed. Standard value is: " << th_seeds_);
    }
    if(!private_nh_.param<int>("road_type", road_type_,1)){
        ROS_WARN_STREAM("Did not load road_type. Standard value is: " << road_type_);
    }
    if(!private_nh_.param<double>("z_up", z_up_,0.7)){
        ROS_WARN_STREAM("Did not load z_up. Standard value is: " << z_up_);
    }
    if(!private_nh_.param<double>("z_down", z_down_,-1.0)){
        ROS_WARN_STREAM("Did not load road_type. Standard value is: " << z_down_);
    }
    if(!private_nh_.param<int>("vis", vis_,0)){
        ROS_WARN_STREAM("Did not load vis. Standard value is: " << vis_);
    }
    if(!private_nh_.param<std::string>("str_range", str_range_,"15,30,45,60")){
        ROS_WARN_STREAM("Did not load str_range. Standard value is: " << str_range_);
    }
    if(!private_nh_.param<std::string>("str_seg_distance", str_seg_distance_,"0.5,1.1,1.6,2.1,2.6")){
        ROS_WARN_STREAM("Did not load str_seg_distance. Standard value is: " << str_seg_distance_);
    }
    ROS_INFO("clip_height: %f", clip_height_);
    ROS_INFO("sensor_height: %f", sensor_height_);
    ROS_INFO("min_distance: %f", min_distance_);
    ROS_INFO("max_distance: %f", max_distance_);
    ROS_INFO("sensor_model: %d", sensor_model_);
    ROS_INFO("num_iter: %d", num_iter_);
    ROS_INFO("num_lpr: %d", num_lpr_);
    ROS_INFO("th_seeds: %f", th_seeds_);
    ROS_INFO("th_dist: %f", th_dist_);
    ROS_INFO("road_type: %d", road_type_);
    ROS_INFO("z_up: %f", z_up_);
    ROS_INFO("z_down: %f", z_down_);
    ROS_INFO("vis: %d", vis_);

    splitString(str_range_, dis_range);
    splitString(str_seg_distance_, seg_distances);

    getPointClouds = false;
    g_seeds_pc.reset(new pcl::PointCloud<PointType>());
    g_ground_pc.reset(new pcl::PointCloud<PointType>());
    g_not_ground_pc.reset(new pcl::PointCloud<PointType>());
    g_all_pc.reset(new pcl::PointCloud<PointType>());
    current_pc_ptr.reset(new pcl::PointCloud<PointType>());
    cloud_filtered.reset(new pcl::PointCloud<PointType>());
    skidpad_detection_pc.reset(new pcl::PointCloud<PointType>());
}

void lidar_cluster::splitString(const std::string& in_string, std::vector<double>& out_array)
{
    std::string tmp;
    std::istringstream in(in_string);
    while (std::getline(in, tmp, ',')) {
        out_array.push_back(stod(tmp));
    }
}

void lidar_cluster::PassThrough(pcl::PointCloud<PointType>::Ptr& cloud_filtered,int type,double z_up,double z_down){
  // auto startTime = std::chrono::steady_clock::now();
  pcl::PassThrough<pcl::PointXYZ> pass;
  // std::cout<<"PassThrough before :"<<cloud_filtered->points.size()<<std::endl;
  pass.setInputCloud(cloud_filtered);
  //当线由车后延到车头这个方向，左右是y，前后是x，上下是z，右y是负的，左y是正的
  // pass.filter(*cloud_filtered);
  // std::cout<<"TYPE:::"<<type<<std::endl;
  if(type == 1){//circle
    point_clip(cloud_filtered);
  // pass.setFilterFieldName("x");//x:-1 represent down +1 represent forward
  // pass.setFilterLimits(0.1,20);
  // pass.filter(*cloud_filtered);
  }else if(type == 2){
  pass.setFilterFieldName("x");
  pass.setFilterLimits(0.1,20);
  pass.filter(*cloud_filtered);
  
  pass.setFilterFieldName("y");//y:-1 represent right,+1 represent left
  pass.setFilterLimits(-4,4);
  pass.filter(*cloud_filtered);
  }else{
  pass.setFilterFieldName("x");
  pass.setFilterLimits(0.1,20);
  pass.filter(*cloud_filtered);
  pass.setFilterFieldName("y");//y:-1 represent right,+1 represent left
  pass.setFilterLimits(-3,3);
  pass.filter(*cloud_filtered);
  }
  pass.setFilterFieldName("z");
  pass.setFilterLimits(-0.35,1);    //-1 1
  pass.filter(*cloud_filtered);

  // std::cout<<"before voxel pointsize:"<<cloud_filtered->points.size()<<std::endl;
  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud(cloud_filtered);
  sor.setLeafSize(0.05f, 0.05f, 0.05f);
  sor.filter(*cloud_filtered);
  // std::cout<<"after voxel pointsize:"<<cloud_filtered->points.size()<<std::endl;
  // auto endTime = std::chrono::steady_clock::now();
  //   auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
  //   std::cout << "PassThrough took " << elapsedTime.count() << " milliseconds" << std::endl;
}




// void lidar_cluster::EucClusterMethod(const pcl::PointCloud<pcl::PointXYZ>::Ptr& in_cloud,
//     std::vector<ClusterPtr>& clusters, const double& max_cluster_dis)
// {
//     pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

//     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_2d(new pcl::PointCloud<pcl::PointXYZ>);

//     pcl::copyPointCloud(*in_cloud, *cloud_2d);

//     for (size_t i = 0; i < cloud_2d->points.size(); i++) {
//         cloud_2d->points[i].z = 0;
//     }

//     tree->setInputCloud(cloud_2d);
//     std::vector<pcl::PointIndices> cluster_indices;

//     pcl::EuclideanClusterExtraction<pcl::PointXYZ> euc;
//     // setClusterTolerance(). If you take a very small value, it can happen that
//     // an actual object can be seen as multiple clusters. On the other hand, if
//     // you set the value too high, it could happen, that multiple objects are
//     // seen as one cluster. So our recommendation is to just test and try out
//     // which value suits your dataset.
//     euc.setClusterTolerance(max_cluster_dis);
//     euc.setMinClusterSize(cluster_min_points);
//     euc.setMaxClusterSize(cluster_max_points);
//     euc.setSearchMethod(tree);
//     euc.setInputCloud(cloud_2d);
//     euc.extract(cluster_indices);

//     for (size_t j = 0; j < cluster_indices.size(); j++) {
//         ClusterPtr one_cluster;
//         one_cluster->setCloud(in_cloud, color_table, cluster_indices[j].indices, j);
//         clusters.push_back(one_cluster);
//     }
// }



void lidar_cluster::EucClusterMethod(pcl::PointCloud<PointType>::Ptr inputcloud,
    std::vector<pcl::PointIndices>& cluster_indices, const double& max_cluster_dis){
   if(inputcloud->points.size()==0)
        return ;
   // auto startTime = std::chrono::steady_clock::now();
   pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (inputcloud);
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec; //聚类对象
  ec.setClusterTolerance (max_cluster_dis); //设置邻近搜索的搜索半径为3cm
  ec.setMinClusterSize (2);    //设置一个聚类需要的最少点云数目为100
  ec.setMaxClusterSize (100);  //最多点云数目为20000
  ec.setSearchMethod (tree);     //设置点云的搜索机制
  ec.setInputCloud (inputcloud); //设置原始点云
  ec.extract (cluster_indices);  //从点云中提取聚类
  // auto endTime = std::chrono::steady_clock::now();
  //   auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
  //   std::cout << "EucCluster took " << elapsedTime.count() << " milliseconds" << std::endl;
}


void lidar_cluster::EuclideanAdaptiveClusterMethod(pcl::PointCloud<PointType>::Ptr inputcloud,
    std::vector<std::vector<pcl::PointIndices>>& cluster_indices, std::vector<pcl::PointCloud<PointType>::Ptr> cloud_segments_array){
    // 根据距离不同，设置不同的聚类阈值
    // 0 => 0-15m d=0.5
    // 1 => 15-30 d=1
    // 2 => 30-45 d=1.6
    // 3 => 45-60 d=2.1
    // 4 => >60   d=2.6
    float interval = 0;
    for (size_t i = 0; i < inputcloud->points.size(); i++) {
        pcl::PointXYZ p;
        p.x = inputcloud->points[i].x;
        p.y = inputcloud->points[i].y;
        p.z = inputcloud->points[i].z;
        // std::cout<< dis_range[0] << dis_range[1]<< std::endl;
        float origin_dis = sqrt(pow(p.x, 2) + pow(p.y, 2));
        // float origin_dis = p.x;
        if (origin_dis < dis_range[0]) {
            cloud_segments_array[0]->points.push_back(p);
        } else if (origin_dis < dis_range[1]) {
            cloud_segments_array[1]->points.push_back(p);
        } else if (origin_dis < dis_range[2]) {
            cloud_segments_array[2]->points.push_back(p);
        } else if ( origin_dis < dis_range[3]) {
            cloud_segments_array[3]->points.push_back(p);
        } else {
            cloud_segments_array[4]->points.push_back(p);
        }
    }
    std::vector<pcl::PointIndices> v;
    for (size_t i = 0; i < cloud_segments_array.size(); i++) {
        v.clear();
        EucClusterMethod(cloud_segments_array[i], v, seg_distances[i]);
        cluster_indices.push_back(v);
  }

}


void lidar_cluster::clusterMethod32(){
    if(vis_){
        bool vis_init_status = vis_init();
    }
    orderCloud.clear();
    position.points.clear();
    position.pc.clear();
    skidpad_detection_pc->points.clear();
        std::vector<std::vector<pcl::PointIndices>> cluster_indices;
  // std::vector<pcl::P
  // split_points_MultiEuc(g_not_ground_pc);
      std::vector<pcl::PointCloud<PointType>::Ptr> cloud_segments_array(5);
    for (size_t i = 0; i < cloud_segments_array.size(); i++) {
            pcl::PointCloud<pcl::PointXYZ>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZ>);
            cloud_segments_array[i] = tmp;
      }
    EuclideanAdaptiveClusterMethod(g_not_ground_pc, cluster_indices, cloud_segments_array);
  
  // EucClusterMethod(g_not_ground_pc, cluster_indices, 0.3);
  int j = 0;
  int clusterId = 0;
  pcl::PointCloud<pcl::PointXYZ>::Ptr final_cluster (new pcl::PointCloud<pcl::PointXYZ>);
  //for(pcl::PointIndices getIndices: cluster_indices){}
  for( int i = 0;i<cluster_indices.size(); ++i){
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices[i].begin (); it != cluster_indices[i].end (); ++it)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    //for(int index : getIndices.indices)
    // intensity = 0.0;
    intensity_count=0;
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
      //cloudCluster->point.push_bach(cloud->points[index]);
    {
        // intensity_count++;
      // cloud_cluster->points.push_back (g_not_ground_pc->points[*pit]); //*
      cloud_cluster->points.push_back (cloud_segments_array[i]->points[*pit]); //*
      // intensity += g_not_ground_pc->points[*pit].intensity;
    }
    // intensity = intensity / cloud_cluster->points.size();
    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud_cluster,centroid);

    pcl::PointXYZ min;
    pcl::PointXYZ max;
    pcl::getMinMax3D(*cloud_cluster,min,max);
    double xx = (double)std::fabs(max.x-min.x); 
    double yy = (double)std::fabs(max.y-min.y);
    double zz = (double)std::fabs(max.z-min.z);
    // if( (xx < 0.4 && yy < 0.4 && zz < 1 && (zz > 0.1 || centroid[2]>0.05) && centroid[2]< 0.7 && centroid[2] > -0.1) )//x
    if( (xx < 0.4 && yy < 0.4 && zz < 0.4))//x
    // if( (xx < 0.4 && yy < 0.4 && zz < 1 && centroid[2]< 0.6 && centroid[2] > -0.1) )//x
    {
         *final_cluster = *final_cluster + *cloud_cluster;
        pcl::toROSMsg(*cloud_cluster, cluster_for_publish);
        
        cluster_for_publish.header = in_pc.header;
        position.pc.push_back(cluster_for_publish);

        geometry_msgs::Point32 tmp;
        tmp.x=centroid[0];
        tmp.y=centroid[1];
        tmp.z=centroid[2];
        euc = float(sqrt(centroid[0]*centroid[0] +centroid[1]*centroid[1]));
        position.points.push_back(tmp);//cone position of centroid published
        position.obj_dist.push_back(euc);
        if(vis_ && vis_init_status){
            vis_for_marker(max,min,euc);
        }
    }
    else
        continue;
  // std::cout<<clusterId<<"Cluster_Centroid"<<"("
  // << centroid[0] << ","
  // << centroid[1] << ","
  // << centroid[2] << ")."<<std::endl;
  ++clusterId;
  ++j;
  // geometry_msgs::Point32 tmp;
  // pcl::PointXYZ tmp_pc;
  // tmp.x=centroid[0];
  // tmp.y=centroid[1];
  // tmp.z=centroid[2];
  pcl::PointXYZ tmp_pXYZ;
  tmp_pXYZ.x = centroid[0];
  tmp_pXYZ.y = centroid[1];
  tmp_pXYZ.z = centroid[2];
  // tmp_pc.x=centroid[0];
  // tmp_pc.y=centroid[1];
  // tmp_pc.z=centroid[2];
  skidpad_detection_pc->points.push_back(tmp_pXYZ);
  // orderCloud.insert(tmp_pc);

  // out_pc.points.push_back(tmp);//only centroid points
  // position.points.push_back(tmp);//cone position of centroid published
  // delete cloud_cluster;
  }
}

markerPub.publish(marker_array);
    //sensor_msgs::PointCloud2 pub_pc;

pcl::toROSMsg(*final_cluster, pub_pc);
pub_pc.header = in_pc.header;
pub_filtered_points__.publish(pub_pc);

position.pc_whole = pub_pc;
position.pc_whole.header = in_pc.header;
// markerPub.publish(marker_array);

pcl::toROSMsg(*skidpad_detection_pc, pub_pc);
pub_pc.header = in_pc.header;
skidpad_detection.publish(pub_pc);
position.header = in_pc.header;
// ROS_INFO_STREAM("PUBLISH HEADERS"<<in_pc.header.stamp.toSec())

cone_position.publish(position);
}
void lidar_cluster::clusterMethod16(){
    if(vis_){
        bool vis_init_status = vis_init();
    }
    orderCloud.clear();
    position.points.clear();
    position.pc.clear();
    skidpad_detection_pc->points.clear();
    std::vector<pcl::PointIndices> cluster_indices;
  
  EucClusterMethod(g_not_ground_pc, cluster_indices, 0.3);
  int j = 0;
  int clusterId = 0;
  pcl::PointCloud<pcl::PointXYZ>::Ptr final_cluster (new pcl::PointCloud<pcl::PointXYZ>);
  //for(pcl::PointIndices getIndices: cluster_indices){}
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
  // for(const auto *it : cluster_indices){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    //for(int index : getIndices.indices)
    // intensity = 0.0;
    intensity_count=0;
    for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++){
    // for(const auto *pit:it->indices){
      //cloudCluster->point.push_bach(cloud->points[index]);
        // intensity_count++;
      cloud_cluster->points.push_back (g_not_ground_pc->points[*pit]); //*
      // cloud_cluster->points.push_back (cloud_segments_array[i]->points[*pit]); //*
      // intensity += g_not_ground_pc->points[*pit].intensity;
    }
    // intensity = intensity / cloud_cluster->points.size();
    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud_cluster,centroid);

    pcl::PointXYZ min;
    pcl::PointXYZ max;
    pcl::getMinMax3D(*cloud_cluster,min,max);
    double xx = (double)std::fabs(max.x-min.x); 
    double yy = (double)std::fabs(max.y-min.y);
    double zz = (double)std::fabs(max.z-min.z);
    // if( (xx < 0.4 && yy < 0.4 && zz < 1 && (zz > 0.1 || centroid[2]>0.05) && centroid[2]< 0.7 && centroid[2] > -0.1) )//x
    if( (xx < 0.4 && yy < 0.4 && zz < 1 ) )//x
    // if( (xx < 0.4 && yy < 0.4 && zz < 1 && centroid[2]< 0.6 && centroid[2] > -0.1) )//x
    {
         *final_cluster = *final_cluster + *cloud_cluster;
        pcl::toROSMsg(*cloud_cluster, cluster_for_publish);
        cluster_for_publish.header = in_pc.header;
        position.pc.push_back(cluster_for_publish);

        geometry_msgs::Point32 tmp;
        tmp.x=centroid[0];
        tmp.y=centroid[1];
        tmp.z=centroid[2];
        euc = float(sqrt(centroid[0]*centroid[0] +centroid[1]*centroid[1]));
        position.points.push_back(tmp);//cone position of centroid published
        position.obj_dist.push_back(euc);
        if(vis_ && vis_init_status){
            vis_for_marker(max,min,euc);
        }
    }
    else
        continue;
  // std::cout<<clusterId<<"Cluster_Centroid"<<"("
  // << centroid[0] << ","
  // << centroid[1] << ","
  // << centroid[2] << ")."<<std::endl;
  ++clusterId;
  ++j;
  // geometry_msgs::Point32 tmp;
  // pcl::PointXYZ tmp_pc;
  // tmp.x=centroid[0];
  // tmp.y=centroid[1];
  // tmp.z=centroid[2];
  pcl::PointXYZ tmp_pXYZ;
  tmp_pXYZ.x = centroid[0];
  tmp_pXYZ.y = centroid[1];
  tmp_pXYZ.z = centroid[2];
  // tmp_pc.x=centroid[0];
  // tmp_pc.y=centroid[1];
  // tmp_pc.z=centroid[2];
  skidpad_detection_pc->points.push_back(tmp_pXYZ);
  // orderCloud.insert(tmp_pc);

  // out_pc.points.push_back(tmp);//only centroid points
  // position.points.push_back(tmp);//cone position of centroid published
  // delete cloud_cluster;
  }
  
//gettimeofday(&end,NULL);

markerPub.publish(marker_array);
    //sensor_msgs::PointCloud2 pub_pc;

pcl::toROSMsg(*final_cluster, pub_pc);
pub_pc.header = in_pc.header;
pub_filtered_points__.publish(pub_pc);

// markerPub.publish(marker_array);

pcl::toROSMsg(*skidpad_detection_pc, pub_pc);
pub_pc.header = in_pc.header;
skidpad_detection.publish(pub_pc);

cone_position.publish(position);

}


// void lidar_cluster::clusterMethod(){
//     if(vis_){
//         bool vis_init_status = vis_init();
//     }
//     orderCloud.clear();
//     position.points.clear();
//     skidpad_detection_pc->points.clear();
//   // std::vector<std::vector<pcl::PointIndices>> cluster_indices;
//   std::vector<pcl::PointIndices> cluster_indices;
//   // std::vector<pcl::P
//   // split_points_MultiEuc(g_not_ground_pc);

//   EuclideanAdaptiveClusterMethod(g_not_ground_pc, cluster_indices);
//   // EucClusterMethod(g_not_ground_pc, cluster_indices, 0.3);
//   int j = 0;
//   int clusterId = 0;
//   pcl::PointCloud<pcl::PointXYZ>::Ptr final_cluster (new pcl::PointCloud<pcl::PointXYZ>);
//   //for(pcl::PointIndices getIndices: cluster_indices){}
//   // for( int i = 0;i<cluster_indices.size(); ++i){
//   for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
//   {
//     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
//     //for(int index : getIndices.indices)
//     // intensity = 0.0;
//     intensity_count=0;
//     for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
//       //cloudCluster->point.push_bach(cloud->points[index]);
//     {
//         // intensity_count++;
//       // cloud_cluster->points.push_back (g_not_ground_pc->points[*pit]); //*
//       cloud_cluster->points.push_back (g_not_ground_pc->points[*pit]); //*
//       // intensity += g_not_ground_pc->points[*pit].intensity;
//     }
//     // intensity = intensity / cloud_cluster->points.size();
//     cloud_cluster->width = cloud_cluster->points.size ();
//     cloud_cluster->height = 1;
//     cloud_cluster->is_dense = true;
//     Eigen::Vector4f centroid;
//     pcl::compute3DCentroid(*cloud_cluster,centroid);

//     pcl::PointXYZ min;
//     pcl::PointXYZ max;
//     pcl::getMinMax3D(*cloud_cluster,min,max);
//     double xx = (double)std::fabs(max.x-min.x); 
//     double yy = (double)std::fabs(max.y-min.y);
//     double zz = (double)std::fabs(max.z-min.z);
//     if( (xx < 0.4 && yy < 0.4 && zz < 1 && (zz > 0.1 || centroid[2]>0.05) && centroid[2]< 0.3 && centroid[2] > -0.1) )//x
//     // if( (xx < 0.4 && yy > 0.05 && yy < 0.4 && zz < 1  && centroid[2]< 0.6 && centroid[2] > -0.1) )//x
//     {
//         *final_cluster = *final_cluster + *cloud_cluster;
//         pcl::toROSMsg(*cloud_cluster, cluster_for_publish);
//         cluster_for_publish.header = in_pc.header;
//         position.pc.push_back(cluster_for_publish);

//         geometry_msgs::Point32 tmp;
//         tmp.x=centroid[0];
//         tmp.y=centroid[1];
//         tmp.z=centroid[2];
//         position.points.push_back(tmp);//cone position of centroid published
//         if(vis_ && vis_init_status){
//             euc = float(sqrt(centroid[0]*centroid[0] +centroid[1]*centroid[1]));
//             vis_for_marker(max,min,euc);
//         }
//     }
//     else
//         continue;
//   // std::cout<<clusterId<<"Cluster_Centroid"<<"("
//   // << centroid[0] << ","
//   // << centroid[1] << ","
//   // << centroid[2] << ")."<<std::endl;
//   ++clusterId;
//   ++j;
//   // geometry_msgs::Point32 tmp;
//   // pcl::PointXYZ tmp_pc;
//   // tmp.x=centroid[0];
//   // tmp.y=centroid[1];
//   // tmp.z=centroid[2];
//   pcl::PointXYZ tmp_pXYZ;
//   tmp_pXYZ.x = centroid[0];
//   tmp_pXYZ.y = centroid[1];
//   tmp_pXYZ.z = centroid[2];
//   // tmp_pc.x=centroid[0];
//   // tmp_pc.y=centroid[1];
//   // tmp_pc.z=centroid[2];
//   skidpad_detection_pc->points.push_back(tmp_pXYZ);
//   // orderCloud.insert(tmp_pc);

//   // out_pc.points.push_back(tmp);//only centroid points
//   }
// // }
  
// //gettimeofday(&end,NULL);

// markerPub.publish(marker_array);
//     //sensor_msgs::PointCloud2 pub_pc;

// pcl::toROSMsg(*final_cluster, pub_pc);
// pub_pc.header = in_pc.header;
// pub_filtered_points__.publish(pub_pc);

// // markerPub.publish(marker_array);

// pcl::toROSMsg(*skidpad_detection_pc, pub_pc);
// pub_pc.header = in_pc.header;
// skidpad_detection.publish(pub_pc);

// cone_position.publish(position);

// }