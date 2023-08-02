#include <utility.h>
#include <set>


bool point_cmp(pcl::PointXYZ a, pcl::PointXYZ b)
{
    return a.z < b.z;
}

void lidar_cluster::ground_segmentation(const pcl::PointCloud<PointType>::Ptr &in_pc,
                                        pcl::PointCloud<PointType>::Ptr &g_not_ground_pc){
  // 1.Msg to pointcloud
    pcl::PointCloud<pcl::PointXYZ> laserCloudIn , laserCloudIn_org;
    laserCloudIn = laserCloudIn_org = *in_pc;
    //cloud = *cloud_Ptr;
    //cloud_Ptr = cloud.makeShared();
    // For mark ground points and hold all points
    pcl::PointXYZ point;
    g_all_pc = laserCloudIn.makeShared();
    //std::vector<int> indices;
    //pcl::removeNaNFromPointCloud(laserCloudIn, laserCloudIn,indices);
    // 2.Sort on Z-axis value.
    sort(laserCloudIn.points.begin(), laserCloudIn.end(), point_cmp);
    // 3.Error point removal
    // As there are some error mirror reflection under the ground,
    // here regardless point under 2* sensor_height
    // Sort point according to height, here uses z-axis in default
    pcl::PointCloud<pcl::PointXYZ>::iterator it = laserCloudIn.points.begin();
    for (int i = 0; i < laserCloudIn.points.size(); i++)
    {
        if (laserCloudIn.points[i].z < -1 * sensor_height_)//0.25
        {
            it++;
        }
        else
        {
            break;
        }
    }
    //remove below sensor_height_* - 1.5 points
    laserCloudIn.points.erase(laserCloudIn.points.begin(), it);
    // 4. Extract init ground seeds.
    extract_initial_seeds_(laserCloudIn);
    g_ground_pc = g_seeds_pc;
    // 5. Ground plane fitter mainloop
    for (int i = 0; i < num_iter_; i++)
    {
        estimate_plane_();
        g_ground_pc->clear();
        g_not_ground_pc->clear();

        //pointcloud to matrix
        MatrixXf points(laserCloudIn_org.points.size(), 3);
        int j = 0;
        for (auto p : laserCloudIn_org.points)
        {
            points.row(j++) << p.x, p.y, p.z;
        }
        // ground plane model
        VectorXf result = points * normal_;
        // threshold filter
        for (int r = 0; r < result.rows(); r++)
        {
            if (result[r] < th_dist_d_)
            {
                //g_all_pc->points[r].label = 1u; // means ground
                g_ground_pc->points.push_back(laserCloudIn_org[r]);
            }
            else
            {
                //g_all_pc->points[r].label = 0u; // means not ground and non clusterred
                g_not_ground_pc->points.push_back(laserCloudIn_org[r]);
            }
        }
    }
    g_all_pc->clear();
}


void lidar_cluster::extract_initial_seeds_(const pcl::PointCloud<pcl::PointXYZ> &p_sorted)
{
    // LPR is the mean of low point representative
    double sum = 0;
    int cnt = 0;
    // Calculate the mean height value.
    for (int i = 0; i < p_sorted.points.size() && cnt < num_lpr_; i++)
    {
        sum += p_sorted.points[i].z;
        cnt++;
    }
    double lpr_height = cnt != 0 ? sum / cnt : 0; // in case divide by 0
    g_seeds_pc->clear();
    // iterate pointcloud, filter those height is less than lpr.height+th_seeds_
    for (int i = 0; i < p_sorted.points.size(); i++)
    {
        if (p_sorted.points[i].z < lpr_height + th_seeds_)
        {
            g_seeds_pc->points.push_back(p_sorted.points[i]);
        }
    }
    // return seeds points
}


void lidar_cluster::estimate_plane_(void)
{
    // Create covarian matrix in single pass.
    // TODO: compare the efficiency.
    Eigen::Matrix3f cov;
    Eigen::Vector4f pc_mean;
    pcl::computeMeanAndCovarianceMatrix(*g_ground_pc, cov, pc_mean);
    // Singular Value Decomposition: SVD
    JacobiSVD<MatrixXf> svd(cov, Eigen::DecompositionOptions::ComputeFullU);
    // use the least singular vector as normal
    normal_ = (svd.matrixU().col(2));
    // mean ground seeds value
    Eigen::Vector3f seeds_mean = pc_mean.head<3>();

    // according to normal.T*[x,y,z] = -d
    d_ = -(normal_.transpose() * seeds_mean)(0, 0);
    // set distance threhold to `th_dist - d`
    th_dist_d_ = th_dist_ - d_;

    // return the equation parameters
}