#include "wall_follower/wall_follower.h"


WallFollower::WallFollower(ros::NodeHandle& nh, GridMap::Ptr& grid_map_ptr)
{
    ROS_INFO("Wall follower init begin.");

    grid_map_ptr_ = grid_map_ptr;

    pts_end_fov_ptr_.reset(new PtsEndFov(nh));
    ptsEndFovGeneration();
    plane_fitter_ptr_.reset(new PlaneFitter(nh));

    find_waypoint_timer_ = nh.createTimer(ros::Duration(0.1), &WallFollower::findWayPointCallback, this);
    vis_timer_ = nh.createTimer(ros::Duration(0.2), &WallFollower::visCallback, this);    
}


void WallFollower::visCallback(const ros::TimerEvent& /*event*/)
{
    pts_end_fov_ptr_->publicPtsEndFov();
}


void WallFollower::findWayPointCallback(const ros::TimerEvent& /*event*/)
{
    findNextWayPoint();
}


bool WallFollower::findNextWayPoint()
{
    if (grid_map_ptr_->md_.has_odom_) {
        std::vector<Eigen::Vector3d> pts_end;
        Eigen::Matrix3d camera_r_m;
        Eigen::Vector3d camera_pos;

        pts_end = pts_end_fov_ptr_->pts_end_body_;

        camera_r_m = grid_map_ptr_->md_.camera_r_m_;
        camera_pos = grid_map_ptr_->md_.camera_pos_;
        std::cout << "camera_pos = " << camera_pos.transpose() << std::endl;
        std::cout << "camera_r_m = " << std::endl << camera_r_m << std::endl;

        for (auto& pt_end: pts_end) {
            pt_end = camera_r_m * pt_end + camera_pos;
        }

        /* only for visual test, remember del or comment */
        pts_end_fov_ptr_->pts_end_world_ = pts_end;
    }
}


WallFollower::PtsEndFov::PtsEndFov(ros::NodeHandle& nh)
{
    nh.param("wall_follower/f", f_, 5.0);
    nh.param("wall_follower/deltaY", deltaY_, 2.0);
    nh.param("wall_follower/deltaZ", deltaZ_, 1.0);
    nh.param("wall_follower/X", X_, 1.0);

    nh.param("wall_follower/R_turn_round_00", R_turn_round_(0,0), 1.0);
    nh.param("wall_follower/R_turn_round_01", R_turn_round_(0,1), 0.0);
    nh.param("wall_follower/R_turn_round_02", R_turn_round_(0,2), 0.0);
    nh.param("wall_follower/R_turn_round_10", R_turn_round_(1,0), 0.0);
    nh.param("wall_follower/R_turn_round_11", R_turn_round_(1,1), 1.0);
    nh.param("wall_follower/R_turn_round_12", R_turn_round_(1,2), 0.0);
    nh.param("wall_follower/R_turn_round_20", R_turn_round_(2,0), 0.0);
    nh.param("wall_follower/R_turn_round_21", R_turn_round_(2,1), 0.0);
    nh.param("wall_follower/R_turn_round_22", R_turn_round_(2,2), 1.0);

    width_idx_ = (int)std::ceil(f_/X_ * deltaY_);
    height_idx_ = (int)std::ceil(f_/X_ * deltaZ_);

    cx_ = width_idx_ / 2.0;
    cy_ = height_idx_ / 2.0;

    std::cout << std::endl;
    std::cout << "wall follower param and data info: " << std::endl;
    std::cout << "f_ = " << f_ << std::endl;
    std::cout << "deltaY_ = " << deltaY_ << std::endl;
    std::cout << "deltaZ_ = " << deltaZ_ << std::endl;
    std::cout << "R_turn_round_ = " << R_turn_round_ << std::endl;
    std::cout << "width_idx_ = " << width_idx_ << std::endl;
    std::cout << "height_idx_ = " << height_idx_ << std::endl;
    std::cout << std::endl;
    

    pts_end_fov_pub_ = nh.advertise<sensor_msgs::PointCloud2>("wall_follower/pts_end_fov", 10);

}


void WallFollower::ptsEndFovGeneration()
{
    Eigen::Vector3d pt_end, camera_pos;
    Eigen::Matrix3d camera_r_m, R_turn_round;
    double X, f;
    int width_idx, height_idx;

    PtsEndFov::Ptr pts_end_fov_ptr;
    GridMap::Ptr grid_map_ptr;

    pts_end_fov_ptr = pts_end_fov_ptr_;
    grid_map_ptr = grid_map_ptr_;

    // camera_r_m = grid_map_ptr->md_.camera_r_m_;
    // camera_pos = grid_map_ptr_->md_.camera_pos_;

    X = pts_end_fov_ptr->X_;
    width_idx = pts_end_fov_ptr->width_idx_;
    height_idx = pts_end_fov_ptr->height_idx_;
    f = pts_end_fov_ptr->f_;
    R_turn_round = pts_end_fov_ptr->R_turn_round_;

    pt_end(0) = X;
    for (int u = 0; u < width_idx; u++) {
        for (int v = 0; v < height_idx; v++) {
            pt_end(1) = -X/f * (u - width_idx/2);
            pt_end(2) = -X/f * (v - height_idx/2);
            pt_end = R_turn_round * pt_end;
            // pt_end = camera_r_m * pt_end + camera_pos;
            pts_end_fov_ptr->pts_end_body_.push_back(pt_end);
            // std::cout << "pt_end = " <<  pt_end << std::endl;
        }
    }

    pts_end_fov_ptr->pts_end_world_ = pts_end_fov_ptr->pts_end_body_;

    // std::cout << std::endl;
    // std::cout << "pts_end_body_.size() = " << pts_end_body_.size() << std::endl;
    // std::cout << "pts_end_world_.size() = " << pts_end_world_.size() << std::endl;
    // std::cout << std::endl;

}


WallFollower::PlaneFitter::PlaneFitter(ros::NodeHandle& nh)
{

}


Eigen::Vector3d WallFollower::planeFitting()
{
    return Eigen::Vector3d(1,2,3);
}


void WallFollower::PtsEndFov::publicPtsEndFov()
{
    if (pts_end_fov_pub_.getNumSubscribers() <= 0)
        return;

    pcl::PointXYZ pt;
    pcl::PointCloud<pcl::PointXYZ> cloud;
    std::vector<Eigen::Vector3d> pts_end_world;

    pts_end_world = pts_end_world_;

    for (auto& pt_end_world: pts_end_world) {
        pt.x = pt_end_world(0);
        pt.y = pt_end_world(1);
        pt.z = pt_end_world(2);
        cloud.push_back(pt);
    }

    cloud.width = cloud.points.size();
    cloud.height = 1;
    cloud.is_dense = true;
    cloud.header.frame_id = string("world");
    sensor_msgs::PointCloud2 cloud_msg;

    pcl::toROSMsg(cloud, cloud_msg);
    pts_end_fov_pub_.publish(cloud_msg);

}

