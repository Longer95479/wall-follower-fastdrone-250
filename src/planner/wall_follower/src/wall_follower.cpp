#include "wall_follower/wall_follower.h"


WallFollower::WallFollower(ros::NodeHandle& nh, GridMap::Ptr& grid_map_ptr)
{
    ROS_INFO("Wall follower init begin.");

    grid_map_ptr_ = grid_map_ptr;

    pts_end_fov_ptr_.reset(new PtsEndFov(nh));
    pts_end_fov_ptr_->ptsEndFovGeneration();

    plane_fitter_ptr_.reset(new PlaneFitter(nh));

    pts_end_fov_pub_ = nh.advertise<sensor_msgs::PointCloud2>("wall_follower/pts_end_fov", 10);

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
    Eigen::Vector3d pts_end = pts_end_fov_ptr_->pts_end_body_;
    /* TODO: not use camera_r, use body_r */
    Eigen::Matrix3d camera_r = grid_map_ptr_->md_.camera_r_m_;

    for (auto& pt_end: pts_end) {
        pt_end = camera_r * pt_end;
    }

    /* only for test, remember del or comment */
    pts_end_fov_ptr_->pts_end_world_ = pts_end;
}


WallFollower::PtsEndFov::PtsEndFov(ros::NodeHandle& nh)
{
    nh.param("wall_follower/f", f_, 100);
    nh.param("wall_follower/deltaY", deltaY_, 1.0);
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

}


void WallFollower::PtsEndFov::ptsEndFovGeneration()
{
    Eigen::Vector3d pt_end;

    pt_end(0) = X_;
    for (int u = 0; u < width_idx_; u++) {
        for (int v = 0; v < height_idx_; v++) {
            pt_end(1) = -X_/f_ * (u - width/2);
            pt_end(2) = -X_/f_ * (v - height/2);
            pt_end = R_turn_round_ * pt_end;
            pts_end_body_.push_back(pt_end);
        }
    }
    pts_end_world_ = pts_end_body_;

}


WallFollower::PlaneFitter::PlaneFitter(ros::NodeHandle& nh)
{

}


Eigen::Vector3d WallFollower::PlaneFitter::planeFitting(std::vector<Eigen::Vector3d>& pts_end_body)
{
    return Eigen::Vector3d(1,2,3);
}


void WallFollower::PtsEndFov::publicPtsEndFov()
{
    if (pts_end_fov_pub_.getNumSubscribers() <= 0)
        return;

    pcl::PointXYZ pt;
    pcl::PointCloud<pcl::PointXYZ> cloud;

    pts_end_world = pts_end_fov_ptr_->pts_end_world_;

    for (auto& pt_end_world: pts_end_world) {
        pt.x = pt_end_world(0);
        pt.y = pt_end_world(1);
        pt.z = pt_end_world(2);
    }

    cloud.width = cloud.points.size();
    cloud.height = 1;
    cloud.is_dense = true;
    cloud.header.frame_id = grid_map_ptr->mp_.frame_id_;
    sensor_msgs::PointCloud2 cloud_msg;

    pcl::toROSMsg(cloud, cloud_msg);
    pts_end_fov_pub_.publish(cloud_msg);

}

