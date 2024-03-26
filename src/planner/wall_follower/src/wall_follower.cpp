#include "wall_follower/wall_follower.h"

#define M_PI    3.14159265358979323846  /* pi */

WallFollower::WallFollower(ros::NodeHandle& nh, GridMap::Ptr& grid_map_ptr)
{
    ROS_INFO("Wall follower init begin.");

    grid_map_ptr_ = grid_map_ptr;

    pts_end_fov_ptr_.reset(new PtsEndFov(nh));
    ptsEndFovGeneration();
    plane_fitter_ptr_.reset(new PlaneFitter(nh));

    odom_sub_ =
      nh.subscribe<nav_msgs::Odometry>("grid_map/odom", 10, &WallFollower::odomCallback, this);
    have_odom_ = false;

    find_waypoint_timer_ = nh.createTimer(ros::Duration(0.1), &WallFollower::findWayPointCallback, this);
    vis_timer_ = nh.createTimer(ros::Duration(0.2), &WallFollower::visCallback, this);    
}


void WallFollower::visCallback(const ros::TimerEvent& /*event*/)
{
    pts_end_fov_ptr_->publicPtsEndFov();
    plane_fitter_ptr_->publicOccupiedPts();
}


void WallFollower::findWayPointCallback(const ros::TimerEvent& /*event*/)
{
    findNextWayPoint();
}


void WallFollower::odomCallback(const nav_msgs::OdometryConstPtr &odom)
{
  body_pos_(0) = odom->pose.pose.position.x;
  body_pos_(1) = odom->pose.pose.position.y;
  body_pos_(2) = odom->pose.pose.position.z;

  body_r_m_ = Eigen::Quaterniond(odom->pose.pose.orientation.w,
                                       odom->pose.pose.orientation.x,
                                       odom->pose.pose.orientation.y,
                                       odom->pose.pose.orientation.z).toRotationMatrix();

  have_odom_ = true;
}

bool WallFollower::findNextWayPoint()
{
    if (have_odom_) {
        std::vector<Eigen::Vector3d> pts_end, occupied_pts;
        Eigen::Matrix3d body_r_m;
        Eigen::Vector3d body_pos, ray_pt;
        Eigen::Vector3i id;
        double map_resolution;
        RayCaster raycaster;

        Eigen::Vector3d half = Eigen::Vector3d(0.5, 0.5, 0.5);

        pts_end = pts_end_fov_ptr_->pts_end_body_;

        body_r_m = body_r_m_;
        body_pos = body_pos_;

        map_resolution = grid_map_ptr_->mp_.resolution_;

        for (auto& pt_end: pts_end) {
            pt_end = body_r_m * pt_end + body_pos;

            raycaster.setInput(body_pos/map_resolution, pt_end/map_resolution);
            while (raycaster.step(ray_pt)) {
                Eigen::Vector3d tmp = (ray_pt + half) * map_resolution;
                grid_map_ptr_->posToIndex(tmp, id);
                if (grid_map_ptr_->isKnownOccupied(id)) {
                    occupied_pts.push_back(tmp);
                    break;
                }
            }
        }

        /* only for visual test, remember del or comment */
        pts_end_fov_ptr_->pts_end_world_ = pts_end;
        plane_fitter_ptr_->occupied_pts_ = occupied_pts;
    }
}


WallFollower::PtsEndFov::PtsEndFov(ros::NodeHandle& nh)
{
    nh.param("wall_follower/f", f_, 10.0);
    nh.param("wall_follower/deltaY", deltaY_, 4.0);
    nh.param("wall_follower/deltaZ", deltaZ_, 2.0);
    nh.param("wall_follower/X", X_, 2.0);

    Eigen::Matrix3d R_turn_round =  Eigen::Quaterniond(cos(-M_PI/4.0/2.0), 0, 0, sin(-M_PI/4.0/2.0)).toRotationMatrix();
    // Eigen::Matrix3d R_turn_round =  Eigen::Quaterniond(1, 0, 0, 0).toRotationMatrix();

    nh.param("wall_follower/R_turn_round_00", R_turn_round_(0,0), R_turn_round(0,0));
    nh.param("wall_follower/R_turn_round_01", R_turn_round_(0,1), R_turn_round(0,1));
    nh.param("wall_follower/R_turn_round_02", R_turn_round_(0,2), R_turn_round(0,2));
    nh.param("wall_follower/R_turn_round_10", R_turn_round_(1,0), R_turn_round(1,0));
    nh.param("wall_follower/R_turn_round_11", R_turn_round_(1,1), R_turn_round(1,1));
    nh.param("wall_follower/R_turn_round_12", R_turn_round_(1,2), R_turn_round(1,2));
    nh.param("wall_follower/R_turn_round_20", R_turn_round_(2,0), R_turn_round(2,0));
    nh.param("wall_follower/R_turn_round_21", R_turn_round_(2,1), R_turn_round(2,1));
    nh.param("wall_follower/R_turn_round_22", R_turn_round_(2,2), R_turn_round(2,2));

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
    Eigen::Vector3d pt_end;
    Eigen::Matrix3d R_turn_round;
    double X, f;
    int width_idx, height_idx;

    PtsEndFov::Ptr pts_end_fov_ptr;

    pts_end_fov_ptr = pts_end_fov_ptr_;

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
            pts_end_fov_ptr->pts_end_body_.push_back(R_turn_round * pt_end);
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
    occupied_pts_updated_ = false;

    occupied_pts_pub_ = nh.advertise<sensor_msgs::PointCloud2>("wall_follower/occupied_pts", 10);
}

void WallFollower::PlaneFitter::publicOccupiedPts()
{
    if (occupied_pts_pub_.getNumSubscribers() <= 0)
            return;

    pcl::PointXYZ pt;
    pcl::PointCloud<pcl::PointXYZ> cloud;
    std::vector<Eigen::Vector3d> occupied_pts;

    occupied_pts = occupied_pts_;

    for (auto& occupied_pt: occupied_pts) {
        pt.x = occupied_pt(0);
        pt.y = occupied_pt(1);
        pt.z = occupied_pt(2);
        cloud.push_back(pt);
    }

    cloud.width = cloud.points.size();
    cloud.height = 1;
    cloud.is_dense = true;
    cloud.header.frame_id = string("world");
    sensor_msgs::PointCloud2 cloud_msg;

    pcl::toROSMsg(cloud, cloud_msg);
    occupied_pts_pub_.publish(cloud_msg);
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

