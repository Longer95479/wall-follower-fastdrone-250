#ifndef _WALLFOLLOWER_H_
#define _WALLFOLLOWER_H_

#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>
#include <Eigen/Eigen>
#include <plan_env/grid_map.h>
#include <queue>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

class WallFollower {
public:
    typedef std::shared_ptr<WallFollower> Ptr;

    WallFollower(ros::NodeHandle& nh, GridMap::Ptr& grid_map_ptr);
	~WallFollower(){}


private:
    struct PtsEndFov {
        /* param */
        double f_;
        double deltaY_, deltaZ_;        // width and height in metric unit
        double X_;                      // depth
        Eigen::Matrix3d R_turn_round_;
        ros::Publisher pts_end_fov_pub_;// only for test

        /* data */
        int width_idx_, height_idx_;
        double cx_, cy_;
        std::vector<Eigen::Vector3d> pts_end_body_;
        std::vector<Eigen::Vector3d> pts_end_world_;

        typedef std::shared_ptr<PtsEndFov> Ptr;

        PtsEndFov(ros::NodeHandle& nh);
        void publicPtsEndFov();
    };

    struct PlaneFitter {
        /* data */
        std::vector<Eigen::Vector3d> occupied_pts;

        typedef std::shared_ptr<PlaneFitter> Ptr;

        PlaneFitter(ros::NodeHandle& nh);
    };

    PtsEndFov::Ptr pts_end_fov_ptr_;
    PlaneFitter::Ptr plane_fitter_ptr_;
    
    GridMap::Ptr grid_map_ptr_;

    ros::Timer vis_timer_, find_waypoint_timer_;

    void ptsEndFovGeneration();
    Eigen::Vector3d planeFitting();
    bool findNextWayPoint();
    void visCallback(const ros::TimerEvent& /*event*/);
    void findWayPointCallback(const ros::TimerEvent& /*event*/);

};


#endif
