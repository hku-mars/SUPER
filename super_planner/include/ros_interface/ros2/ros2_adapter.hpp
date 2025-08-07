/**
* This file is part of SUPER
*
* Copyright 2025 Yunfan REN, MaRS Lab, University of Hong Kong, <mars.hku.hk>
* Developed by Yunfan REN <renyf at connect dot hku dot hk>
* for more information see <https://github.com/hku-mars/SUPER>.
* If you use this code, please cite the respective publications as
* listed on the above website.
*
* SUPER is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* SUPER is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with SUPER. If not, see <http://www.gnu.org/licenses/>.
*/


#ifdef USE_ROS2
#ifndef SRC_ROS2_ADAPTER_HPP
#define SRC_ROS2_ADAPTER_HPP

#include "rclcpp/rclcpp.hpp"
#include "ros_interface/ros_interface.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "nav_msgs/msg/path.hpp"
#include "data_structure/base/trajectory.h"
#include "utils/header/color_text.hpp"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include <random>
#include "mars_quadrotor_msgs/msg/position_command.hpp"
#include "mars_quadrotor_msgs/msg/polynomial_trajectory.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "rosgraph_msgs/msg/clock.hpp"

namespace ros_interface {
    using namespace geometry_utils;
    using namespace color_text;

    using MarkerArrayPubPtr = rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr;
    using MarkerPubPtr = rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr;
    using PointCloud2PubPtr = rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr;
    using PathPubPtr = rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr;
    using PositionCommandPubPtr = rclcpp::Publisher<mars_quadrotor_msgs::msg::PositionCommand>::SharedPtr;
    using PolynomialTrajectoryPubPtr = rclcpp::Publisher<mars_quadrotor_msgs::msg::PolynomialTrajectory>::SharedPtr;

    const std::string DEFAULT_FRAME_ID = "world";

    class Ros1Adapter {
    public:
        /* For visualization ============================================================*/
        static void deleteAllMarkerArray(const MarkerArrayPubPtr &pub_) {
            visualization_msgs::msg::Marker del;
            visualization_msgs::msg::MarkerArray arr;
            del.action = visualization_msgs::msg::Marker::DELETEALL;
            arr.markers.push_back(del);
            pub_->publish(arr);
        }

        static void deleteAllMarker(const MarkerPubPtr &pub_) {
            visualization_msgs::msg::Marker del;
            del.action = visualization_msgs::msg::Marker::DELETEALL;
            pub_->publish(del);
        }

        static void addVecPointsToPointCloud2(const vec_Vec3f &points, sensor_msgs::msg::PointCloud2 &pc2) {
            pcl::PointCloud<pcl::PointXYZ> cloud;
            for (const auto &p: points) {
                pcl::PointXYZ pt;
                pt.x = p.x();
                pt.y = p.y();
                pt.z = p.z();
                cloud.push_back(pt);
            }
            pcl::toROSMsg(cloud, pc2);
            pc2.header.frame_id = DEFAULT_FRAME_ID;
            auto sim_clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
            pc2.header.stamp = sim_clock->now();
        }

        static void addPathToMarkerArray(visualization_msgs::msg::MarkerArray &mkr_ary,
                                         const vec_E<Vec3f> &path,
                                         Color color,
                                         string ns,
                                         double pt_size,
                                         double line_size) {
            visualization_msgs::msg::Marker line_list;
            if (path.size() <= 0) {
                std::cout << YELLOW << "Try to publish empty path, return.\n" << RESET << std::endl;
                return;
            }
            Vec3f cur_pt = path[0], last_pt;
            static int point_id = 0;
            static int line_cnt = 0;
            auto sim_clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
            for (size_t i = 0; i < path.size(); i++) {
                last_pt = cur_pt;
                cur_pt = path[i];

                /* Publish point */
                visualization_msgs::msg::Marker point;
                point.header.frame_id = DEFAULT_FRAME_ID;
                auto sim_clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
                point.header.stamp = sim_clock->now();
                point.ns = ns.c_str();
                point.id = point_id++;
                point.action = visualization_msgs::msg::Marker::ADD;
                point.pose.orientation.w = 1.0;
                point.type = visualization_msgs::msg::Marker::SPHERE;
                // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
                point.scale.x = pt_size;
                point.scale.y = pt_size;
                point.scale.z = pt_size;
                // Line list is blue
                point.color = color;
                point.color.a = 1.0;
                // Create the vertices for the points and lines
                geometry_msgs::msg::Point p;
                p.x = cur_pt.x();
                p.y = cur_pt.y();
                p.z = cur_pt.z();
                point.pose.position = p;
                mkr_ary.markers.push_back(point);
                /* publish lines */
                if (i > 0) {
                    geometry_msgs::msg::Point p;
                    // publish lines
                    visualization_msgs::msg::Marker line_list;
                    line_list.header.frame_id = DEFAULT_FRAME_ID;
                    line_list.header.stamp = sim_clock->now();
                    line_list.ns = ns + "_line";
                    line_list.id = line_cnt++;
                    line_list.action = visualization_msgs::msg::Marker::ADD;
                    line_list.pose.orientation.w = 1.0;
                    line_list.type = visualization_msgs::msg::Marker::LINE_LIST;
                    // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
                    line_list.scale.x = line_size;
                    // Line list is blue
                    line_list.color = color;
                    // Create the vertices for the points and lines

                    p.x = last_pt.x();
                    p.y = last_pt.y();
                    p.z = last_pt.z();
                    // The line list needs two points for each line
                    line_list.points.push_back(p);
                    p.x = cur_pt.x();
                    p.y = cur_pt.y();
                    p.z = cur_pt.z();
                    // The line list needs
                    line_list.points.push_back(p);

                    mkr_ary.markers.push_back(line_list);
                }
            }
        }

        static void addTrajectoryToMarkerArray(
                visualization_msgs::msg::MarkerArray &mkr_arr,
                const Trajectory &traj,
                const std::string &name_space,
                const Color &color = Color::Chartreuse(),
                const double &size = 0.1,
                const bool &show_waypoint = false,
                const bool &color_in_vel = false,
                const tinycolormap::ColormapType &cmp = tinycolormap::ColormapType::Jet) {
            // * 1) Visualize the color
            double t_sum = traj.getTotalDuration();
            auto sim_clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
            if (t_sum < 1e-2 || std::isnan(t_sum) || traj.empty()) {
                std::cout << REDPURPLE << "[Trajectory::Visualize] ERROR, the total duration is too small" << RESET
                          << std::endl;
                return;
            }
            double eval_t = 0.001;
            Vec3f cur_vel_dir, last_pos, cur_pos, end_point;
            Vec3f cur_vel;
            visualization_msgs::msg::Marker line_list;
            double vel_max = traj.getMaxVelRate();
            double map_factor = 1.0 / vel_max;
            last_pos = traj.getPos(0);
            line_list.header.frame_id = DEFAULT_FRAME_ID;
            line_list.header.stamp = sim_clock->now();
            line_list.ns = name_space;
            line_list.action = visualization_msgs::msg::Marker::ADD;
            line_list.pose.orientation.w = 1.0;
            line_list.type = visualization_msgs::msg::Marker::ARROW;
            // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
            line_list.scale.x = size;
            line_list.scale.y = 0.0000001;
            line_list.scale.z = 0.0000001;
            while (eval_t + 1e-4 < t_sum) {
                cur_pos = traj.getPos(eval_t);
                if ((cur_pos - last_pos).norm() < size / 2.00) {
                    eval_t += 0.03;
                    continue;
                }
                cur_vel = traj.getVel(eval_t);
                {

                    // publish lines
                    static int cnt = 0;
                    line_list.id = cnt++;
                    line_list.points.clear();
                    // Line list is blue
                    Eigen::Vector3d color_mag;

                    if (color_in_vel) {
                        double color_id = ((cur_vel.norm() * map_factor));
                        color_mag = tinycolormap::GetColor(color_id, cmp).ConvertToEigen();
                    } else {
                        color_mag.x() = color.r;
                        color_mag.y() = color.g;
                        color_mag.z() = color.b;
                    }

                    line_list.color.r = static_cast<float>(color_mag[0]);
                    line_list.color.g = static_cast<float >(color_mag[1]);
                    line_list.color.b = static_cast<float >(color_mag[2]);
                    line_list.color.a = 1;
                    // Create the vertices for the points and lines

                    geometry_msgs::msg::Point p;
                    p.x = last_pos.x();
                    p.y = last_pos.y();
                    p.z = last_pos.z();
                    // The line list needs two points for each line
                    line_list.points.push_back(p);
                    p.x = cur_pos.x();
                    p.y = cur_pos.y();
                    p.z = cur_pos.z();
                    // The line list needs
                    line_list.points.push_back(p);

                    mkr_arr.markers.push_back(line_list);
                }
                last_pos = cur_pos;
                eval_t += 0.03;
            }
            // * 2) Publish the waypoint
            if (show_waypoint) {
                visualization_msgs::msg::Marker marker_ball;
                static int ball_id = 0;
                marker_ball.header.frame_id = DEFAULT_FRAME_ID;
                marker_ball.header.stamp = sim_clock->now();
                marker_ball.ns = name_space + "_waypoints";
                marker_ball.action = visualization_msgs::msg::Marker::ADD;
                marker_ball.pose.orientation.w = 1.0;
                marker_ball.type = visualization_msgs::msg::Marker::SPHERE;
                marker_ball.scale.x = size * 3;
                marker_ball.scale.y = size * 3;
                marker_ball.scale.z = size * 3;
                marker_ball.color = color;
                marker_ball.color.a = 1.0;
                geometry_msgs::msg::Point p;
                for (size_t i = 0; i < traj.size() - 1; i++) {
                    cur_pos = traj[i].getPos(traj[i].getDuration());
                    p.x = cur_pos.x();
                    p.y = cur_pos.y();
                    p.z = cur_pos.z();
                    marker_ball.pose.position = p;
                    marker_ball.id = ball_id++;
                    mkr_arr.markers.push_back(marker_ball);
                }
            }
        }


        /// Sort a set of points in a plane.
        static vec_E<Vec3f> SortPtsInClockWise(vec_E<Vec3f> &pts, Vec3f normal) {
            Vec3f center(0, 0, 0);
            for (auto pt: pts) {
                center += pt;
            }
            center = center / pts.size();
            Eigen::Quaterniond q = Eigen::Quaterniond::FromTwoVectors(Vec3f(0, 0, 1), normal);
            center = q.matrix() * center;
            std::vector<std::pair<double, Vec3f>> pts_valued;
            pts_valued.resize(pts.size());
            Vec3f temp_p;
            for (size_t i = 0; i < pts.size(); i++) {
                temp_p = q.matrix() * pts[i];
                double theta = atan2(temp_p(1) - center(1), temp_p(0) - center(0));
                pts_valued[i] = std::make_pair(theta, pts[i]);
            }

            std::sort(
                    pts_valued.begin(), pts_valued.end(),
                    [](const std::pair<double, Vec3f> &i,
                       const std::pair<double, Vec3f> &j) { return i.first < j.first; });
            vec_E<Vec3f> pts_sorted(pts_valued.size());
            for (size_t i = 0; i < pts_valued.size(); i++)
                pts_sorted[i] = pts_valued[i].second;
            return pts_sorted;
        }

        static void addPolytopeToMarkerArray(visualization_msgs::msg::MarkerArray &mkr_arr,
                                             const Polytope &poly,
                                             const std::string &ns, const bool &use_random_color,
                                             const Color &surf_color, const Color &edge_color,
                                             const Color &vertex_color,
                                             const double &alpha, const double &edge_width) {
            const auto &planes = poly.GetPlanes();
            const auto &have_seed_line = poly.HaveSeedLine();
            const auto &seed_line = poly.seed_line;
            auto sim_clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
            if (std::isnan(planes.sum())) {
                printf(" -- [Poly] ERROR, try to visualize polytope with NaN, force return.\n");
                return;
            }
            // Due to the fact that H-representation cannot be directly visualized
            // We first conduct vertex enumeration of them, then apply quickhull
            // to obtain triangle meshs of polyhedra
            Eigen::Matrix3Xd mesh(3, 0), curTris(3, 0), oldTris(3, 0);
            oldTris = mesh;
            Eigen::Matrix<double, 3, -1, Eigen::ColMajor> vPoly;

            if (!geometry_utils::enumerateVs(planes, vPoly)) {
                printf(" -- [WARN] Trying to publish ill polytope.\n");
                return;
            }

            geometry_utils::QuickHull<double> tinyQH;
            const auto polyHull = tinyQH.getConvexHull(vPoly.data(), vPoly.cols(), false, true);
            const auto &idxBuffer = polyHull.getIndexBuffer();
            int hNum = idxBuffer.size() / 3;

            curTris.resize(3, hNum * 3);
            for (int i = 0; i < hNum * 3; i++) {
                curTris.col(i) = vPoly.col(idxBuffer[i]);
            }
            mesh.resize(3, oldTris.cols() + curTris.cols());
            mesh.leftCols(oldTris.cols()) = oldTris;
            mesh.rightCols(curTris.cols()) = curTris;

            if (std::isnan(mesh.sum())) {
                printf(" -- [WARN] Trying to publish ill polytope.\n");
                return;
            }

            // RVIZ support tris for visualization
            visualization_msgs::msg::Marker meshMarker, edgeMarker, seedMarker;
            static int mesh_id = 0;
            static int edge_id = 0;

            static std::random_device rd;
            static std::mt19937 mt(rd());
            static std::uniform_real_distribution<double> dist(0.0, 1.0);
            Color random_color(dist(mt), dist(mt), dist(mt));
            if (have_seed_line) {
                /// * 1) Publish Seed line
                static int point_id = 0;
                static int line_cnt = 0;
                /* Publish point */
                visualization_msgs::msg::Marker point;
                point.header.frame_id = DEFAULT_FRAME_ID;
                point.header.stamp = sim_clock->now();
                point.ns = ns + " pt";
                point.id = point_id++;
                point.action = visualization_msgs::msg::Marker::ADD;
                point.pose.orientation.w = 1.0;
                point.type = visualization_msgs::msg::Marker::SPHERE;
                // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
                point.scale.x = 0.2;
                point.scale.y = 0.2;
                point.scale.z = 0.2;
                // Line list is blue
                if (use_random_color) {
                    point.color = random_color;// Color::Blue();
                } else {
                    point.color = edge_color;
                }

                point.color.a = 1.0;
                // Create the vertices for the points and lines
                geometry_msgs::msg::Point p;
                p.x = seed_line.first.x();
                p.y = seed_line.first.y();
                p.z = seed_line.first.z();
                point.pose.position = p;
                mkr_arr.markers.push_back(point);

                p.x = seed_line.second.x();
                p.y = seed_line.second.y();
                p.z = seed_line.second.z();
                point.pose.position = p;
                point.id = point_id++;
                mkr_arr.markers.push_back(point);

                // publish lines
                visualization_msgs::msg::Marker line_list;
                line_list.header.frame_id = DEFAULT_FRAME_ID;
                line_list.header.stamp = sim_clock->now();
                line_list.ns = ns + " line";
                line_list.id = line_cnt++;
                line_list.action = visualization_msgs::msg::Marker::ADD;
                line_list.pose.orientation.w = 1.0;
                line_list.type = visualization_msgs::msg::Marker::LINE_LIST;
                // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
                line_list.scale.x = 0.1;
                // Line list is blue
                if (use_random_color) {
                    line_list.color = random_color;// Color::Blue();
                } else {
                    line_list.color = edge_color;
                }
                line_list.color.a = 1.0;
                // Create the vertices for the points and lines

                p.x = seed_line.first.x();
                p.y = seed_line.first.y();
                p.z = seed_line.first.z();
                // The line list needs two points for each line
                line_list.points.push_back(p);
                p.x = seed_line.second.x();
                p.y = seed_line.second.y();
                p.z = seed_line.second.z();
                // The line list needs
                line_list.points.push_back(p);

                mkr_arr.markers.push_back(line_list);
            }

            geometry_msgs::msg::Point point;
            meshMarker.id = mesh_id++;
            meshMarker.header.stamp = sim_clock->now();
            meshMarker.header.frame_id = DEFAULT_FRAME_ID;
            meshMarker.pose.orientation.w = 1.00;
            meshMarker.action = visualization_msgs::msg::Marker::ADD;
            meshMarker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
            meshMarker.ns = ns + " mesh";
            meshMarker.color = surf_color;
            meshMarker.color.a = alpha;
            meshMarker.scale.x = 1.0;
            meshMarker.scale.y = 1.0;
            meshMarker.scale.z = 1.0;

            edgeMarker = meshMarker;
            edgeMarker.header.stamp = sim_clock->now();
            edgeMarker.header.frame_id = DEFAULT_FRAME_ID;
            edgeMarker.id = edge_id++;
            edgeMarker.type = visualization_msgs::msg::Marker::LINE_LIST;
            edgeMarker.ns = ns + " edge";
            if (use_random_color) {
                edgeMarker.color = random_color;// Color::Blue();
            } else {
                edgeMarker.color = edge_color;
            }
            edgeMarker.color.a = 1.00;
            edgeMarker.scale.x = edge_width;


            long unsigned int ptnum = mesh.cols();

            for (long unsigned int i = 0; i < ptnum; i++) {
                point.x = mesh(0, i);
                point.y = mesh(1, i);
                point.z = mesh(2, i);
                meshMarker.points.push_back(point);
            }


            /// 接下来遍历选择定点属于哪些面
            vec_E<Vec3f> edges;

            for (long int i = 0; i < planes.rows(); i++) {
                edges.clear();
                Eigen::VectorXd temp = (planes.row(i).head(3) * vPoly);
                Eigen::VectorXd d(temp.size());
                d.setConstant(planes(i, 3));
                temp = temp + d;
                for (int j = 0; j < temp.size(); j++) {
                    if (std::abs(temp(j)) < super_utils::epsilon_) {
                        edges.emplace_back(vPoly.col(j));
                    }
                }
                if (edges.size() < 2) {
                    continue;
                }

                vec_E<Vec3f> pts_sorted = SortPtsInClockWise(edges, planes.row(i).head(3));
                int pts_num = pts_sorted.size();
                for (int k = 0; k < pts_num - 1; k++) {
                    point.x = pts_sorted[k].x();
                    point.y = pts_sorted[k].y();
                    point.z = pts_sorted[k].z();
                    edgeMarker.points.push_back(point);
                    point.x = pts_sorted[k + 1].x();
                    point.y = pts_sorted[k + 1].y();
                    point.z = pts_sorted[k + 1].z();
                    edgeMarker.points.push_back(point);
                }
                point.x = pts_sorted[0].x();
                point.y = pts_sorted[0].y();
                point.z = pts_sorted[0].z();
                edgeMarker.points.push_back(point);
                point.x = pts_sorted[pts_num - 1].x();
                point.y = pts_sorted[pts_num - 1].y();
                point.z = pts_sorted[pts_num - 1].z();
                edgeMarker.points.push_back(point);
            }
            mkr_arr.markers.push_back(meshMarker);
            mkr_arr.markers.push_back(edgeMarker);
        }

        static void addYawTrajectoryToMarkerArray(visualization_msgs::msg::MarkerArray &mkr_arr,
                                                  const Trajectory &pos_traj,
                                                  const Trajectory &yaw_traj,
                                                  string ns = "yaw_traj") {
            double t_sum = yaw_traj.getTotalDuration();
            double eval_t = 0.001;
            Vec3f cur_pos, end_point;
            tinycolormap::ColormapType cmp = tinycolormap::ColormapType::Jet;
            visualization_msgs::msg::Marker line_list;
            auto sim_clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
            while (eval_t + 1e-4 < t_sum) {
                visualization_msgs::msg::Marker line_list;
                cur_pos = pos_traj.getPos(eval_t);
                double cur_yaw = yaw_traj.getPos(eval_t)[0];
                {
                    static int cnt = 0;
                    // publish lines
                    visualization_msgs::msg::Marker line_list;
                    line_list.header.frame_id = "world";
                    line_list.header.stamp = sim_clock->now();
                    line_list.ns = ns;
                    line_list.id = cnt++;
                    line_list.action = visualization_msgs::msg::Marker::ADD;
                    line_list.pose.orientation.w = 1.0;
                    line_list.type = visualization_msgs::msg::Marker::ARROW;
                    // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
                    line_list.scale.x = 0.05;
                    line_list.scale.y = 0.1;
                    line_list.scale.z = 0.2;
                    // Line list is blue
                    // Line list is blue
                    double color_id = ((eval_t / t_sum));

                    Eigen::Vector3d color_mag = tinycolormap::GetColor(color_id, cmp).ConvertToEigen();
                    line_list.color.r = color_mag[0];
                    line_list.color.g = color_mag[1];
                    line_list.color.b = color_mag[2];
                    line_list.color.a = 1;
                    // Create the vertices for the points and lines
                    Vec3f p2 = cur_pos + Vec3f(cos(cur_yaw), sin(cur_yaw), 0);
                    geometry_msgs::msg::Point p;
                    p.x = cur_pos.x();
                    p.y = cur_pos.y();
                    p.z = cur_pos.z();
                    // The line list needs two points for each line
                    line_list.points.push_back(p);
                    p.x = p2.x();
                    p.y = p2.y();
                    p.z = p2.z();
                    // The line list needs
                    line_list.points.push_back(p);

                    mkr_arr.markers.push_back(line_list);
                }
                eval_t += 0.05;
            }
        }

        static void addBoundingBoxToMarkerArray(visualization_msgs::msg::MarkerArray &mkrarr,
                                                const Vec3f &box_min,
                                                const Vec3f &box_max,
                                                const string &ns,
                                                const Color &color,
                                                const double &size_x = 0.1,
                                                const double &alpha = 1.0,
                                                const bool &print_ns = true) {
            Vec3f size = (box_max - box_min) / 2;
            Vec3f vis_pos_world = (box_min + box_max) / 2;
            double width = size.x();
            double length = size.y();
            double hight = size.z();
            auto sim_clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
            //Publish Bounding box
            int id = 0;
            visualization_msgs::msg::Marker line_strip;
            line_strip.header.stamp = sim_clock->now();
            line_strip.header.frame_id = "world";
            line_strip.action = visualization_msgs::msg::Marker::ADD;
            line_strip.ns = ns;
            line_strip.pose.orientation.w = 1.0;
            line_strip.id = id++; //unique id, useful when multiple markers exist.
            line_strip.type = visualization_msgs::msg::Marker::LINE_STRIP; //marker type
            line_strip.scale.x = size_x;


            line_strip.color = color;
            line_strip.color.a = alpha; //不透明度，设0则全透明
            geometry_msgs::msg::Point p[8];

            //vis_pos_world是目标物的坐标
            p[0].x = vis_pos_world(0) - width;
            p[0].y = vis_pos_world(1) + length;
            p[0].z = vis_pos_world(2) + hight;
            p[1].x = vis_pos_world(0) - width;
            p[1].y = vis_pos_world(1) - length;
            p[1].z = vis_pos_world(2) + hight;
            p[2].x = vis_pos_world(0) - width;
            p[2].y = vis_pos_world(1) - length;
            p[2].z = vis_pos_world(2) - hight;
            p[3].x = vis_pos_world(0) - width;
            p[3].y = vis_pos_world(1) + length;
            p[3].z = vis_pos_world(2) - hight;
            p[4].x = vis_pos_world(0) + width;
            p[4].y = vis_pos_world(1) + length;
            p[4].z = vis_pos_world(2) - hight;
            p[5].x = vis_pos_world(0) + width;
            p[5].y = vis_pos_world(1) - length;
            p[5].z = vis_pos_world(2) - hight;
            p[6].x = vis_pos_world(0) + width;
            p[6].y = vis_pos_world(1) - length;
            p[6].z = vis_pos_world(2) + hight;
            p[7].x = vis_pos_world(0) + width;
            p[7].y = vis_pos_world(1) + length;
            p[7].z = vis_pos_world(2) + hight;
            //LINE_STRIP类型仅仅将line_strip.points中相邻的两个点相连，如0和1，1和2，2和3
            for (int i = 0; i < 8; i++) {
                line_strip.points.push_back(p[i]);
            }
            //为了保证矩形框的八条边都存在：
            line_strip.points.push_back(p[0]);
            line_strip.points.push_back(p[3]);
            line_strip.points.push_back(p[2]);
            line_strip.points.push_back(p[5]);
            line_strip.points.push_back(p[6]);
            line_strip.points.push_back(p[1]);
            line_strip.points.push_back(p[0]);
            line_strip.points.push_back(p[7]);
            line_strip.points.push_back(p[4]);
            mkrarr.markers.push_back(line_strip);
        }

        static void addPointToMarkerArray(visualization_msgs::msg::MarkerArray &mkr_arr,
                                          const Vec3f &pt,
                                          Color color = Color::Chartreuse(),
                                          string ns = "undefined_point",
                                          double size = 0.1, int id = 0) {
            visualization_msgs::msg::Marker marker_ball;
            auto sim_clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
            static int cnt = 0;
            Vec3f cur_pos = pt;
            if (std::isnan(pt.x()) || std::isnan(pt.y()) || std::isnan(pt.z())) {
                return;
            }
            marker_ball.header.frame_id = "world";
            marker_ball.header.stamp = sim_clock->now();
            marker_ball.ns = ns.c_str();
            marker_ball.id = id >= 0 ? id : cnt++;
            marker_ball.action = visualization_msgs::msg::Marker::ADD;
            marker_ball.pose.orientation.w = 1.0;
            marker_ball.type = visualization_msgs::msg::Marker::SPHERE;
            marker_ball.scale.x = size;
            marker_ball.scale.y = size;
            marker_ball.scale.z = size;
            marker_ball.color = color;

            geometry_msgs::msg::Point p;
            p.x = cur_pos.x();
            p.y = cur_pos.y();
            p.z = cur_pos.z();

            marker_ball.pose.position = p;
            mkr_arr.markers.push_back(marker_ball);
        }

        static void addEllipsoidToMarkerArray(visualization_msgs::msg::MarkerArray &mkr_arr,
                                              const Ellipsoid &ellipsoid,
                                              const std::string &ns = "ellipsoid",
                                              Color color = Color(Color::Orange(), 0.3)) {
            visualization_msgs::msg::Marker mkr;
            const auto R_ = ellipsoid.R();
            const auto d_ = ellipsoid.d();
            const auto r_ = ellipsoid.r();
            auto sim_clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
            Eigen::Quaterniond q(R_);

            static int id = 0;
            mkr.id = id++;
            mkr.type = visualization_msgs::msg::Marker::SPHERE;
            mkr.header.frame_id = "world";
            mkr.header.stamp = sim_clock->now();
            mkr.ns = ns;
            mkr.id = id++;
            mkr.action = visualization_msgs::msg::Marker::ADD;
            mkr.pose.orientation.w = q.w();
            mkr.pose.orientation.x = q.x();
            mkr.pose.orientation.y = q.y();
            mkr.pose.orientation.z = q.z();
            mkr.pose.position.x = d_.x();
            mkr.pose.position.y = d_.y();
            mkr.pose.position.z = d_.z();
            mkr.scale.x = r_.x() * 2;
            mkr.scale.y = r_.y() * 2;
            mkr.scale.z = r_.z() * 2;
            mkr.color = color;
            mkr_arr.markers.push_back(mkr);
        }

        static void addLineToMarkerArray(visualization_msgs::msg::MarkerArray &mkr_arr,
                                         const Vec3f &p1, const Vec3f &p2,
                                         Color color_pt = Color::Pink(), Color color = Color::Orange(),
                                         std::string ns = "line",
                                         double pt_size = 0.1,
                                         double line_size = 0.05) {
            static int point_id = 0;
            static int line_cnt = 0;
            auto sim_clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
            /* Publish point */
            visualization_msgs::msg::Marker point;
            point.header.frame_id = "world";
            point.header.stamp = sim_clock->now();
            point.ns = ns.c_str();
            point.id = point_id++;
            point.action = visualization_msgs::msg::Marker::ADD;
            point.pose.orientation.w = 1.0;
            point.type = visualization_msgs::msg::Marker::SPHERE;
            // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
            point.scale.x = pt_size;
            point.scale.y = pt_size;
            point.scale.z = pt_size;
            // Line list is blue
            point.color = color_pt;
            // Create the vertices for the points and lines
            geometry_msgs::msg::Point p;
            p.x = p1.x();
            p.y = p1.y();
            p.z = p1.z();
            point.pose.position = p;
            mkr_arr.markers.push_back(point);

            p.x = p2.x();
            p.y = p2.y();
            p.z = p2.z();
            point.pose.position = p;
            point.id = point_id++;
            mkr_arr.markers.push_back(point);

            // publish lines
            visualization_msgs::msg::Marker line_list;
            line_list.header.frame_id = "world";
            line_list.header.stamp = sim_clock->now();
            line_list.ns = ns + "line";
            line_list.id = line_cnt++;
            line_list.action = visualization_msgs::msg::Marker::ADD;
            line_list.pose.orientation.w = 1.0;
            line_list.type = visualization_msgs::msg::Marker::LINE_LIST;
            // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
            line_list.scale.x = line_size;
            // Line list is blue
            line_list.color = color;
            // Create the vertices for the points and lines

            p.x = p1.x();
            p.y = p1.y();
            p.z = p1.z();
            // The line list needs two points for each line
            line_list.points.push_back(p);
            p.x = p2.x();
            p.y = p2.y();
            p.z = p2.z();
            // The line list needs
            line_list.points.push_back(p);

            mkr_arr.markers.push_back(line_list);
        }

    };

}

#endif //SRC_ROS1_VISUALIZER_HPP
#endif
