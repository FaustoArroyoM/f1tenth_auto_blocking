#include <memory>
#include <math.h>
#include <string>
#include <cstdlib> 
#include <vector> 
#include <sstream> 
#include <iostream> 
#include <fstream>
#include <algorithm> 
#include <map>
#include <Eigen/Eigen>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "std_msgs/msg/int32.hpp" // Added for lane switching
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#define _USE_MATH_DEFINES
using std::placeholders::_1;
using namespace std::chrono_literals;

class PurePursuitMulti : public rclcpp::Node {

public:
    PurePursuitMulti() : Node("pure_pursuit_multi_node") {
        // Global Parameters
        this->declare_parameter("odom_topic", "/ego_racecar/odom");
        this->declare_parameter("car_refFrame", "ego_racecar/base_link");
        this->declare_parameter("drive_topic", "/drive");
        this->declare_parameter("rviz_waypointselected_topic", "/waypoints");
        this->declare_parameter("global_refFrame", "map");
        this->declare_parameter("steering_limit", 25.0);
        this->declare_parameter("lookahead_ratio", 8.0);

        // Load specific parameters for Inner (0), Middle (1), Outer (2)
        std::vector<std::string> prefixes = {"inner", "middle", "outer"};
        for (int i = 0; i < 3; ++i) {
            std::string p = prefixes[i];
            this->declare_parameter(p + ".waypoints_path", "");
            this->declare_parameter(p + ".min_lookahead", 1.0);
            this->declare_parameter(p + ".max_lookahead", 3.0);
            this->declare_parameter(p + ".K_p", 0.3);
            this->declare_parameter(p + ".velocity_percentage", 1.0);

            LineData ld;
            ld.path = this->get_parameter(p + ".waypoints_path").as_string();
            ld.min_la = this->get_parameter(p + ".min_lookahead").as_double();
            ld.max_la = this->get_parameter(p + ".max_lookahead").as_double();
            ld.kp = this->get_parameter(p + ".K_p").as_double();
            ld.vel_perc = this->get_parameter(p + ".velocity_percentage").as_double();
            
            racing_lines[i] = ld;
            if (!ld.path.empty()) download_waypoints(i);
        }

        // Initialize from parameters
        odom_topic = this->get_parameter("odom_topic").as_string();
        car_refFrame = this->get_parameter("car_refFrame").as_string();
        drive_topic = this->get_parameter("drive_topic").as_string();
        rviz_waypointselected_topic = this->get_parameter("rviz_waypointselected_topic").as_string();
        global_refFrame = this->get_parameter("global_refFrame").as_string();
        steering_limit =  this->get_parameter("steering_limit").as_double();
        
        // Subscribers & Publishers
        subscription_odom = this->create_subscription<nav_msgs::msg::Odometry>(odom_topic, 25, std::bind(&PurePursuitMulti::odom_callback, this, _1));
        subscription_lane = this->create_subscription<std_msgs::msg::Int32>(
            "/active_raceline", 10, [this](const std_msgs::msg::Int32::SharedPtr msg) {
                if (msg->data >= 0 && msg->data <= 2) {
                    active_lane = msg->data;
                    RCLCPP_INFO(this->get_logger(), "Switched to lane: %d", active_lane);
                }
            });

        publisher_drive = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(drive_topic, 25);
        vis_point_pub = this->create_publisher<visualization_msgs::msg::Marker>(rviz_waypointselected_topic, 10);

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        RCLCPP_INFO(this->get_logger(), "Multi-Lane Pure Pursuit node has been launched");
    }

private:
    struct LineData {
        std::string path;
        std::vector<double> X, Y, V;
        double min_la, max_la, kp, vel_perc;
        int index = 0;
        int velocity_index = 0;
    };

    std::map<int, LineData> racing_lines;
    int active_lane = 1; // Default to middle lane 
    geometry_msgs::msg::Quaternion odom_quat;
    Eigen::Matrix3d rotation_m;
    Eigen::Vector3d p1_car;
    double x_car_world, y_car_world;
    double curr_velocity = 0.0;
    double steering_limit;

    std::string odom_topic, car_refFrame, drive_topic, global_refFrame, rviz_waypointselected_topic;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_odom;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_lane;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr publisher_drive; 
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr vis_point_pub; 

    std::shared_ptr<tf2_ros::TransformListener> transform_listener_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    double to_radians(double degrees) { return degrees * M_PI / 180.0; }
    double to_degrees(double radians) { return radians * 180.0 / M_PI; }
    double p2pdist(double x1, double x2, double y1, double y2) { return sqrt(pow((x2-x1),2)+pow((y2-y1),2)); }

    void download_waypoints(int id) {
        std::ifstream file(racing_lines[id].path);
        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open CSV: %s", racing_lines[id].path.c_str());
            return;
        }
        std::string line, word;
        while (std::getline(file, line)) {
            std::stringstream s(line);
            int j = 0;
            while (getline(s, word, ',')) {
                if (j == 0) racing_lines[id].X.push_back(std::stod(word));
                else if (j == 1) racing_lines[id].Y.push_back(std::stod(word));
                else if (j == 2) racing_lines[id].V.push_back(std::stod(word));
                j++;
            }
        }
        RCLCPP_INFO(this->get_logger(), "Loaded lane %d from %s", id, racing_lines[id].path.c_str());
    }

    void update_indices() {
        double vehicle_heading = atan2(2.0 * (odom_quat.z * odom_quat.w + odom_quat.x * odom_quat.y), 
                                       1.0 - 2.0 * (odom_quat.y * odom_quat.y + odom_quat.z * odom_quat.z));
        double ratio = this->get_parameter("lookahead_ratio").as_double();

        for (int i = 0; i < 3; ++i) {
            if (racing_lines[i].X.empty()) continue;

            double longest_distance = 0;
            int final_i = -1;
            int num_wp = racing_lines[i].X.size();
            int start = racing_lines[i].index;
            int end = (start + 500) % num_wp;
            double lookahead = std::min(std::max(racing_lines[i].min_la, racing_lines[i].max_la * curr_velocity / ratio), racing_lines[i].max_la);

            for (int j = start; j != end; j = (j + 1) % num_wp) {
                double dx = racing_lines[i].X[j] - x_car_world;
                double dy = racing_lines[i].Y[j] - y_car_world;
                double waypoint_angle = atan2(dy, dx);
                double angle_diff = abs(waypoint_angle - vehicle_heading);
                double dist = p2pdist(racing_lines[i].X[j], x_car_world, racing_lines[i].Y[j], y_car_world);

                if ((angle_diff < M_PI_2 || angle_diff > 3 * M_PI_2) && dist <= lookahead) {
                    if (dist > longest_distance) {
                        longest_distance = dist;
                        final_i = j;
                    }
                }
            }
            if (final_i != -1) racing_lines[i].index = final_i;
            else if (longest_distance == 0) racing_lines[i].index = (racing_lines[i].index + 1) % num_wp;

            // Velocity Index
            double shortest_dist = p2pdist(racing_lines[i].X[start], x_car_world, racing_lines[i].Y[start], y_car_world);
            int velocity_i = start;
            for (int j = start; j != end; j = (j + 1) % num_wp) {
                double d = p2pdist(racing_lines[i].X[j], x_car_world, racing_lines[i].Y[j], y_car_world);
                if (d < shortest_dist) {
                    shortest_dist = d;
                    velocity_i = j;
                }
            }
            racing_lines[i].velocity_index = velocity_i;
        }
    }

    void quat_to_rot(double q0, double q1, double q2, double q3) {
        rotation_m << (2.0*(q0*q0+q1*q1)-1.0), (2.0*(q1*q2-q0*q3)), (2.0*(q1*q3+q0*q2)),
                      (2.0*(q1*q2+q0*q3)), (2.0*(q0*q0+q2*q2)-1.0), (2.0*(q2*q3-q0*q1)),
                      (2.0*(q1*q3-q0*q2)), (2.0*(q2*q3+q0*q1)), (2.0*(q0*q0+q3*q3)-1.0);
    }

    void transform_waypoint() {
        auto& ld = racing_lines[active_lane];
        if (ld.X.empty()) return;

        Eigen::Vector3d p1_world(ld.X[ld.index], ld.Y[ld.index], 0.0);
        visualize_point(p1_world);

        geometry_msgs::msg::TransformStamped tf;
        try {
            tf = tf_buffer_->lookupTransform(car_refFrame, global_refFrame, tf2::TimePointZero);
            Eigen::Vector3d trans(tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z);
            quat_to_rot(tf.transform.rotation.w, tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z);
            p1_car = (rotation_m * p1_world) + trans;
        } catch (tf2::TransformException & ex) {
            RCLCPP_ERROR(this->get_logger(), "TF Error: %s", ex.what());
        }
    }

    double p_controller() {
        double r = p1_car.norm();
        double y = p1_car(1);
        return (racing_lines[active_lane].kp * 2.0 * y) / pow(r, 2);
    }

    double get_velocity(double steering_angle) {
        auto& ld = racing_lines[active_lane];
        if (ld.V[ld.index] > 0) return ld.V[ld.index] * ld.vel_perc;
        
        double abs_angle = abs(steering_angle);
        if (abs_angle < to_radians(10.0)) return 6.0 * ld.vel_perc;
        if (abs_angle <= to_radians(20.0)) return 2.5 * ld.vel_perc;
        return 2.0 * ld.vel_perc;
    }
    void publish_message (double steering_angle) {
        auto drive_msgObj = ackermann_msgs::msg::AckermannDriveStamped();
        if (steering_angle < 0.0) {
            drive_msgObj.drive.steering_angle = std::max(steering_angle, -to_radians(steering_limit)); //ensure steering angle is dynamically capable
        } else {
            drive_msgObj.drive.steering_angle = std::min(steering_angle, to_radians(steering_limit)); //ensure steering angle is dynamically capable
        }

        curr_velocity = get_velocity(drive_msgObj.drive.steering_angle);
        drive_msgObj.drive.speed = curr_velocity;

       // RCLCPP_INFO(this->get_logger(), "index: %d ... distance: %.2fm ... Speed: %.2fm/s ... Steering Angle: %.2f ... K_p: %.2f ... velocity_percentage: %.2f", waypoints.index, p2pdist(waypoints.X[waypoints.index], x_car_world, waypoints.Y[waypoints.index], y_car_world), drive_msgObj.drive.speed, to_degrees(drive_msgObj.drive.steering_angle), K_p, velocity_percentage);

        publisher_drive->publish(drive_msgObj);
                
        RCLCPP_INFO(this->get_logger(), "Active Lane: %d, Target Index: %d, Speed: %.2f", 
            active_lane, racing_lines[active_lane].index, drive_msgObj.drive.speed);
    }

    void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        // Safety Guard: Check if the current lane actually has data
        if (racing_lines[active_lane].X.empty()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
                                "Waiting for waypoints to load for lane %d...", active_lane);
            return; // Exit early so we don't crash
        }
        odom_quat = msg->pose.pose.orientation;
        x_car_world = msg->pose.pose.position.x;
        y_car_world = msg->pose.pose.position.y;
        curr_velocity = msg->twist.twist.linear.x;

        update_indices();
        transform_waypoint();
        double steering_angle = p_controller();

        publish_message(steering_angle);

    }

    void visualize_point(Eigen::Vector3d p) {
        auto marker = visualization_msgs::msg::Marker();
        marker.header.frame_id = global_refFrame;
        marker.header.stamp = this->get_clock()->now();
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.scale.x = marker.scale.y = marker.scale.z = 0.25;
        marker.color.a = 1.0; marker.color.r = 1.0;
        marker.pose.position.x = p.x(); marker.pose.position.y = p.y();
        vis_point_pub->publish(marker);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PurePursuitMulti>());
    rclcpp::shutdown();
    return 0;
}