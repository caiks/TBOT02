#ifndef ACTOR_H
#define ACTOR_H

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/string.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include "dev.h"

class Actor : public rclcpp::Node
{
public:
	Actor(const std::string&, const std::string&, std::chrono::milliseconds, const std::string&, std::size_t, const std::string&, std::size_t, double);
	~Actor();

private:
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _turn_request_pub;

	rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr _scan_sub;
	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _odom_sub;
	rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _goal_sub;

	rclcpp::TimerBase::SharedPtr _act_timer;

	TBOT02::Record _record;
	bool _pose_updated;
	bool _scan_updated;
	
	std::string _mode;
	std::size_t _act_factor;
	double _majority_fraction;
	
	std::string _room;

	std::shared_ptr<Alignment::System> _uu;
	std::shared_ptr<Alignment::SystemRepa> _ur;
	std::shared_ptr<Alignment::ApplicationRepa> _dr;
	std::map<std::size_t, std::shared_ptr<Alignment::HistoryRepa>> _slice_history;
	std::map<std::string, Alignment::Histogram> _room_location_goal;

	void act_callback();
	void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
	void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
	void goal_callback(const std_msgs::msg::String::SharedPtr msg);
};
#endif // ACTOR_H
