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
	Actor(std::chrono::milliseconds, const std::string&, const std::string&, const std::string&, std::size_t, std::size_t, std::size_t, std::size_t, const std::string&);
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
	
	std::string _struct;
	std::string _model;
	std::string _mode;
	
	std::string _room;

	std::map<std::string, Alignment::Histogram> _room_location_goal;
	
	std::shared_ptr<Alignment::ActiveSystem> _system;
	std::size_t _level1Size;
	std::shared_ptr<Alignment::ActiveEventsRepa> _events;
	std::size_t _activeSize;
	std::vector<std::shared_ptr<Alignment::Active>> _level1;
	std::size_t _induceThresholdLevel1;
	std::vector<std::shared_ptr<Alignment::Active>> _level2;
	std::size_t _induceThreshold;

	void act_callback();
	void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
	void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
	void goal_callback(const std_msgs::msg::String::SharedPtr msg);
};
#endif // ACTOR_H
