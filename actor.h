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
	Actor(const std::string&);
	~Actor();

	TBOT02::Record _record;
	bool _pose_updated;
	bool _scan_updated;
	
	std::string _struct;
	std::string _model;
	std::size_t _induceThreadCount;
	std::chrono::milliseconds _induceInterval;
	std::string _mode;
	
	std::string _room;

	std::map<std::string, Alignment::Histogram> _room_location_goal;
	
	std::shared_ptr<Alignment::ActiveSystem> _system;
	std::shared_ptr<Alignment::ActiveEventsRepa> _events;
	std::vector<std::thread> _threads;
	std::vector<std::shared_ptr<Alignment::Active>> _level1;
	std::size_t _level1Count;
	std::vector<std::shared_ptr<Alignment::Active>> _level2;
	Alignment::ActiveUpdateParameters _updateParameters;
	Alignment::ActiveInduceParameters _induceParametersLevel1;
	Alignment::ActiveInduceParameters _induceParameters;
	std::size_t _eventId;
	
	std::shared_ptr<Alignment::System> _uu;
	std::shared_ptr<Alignment::SystemRepa> _ur;
	
private:
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _turn_request_pub;

	rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr _scan_sub;
	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _odom_sub;
	rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _goal_sub;

	rclcpp::TimerBase::SharedPtr _act_timer;

	void act_callback();
	void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
	void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
	void goal_callback(const std_msgs::msg::String::SharedPtr msg);
};
#endif // ACTOR_H
