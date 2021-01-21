#include "actor.h"
#include <sstream>
#include <rapidjson/document.h>
#include <rapidjson/istreamwrapper.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

using namespace Alignment;
using namespace TBOT02;
using namespace std;
using namespace std::chrono_literals;
namespace js = rapidjson;

#define EVAL(x) { std::ostringstream str; str << #x << ": " << (x); RCLCPP_INFO(this->get_logger(), str.str());}

#define UNLOG ; str.flush(); RCLCPP_INFO(this->get_logger(), str.str());}
#define LOG { std::ostringstream str; str << 

#define ARGS_STRING_DEF(x,y) args.HasMember(#x) && args[#x].IsString() ? args[#x].GetString() : y
#define ARGS_STRING(x) ARGS_STRING_DEF(x,"")
#define ARGS_INT_DEF(x,y) args.HasMember(#x) && args[#x].IsInt() ? args[#x].GetInt() : y
#define ARGS_INT(x) ARGS_INT_DEF(x,0)
#define ARGS_DOUBLE_DEF(x,y) args.HasMember(#x) && args[#x].IsDouble() ? args[#x].GetDouble() : y
#define ARGS_DOUBLE(x) ARGS_DOUBLE_DEF(x,0.0)
#define ARGS_BOOL_DEF(x,y) args.HasMember(#x) && args[#x].IsBool() ? args[#x].GetBool() : y
#define ARGS_BOOL(x) ARGS_BOOL_DEF(x,false)

typedef std::chrono::duration<double> sec;
typedef std::chrono::high_resolution_clock clk;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define DEG2RAD (M_PI / 180.0)
#define RAD2DEG (180.0 / M_PI)

#define CENTER 0
#define LEFT   1
#define RIGHT  2

#define LINEAR_VELOCITY  0.3
#define ANGULAR_VELOCITY 1.5

#define GET_TB3_DIRECTION 0
#define TB3_DRIVE_FORWARD 1
#define TB3_RIGHT_TURN	2
#define TB3_LEFT_TURN	 3

Actor* actor_this = 0;

void actor_log(const std::string& str)
{
	if (actor_this)
	{
		RCLCPP_INFO(actor_this->get_logger(), str);
	}
	else
		std::cout << str << std::endl;
	return;
};
		
void run_induce(Actor& actor, Active& active, std::chrono::milliseconds induceInterval, std::size_t induceThresholdInitial)
{
	while (!active.terminate)
	{
		if (actor._pose_updated && actor._scan_updated && actor._update_updated && actor._eventId >= induceThresholdInitial)
			active.induce(actor._induceParametersLevel1);
		std::this_thread::sleep_for(std::chrono::milliseconds(induceInterval));
	}	
	return;
};

void run_update(Active& active, ActiveUpdateParameters ppu)
{
	if (!active.terminate)
		active.update(ppu);
	return;
};

Actor::Actor(const std::string& args_filename)
: Node("TBOT02_actor_node")
{
	typedef std::tuple<std::string, std::string, std::string> String3;	
	typedef std::vector<String3> String3List;	
	auto add = pairHistogramsAdd_u;
	auto single = histogramSingleton_u;			
			
	js::Document args;
	{
		std::ifstream in;
		try 
		{
			in.open(args_filename);
			js::IStreamWrapper isw(in);
			args.ParseStream(isw);
		}
		catch (const std::exception& e) 
		{
			RCLCPP_INFO(this->get_logger(), "TBOT02 actor node failed to open arguments file");	
			return;
		}	
		if (!args.IsObject())
		{
			RCLCPP_INFO(this->get_logger(), "TBOT02 actor node failed to read arguments file");	
			return;
		}
	}

	_scan_data[0] = 0.0;
	_scan_data[1] = 0.0;
	_scan_data[2] = 0.0;
	_robot_pose = 0.0;
	_prev_robot_pose = 0.0;
	_pose_updated = false;
	_scan_updated = false;
	_update_updated = false;
	_turn_request = "";
	_updateLogging = ARGS_BOOL(logging_update);
	std::chrono::milliseconds updateInterval = (std::chrono::milliseconds)(ARGS_INT_DEF(update_interval,10));
	std::chrono::milliseconds biasInterval = (std::chrono::milliseconds)(ARGS_INT_DEF(bias_interval,0));
	std::chrono::milliseconds turnInterval = (std::chrono::milliseconds)(ARGS_INT_DEF(turn_interval,0));
	_bias_right = true;
	_bias_factor = biasInterval.count() / updateInterval.count();
	_turn_factor = turnInterval.count() / updateInterval.count();
	
	_eventId = 0;
	std::chrono::milliseconds actInterval = (std::chrono::milliseconds)(ARGS_INT_DEF(act_interval,250));
	_room = ARGS_STRING_DEF(room_initial,"room1");
	_struct = ARGS_STRING_DEF(structure,"struct001");
	_model = ARGS_STRING(model);
	std::string modelInitial = ARGS_STRING(model_initial);
	_induceThreadCount = ARGS_INT_DEF(induceThreadCount,4);
	_level1Count = ARGS_INT_DEF(level1Count,12);
	bool level1Logging = ARGS_BOOL(logging_level1);
	std::size_t activeSizeLevel1 = ARGS_INT_DEF(activeSizeLevel1,10000);
	std::size_t induceThresholdLevel1 = ARGS_INT_DEF(induceThresholdLevel1,100);
	std::size_t induceThresholdInitialLevel1 = ARGS_INT_DEF(induceThresholdInitialLevel1,500);
	std::chrono::milliseconds induceIntervalLevel1 = (std::chrono::milliseconds)(ARGS_INT_DEF(induceIntervalLevel1,10));
	bool level2Logging = ARGS_BOOL(logging_level2);
	std::size_t activeSize = ARGS_INT_DEF(activeSize,1000000);
	std::size_t induceThreshold = ARGS_INT_DEF(induceThreshold,100);
	std::size_t induceThresholdInitial = ARGS_INT_DEF(induceThresholdInitial,1000);
	std::chrono::milliseconds induceInterval = (std::chrono::milliseconds)(ARGS_INT_DEF(induceInterval,10));	
	_mode = ARGS_STRING(mode);		
	{
		_induceParametersLevel1.tint = _induceThreadCount;
		_induceParametersLevel1.wmax = ARGS_INT_DEF(induceParametersLevel1.wmax,9);
		_induceParametersLevel1.lmax = ARGS_INT_DEF(induceParametersLevel1.lmax,8);
		_induceParametersLevel1.xmax = ARGS_INT_DEF(induceParametersLevel1.xmax,128);
		_induceParametersLevel1.znnmax = 200000.0 * 2.0 * 300.0 * 300.0 * _induceThreadCount;
		_induceParametersLevel1.omax = ARGS_INT_DEF(induceParametersLevel1.omax,10);
		_induceParametersLevel1.bmax = ARGS_INT_DEF(induceParametersLevel1.bmax,10*3);
		_induceParametersLevel1.mmax = ARGS_INT_DEF(induceParametersLevel1.mmax,3);
		_induceParametersLevel1.umax = ARGS_INT_DEF(induceParametersLevel1.umax,128);
		_induceParametersLevel1.pmax = ARGS_INT_DEF(induceParametersLevel1.pmax,1);
		_induceParametersLevel1.mult = ARGS_INT_DEF(induceParametersLevel1.mult,1);
		_induceParametersLevel1.seed = ARGS_INT_DEF(induceParametersLevel1.seed,5);
		_induceParameters.tint = _induceThreadCount;		
		_induceParameters.wmax = ARGS_INT_DEF(induceParameters.wmax,18);
		_induceParameters.lmax = ARGS_INT_DEF(induceParameters.lmax,8);
		_induceParameters.xmax = ARGS_INT_DEF(induceParameters.xmax,128);
		_induceParameters.znnmax = 200000.0 * 2.0 * 300.0 * 300.0 * _induceThreadCount;
		_induceParameters.omax = ARGS_INT_DEF(induceParameters.omax,10);
		_induceParameters.bmax = ARGS_INT_DEF(induceParameters.bmax,10*3);
		_induceParameters.mmax = ARGS_INT_DEF(induceParameters.mmax,3);
		_induceParameters.umax = ARGS_INT_DEF(induceParameters.umax,128);
		_induceParameters.pmax = ARGS_INT_DEF(induceParameters.pmax,1);
		_induceParameters.mult = ARGS_INT_DEF(induceParameters.mult,1);
		_induceParameters.seed = ARGS_INT_DEF(induceParameters.seed,5);		
	}

	EVAL(_room);
	EVAL(_struct);
	EVAL(_model);
	EVAL(_mode);
	
	if (_struct=="struct001")
	{
		std::unique_ptr<HistoryRepa> hr;
		{
			SystemHistoryRepaTuple xx = recordListsHistoryRepa_4(8, RecordList{ Record() });	
			_uu = std::move(std::get<0>(xx));
			_ur = std::move(std::get<1>(xx));
			hr = std::move(std::get<2>(xx));
		}
		_system = std::make_shared<ActiveSystem>();
		actor_this = this;
		_threads.reserve(_level1Count+1);
		_events = std::make_shared<ActiveEventsRepa>(_level1Count+1);
		for (std::size_t m = 0; m < _level1Count; m++)
			_level1.push_back(std::make_shared<Active>());
		for (std::size_t m = 0; m < _level1Count; m++)
		{			
			auto& activeA = *_level1[m];
			activeA.log = actor_log;
			activeA.system = _system;
			if (modelInitial.size())
			{
				ActiveIOParameters ppio;
				ppio.filename = modelInitial + "_1_" + (m<10 ? "0" : "") + std::to_string(m) +".ac";
				activeA.logging = true;
				if (!activeA.load(ppio))
				{
					RCLCPP_INFO(this->get_logger(), "TBOT02 actor node failed to load model");					
					_system.reset();
					return;
				}								
				_system->block = std::max(_system->block, activeA.varMax() >> activeA.bits);
				if (activeA.underlyingEventUpdateds.size())
					_eventId = std::max(_eventId,*(activeA.underlyingEventUpdateds.rbegin()));					
				else if (activeA.historyOverflow)
					_eventId = std::max(_eventId,activeA.historySize);	
				else					
					_eventId = std::max(_eventId,activeA.historyEvent);	
			}
			else
			{
				activeA.var = activeA.system->next(activeA.bits);
				activeA.varSlice = activeA.system->next(activeA.bits);
				activeA.historySize = activeSizeLevel1;
				activeA.induceThreshold = induceThresholdLevel1;
				activeA.decomp = std::make_unique<DecompFudSlicedRepa>();				
				{
					SizeList vv0;
					{
						auto& mm = _ur->mapVarSize();
						auto vscan = std::make_shared<Variable>("scan");
						int start = 360 - (360/_level1Count/2) + (360/_level1Count)*m;
						int end = start + (360/_level1Count);
						for (int i = start; i < end; i++)
							vv0.push_back(mm[Variable(vscan, std::make_shared<Variable>((i % 360) + 1))]);
					}
					auto hr1 = std::make_shared<HistoryRepa>();
					{
						auto sh = hr->shape;
						auto& mvv = hr->mapVarInt();
						auto n1 = vv0.size();
						hr1->dimension = n1;
						hr1->vectorVar = new std::size_t[n1];
						auto vv1 = hr1->vectorVar;
						hr1->shape = new std::size_t[n1];
						auto sh1 = hr1->shape;
						for (std::size_t i = 0; i < n1; i++)
						{
							auto v = vv0[i];
							vv1[i] = v;
							sh1[i] = sh[mvv[v]];
						}
						hr1->evient = true;
						hr1->size = activeA.historySize;
						auto z1 = hr1->size;
						hr1->arr = new unsigned char[z1*n1];
						auto rr1 = hr1->arr;
						// memset(rr1, 0, z1*n1);			
					}
					activeA.underlyingHistoryRepa.push_back(hr1);
				}
				{
					auto hr = std::make_unique<HistorySparseArray>();
					{
						auto z = activeA.historySize;
						hr->size = z;
						hr->capacity = 1;
						hr->arr = new std::size_t[z];		
					}		
					activeA.historySparse = std::move(hr);			
				}
			}
			activeA.name = (_model!="" ? _model : "model") + "_1_" + (m<10 ? "0" : "") + std::to_string(m);			
			activeA.logging = level1Logging;
			activeA.underlyingEventsRepa.push_back(_events);
			activeA.eventsSparse = std::make_shared<ActiveEventsArray>(1);
			std::size_t sizeA = activeA.historyOverflow ? activeA.historySize : activeA.historyEvent;
			if (sizeA)
			{
				LOG activeA.name << "\tfuds cardinality: " << activeA.decomp->fuds.size() << "\tmodel cardinality: " << activeA.decomp->fudRepasSize << "\tactive size: " << sizeA << "\tfuds per threshold: " << (double)activeA.decomp->fuds.size() * activeA.induceThreshold / sizeA UNLOG				
			}
			_threads.push_back(std::thread(run_induce, std::ref(*this), std::ref(activeA), induceIntervalLevel1, induceThresholdInitialLevel1));
		}
	
		_level2.push_back(std::make_shared<Active>());
		{
			auto& activeA = *_level2.front();
			activeA.log = actor_log;
			activeA.system = _system;
			if (modelInitial.size())
			{
				ActiveIOParameters ppio;
				ppio.filename = modelInitial + "_2" +".ac";
				activeA.logging = true;
				if (!activeA.load(ppio))
				{
					RCLCPP_INFO(this->get_logger(), "TBOT02 actor node failed to load model");					
					_system.reset();
					return;
				}								
				_system->block = std::max(_system->block, activeA.varMax() >> activeA.bits);
				if (activeA.underlyingEventUpdateds.size())
					_eventId = std::max(_eventId,*(activeA.underlyingEventUpdateds.rbegin()));					
				else if (activeA.historyOverflow)
					_eventId = std::max(_eventId,activeA.historySize);	
				else					
					_eventId = std::max(_eventId,activeA.historyEvent);	
			}
			else
			{			
				activeA.var = activeA.system->next(activeA.bits);
				activeA.varSlice = activeA.system->next(activeA.bits);
				activeA.historySize = activeSize;
				activeA.induceThreshold = induceThreshold;
				activeA.decomp = std::make_unique<DecompFudSlicedRepa>();
				{
					SizeList vv0;
					{
						auto& mm = _ur->mapVarSize();
						vv0.push_back(mm[Variable("motor")]);
						vv0.push_back(mm[Variable("location")]);
						for (auto v : vv0)
							activeA.induceVarExclusions.insert(v);
					}
					auto hr1 = std::make_shared<HistoryRepa>();
					{
						auto n = hr->dimension;
						auto vv = hr->vectorVar;
						auto sh = hr->shape;
						auto& mvv = hr->mapVarInt();
						auto n1 = vv0.size();
						hr1->dimension = n1;
						hr1->vectorVar = new std::size_t[n1];
						auto vv1 = hr1->vectorVar;
						hr1->shape = new std::size_t[n1];
						auto sh1 = hr1->shape;
						for (std::size_t i = 0; i < n1; i++)
						{
							auto v = vv0[i];
							vv1[i] = v;
							sh1[i] = sh[mvv[v]];
						}
						hr1->evient = true;
						hr1->size = activeA.historySize;
						auto z1 = hr1->size;
						hr1->arr = new unsigned char[z1*n1];
						auto rr1 = hr1->arr;
						// memset(rr1, 0, z1*n1);			
					}
					activeA.underlyingHistoryRepa.push_back(hr1);
				}
				for (std::size_t m = 0; m < _level1Count; m++)
				{
					auto& activeB = *_level1[m];
					activeA.underlyingHistorySparse.push_back(std::make_shared<HistorySparseArray>(activeA.historySize,1));
				}
				{
					auto hr = std::make_unique<HistorySparseArray>();
					{
						auto z = activeA.historySize;
						hr->size = z;
						hr->capacity = 1;
						hr->arr = new std::size_t[z];		
					}		
					activeA.historySparse = std::move(hr);			
				}
			}
			activeA.name = (_model!="" ? _model : "model") + "_2";
			activeA.logging = level2Logging;
			activeA.underlyingEventsRepa.push_back(_events);
			for (std::size_t m = 0; m < _level1Count; m++)
			{
				auto& activeB = *_level1[m];
				activeA.underlyingEventsSparse.push_back(activeB.eventsSparse);
			}
			activeA.eventsSparse = std::make_shared<ActiveEventsArray>(1);	
			std::size_t sizeA = activeA.historyOverflow ? activeA.historySize : activeA.historyEvent;
			if (sizeA)
			{
				LOG activeA.name << "\tfuds cardinality: " << activeA.decomp->fuds.size() << "\tmodel cardinality: " << activeA.decomp->fudRepasSize << "\tactive size: " << sizeA << "\tfuds per threshold: " << (double)activeA.decomp->fuds.size() * activeA.induceThreshold / sizeA UNLOG				
			}			
			_threads.push_back(std::thread(run_induce, std::ref(*this), std::ref(activeA), induceInterval, induceThresholdInitial));			
		}
	}
	
	{
		Variable location("location");
		Variable room_next("room_next");
		String3List ll{
			String3("room1","door12","room1"),
			String3("room1","door13","room1"),
			String3("room1","door14","room1"),
			String3("room1","door45","room4"),
			String3("room1","door56","room5"),
			String3("room1","room1","room1"),
			String3("room1","room2","room1"),
			String3("room1","room3","room1"),
			String3("room1","room4","room1"),
			String3("room1","room5","room4"),
			String3("room1","room6","room5"),
			String3("room2","door12","room2"),
			String3("room2","door13","room1"),
			String3("room2","door14","room1"),
			String3("room2","door45","room4"),
			String3("room2","door56","room5"),
			String3("room2","room1","room2"),
			String3("room2","room2","room2"),
			String3("room2","room3","room1"),
			String3("room2","room4","room1"),
			String3("room2","room5","room4"),
			String3("room2","room6","room5"),		
			String3("room3","door12","room1"),
			String3("room3","door13","room3"),
			String3("room3","door14","room1"),
			String3("room3","door45","room4"),
			String3("room3","door56","room5"),
			String3("room3","room1","room3"),
			String3("room3","room2","room1"),
			String3("room3","room3","room3"),
			String3("room3","room4","room1"),
			String3("room3","room5","room4"),
			String3("room3","room6","room5"),	
			String3("room4","door12","room1"),
			String3("room4","door13","room2"),
			String3("room4","door14","room4"),
			String3("room4","door45","room4"),
			String3("room4","door56","room5"),
			String3("room4","room1","room4"),
			String3("room4","room2","room1"),
			String3("room4","room3","room1"),
			String3("room4","room4","room4"),
			String3("room4","room5","room4"),
			String3("room4","room6","room5"),	
			String3("room5","door12","room1"),
			String3("room5","door13","room1"),
			String3("room5","door14","room4"),
			String3("room5","door45","room5"),
			String3("room5","door56","room5"),
			String3("room5","room1","room4"),
			String3("room5","room2","room1"),
			String3("room5","room3","room1"),
			String3("room5","room4","room5"),
			String3("room5","room5","room5"),
			String3("room5","room6","room5"),	
			String3("room6","door12","room1"),
			String3("room6","door13","room1"),
			String3("room6","door14","room4"),
			String3("room6","door45","room5"),
			String3("room6","door56","room6"),
			String3("room6","room1","room4"),
			String3("room6","room2","room1"),
			String3("room6","room3","room1"),
			String3("room6","room4","room5"),
			String3("room6","room5","room6"),
			String3("room6","room6","room6")
		};
		for (auto t : ll)
			_room_location_goal[std::get<0>(t)] = *add(_room_location_goal[std::get<0>(t)], *single(State(VarValPairList{VarValPair(location, std::get<1>(t)),VarValPair(room_next, std::get<2>(t))}),1));
	}
	
	{
	_cmd_vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", rclcpp::QoS(rclcpp::KeepLast(10)));
	
	_scan_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
		"scan", rclcpp::SensorDataQoS(), std::bind(&Actor::scan_callback, this, std::placeholders::_1));
	_odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
		"odom", rclcpp::QoS(rclcpp::KeepLast(10)), std::bind(&Actor::odom_callback, this, std::placeholders::_1));
	_goal_sub = this->create_subscription<std_msgs::msg::String>(
		"goal", 10, std::bind(&Actor::goal_callback, this, std::placeholders::_1));

	_update_timer = this->create_wall_timer(updateInterval, std::bind(&Actor::update_callback, this));
	_act_timer = this->create_wall_timer(actInterval, std::bind(&Actor::act_callback, this));
	}

	RCLCPP_INFO(this->get_logger(), "TBOT02 actor node has been initialised");
}

Actor::~Actor()
{
	if (_system && _struct=="struct001")
	{
		for (std::size_t m = 0; m < _level1Count; m++)
		{
			auto& activeA = *_level1[m];
			activeA.terminate = true;
			if ( _model!="")
			{
				ActiveIOParameters ppio;
				ppio.filename = activeA.name+".ac";
				activeA.logging = true;
				activeA.dump(ppio);						
			}
		}
		{
			auto& activeA = *_level2.front();	
			activeA.terminate = true;
			if ( _model!="")
			{
				ActiveIOParameters ppio;
				ppio.filename = activeA.name+".ac";
				activeA.dump(ppio);		
			}			
		}	
		for (auto& t : _threads)
			t.join();	
	}
	RCLCPP_INFO(this->get_logger(), "TBOT02 actor node has been terminated");
}

void Actor::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
	tf2::Quaternion q(
		msg->pose.pose.orientation.x,
		msg->pose.pose.orientation.y,
		msg->pose.pose.orientation.z,
		msg->pose.pose.orientation.w);
	tf2::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);
	_robot_pose = yaw;

	_record.sensor_pose[0] = msg->pose.pose.position.x;
	_record.sensor_pose[1] = msg->pose.pose.position.y;
	_record.sensor_pose[2] = msg->pose.pose.position.z;
	_record.sensor_pose[3] = msg->pose.pose.orientation.x;
	_record.sensor_pose[4] = msg->pose.pose.orientation.y;
	_record.sensor_pose[5] = msg->pose.pose.orientation.z;
	_record.sensor_pose[6] = msg->pose.pose.orientation.w;
	_pose_updated = true;
	if (_record.sensor_pose[2] >= 0.02)
	{
		_crashed = true;
		RCLCPP_INFO(this->get_logger(), "Crashed");		
	}	
}

void Actor::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
	uint16_t scan_angle[3] = {0, 30, 330};

	for (int num = 0; num < 3; num++)
	{
		if (std::isinf(msg->ranges.at(scan_angle[num])))
			_scan_data[num] = msg->range_max;
		else
			_scan_data[num] = msg->ranges.at(scan_angle[num]);
	}

	for (std::size_t i = 0; i < 360; i++)
	{
		if (std::isinf(msg->ranges.at(i)))
			_record.sensor_scan[i] = msg->range_max;
		else
			_record.sensor_scan[i] = msg->ranges.at(i);
	}
	_scan_updated = true;
}


void Actor::update_cmd_vel(double linear, double angular)
{
	geometry_msgs::msg::Twist cmd_vel;
	cmd_vel.linear.x  = linear;
	cmd_vel.angular.z = angular;

	_cmd_vel_pub->publish(cmd_vel);

	_record.action_linear = cmd_vel.linear.x;
	_record.action_angular = cmd_vel.angular.z;
	_update_updated = true;
}

void Actor::update_callback()
{
	static uint8_t turtlebot3_state_num = 0;
	double escape_range = 30.0 * DEG2RAD;
	double check_forward_dist = 0.7;
	double check_side_dist = 0.6;

	if (_crashed)
		return;
		
	if (_turn_request == "" && _bias_factor > 0 && (rand() % _bias_factor) == 0)
	{
		_bias_right = !_bias_right;
		if (_updateLogging)
		{
			RCLCPP_INFO(this->get_logger(), "Random bias: '%s'", _bias_right ? "right" : "left");
		}
	}
	
	switch (turtlebot3_state_num)
	{
	case GET_TB3_DIRECTION:
		if (_scan_data[CENTER] > check_forward_dist)
		{
			if (_bias_right && _scan_data[LEFT] < check_side_dist)
			{
				_prev_robot_pose = _robot_pose;
				turtlebot3_state_num = TB3_RIGHT_TURN;
			}
			else if (_scan_data[RIGHT] < check_side_dist)
			{
				_prev_robot_pose = _robot_pose;
				turtlebot3_state_num = TB3_LEFT_TURN;
			}
			else if (_scan_data[LEFT] < check_side_dist)
			{
				_prev_robot_pose = _robot_pose;
				turtlebot3_state_num = TB3_RIGHT_TURN;
			}
			else
			{
				turtlebot3_state_num = TB3_DRIVE_FORWARD;
			}
		}

		if (_scan_data[CENTER] < check_forward_dist)
		{
			_prev_robot_pose = _robot_pose;
			turtlebot3_state_num = _bias_right ? TB3_RIGHT_TURN : TB3_LEFT_TURN;
		}
		break;

	case TB3_DRIVE_FORWARD:
		if (_turn_factor > 0 && (rand() % _turn_factor) == 0)
		{
			_prev_robot_pose = _robot_pose;
			bool right = (rand() % 2) == 0;
			turtlebot3_state_num = right ? TB3_RIGHT_TURN : TB3_LEFT_TURN;
			if (_updateLogging)
			{
				RCLCPP_INFO(this->get_logger(), "Random turn: '%s'", right ? "right" : "left");
			}
			break;
		}
		if (_turn_request != "")
		{
			_prev_robot_pose = _robot_pose;
			bool right = _turn_request[0] == 'r' || _turn_request[0] == 'R';
			turtlebot3_state_num = right ? TB3_RIGHT_TURN : TB3_LEFT_TURN;
			if (_updateLogging)
			{
				
				RCLCPP_INFO(this->get_logger(), "Executing turn request: '%s'", right ? "right" : "left");
			}
			_turn_request = "";
			break;
		}
		update_cmd_vel(LINEAR_VELOCITY, 0.0);
		turtlebot3_state_num = GET_TB3_DIRECTION;
		break;

	case TB3_RIGHT_TURN:
		if (_turn_request != "")
		{
			bool right = _turn_request[0] == 'r' || _turn_request[0] == 'R';
			if (_updateLogging)
			{
				
				RCLCPP_INFO(this->get_logger(), "Ignoring turn request: '%s'", right ? "right" : "left");
			}
			_turn_request = "";
		}
		if (fabs(_prev_robot_pose - _robot_pose) >= escape_range)
			turtlebot3_state_num = GET_TB3_DIRECTION;
		else
			update_cmd_vel(0.0, -1 * ANGULAR_VELOCITY);
		break;

	case TB3_LEFT_TURN:
		if (_turn_request != "")
		{
			bool right = _turn_request[0] == 'r' || _turn_request[0] == 'R';
			if (_updateLogging)
			{
				
				RCLCPP_INFO(this->get_logger(), "Ignoring turn request: '%s'", right ? "right" : "left");
			}
			_turn_request = "";
		}
		if (fabs(_prev_robot_pose - _robot_pose) >= escape_range)
			turtlebot3_state_num = GET_TB3_DIRECTION;
		else
			update_cmd_vel(0.0, ANGULAR_VELOCITY);
		break;

	default:
		turtlebot3_state_num = GET_TB3_DIRECTION;
		break;
	}
}

void Actor::act_callback()
{
	if (_crashed || !_pose_updated || !_scan_updated || !_update_updated || !_system)
		return;
		
	std::unique_ptr<HistoryRepa> hr;
	{
		SystemHistoryRepaTuple xx = recordListsHistoryRepa_4(8, RecordList{ _record });	
		hr = std::move(std::get<2>(xx));
	}
	_eventId++;
	_events->mapIdEvent[_eventId] = HistoryRepaPtrSizePair(std::move(hr),_events->references);			
	{		
		std::vector<std::thread> threadsLevel1;
		threadsLevel1.reserve(_level1Count);
		for (std::size_t m = 0; m < _level1Count; m++)
		{
			auto& activeA = *_level1[m];
			threadsLevel1.push_back(std::thread(run_update, std::ref(activeA), _updateParameters));
		}
		for (auto& t : threadsLevel1)
			t.join();			
	}
	{
		auto& activeA = *_level2.front();	
		if (!activeA.terminate)		
			activeA.update(_updateParameters);
	}
	if (_struct=="struct001" && _mode=="mode001")
	{		
		auto single = histogramSingleton_u;		
		auto mul = pairHistogramsMultiply;
		auto sub = pairHistogramsSubtract_u;
		auto size = [](const Histogram& aa)
		{
			return (double)histogramsSize(aa).getNumerator();
		};		
		auto trim = histogramsTrim;
		auto ared = [](const Histogram& aa, const VarUSet& vv)
		{
			return setVarsHistogramsReduce(vv, aa);
		};		
		auto hraa = [](const System& uu, const SystemRepa& ur, const HistoryRepa& hr)
		{
			return historiesHistogram(*systemsHistoryRepasHistory_u(uu,ur,hr));
		};
		auto hrsel = eventsHistoryRepasHistoryRepaSelection_u;
		Variable motor("motor");
		Variable location("location");
		Variable room_next("room_next");
		bool ok = true;
		
		auto& activeA = *_level2.front();
		if (!activeA.terminate)	
		{			
			auto mark = clk::now();
			std::lock_guard<std::mutex> guard(activeA.mutex);
			ok = ok && (activeA.historyOverflow	|| activeA.historyEvent);
			std::size_t historyEventA = ok ? (activeA.historyEvent ? activeA.historyEvent - 1 : activeA.historyOverflow - 1) : 0;
			std::size_t sliceA = ok ? activeA.historySparse->arr[historyEventA] : 0;
			SizeSet setEventA = ok ? activeA.historySlicesSetEvent[sliceA] : SizeSet();
			std::shared_ptr<HistoryRepa> hr = ok ? activeA.underlyingHistoryRepa.front() : 0;
			Histogram histogramA;
			if (ok)
			{
				SizeList ev(setEventA.begin(),setEventA.end());
				histogramA = *trim(*hraa(*_uu, *_ur, *hrsel(ev.size(), ev.data(), *hr)));
			}
			// EVAL(size(histogramA))
			// EVAL(histogramA);
			// EVAL(*ared(histogramA, VarUSet{location}));
			// EVAL(*ared(histogramA, VarUSet{motor}));
			// EVAL(_room);
			// EVAL(_room_location_goal[_room]);	
			{
				std::size_t sizeA = activeA.historyOverflow ? activeA.historySize : activeA.historyEvent;
				if (sizeA)
				{
					LOG activeA.name << "\tmode: " << _mode << "\tfuds cardinality: " << activeA.decomp->fuds.size() << "\tmodel cardinality: " << activeA.decomp->fudRepasSize << "\tactive size: " << sizeA << "\tfuds per threshold: " << (double)activeA.decomp->fuds.size() * activeA.induceThreshold / sizeA << "\ttime " << ((sec)(clk::now() - mark)).count() << "s" UNLOG				
				}					
			}


		}		
	}
}

void Actor::goal_callback(const std_msgs::msg::String::SharedPtr msg)
{
	_room = msg->data;
	std::ostringstream str; 
	str << "Received goal: " << msg->data;
	RCLCPP_INFO(this->get_logger(), str.str());
}

int main(int argc, char** argv)
{
	std::string args_filename = string(argc > 1 ? argv[1] : "actor.json");

	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<Actor>(args_filename));
	rclcpp::shutdown();

	return 0;
}

