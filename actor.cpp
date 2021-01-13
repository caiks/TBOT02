#include "actor.h"
#include <sstream>

using namespace Alignment;
using namespace TBOT02;
using namespace std;
using namespace std::chrono_literals;

#define EVAL(x) { std::ostringstream str; str << #x << ": " << (x); RCLCPP_INFO(this->get_logger(), str.str());}

typedef std::chrono::duration<double> sec;
typedef std::chrono::high_resolution_clock clk;

Actor::Actor(std::chrono::milliseconds act_interval, const std::string& room_initial, const std::string& structure, const std::string& model, std::size_t level1Size, std::size_t activeSize, std::size_t induceThresholdLevel1, std::size_t induceThreshold, const std::string& mode)
: Node("TBOT02_actor_node")
{
	typedef std::tuple<std::string, std::string, std::string> String3;	
	typedef std::vector<String3> String3List;	
	auto add = pairHistogramsAdd_u;
	auto single = histogramSingleton_u;		
		
	_pose_updated = false;
	_scan_updated = false;
	
	_room = room_initial;
	_struct = structure;
	_model = model;
	_level1Size = level1Size;
	_activeSize = activeSize;
	_induceThresholdLevel1 = induceThresholdLevel1;
	_induceThreshold = induceThreshold;
	_mode = mode;
	
	EVAL(_room);
	EVAL(_struct);
	EVAL(_model);
	EVAL(_level1Size);
	EVAL(_activeSize);
	EVAL(_induceThresholdLevel1);
	EVAL(_induceThreshold);
	EVAL(_mode);
	
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
	_turn_request_pub = this->create_publisher<std_msgs::msg::String>("turn_request", rclcpp::QoS(rclcpp::KeepLast(10)));
	
	_scan_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
		"scan", rclcpp::SensorDataQoS(), std::bind(&Actor::scan_callback, this, std::placeholders::_1));
	_odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
		"odom", rclcpp::QoS(rclcpp::KeepLast(10)), std::bind(&Actor::odom_callback, this, std::placeholders::_1));
	_goal_sub = this->create_subscription<std_msgs::msg::String>(
		"goal", 10, std::bind(&Actor::goal_callback, this, std::placeholders::_1));

	_act_timer = this->create_wall_timer(act_interval, std::bind(&Actor::act_callback, this));
	}

	RCLCPP_INFO(this->get_logger(), "TBOT02 actor node has been initialised");
}

Actor::~Actor()
{
	RCLCPP_INFO(this->get_logger(), "TBOT02 actor node has been terminated");
}

void Actor::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
	_record.sensor_pose[0] = msg->pose.pose.position.x;
	_record.sensor_pose[1] = msg->pose.pose.position.y;
	_record.sensor_pose[2] = msg->pose.pose.position.z;
	_record.sensor_pose[3] = msg->pose.pose.orientation.x;
	_record.sensor_pose[4] = msg->pose.pose.orientation.y;
	_record.sensor_pose[5] = msg->pose.pose.orientation.z;
	_record.sensor_pose[6] = msg->pose.pose.orientation.w;
	_pose_updated = true;
}

void Actor::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
	for (std::size_t i = 0; i < 360; i++)
	{
		if (std::isinf(msg->ranges.at(i)))
			_record.sensor_scan[i] = msg->range_max;
		else
			_record.sensor_scan[i] = msg->ranges.at(i);
	}
	_scan_updated = true;
}

void Actor::act_callback()
{
	if (!_pose_updated || !_scan_updated)
		return;

	Variable motor("motor");
	Variable location("location");
	Variable room_next("room_next");
	
	std::unique_ptr<SystemRepa> ur;
	std::unique_ptr<HistoryRepa> hr;
	{
		SystemHistoryRepaTuple xx = recordListsHistoryRepa_4(8, RecordList{ _record });	
		ur = std::move(std::get<1>(xx));
		hr = std::move(std::get<2>(xx));
	}
	
	if (!_system && _struct=="struct001")
	{
		_system = std::make_shared<ActiveSystem>();
		_events = std::make_shared<ActiveEventsRepa>(_level1Size+1);
		for (std::size_t m = 0; m < _level1Size; m++)
			_level1.push_back(std::make_shared<Active>());
		for (std::size_t m = 0; m < _level1Size; m++)
		{			
			auto& activeA = *_level1[m];
			activeA.name = _model + "_1_" + (m<10 ? "0" : "") + std::to_string(m);
			activeA.system = _system;
			activeA.var = activeA.system->next(activeA.bits);
			activeA.varSlice = activeA.system->next(activeA.bits);
			activeA.historySize = _activeSize;
			activeA.induceThreshold = _induceThresholdLevel1;
			activeA.logging = false;
			activeA.decomp = std::make_unique<DecompFudSlicedRepa>();
			activeA.underlyingEventsRepa.push_back(_events);
			{
				SizeList vv0;
				{
					auto& mm = ur->mapVarSize();
					auto vscan = std::make_shared<Variable>("scan");
					int start = 360 - (360/_level1Size/2) + (360/_level1Size)*m;
					int end = start + (360/_level1Size);
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
			activeA.eventsSparse = std::make_shared<ActiveEventsArray>(1);
		}
	
		_level2.push_back(std::make_shared<Active>());
		{
			auto& activeA = *_level2.front();
			activeA.name = _model + "_2";
			activeA.system = _system;
			activeA.var = activeA.system->next(activeA.bits);
			activeA.varSlice = activeA.system->next(activeA.bits);
			activeA.historySize = _activeSize;
			activeA.induceThreshold = _induceThreshold;
			activeA.logging = false;
			activeA.decomp = std::make_unique<DecompFudSlicedRepa>();
			activeA.underlyingEventsRepa.push_back(_events);
			{
				SizeList vv0;
				{
					auto& mm = ur->mapVarSize();
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
			for (std::size_t m = 0; m < _level1Size; m++)
			{
				auto& activeB = *_level1[m];
				activeA.underlyingEventsSparse.push_back(activeB.eventsSparse);
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
			activeA.eventsSparse = std::make_shared<ActiveEventsArray>(1);			
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
	std::size_t arg = 1;
	std::chrono::milliseconds act_interval(argc > arg ? std::atol(argv[arg++]) : 250);
	std::string room_initial = string(argc > arg ? argv[arg++] : "room1");
	string structure = string(argc > arg ? argv[arg++] : "struct001");
	std::string model;
	std::size_t level1Size;
	std::size_t activeSize;
	std::size_t induceThresholdLevel1;
	std::size_t induceThreshold;
	if (structure == "struct001")
	{
		model = string(argc > arg ? argv[arg++] : "");	
		level1Size = argc > arg ? std::atol(argv[arg++]) : 12;
		activeSize = argc > arg ? std::atol(argv[arg++]) : 1000000;
		induceThresholdLevel1 = argc > arg ? std::atol(argv[arg++]) : 100;
		induceThreshold = argc > arg ? std::atol(argv[arg++]) : 100;
	}
	string mode = string(argc >= arg ? argv[arg++] : "mode001");

	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<Actor>(act_interval, room_initial, structure, model, level1Size, activeSize, induceThresholdLevel1, induceThreshold, mode));
	rclcpp::shutdown();

	return 0;
}

