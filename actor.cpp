#include "actor.h"
#include <sstream>
#include <rapidjson/document.h>
#include <rapidjson/istreamwrapper.h>

using namespace Alignment;
using namespace TBOT02;
using namespace std;
using namespace std::chrono_literals;
namespace js = rapidjson;

#define EVAL(x) { std::ostringstream str; str << #x << ": " << (x); RCLCPP_INFO(this->get_logger(), str.str());}

#define ARGS_STRING(x) args.HasMember(#x) && args[#x].IsString() ? args[#x].GetString() : ""
#define ARGS_STRING_DEF(x,y) args.HasMember(#x) && args[#x].IsString() ? args[#x].GetString() : y
#define ARGS_INT(x) args.HasMember(#x) && args[#x].IsInt() ? args[#x].GetInt() : 0
#define ARGS_INT_DEF(x,y) args.HasMember(#x) && args[#x].IsInt() ? args[#x].GetInt() : y

typedef std::chrono::duration<double> sec;
typedef std::chrono::high_resolution_clock clk;

Actor* actor_this;

void actor_log(const std::string& str)
{
	RCLCPP_INFO(actor_this->get_logger(), str);
	return;
};

Actor::Actor(const std::string& args_filename)
: Node("TBOT02_actor_node")
{
	typedef std::tuple<std::string, std::string, std::string> String3;	
	typedef std::vector<String3> String3List;	
	auto add = pairHistogramsAdd_u;
	auto single = histogramSingleton_u;			
		
	_pose_updated = false;
	_scan_updated = false;
	_eventId = 0;
	
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
			
	std::chrono::milliseconds actInterval = (std::chrono::milliseconds)(ARGS_INT_DEF(actInterval,250));
	_room = ARGS_STRING_DEF(room_initial,"room1");
	_struct = ARGS_STRING_DEF(structure,"struct001");
	_model = ARGS_STRING(model);
	_induceThreadCount = ARGS_INT_DEF(induceThreadCount,4);
	_induceInterval = (std::chrono::milliseconds)(ARGS_INT_DEF(induceInterval,10));
	_level1Count = ARGS_INT_DEF(level1Count,12);
	std::size_t activeSizeLevel1 = ARGS_INT_DEF(activeSizeLevel1,10000);
	std::size_t induceThresholdLevel1 = ARGS_INT_DEF(induceThresholdLevel1,100);
	_induceThresholdInitialLevel1 = ARGS_INT_DEF(induceThresholdInitialLevel1,500);
	std::size_t activeSize = ARGS_INT_DEF(activeSize,1000000);
	std::size_t induceThreshold = ARGS_INT_DEF(induceThreshold,100);
	_induceThresholdInitial = ARGS_INT_DEF(induceThresholdInitial,1000);
	_mode = ARGS_STRING_DEF(mode,"mode001");
		
	EVAL(_room);
	EVAL(_struct);
	EVAL(_model);
	EVAL(_mode);
	
	if (_struct=="struct001")
	{
		std::unique_ptr<SystemRepa> ur;
		std::unique_ptr<HistoryRepa> hr;
		{
			SystemHistoryRepaTuple xx = recordListsHistoryRepa_4(8, RecordList{ Record() });	
			ur = std::move(std::get<1>(xx));
			hr = std::move(std::get<2>(xx));
		}
		_system = std::make_shared<ActiveSystem>();
		actor_this = this;
		_events = std::make_shared<ActiveEventsRepa>(_level1Count+1);
		for (std::size_t m = 0; m < _level1Count; m++)
			_level1.push_back(std::make_shared<Active>());
		for (std::size_t m = 0; m < _level1Count; m++)
		{			
			auto& activeA = *_level1[m];
			activeA.log = actor_log;
			activeA.name = (_model!="" ? _model : "model") + "_1_" + (m<10 ? "0" : "") + std::to_string(m);
			activeA.system = _system;
			activeA.var = activeA.system->next(activeA.bits);
			activeA.varSlice = activeA.system->next(activeA.bits);
			activeA.historySize = activeSizeLevel1;
			activeA.induceThreshold = induceThresholdLevel1;
			activeA.logging = true;
			activeA.decomp = std::make_unique<DecompFudSlicedRepa>();
			activeA.underlyingEventsRepa.push_back(_events);
			{
				SizeList vv0;
				{
					auto& mm = ur->mapVarSize();
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
			activeA.eventsSparse = std::make_shared<ActiveEventsArray>(1);
		}
	
		_level2.push_back(std::make_shared<Active>());
		{
			auto& activeA = *_level2.front();
			activeA.log = actor_log;
			activeA.name = (_model!="" ? _model : "model") + "_2";
			activeA.system = _system;
			activeA.var = activeA.system->next(activeA.bits);
			activeA.varSlice = activeA.system->next(activeA.bits);
			activeA.historySize = activeSize;
			activeA.induceThreshold = induceThreshold;
			activeA.logging = true;
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
			for (std::size_t m = 0; m < _level1Count; m++)
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

	_act_timer = this->create_wall_timer(actInterval, std::bind(&Actor::act_callback, this));
	}

	RCLCPP_INFO(this->get_logger(), "TBOT02 actor node has been initialised");
}

Actor::~Actor()
{
	if (_system && _struct=="struct001" && _model!="")
	{
		for (std::size_t m = 0; m < _level1Count; m++)
		{
			auto& activeA = *_level1[m];
			activeA.terminate = true;
			ActiveIOParameters ppio;
			ppio.filename = activeA.name+".ac";
			activeA.logging = true;
			activeA.dump(ppio);			
		}
		{
			auto& activeA = *_level2.front();	
			activeA.terminate = true;
			ActiveIOParameters ppio;
			ppio.filename = activeA.name+".ac";
			activeA.dump(ppio);							
		}	
	}
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
	if (!_pose_updated || !_scan_updated || !_system)
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
	_eventId++;
	_events->mapIdEvent[_eventId] = HistoryRepaPtrSizePair(std::move(hr),_events->references);			
	auto do_update = [](Active& active, ActiveUpdateParameters ppu)
	{
		active.update(ppu);
		return;
	};
	{		
		std::vector<std::thread> threadsLevel1;
		threadsLevel1.reserve(_level1Count);
		for (std::size_t m = 0; m < _level1Count; m++)
		{
			auto& activeA = *_level1[m];
			threadsLevel1.push_back(std::thread(do_update, std::ref(activeA), _updateParameters));
		}
		for (auto& t : threadsLevel1)
			t.join();			
	}
	{
		auto& activeA = *_level2.front();	
		activeA.update(_updateParameters);
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

