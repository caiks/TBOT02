#include "actor.h"
#include <sstream>

using namespace Alignment;
using namespace TBOT02;
using namespace std;
using namespace std::chrono_literals;

#define EVAL(x) { std::ostringstream str; str << #x << ": " << (x); RCLCPP_INFO(this->get_logger(), str.str());}

typedef std::chrono::duration<double> sec;
typedef std::chrono::high_resolution_clock clk;

Actor::Actor(const std::string& model, const std::string& room_initial, std::chrono::milliseconds act_interval, const std::string& dataset, std::size_t chunks, const std::string& mode, std::size_t act_factor, double majority_fraction)
: Node("TBOT02_actor_node")
{
	typedef std::tuple<std::string, std::string, std::string> String3;	
	typedef std::vector<String3> String3List;	
	typedef std::vector<std::string> StringList;	
	auto add = pairHistogramsAdd_u;
	auto single = histogramSingleton_u;		
	auto hrsel = eventsHistoryRepasHistoryRepaSelection_u;
	auto hrhrred = setVarsHistoryRepasHistoryRepaReduced_u;
	auto frmul = historyRepasFudRepasMultiply_u;
		
	_pose_updated = false;
	_scan_updated = false;
	
	_act_factor = act_factor;
	_majority_fraction = majority_fraction;
	_mode = mode;
	_room = room_initial;
	
	EVAL(_room);
	EVAL(_mode);
	EVAL(_act_factor);
	EVAL(_majority_fraction);
	
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
		if (_mode == "mode004" || _mode == "mode005")
		{
			StringList rooms{string("room1"),string("room2"),string("room3"),string("room4"),string("room5"),string("room6")};
			for (auto room : rooms)
			{
				ll.push_back(String3(room,"room2z","room2"));
				ll.push_back(String3(room,"room3z","room3"));
				ll.push_back(String3(room,"room6z","room6"));
			}
		}
		for (auto t : ll)
			_room_location_goal[std::get<0>(t)] = *add(_room_location_goal[std::get<0>(t)], *single(State(VarValPairList{VarValPair(location, std::get<1>(t)),VarValPair(room_next, std::get<2>(t))}),1));
	}
	
	{	
		std::unique_ptr<Alignment::HistoryRepa> hr;
		{
			std::vector<std::string> files{
				"data002_room1.bin",
				"data002_room2.bin",
				"data002_room2_2.bin",
				"data002_room3.bin",
				"data002_room4.bin",
				"data002_room5.bin",
				"data002_room5_2.bin"
			};
			if (dataset == "data003")
			{
				files.clear();
				files.push_back("data003.bin");
			}
			else if (dataset == "data004")
			{
				files.clear();
				files.push_back("data003.bin");
				files.push_back("data004_01.bin");
				files.push_back("data004_02.bin");
				files.push_back("data004_03.bin");
				files.push_back("data004_04.bin");
				files.push_back("data004_05.bin");
			}
			else if (dataset != "data002")
			{
				files.clear();
				files.push_back(dataset+".bin");
			}	
			HistoryRepaPtrList ll;
			for (auto& f : files)
			{
				std::ifstream in(f, std::ios::binary);
				auto qq = persistentsRecordList(in);
				in.close();			
				SystemHistoryRepaTuple xx;
				if (_mode == "mode004" || _mode == "mode005")
					xx = recordListsHistoryRepa_6(8, *qq);		
				else		
					xx = recordListsHistoryRepa_4(8, *qq);							
				_uu = std::move(std::get<0>(xx));
				_ur = std::move(std::get<1>(xx));
				ll.push_back(std::move(std::get<2>(xx)));
			}
			hr = vectorHistoryRepasConcat_u(ll);
		}
		
		EVAL(hr->size);
		
		auto& llu = _ur->listVarSizePair;
		{
			std::unique_ptr<Alignment::SystemRepa> ur1;
			StrVarPtrMap m;
			std::ifstream in(model + ".dr", std::ios::binary);
			ur1 = persistentsSystemRepa(in, m);
			_dr = persistentsApplicationRepa(in);
			in.close();
			auto& llu1 = ur1->listVarSizePair;			
			SizeSizeUMap nn;
			for (auto& ll : _dr->fud->layers)
				for (auto& tr : ll)
				{
					auto x = tr->derived;
					auto& p = llu1[x];
					llu.push_back(VarSizePair(p.first, p.second));
					nn[x] = llu.size() - 1;
				}
			_dr->reframe_u(nn);
		}	
		
		EVAL(treesSize(*_dr->slices));
		EVAL(treesLeafElements(*_dr->slices)->size());		
		
		VarSet vvl;
		vvl.insert(Variable("motor"));
		vvl.insert(Variable("location"));
		vvl.insert(Variable("room_next"));

		auto& vvi = _ur->mapVarSize();
		SizeList vvl1;
		for (auto& v : vvl)
			vvl1.push_back(vvi[v]);
			
		if (!chunks)
		{
			auto hr1 = frmul(*hr, *_dr->fud);
			EVAL(hr1->size);	
			auto hr2 = hrhrred(vvl1.size(), vvl1.data(), *hr);
			EVAL(hr2->size);	
			if (hr1->evient)
				hr1->transpose();
			auto z = hr1->size;
			auto& mvv = hr1->mapVarInt();
			auto sh = hr1->shape;
			auto rr = hr1->arr;
			auto nn = treesLeafNodes(*_dr->slices);
			for (auto& s : *nn)
			{
				SizeList ev;
				auto pk = mvv[s.first];
				for (std::size_t j = 0; j < z; j++)
				{
					std::size_t u = rr[pk*z + j];
					if (u)
					{
						ev.push_back(j);
					}
				}
				if (ev.size() > 0)	
					_slice_history[s.first] = std::move(hrsel(ev.size(), ev.data(), *hr2));
			}
		}
		else		
		{
			auto z0 = hr->size;
			SizeList sl;
			sl.reserve(z0);
			auto nn = treesLeafNodes(*_dr->slices);
			for (std::size_t k = 0; k <= chunks; k++)
			{
				EVAL(k);					
				SizeList ev;
				ev.reserve(2*z0/chunks);
				for (std::size_t j = k * (z0/chunks); j < (k+1) * (z0/chunks) && j < z0; j++)
					ev.push_back(j);
				if (ev.size())
				{
					auto hr1 = frmul(*hrsel(ev.size(), ev.data(), *hr), *_dr->fud);
					EVAL(hr1->size);	
					if (hr1->evient)
						hr1->transpose();
					auto z = hr1->size;
					auto& mvv = hr1->mapVarInt();
					auto sh = hr1->shape;
					auto rr = hr1->arr;
					for (std::size_t j = 0; j < z; j++)
					{
						for (auto& s : *nn)
						{
							auto pk = mvv[s.first];
							std::size_t u = rr[pk*z + j];
							if (u)
							{
								sl.push_back(s.first);
								break;
							}
						}
					}								
				}
			}
			auto hr2 = hrhrred(vvl1.size(), vvl1.data(), *hr);
			EVAL(hr2->size);	
			EVAL(sl.size());	
			for (auto& s : *nn)
			{
				SizeList ev;
				for (std::size_t j = 0; j < z0; j++)	
				{
					if (s.first == sl[j])
						ev.push_back(j);
				}
				if (ev.size() > 0)	
					_slice_history[s.first] = std::move(hrsel(ev.size(), ev.data(), *hr2));
			}
		}	
		EVAL(_slice_history.size());
	}

	_turn_request_pub = this->create_publisher<std_msgs::msg::String>("turn_request", rclcpp::QoS(rclcpp::KeepLast(10)));
	
	_scan_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
		"scan", rclcpp::SensorDataQoS(), std::bind(&Actor::scan_callback, this, std::placeholders::_1));
	_odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
		"odom", rclcpp::QoS(rclcpp::KeepLast(10)), std::bind(&Actor::odom_callback, this, std::placeholders::_1));
	_goal_sub = this->create_subscription<std_msgs::msg::String>(
		"goal", 10, std::bind(&Actor::goal_callback, this, std::placeholders::_1));

	_act_timer = this->create_wall_timer(act_interval, std::bind(&Actor::act_callback, this));

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
	auto state = [](const Variable& v, const Value& u)
	{
		return State(VarValPairList{VarValPair(v, u)});
	};
	auto smax = [](const Histogram& aa)
	{
		std::vector<std::pair<Rational,State>> ll;
		auto ll0 = *histogramsList(aa);
		for (auto p : ll0)
			ll.push_back(std::pair<Rational,State>(p.second,p.first));
		auto ll1 = sorted(ll);
		return ll1.back().second;
	};	
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
	auto hrhrred = setVarsHistoryRepasHistoryRepaReduced_u;
	auto frmul = historyRepasFudRepasMultiply_u;
		
	if (!_pose_updated || !_scan_updated)
		return;

	Variable motor("motor");
	Variable location("location");
	Variable room_next("room_next");
	
	std::size_t s = 0;
	{
		SystemHistoryRepaTuple xx;
		if (_mode == "mode004" || _mode == "mode005")
			xx = recordListsHistoryRepa_6(8, RecordList{ _record });		
		else		
			xx = recordListsHistoryRepa_4(8, RecordList{ _record });	
		auto hr = std::move(std::get<2>(xx));
		SizeList ww;
		auto nn = treesLeafNodes(*_dr->slices);
		for (auto& s : *nn)
			ww.push_back(s.first);
		auto hr1 = hrhrred(ww.size(), ww.data(), *frmul(*hr, *_dr->fud));
		auto n = hr1->dimension;
		auto vv = hr1->vectorVar;
		auto rr = hr1->arr;
		bool found = false;
		for (std::size_t i = 0; !found && i < n; i++)
		{
			std::size_t u = rr[i];
			if (u)
			{
				s = vv[i];
				found = true;
			}
		}	
		if (!found)
		{
			RCLCPP_INFO(this->get_logger(), "act_callback: error: no slice");
			return;
		}
	}
	
	if (_slice_history.find(s) == _slice_history.end())
	{
		RCLCPP_INFO(this->get_logger(), "no slice history");
		return;
	}
		
	{
		VarSet vvl;
		vvl.insert(motor);
		vvl.insert(location);
		vvl.insert(room_next);

		auto& vvi = _ur->mapVarSize();
		auto& llu = _ur->listVarSizePair;
		SizeList vvl1;
		for (auto& v : vvl)
			vvl1.push_back(vvi[v]);
	
		// EVAL(*llu[s].first);
		auto aa = *trim(*hraa(*_uu, *_ur, *_slice_history[s]));
		// EVAL(size(aa))
		// EVAL(aa);
		// EVAL(*ared(aa, VarUSet{location}));
		// EVAL(_room);
		// EVAL(_room_location_goal[_room]);			
		auto next_size = size(aa);
		EVAL(next_size);
		if (next_size > 0 && (_mode == "mode003" || _mode == "mode005"))
		{
			auto aa1 = *mul(aa,_room_location_goal[_room]);	
			// EVAL(*ared(aa1, VarUSet{motor}));		
			// EVAL(aa1);	
			auto aa2 = *sub(aa,aa1);	
			// EVAL(*ared(aa2, VarUSet{motor}));		
			// EVAL(aa2);				
			auto next_size_left = size(*mul(aa1,*single(state(motor,Value(0)),1))) - size(*mul(aa2,*single(state(motor,Value(0)),1)));		
			EVAL(next_size_left);
			auto next_size_right = size(*mul(aa1,*single(state(motor,Value(2)),1))) - size(*mul(aa2,*single(state(motor,Value(2)),1)));		
			EVAL(next_size_right);	
			auto aa3 = *single(smax(*ared(aa,VarUSet{location})),1);			
			// EVAL(aa3);
			bool single_exit = _mode == "mode003" &&
								(aa3 == *single(state(location,Value("room2")),1) 
									|| aa3 == *single(state(location,Value("room3")),1)
									|| aa3 == *single(state(location,Value("room6")),1));
			EVAL(single_exit);
			if (!single_exit && next_size_left > next_size_right && (next_size_left - next_size_right) / next_size >= _majority_fraction)
			{
				std_msgs::msg::String msg;
				msg.data = "left";
				_turn_request_pub->publish(msg);
				RCLCPP_INFO(this->get_logger(), "Published turn request: left");
			}
			else if (!single_exit && next_size_right > next_size_left && (next_size_right - next_size_left) / next_size >= _majority_fraction)
			{
				std_msgs::msg::String msg;
				msg.data = "right";
				_turn_request_pub->publish(msg);
				RCLCPP_INFO(this->get_logger(), "Published turn request: right");
			}	
		}
		else if (next_size > 0 && (_mode == "mode002" || _mode == "mode004"))
		{
			auto aa1 = *mul(aa,_room_location_goal[_room]);	
			// EVAL(*ared(aa1, VarUSet{motor}));		
			// EVAL(aa1);	
			auto aa2 = *sub(aa,aa1);	
			// EVAL(*ared(aa2, VarUSet{motor}));		
			// EVAL(aa2);				
			auto next_size_left = size(*mul(aa1,*single(state(motor,Value(0)),1))) + size(*mul(aa2,*single(state(motor,Value(2)),1)));		
			EVAL(next_size_left);
			auto next_size_right = size(*mul(aa1,*single(state(motor,Value(2)),1))) + size(*mul(aa2,*single(state(motor,Value(0)),1)));		
			EVAL(next_size_right);	
			auto aa3 = *single(smax(*ared(aa,VarUSet{location})),1);			
			// EVAL(aa3);
			bool single_exit = _mode == "mode002" &&
								(aa3 == *single(state(location,Value("room2")),1) 
									|| aa3 == *single(state(location,Value("room3")),1)
									|| aa3 == *single(state(location,Value("room6")),1));
			EVAL(single_exit);
			bool turn_left = false;
			bool turn_right = false;
			for (int i = 0; !single_exit && !turn_right && !turn_left && i < _act_factor; i++)
			{
				auto j = rand() % (int)next_size;
				turn_left = j < next_size_left;
				turn_right = j >= next_size_left && j < next_size_left + next_size_right;
			}
			if (turn_left)
			{
				std_msgs::msg::String msg;
				msg.data = "left";
				_turn_request_pub->publish(msg);
				RCLCPP_INFO(this->get_logger(), "Published turn request: left");
			}
			else if (turn_right)
			{
				std_msgs::msg::String msg;
				msg.data = "right";
				_turn_request_pub->publish(msg);
				RCLCPP_INFO(this->get_logger(), "Published turn request: right");
			}			
		}
		else if (next_size > 0)
		{
			auto aa1 = *mul(aa,_room_location_goal[_room]);	
			// EVAL(*ared(aa1, VarUSet{motor}));		
			// EVAL(aa1);			
			next_size = size(aa1);
			EVAL((int)next_size);
			auto next_size_left = size(*mul(aa1,*single(state(motor,Value(0)),1)));		
			EVAL(next_size_left);
			auto next_size_right = size(*mul(aa1,*single(state(motor,Value(2)),1)));		
			EVAL(next_size_right);
			bool turn_left = false;
			bool turn_right = false;
			for (int i = 0; next_size > 0 && !turn_right && !turn_left && i < _act_factor; i++)
			{
				auto j = rand() % (int)next_size;
				turn_left = j < next_size_left;
				turn_right = j >= next_size_left && j < next_size_left + next_size_right;
			}
			if (turn_left)
			{
				std_msgs::msg::String msg;
				msg.data = "left";
				_turn_request_pub->publish(msg);
				RCLCPP_INFO(this->get_logger(), "Published turn request: left");
			}
			else if (turn_right)
			{
				std_msgs::msg::String msg;
				msg.data = "right";
				_turn_request_pub->publish(msg);
				RCLCPP_INFO(this->get_logger(), "Published turn request: right");
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
	std::string model = string(argc >= 2 ? argv[1] : "model006_location");
	std::string room_initial = string(argc >= 3 ? argv[2] : "room1");
	std::chrono::milliseconds act_interval(argc >= 4 ? std::atol(argv[3]) : 5*60);
	string dataset = string(argc >= 5 ? argv[4] : "data002");
	std::size_t chunks(argc >= 6 ? std::atol(argv[5]) : 0);
	string mode = string(argc >= 6 ? argv[6] : "mode003");
	auto twofiftyms = 250ms;
	std::size_t act_factor((mode == "mode001" || mode == "mode002" || mode == "mode004") && argc >= 8 ? std::atol(argv[7]) : act_interval.count() / twofiftyms.count());
	double majority_fraction((mode == "mode003" || mode == "mode005") && argc >= 8 ? std::atof(argv[7]) : 0.0);

	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<Actor>(model, room_initial, act_interval, dataset, chunks, mode, act_factor, majority_fraction));
	rclcpp::shutdown();

	return 0;
}

