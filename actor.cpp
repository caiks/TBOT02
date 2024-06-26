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

#define EVAL(x) { std::ostringstream str; str << #x << ": " << (x); RCLCPP_INFO(actor_this->get_logger(), str.str());}

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

#define UNLOG ; str.flush(); RCLCPP_INFO(actor_this->get_logger(), str.str());}
#define LOG { std::ostringstream str; str << 

void actor_log(Active& active, const std::string& str)
{
	if (actor_this)
	{
		RCLCPP_INFO(actor_this->get_logger(), str);
	}
	else
		std::cout << str << std::endl;
	return;
};

void layerer_actor_log(const std::string& str)
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
	while (!actor._terminate && !active.terminate)
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

void run_act(Actor& actor)
{
	while (!actor._terminate)
	{
		auto mark = clk::now();
		if (!actor._crashed 
			&& actor._pose_updated && actor._scan_updated && actor._update_updated 
			&& actor._system)
		{
			{
				std::unique_ptr<HistoryRepa> hr;
				{
					SystemHistoryRepaTuple xx = recordListsHistoryRepa_4(8, RecordList{ actor._record });	
					hr = std::move(std::get<2>(xx));
				}
				actor._eventId++;
				actor._events->mapIdEvent[actor._eventId] = HistoryRepaPtrSizePair(std::move(hr),actor._events->references);			
				{		
					std::vector<std::thread> threadsLevel;
					threadsLevel.reserve(actor._level1.size());
					for (auto& activeA  : actor._level1)
					{
						threadsLevel.push_back(std::thread(run_update, std::ref(*activeA), actor._updateParameters));
					}
					for (auto& t : threadsLevel)
						t.join();			
				}
				for (auto& activeA  : actor._level2)
				{
					if (!activeA->terminate)		
						activeA->update(actor._updateParameters);
				}
				{		
					std::vector<std::thread> threadsLevel;
					threadsLevel.reserve(actor._level3.size());
					for (auto& activeA  : actor._level3)
					{
						threadsLevel.push_back(std::thread(run_update, std::ref(*activeA), actor._updateParameters));
					}
					for (auto& t : threadsLevel)
						t.join();			
				}
				if (actor._actLogging)
				{
					LOG "actor\tevent id: " << actor._eventId << "\ttime " << ((sec)(clk::now() - mark)).count() << "s" UNLOG								
				}		
			}	
			if (actor._struct=="struct001" && actor._mode=="mode001")
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

				bool ok = true;		
				auto& activeA = *actor._level2.front();
				if (!activeA.terminate)	
				{			
					auto mark = clk::now();
					std::lock_guard<std::mutex> guard(activeA.mutex);
					ok = ok && (activeA.historyOverflow	|| activeA.historyEvent);
					std::size_t historyEventA = ok ? (activeA.historyEvent ? activeA.historyEvent - 1 : activeA.historySize - 1) : 0;
					std::size_t sliceA = ok ? activeA.historySparse->arr[historyEventA] : 0;
					SizeSet setEventA = ok ? activeA.historySlicesSetEvent[sliceA] : SizeSet();
					std::shared_ptr<HistoryRepa> hr = ok ? activeA.underlyingHistoryRepa.front() : 0;
					// now calculate the present value for each motor value
					if (ok)
					{
						std::vector<std::string> locations{ "door12", "door13", "door14", "door45", "door56", "room1", "room2", "room3", "room4", "room5", "room6" };
						std::map<std::string,std::size_t> locationsInt;
						for (std::size_t i = 0; i < locations.size(); i++)
							locationsInt[locations[i]] = i;	
						const char turn_left = 0;
						const char ahead = 1;
						const char turn_right = 2;
						auto locationsGoal = actor._goalsLocationsNext[actor._goal];
						// EVAL(locationsGoal);
						auto over = activeA.historyOverflow;
						auto& mm = actor._ur->mapVarSize();
						auto& mvv = hr->mapVarInt();
						auto motor = mvv[mm[Variable("motor")]];
						auto location = mvv[mm[Variable("location")]];
						std::map<std::size_t, double> actionsPV;
						auto n = hr->dimension;
						auto z = hr->size;
						auto y = activeA.historyEvent;
						auto rr = hr->arr;	
						double	steps = 0.0;
						if (actor._mode1GuessLocation)
						{
							std::map<std::size_t, std::size_t> locationsCurr;
							for (auto ev : setEventA)
								locationsCurr[rr[ev*n+location]]++;
							std::size_t locationMax = 0;
							std::size_t locationCount = 0;
							for (auto& p : locationsCurr)
							{
								if (locationCount < p.second)
								{
									locationCount = p.second;
									locationMax = p.first;
								}
							}
							// EVAL(locations[locationMax]);
							// EVAL(locationsGoal[locations[locationMax]]);
							SizeSet setEventB;
							for (auto ev : setEventA)
								if (rr[ev*n+location] == locationMax)
									setEventB.insert(ev);
							setEventA = setEventB;
						}
						if (actor._mode1Shortest)
						{
							std::size_t shortest = 0;
							bool shortestFound = false;
							for (auto ev : setEventA)
							{
								auto curr = rr[ev*n+location];
								auto next = locationsInt[locationsGoal[locations[curr]]];
								if (next == curr)
									continue;
								auto j = ev + 1;	
								while ((j < y || (over && j > y && j < y+z)))
								{
									auto loc = rr[(j%z)*n+location];
									if (loc == next)
									{
										shortest = shortestFound ? std::min(shortest, j-ev) : j-ev;
										shortestFound = true;
									}
									if (loc != curr)
										break;
									j++;
								}
							}	
							if (shortest)
								steps = (double)shortest;
						}
						else
						{
							std::size_t stepsCount = 0;
							for (auto ev : setEventA)
							{
								auto curr = rr[ev*n+location];
								auto next = locationsInt[locationsGoal[locations[curr]]];
								if (next == curr)
									continue;
								auto j = ev + 1;	
								while ((j < y || (over && j > y && j < y+z)))
								{
									auto loc = rr[(j%z)*n+location];
									if (loc == next)
									{
										steps += j-ev;
										stepsCount++;
									}
									if (loc != curr)
										break;
									j++;
								}
							}	
							if (stepsCount)
								steps /= stepsCount;
						}
						// EVAL(steps);
						if (steps > 0.0)
						{
							std::map<std::size_t, double> actionsCount;
							double discount = actor._mode1DiscountRate / steps;
							for (auto ev : setEventA)
							{
								auto curr = rr[ev*n+location];
								auto next = locationsInt[locationsGoal[locations[curr]]];
								if (next == curr)
									continue;
								auto j = ev + 1;	
								bool nextFound = false;
								bool found = false;
								while ((j < y || (over && j > y && j < y+z)))
								{
									auto loc = rr[(j%z)*n+location];
									nextFound = loc == next;
									if (loc != curr)
									{
										found = true;
										break;
									}
									j++;
								}
								if (!found)
									continue;
								auto action = rr[ev*n+motor];
								if (nextFound)
								{
									actionsCount[action] += 1.0;
									actionsPV[action] += std::exp(-1.0 * discount * (j-ev));
								}
								else if (actor._mode1Repulsive && action == turn_left)
								{
									actionsCount[turn_right] += 1.0;
									actionsPV[turn_right] += std::exp(-1.0 * discount * (j-ev));
								}
								else if (actor._mode1Repulsive && action == turn_right)
								{
									actionsCount[turn_left] += 1.0;
									actionsPV[turn_left] += std::exp(-1.0 * discount * (j-ev));
								}
								else if (actor._mode1Repulsive)
								{
									actionsCount[turn_left] += actor._mode1Turnaway * 0.5;
									actionsPV[turn_left] += actor._mode1Turnaway * 0.5 * std::exp(-1.0 * discount * (j-ev));
									actionsCount[turn_right] += actor._mode1Turnaway * 0.5;
									actionsPV[turn_right] += actor._mode1Turnaway * 0.5 * std::exp(-1.0 * discount * (j-ev));
								}
							}	
							if (actor._mode1ExpectedPV)
							{
								for (auto& p : actionsCount)
									if (p.second > 0.0)
										actionsPV[p.first] /= p.second;						
							}
							double norm = 0.0;
							for (auto& p : actionsPV)
								norm += p.second;	
							for (auto& p : actionsPV)
								actionsPV[p.first] /= norm;							
							// EVAL(actionsPV);
							// EVAL(actionsCount);				
						}
						if (steps > 0.0)
						{
							char action = ahead;
							if (actor._mode1Probabilistic)
							{
								auto r = (double) rand() / (RAND_MAX);
								double accum = 0.0;
								for (auto& p : actionsPV)
								{
									accum += p.second;
									if (r < accum)
									{
										action = p.first;
										break;
									}
								}						
							}
							else
							{
								if (actionsPV[turn_left] > actionsPV[ahead] && actionsPV[turn_left] > actionsPV[turn_right])
									action = turn_left;
								else if (actionsPV[turn_right] > actionsPV[ahead] && actionsPV[turn_right] > actionsPV[turn_left])
									action = turn_right;	
							}	
							// locking here? TODO
							if (action == turn_left)
							{
								actor._turn_request = "left";
								actor._bias_right = false;
							}
							else if (action == turn_right)
							{
								actor._turn_request = "right";
								actor._bias_right = true;
							}					
						}
					}
					if (ok && actor._modeLogging)
					{
						std::size_t sizeA = activeA.historyOverflow ? activeA.historySize : activeA.historyEvent;
						LOG activeA.name << "\t" << actor._mode << "\trequest: " << actor._turn_request << "\tfuds cardinality: " << activeA.decomp->fuds.size() << "\tmodel cardinality: " << activeA.decomp->fudRepasSize << "\tactive size: " << sizeA << "\tfuds per threshold: " << (double)activeA.decomp->fuds.size() * activeA.induceThreshold / sizeA << "\ttime " << ((sec)(clk::now() - mark)).count() << "s" UNLOG								
					}
				}		
			}
			else if (actor._struct=="struct001" && actor._mode=="mode002")
			{		
				bool ok = true;		
				auto& activeA = *actor._level2.front();
				if (!activeA.terminate)	
				{			
					auto mark = clk::now();
					std::lock_guard<std::mutex> guard(activeA.mutex);
					ok = ok && (activeA.historyOverflow	|| activeA.historyEvent);
					if (ok)
					{
						auto historyEventA = activeA.historyEvent ? activeA.historyEvent - 1 : activeA.historySize - 1;
						auto sliceA = activeA.historySparse->arr[historyEventA];
						auto setEventA = activeA.historySlicesSetEvent[sliceA];
						std::shared_ptr<HistoryRepa> hr = activeA.underlyingHistoryRepa.front();
						auto& hs = *activeA.historySparse;
						std::vector<std::string> locations{ "door12", "door13", "door14", "door45", "door56", "room1", "room2", "room3", "room4", "room5", "room6" };
						std::map<std::string,std::size_t> locationsInt;
						for (std::size_t i = 0; i < locations.size(); i++)
							locationsInt[locations[i]] = i;	
						std::size_t goal = locationsInt[actor._goal];
						const char turn_left = 0;
						const char ahead = 1;
						const char turn_right = 2;
						auto over = activeA.historyOverflow;
						auto& mm = actor._ur->mapVarSize();
						auto& mvv = hr->mapVarInt();
						auto motor = mvv[mm[Variable("motor")]];
						auto location = mvv[mm[Variable("location")]];
						auto n = hr->dimension;
						auto z = hr->size;
						auto y = activeA.historyEvent;
						auto rr = hr->arr;	
						auto rs = hs.arr;
						auto sliceCount = activeA.historySlicesSetEvent.size();
						std::unordered_map<std::size_t, std::size_t> slicesLocation;
						slicesLocation.reserve(sliceCount);				
						{
							for (auto& p : activeA.historySlicesSetEvent)
							{
								std::map<std::size_t, std::size_t> locsCount;
								for (auto ev : p.second)
									locsCount[rr[ev*n+location]]++;
								std::size_t most = 0;
								std::size_t loc = 0;
								for (auto& q : locsCount)	
									if (!most || most < q.second)
									{
										most = q.second;
										loc = q.first;
									}
								slicesLocation.insert_or_assign(p.first,loc);
							}
						}
						std::unordered_map<std::size_t, SizeSet> slicesSliceSetPrev;
						slicesSliceSetPrev.reserve(sliceCount);
						std::unordered_map<std::size_t, SizeSet> slicesEventSetNext;
						slicesEventSetNext.reserve(sliceCount);
						{
							auto j = over ? y : z;	
							auto eventB = j%z;
							auto sliceB = rs[j%z];
							auto locB = rr[(j%z)*n+location];
							j++;
							while (j < y+z)
							{
								auto sliceC = rs[j%z];
								auto locC = rr[(j%z)*n+location];
								if (sliceC != sliceB)
								{
									if (slicesLocation[sliceB] == locB && slicesLocation[sliceC] == locC)
									{
										slicesSliceSetPrev[sliceC].insert(sliceB);
										if (sliceB == sliceA)
											slicesEventSetNext[sliceC].insert(eventB);
									}
									sliceB = sliceC;
									locB = locC;
									eventB = j%z;
								}
								j++;
							}					
						}
						std::unordered_map<std::size_t, std::size_t> slicesStepCount;
						slicesStepCount.reserve(sliceCount);
						SizeUSet sliceCurrents;
						sliceCurrents.reserve(sliceCount);			
						for (auto& p : slicesLocation)
							if (p.second == goal)
							{
								slicesStepCount.insert_or_assign(p.first,0);
								sliceCurrents.insert(p.first);
							}
						// EVAL(sliceCurrents.size());
						// EVAL(sliceCount);
						while (sliceCurrents.size())
						{
							SizeList sliceCurrentBs;
							sliceCurrentBs.reserve(sliceCount);
							for (auto sliceB : sliceCurrents)		
							{
								auto countC = slicesStepCount[sliceB] + 1;
								for (auto sliceC : slicesSliceSetPrev[sliceB])
								{							
									auto it = slicesStepCount.find(sliceC);
									if (it == slicesStepCount.end())
									{
										slicesStepCount.insert_or_assign(sliceC, countC);
										sliceCurrentBs.push_back(sliceC);
									}															
								}
							}
							sliceCurrents.clear();
							sliceCurrents.insert(sliceCurrentBs.begin(), sliceCurrentBs.end());
						}
						// EVAL(slicesStepCount);
						// EVAL(sliceA);
						// {
							// std::map<std::string, std::size_t> locsCount;
							// auto j = over ? y : z;	
							// while (j < y+z)
							// {
								// if (rs[j%z] == sliceA)
									// locsCount[locations[rr[(j%z)*n+location]]]++;
								// j++;
							// }	
							// EVAL(locsCount);
						// }
						std::map<std::size_t, std::size_t> actionsCount;
						{
							std::size_t least = 0;
							for (auto& p : slicesEventSetNext)
							{
								auto it = slicesStepCount.find(p.first);
								if (it != slicesStepCount.end())
									least = least ? std::min(least,it->second) : it->second;
							}
							// EVAL(least);
							for (auto& p : slicesEventSetNext)
							{
								auto it = slicesStepCount.find(p.first);
								if (least && it != slicesStepCount.end() && least == it->second)
								{
									{
									// EVAL(p.first);
									// EVAL(slicesStepCount[p.first]);
									// {
										// std::map<std::string, std::size_t> locsCount;
										// auto j = over ? y : z;	
										// while (j < y+z)
										// {
											// if (rs[j%z] == p.first)
												// locsCount[locations[rr[(j%z)*n+location]]]++;
											// j++;
										// }	
										// EVAL(locsCount);
									// }
									// for (auto& q : slicesSliceSetPrev)
									// {
										// for (auto& sliceB : q.second)		
											// if (sliceB == p.first && slicesStepCount[q.first] == 0)
											// {
												// EVAL(q.first);
												// // EVAL(slicesGoalCount[q.first] > activeA.historySlicesSetEvent[q.first].size() / 2);
												// {
													// std::map<std::string, std::size_t> locsCount;
													// auto j = over ? y : z;	
													// while (j < y+z)
													// {
														// if (rs[j%z] == q.first)
															// locsCount[locations[rr[(j%z)*n+location]]]++;
														// j++;
													// }	
													// EVAL(locsCount);
												// }
											// }
									// }								
									}
									for (auto ev : p.second)
										actionsCount[rr[ev*n+motor]]++;
								}
							}					
							// EVAL(actionsCount);
						}
						if (actionsCount.size())
						{
							char action = ahead;
							if (actor._modeProbabilistic)
							{
								std::size_t total = 0;
								for (auto& p : actionsCount)	
									total += p.second;
								auto r = rand() % total;
								std::size_t accum = 0.0;
								for (auto& p : actionsCount)
								{
									accum += p.second;
									if (r < accum)
									{
										action = p.first;
										break;
									}
								}						
							}
							else
							{
								if (actionsCount[turn_left] > actionsCount[ahead] && actionsCount[turn_left] > actionsCount[turn_right])
									action = turn_left;
								else if (actionsCount[turn_right] > actionsCount[ahead] && actionsCount[turn_right] > actionsCount[turn_left])
									action = turn_right;	
							}	
							// locking here? TODO
							if (action == turn_left)
							{
								actor._turn_request = "left";
								actor._bias_right = false;
							}
							else if (action == turn_right)
							{
								actor._turn_request = "right";
								actor._bias_right = true;
							}					
						}
					}
					if (ok && actor._modeLogging)
					{
						std::size_t sizeA = activeA.historyOverflow ? activeA.historySize : activeA.historyEvent;
						LOG activeA.name << "\t" << actor._mode << "\trequest: " << actor._turn_request << "\tfuds cardinality: " << activeA.decomp->fuds.size() << "\tmodel cardinality: " << activeA.decomp->fudRepasSize << "\tactive size: " << sizeA << "\tfuds per threshold: " << (double)activeA.decomp->fuds.size() * activeA.induceThreshold / sizeA << "\ttime " << ((sec)(clk::now() - mark)).count() << "s" UNLOG								
					}
				}		
			}
			else if (actor._struct=="struct001" && actor._mode=="mode003")
			{		
				bool ok = true;		
				auto& activeA = *actor._level2.front();
				if (!activeA.terminate)	
				{			
					auto mark = clk::now();
					std::lock_guard<std::mutex> guard(activeA.mutex);
					ok = ok && (activeA.historyOverflow	|| activeA.historyEvent);
					if (ok)
					{
						auto historyEventA = activeA.historyEvent ? activeA.historyEvent - 1 : activeA.historySize - 1;
						auto sliceA = activeA.historySparse->arr[historyEventA];
						std::shared_ptr<HistoryRepa> hr = activeA.underlyingHistoryRepa.front();
						auto& hs = *activeA.historySparse;
						std::vector<std::string> locations{ "door12", "door13", "door14", "door45", "door56", "room1", "room2", "room3", "room4", "room5", "room6" };
						std::map<std::string,std::size_t> locationsInt;
						for (std::size_t i = 0; i < locations.size(); i++)
							locationsInt[locations[i]] = i;	
						std::size_t goal = locationsInt[actor._goal];
						const char turn_left = 0;
						const char ahead = 1;
						const char turn_right = 2;
						auto over = activeA.historyOverflow;
						auto& mm = actor._ur->mapVarSize();
						auto& mvv = hr->mapVarInt();
						auto motor = mvv[mm[Variable("motor")]];
						auto location = mvv[mm[Variable("location")]];
						auto n = hr->dimension;
						auto z = hr->size;
						auto y = activeA.historyEvent;
						auto rr = hr->arr;	
						auto rs = hs.arr;
						auto sliceCount = activeA.historySlicesSetEvent.size();
						std::map<std::size_t, std::size_t> neighbours;
						for (auto sliceB : actor._mode3SlicesSliceSetNext[sliceA])
						{
							std::size_t steps = 1;
							if (actor._mode3SlicesLocation[sliceB] != goal)
							{
								bool found = false;
								SizeSet slicePrevious;	
								slicePrevious.insert(sliceA);							
								slicePrevious.insert(sliceB);							
								SizeSet sliceCurrents;		
								sliceCurrents.insert(sliceB);
								while (!found && sliceCurrents.size())
								{
									steps++;
									SizeList sliceCurrentBs;
									sliceCurrentBs.reserve(sliceCurrents.size() * 20);
									for (auto sliceC : sliceCurrents)		
									{
										for (auto sliceD : actor._mode3SlicesSliceSetNext[sliceC])
										{					
											if (slicePrevious.find(sliceD) == slicePrevious.end())
											{
												if (actor._mode3SlicesLocation[sliceD] == goal)
												{
													found = true;
													break;
												}
												sliceCurrentBs.push_back(sliceD);			
												slicePrevious.insert(sliceD);			
											}
										}
										if (found)
											break;
									}	
									if (found)
										break;	
									sliceCurrents.clear();
									sliceCurrents.insert(sliceCurrentBs.begin(), sliceCurrentBs.end());
								}
								if (found)
									neighbours[sliceB] = steps;
							}
							else
								neighbours[sliceB] = steps;
						}
						// EVAL(neighbours);
						std::set<std::size_t> neighbourLeasts;
						{
							std::size_t least = 0;
							for (auto& p : neighbours)	
								if (!least || least > p.second)
									least = p.second;
							for (auto& p : neighbours)	
								if (p.second == least)	
									neighbourLeasts.insert(p.first);
							// EVAL(least);
						}
						// EVAL(neighbourLeasts);
						std::map<std::size_t, std::size_t> actionsCount;
						{
							for (auto ev : activeA.historySlicesSetEvent[sliceA])
							{
								auto j = ev + (ev >= y ? 0 : z)  + 1;	
								while (j < y+z)
								{
									auto sliceB = rs[j%z];
									if (sliceB != sliceA)
									{
										if (neighbourLeasts.find(sliceB) != neighbourLeasts.end())
											actionsCount[rr[ev*n+motor]]++;
										break;
									}
									j++;
								}									
							}
							
						}
						// EVAL(actionsCount);
						if (actionsCount.size())
						{
							char action = ahead;
							if (actor._modeProbabilistic)
							{
								std::size_t total = 0;
								for (auto& p : actionsCount)	
									total += p.second;
								auto r = rand() % total;
								std::size_t accum = 0.0;
								for (auto& p : actionsCount)
								{
									accum += p.second;
									if (r < accum)
									{
										action = p.first;
										break;
									}
								}						
							}
							else
							{
								if (actionsCount[turn_left] * actor._acts_per_turn * 2 > actionsCount[ahead] && actionsCount[turn_left] > actionsCount[turn_right])
									action = turn_left;
								else if (actionsCount[turn_right] * actor._acts_per_turn * 2 > actionsCount[ahead] && actionsCount[turn_right] > actionsCount[turn_left])
									action = turn_right;	
							}	
							// locking here? TODO
							if (action == turn_left)
							{
								actor._turn_request = "left";
								actor._bias_right = false;
							}
							else if (action == turn_right)
							{
								actor._turn_request = "right";
								actor._bias_right = true;
							}					
						}
					}
					if (ok && actor._modeLogging)
					{
						std::size_t sizeA = activeA.historyOverflow ? activeA.historySize : activeA.historyEvent;
						LOG activeA.name << "\t" << actor._mode << "\trequest: " << actor._turn_request << "\tfuds cardinality: " << activeA.decomp->fuds.size() << "\tmodel cardinality: " << activeA.decomp->fudRepasSize << "\tactive size: " << sizeA << "\tfuds per threshold: " << (double)activeA.decomp->fuds.size() * activeA.induceThreshold / sizeA << "\ttime " << ((sec)(clk::now() - mark)).count() << "s" UNLOG								
					}
				}		
			}
			else if (actor._struct=="struct001" && actor._mode=="mode004")
			{		
				bool ok = true;		
				auto& activeA = *actor._level2.front();
				if (!activeA.terminate)	
				{			
					auto mark = clk::now();
					std::lock_guard<std::mutex> guard(activeA.mutex);
					ok = ok && (activeA.historyOverflow	|| activeA.historyEvent);
					if (ok)
					{
						std::vector<std::string> locations{ "door12", "door13", "door14", "door45", "door56", "room1", "room2", "room3", "room4", "room5", "room6" };
						auto nloc = locations.size();
						std::map<std::string,std::size_t> locationsInt;
						for (std::size_t i = 0; i < locations.size(); i++)
							locationsInt[locations[i]] = i;	

						auto historyEventA = activeA.historyEvent ? activeA.historyEvent - 1 : activeA.historySize - 1;
						auto sliceA = activeA.historySparse->arr[historyEventA];
						std::shared_ptr<HistoryRepa> hr = activeA.underlyingHistoryRepa.front();
						auto& hs = *activeA.historySparse;
						std::size_t goal = locationsInt[actor._goal];
						const char turn_left = 0;
						const char ahead = 1;
						const char turn_right = 2;
						auto over = activeA.historyOverflow;
						auto& mm = actor._ur->mapVarSize();
						auto& mvv = hr->mapVarInt();
						auto motor = mvv[mm[Variable("motor")]];
						auto location = mvv[mm[Variable("location")]];
						auto n = hr->dimension;
						auto z = hr->size;
						auto y = activeA.historyEvent;
						auto rr = hr->arr;	
						auto rs = hs.arr;
						auto locA = rr[historyEventA*n+location];
						auto sliceLocA = sliceA*nloc+locA;
						auto sliceCount = activeA.historySlicesSetEvent.size();
						std::map<std::size_t, std::size_t> neighbours;
						if (!actor._mode4Caching)
						{
							for (auto sliceLocB : actor._mode4SlicesSliceSetNext[sliceLocA])
							{
								std::size_t steps = 1;
								if (sliceLocB % nloc != goal)
								{
									bool found = false;
									SizeUSet slicePrevious;	
									slicePrevious.reserve(sliceCount);
									slicePrevious.insert(sliceLocA);							
									slicePrevious.insert(sliceLocB);							
									SizeUSet sliceCurrents;		
									sliceCurrents.reserve(sliceCount);
									sliceCurrents.insert(sliceLocB);
									while (!found && sliceCurrents.size())
									{
										steps++;
										SizeList sliceCurrentBs;
										sliceCurrentBs.reserve(sliceCurrents.size() * 20);
										for (auto sliceLocC : sliceCurrents)		
										{
											for (auto sliceLocD : actor._mode4SlicesSliceSetNext[sliceLocC])
											{					
												if (slicePrevious.find(sliceLocD) == slicePrevious.end())
												{
													if (sliceLocD % nloc == goal)
													{
														found = true;
														break;
													}
													sliceCurrentBs.push_back(sliceLocD);			
													slicePrevious.insert(sliceLocD);			
												}
											}
											if (found)
												break;
										}	
										if (found)
											break;	
										sliceCurrents.clear();
										sliceCurrents.insert(sliceCurrentBs.begin(), sliceCurrentBs.end());
									}
									if (found)
										neighbours[sliceLocB] = steps;
								}
								else
									neighbours[sliceLocB] = steps;
							}
						}
						// EVAL(neighbours);							
						else
						{
							// neighbours.clear();
							auto& slicesStepCount = actor._mode4locationsSlicesStepCount[goal];
							for (auto sliceLocB : actor._mode4SlicesSliceSetNext[sliceLocA])
							{
								auto it = slicesStepCount.find(sliceLocB);
								if (it != slicesStepCount.end())
									neighbours[sliceLocB] = it->second;								
							}
						}
						// EVAL(sliceA);							
						// EVAL(locA);							
						EVAL(sliceLocA);							
						EVAL(neighbours);							
						std::set<std::size_t> neighbourLeasts;
						{
							bool found = false;
							std::size_t least = 0;
							for (auto& p : neighbours)	
								if (!found || least > p.second)
								{
									least = p.second;
									found = true;
								}
							for (auto& p : neighbours)	
								if (p.second == least)	
									neighbourLeasts.insert(p.first);
							EVAL(least);
						}
						EVAL(neighbourLeasts);
						if (sliceLocA != actor._mode4SliceLocA)
						{
							if (actor._mode4Neighbours.find(sliceLocA) == actor._mode4Neighbours.end())
								actor._mode4TransistionNullCount++;
							else 
							{
								if (actor._mode4NeighbourLeasts.find(sliceLocA) != actor._mode4NeighbourLeasts.end())
									actor._mode4TransistionSuccessCount++;
								actor._mode4TransistionExpectedSuccessCount += (double) actor._mode4NeighbourLeasts.size() / (double) actor._mode4Neighbours.size();
							}
							actor._mode4TransistionCount++;
							double transition_success_rate = (double) actor._mode4TransistionSuccessCount * 100.0 / (double) actor._mode4TransistionCount;
							double transition_expected_success_rate = (double) actor._mode4TransistionExpectedSuccessCount * 100.0 / (double) actor._mode4TransistionCount;
							double transition_null_rate = (double) actor._mode4TransistionNullCount * 100.0 / (double) actor._mode4TransistionCount;
							EVAL(transition_success_rate);
							EVAL(transition_expected_success_rate);
							EVAL(transition_null_rate);
							actor._mode4SliceLocA = sliceLocA;
							actor._mode4NeighbourLeasts = neighbourLeasts;
							actor._mode4Neighbours.clear();
							for (auto& p : neighbours)
								actor._mode4Neighbours.insert(p.first);
						}
						if (neighbourLeasts.size() && neighbourLeasts.size() < neighbours.size())
						{
							std::map<std::size_t, std::size_t> actionsCount;
							if (actor._mode4Stepwise)
							{
								std::map<std::size_t, std::map<std::size_t, std::size_t>> stepsActionsCount;
								std::size_t stepsEventA = 0;
								for (auto ev : activeA.historySlicesSetEvent[sliceA])
								{
									if (rr[ev*n+location] == locA)
									{
										std::size_t steps = 0;
										{
											auto j = ev + (ev >= y ? 0 : z) - 1;	
											while (j >= (over ? y : z))
											{
												auto sliceLocB = rs[j%z]*nloc + rr[(j%z)*n+location];
												if (sliceLocB != sliceLocA)
													break;
												j--;
												steps++;
											}												
										}
										if (historyEventA == ev)
											stepsEventA = steps;
										{
											auto j = ev + (ev >= y ? 0 : z)  + 1;	
											while (j < y+z)
											{
												auto sliceLocB = rs[j%z]*nloc + rr[(j%z)*n+location];
												if (sliceLocB != sliceLocA)
												{
													if (neighbourLeasts.find(sliceLocB) != neighbourLeasts.end())
														stepsActionsCount[steps][rr[((ev+actor._mode4Lag)%z)*n+motor]]++;
													break;
												}
												j++;
											}												
										}
									}
								}	
								EVAL(stepsEventA);
								EVAL(stepsActionsCount);
								if (stepsActionsCount.size())
								{
									auto it = stepsActionsCount.find(stepsEventA);
									if (it != stepsActionsCount.end())
										actionsCount = it->second;
									else			
										actionsCount = stepsActionsCount.rbegin()->second;
								}								
							}
							else
							{
								for (auto ev : activeA.historySlicesSetEvent[sliceA])
								{
									if (rr[ev*n+location] == locA)
									{
										auto j = ev + (ev >= y ? 0 : z)  + 1;	
										while (j < y+z)
										{
											auto sliceLocB = rs[j%z]*nloc + rr[(j%z)*n+location];
											if (sliceLocB != sliceLocA)
											{
												if (neighbourLeasts.find(sliceLocB) != neighbourLeasts.end())
													actionsCount[rr[((ev+actor._mode4Lag)%z)*n+motor]]++;
												break;
											}
											j++;
										}										
									}
								}
							}
							EVAL(actionsCount);
							if (actionsCount.size())
							{
								char action = ahead;
								if (actor._modeProbabilistic)
								{
									std::size_t total = 0;
									for (auto& p : actionsCount)	
										total += p.second;
									auto r = rand() % total;
									std::size_t accum = 0.0;
									for (auto& p : actionsCount)
									{
										accum += p.second;
										if (r < accum)
										{
											action = p.first;
											break;
										}
									}						
								}
								else
								{
									if (actionsCount[turn_left] * actor._acts_per_turn * 2 > actionsCount[ahead] && actionsCount[turn_left] > actionsCount[turn_right])
										action = turn_left;
									else if (actionsCount[turn_right] * actor._acts_per_turn * 2 > actionsCount[ahead] && actionsCount[turn_right] > actionsCount[turn_left])
										action = turn_right;	
								}	
								// locking here? TODO
								if (action == turn_left)
								{
									actor._turn_request = "left";
									actor._bias_right = false;
								}
								else if (action == turn_right)
								{
									actor._turn_request = "right";
									actor._bias_right = true;
								}					
							}							
						}
					}
					if (ok && actor._modeLogging)
					{
						std::size_t sizeA = activeA.historyOverflow ? activeA.historySize : activeA.historyEvent;
						LOG activeA.name << "\t" << actor._mode << "\trequest: " << actor._turn_request << "\tfuds cardinality: " << activeA.decomp->fuds.size() << "\tmodel cardinality: " << activeA.decomp->fudRepasSize << "\tactive size: " << sizeA << "\tfuds per threshold: " << (double)activeA.decomp->fuds.size() * activeA.induceThreshold / sizeA << "\ttime " << ((sec)(clk::now() - mark)).count() << "s" UNLOG								
					}
				}		
			}
		}
		auto t = clk::now() - mark;
		if (t < actor._actInterval)
		{
			std::this_thread::sleep_for(actor._actInterval-t);
		}
		else if (actor._actWarning)		
		{
			LOG "actor warning\ttime " << ((sec)(clk::now() - mark)).count() << "s" UNLOG
		}		
	}
};

Actor::Actor(const std::string& args_filename)
: Node("TBOT02_actor_node")
{
	typedef std::tuple<std::string, std::string, std::string> String3;	
	typedef std::vector<String3> String3List;	
	auto add = pairHistogramsAdd_u;
	auto single = histogramSingleton_u;		

	_terminate = false;
			
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
	_actLogging = ARGS_BOOL(logging_action);
	_actWarning = ARGS_BOOL(warning_action);
	std::chrono::milliseconds updateInterval = (std::chrono::milliseconds)(ARGS_INT_DEF(update_interval,10));
	std::chrono::milliseconds biasInterval = (std::chrono::milliseconds)(ARGS_INT_DEF(bias_interval,0));
	std::chrono::milliseconds turnInterval = (std::chrono::milliseconds)(ARGS_INT_DEF(turn_interval,0));
	_bias_right = true;
	_bias_factor = biasInterval.count() / updateInterval.count();
	_turn_factor = turnInterval.count() / updateInterval.count();
	_eventId = 0;
	_actInterval = (std::chrono::milliseconds)(ARGS_INT_DEF(act_interval,250));
	_acts_per_turn = ARGS_INT_DEF(acts_per_turn,20);
	_goal = ARGS_STRING_DEF(goal_initial,"room5");
	_struct = ARGS_STRING(structure);
	_model = ARGS_STRING(model);
	std::string modelInitial = ARGS_STRING(model_initial);
	std::string structInitial = ARGS_STRING(structure_initial);
	_induceThreadCount = ARGS_INT_DEF(induceThreadCount,4);
	_level1Count = ARGS_INT_DEF(level1Count,12);
	bool level1Logging = ARGS_BOOL(logging_level1);
	bool level1Summary = ARGS_BOOL(summary_level1);
	std::size_t activeSizeLevel1 = ARGS_INT_DEF(activeSizeLevel1,10000);
	std::size_t induceThresholdLevel1 = ARGS_INT_DEF(induceThresholdLevel1,100);
	std::size_t induceThresholdInitialLevel1 = ARGS_INT_DEF(induceThresholdInitialLevel1,500);
	std::chrono::milliseconds induceIntervalLevel1 = (std::chrono::milliseconds)(ARGS_INT_DEF(induceIntervalLevel1,10));
	bool level2Logging = ARGS_BOOL(logging_level2);
	bool level2Summary = ARGS_BOOL(summary_level2);
	std::size_t activeSize = ARGS_INT_DEF(activeSize,1000000);
	std::size_t induceThreshold = ARGS_INT_DEF(induceThreshold,100);
	std::size_t induceThresholdInitial = ARGS_INT_DEF(induceThresholdInitial,1000);
	std::chrono::milliseconds induceInterval = (std::chrono::milliseconds)(ARGS_INT_DEF(induce_interval,10));	
	bool level3Logging = ARGS_BOOL(logging_level3);
	bool level3Summary = ARGS_BOOL(summary_level3);
	bool induceNot = ARGS_BOOL(no_induce);
	_mode = ARGS_STRING(mode);	
	_modeLogging = ARGS_BOOL(mode_logging);	
	_mode1DiscountRate = ARGS_DOUBLE_DEF(discount_rate,3.0);
	_mode1Turnaway = ARGS_DOUBLE_DEF(turn_away_probability,0.0);
	_mode1Probabilistic = ARGS_BOOL(probabilistic_pv);
	_mode1Shortest = ARGS_BOOL(shortest_success);
	_mode1ExpectedPV = ARGS_BOOL(expected_pv);
	_mode1Repulsive = ARGS_BOOL_DEF(repulsive,true);
	_mode1GuessLocation = ARGS_BOOL_DEF(guess_location,true);
	_modeProbabilistic = ARGS_BOOL(probabilistic);
	_modeMultipleTransition = ARGS_BOOL(multiple_transition);
	_mode4Caching = ARGS_BOOL_DEF(caching,true);
	_mode4Lag = ARGS_INT_DEF(lag,1);
	_mode4Stepwise = ARGS_BOOL(stepwise);
	_mode4TransistionSuccessCount = 0;
	_mode4TransistionCount = 0;
	_mode4TransistionExpectedSuccessCount = 0.0;
	_mode4TransistionNullCount = 0;
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

	actor_this = this;
		
	EVAL(_goal);
	EVAL(_struct);
	EVAL(_model);
	EVAL(_mode);
	
	if (_struct=="struct001" || _struct=="struct002")
	{
		std::unique_ptr<HistoryRepa> hr;
		{
			SystemHistoryRepaTuple xx = recordListsHistoryRepa_4(8, RecordList{ Record() });	
			_uu = std::move(std::get<0>(xx));
			_ur = std::move(std::get<1>(xx));
			hr = std::move(std::get<2>(xx));
		}
		_system = std::make_shared<ActiveSystem>();
		_threads.reserve(1+_level1Count+1+6);
		_events = std::make_shared<ActiveEventsRepa>(_level1Count+1);
		for (std::size_t m = 0; m < _level1Count; m++)
			_level1.push_back(std::make_shared<Active>());
		for (std::size_t m = 0; m < _level1Count; m++)
		{			
			auto& activeA = *_level1[m];
			activeA.log = actor_log;
			activeA.layerer_log = layerer_actor_log;
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
			activeA.summary = level1Summary;
			activeA.underlyingEventsRepa.push_back(_events);
			activeA.eventsSparse = std::make_shared<ActiveEventsArray>(1);
			std::size_t sizeA = activeA.historyOverflow ? activeA.historySize : activeA.historyEvent;
			if (sizeA)
			{
				LOG activeA.name << "\tfuds cardinality: " << activeA.decomp->fuds.size() << "\tmodel cardinality: " << activeA.decomp->fudRepasSize << "\tactive size: " << sizeA << "\tfuds per threshold: " << (double)activeA.decomp->fuds.size() * activeA.induceThreshold / sizeA UNLOG				
			}
			if (!induceNot)
				_threads.push_back(std::thread(run_induce, std::ref(*this), std::ref(activeA), induceIntervalLevel1, induceThresholdInitialLevel1));
		}	
		{
			_level2.push_back(std::make_shared<Active>());
			auto& activeA = *_level2.front();
			activeA.log = actor_log;
			activeA.layerer_log = layerer_actor_log;
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
			activeA.summary = level2Summary;
			activeA.underlyingEventsRepa.push_back(_events);
			for (std::size_t m = 0; m < _level1Count; m++)
			{
				auto& activeB = *_level1[m];
				activeA.underlyingEventsSparse.push_back(activeB.eventsSparse);
			}
			if (_struct=="struct002")
				activeA.eventsSparse = std::make_shared<ActiveEventsArray>(0);	
			std::size_t sizeA = activeA.historyOverflow ? activeA.historySize : activeA.historyEvent;
			if (sizeA)
			{
				LOG activeA.name << "\tfuds cardinality: " << activeA.decomp->fuds.size() << "\tmodel cardinality: " << activeA.decomp->fudRepasSize << "\tactive size: " << sizeA << "\tfuds per threshold: " << (double)activeA.decomp->fuds.size() * activeA.induceThreshold / sizeA UNLOG				
			}			
			if (!induceNot)
				_threads.push_back(std::thread(run_induce, std::ref(*this), std::ref(activeA), induceInterval, induceThresholdInitial));			
		}
		if (_struct=="struct002")
		{
			std::vector<SizeList> under 
			{
				SizeList{0,1,3,6,10,15,21,28,36}, 
				SizeList{0,1,3}, 
				SizeList{0,1,3,6,10}, 
				SizeList{0,1,3,6,10,15,21,28,36}, 
				SizeList{0,2,4,8,16,32}, 
				SizeList{0,2,4,8,16,32}
			};
			std::vector<SizeList> self 
			{
				SizeList{}, 
				SizeList{6,10,15}, 
				SizeList{16,32,64}, 
				SizeList{4,8,16,32,64}, 
				SizeList{}, 
				SizeList{16,32,64}
			};			
			for (std::size_t m = 0; m < under.size(); m++)
			{
				_level3.push_back(std::make_shared<Active>());
				auto& activeA = *_level3.back();
				activeA.log = actor_log;
				activeA.layerer_log = layerer_actor_log;
				activeA.system = _system;
				if (modelInitial.size() && structInitial != "struct001")
				{
					ActiveIOParameters ppio;
					ppio.filename = modelInitial + "_3_" + (m<10 ? "0" : "") + std::to_string(m) +".ac";
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
					{
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
				activeA.name = (_model!="" ? _model : "model") + "_3_" + (m<10 ? "0" : "") + std::to_string(m);
				activeA.logging = level3Logging;
				activeA.summary = level3Summary;
				activeA.underlyingEventsRepa.push_back(_events);
				_events->references++;
				{
					auto& activeB = *_level2.back();
					activeA.underlyingEventsSparse.push_back(activeB.eventsSparse);
					activeB.eventsSparse->references++;
				}
				// activeA.eventsSparse = std::make_shared<ActiveEventsArray>(1);	
				for (auto i : under[m])
					activeA.frameUnderlyings.push_back(i);
				for (auto i : self[m])
					activeA.frameHistorys.push_back(i);
				std::size_t sizeA = activeA.historyOverflow ? activeA.historySize : activeA.historyEvent;
				if (sizeA)
				{
					LOG activeA.name << "\tfuds cardinality: " << activeA.decomp->fuds.size() << "\tmodel cardinality: " << activeA.decomp->fudRepasSize << "\tactive size: " << sizeA << "\tfuds per threshold: " << (double)activeA.decomp->fuds.size() * activeA.induceThreshold / sizeA UNLOG				
				}			
				if (!induceNot)
					_threads.push_back(std::thread(run_induce, std::ref(*this), std::ref(activeA), induceInterval, induceThresholdInitial));			
			}			
		}
		_threads.push_back(std::thread(run_act, std::ref(*this)));	
	}
	
	{
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
			_goalsLocationsNext[std::get<0>(t)][std::get<1>(t)] = std::get<2>(t);
	}
	
	if (_struct=="struct001" && _mode=="mode003")
	{	
		auto& activeA = *_level2.front();	
		auto historyEventA = activeA.historyEvent ? activeA.historyEvent - 1 : activeA.historySize - 1;
		std::shared_ptr<HistoryRepa> hr = activeA.underlyingHistoryRepa.front();
		auto& hs = *activeA.historySparse;
		auto over = activeA.historyOverflow;
		auto& mm = _ur->mapVarSize();
		auto& mvv = hr->mapVarInt();
		auto location = mvv[mm[Variable("location")]];
		auto n = hr->dimension;
		auto z = hr->size;
		auto y = activeA.historyEvent;
		auto rr = hr->arr;	
		auto rs = hs.arr;
		auto sliceCount = activeA.historySlicesSetEvent.size();
		_mode3SlicesLocation.reserve(sliceCount);				
		{
			for (auto& p : activeA.historySlicesSetEvent)
			{
				std::map<std::size_t, std::size_t> locsCount;
				for (auto ev : p.second)
					locsCount[rr[ev*n+location]]++;
				std::size_t most = 0;
				std::size_t loc = 0;
				for (auto& q : locsCount)	
					if (!most || most < q.second)
					{
						most = q.second;
						loc = q.first;
					}
				_mode3SlicesLocation.insert_or_assign(p.first,loc);
			}
		}
		std::unordered_map<std::size_t, std::map<std::size_t, std::size_t>> slicesSliceSetNext;
		slicesSliceSetNext.reserve(sliceCount);
		{
			auto j = over ? y : z;	
			auto sliceB = rs[j%z];
			j++;
			while (j < y+z)
			{
				auto sliceC = rs[j%z];
				if (sliceC != sliceB)
				{
					slicesSliceSetNext[sliceB][sliceC]++;
					sliceB = sliceC;
				}
				j++;
			}					
		}
		_mode3SlicesSliceSetNext.reserve(slicesSliceSetNext.size());
		for (auto& p : slicesSliceSetNext)
			for (auto& q : p.second)
			{
				if (!_modeMultipleTransition || q.second > 1)
					_mode3SlicesSliceSetNext[p.first].insert(q.first);
			}
	}
	else if (_struct=="struct001" && _mode=="mode004")
	{	
		std::vector<std::string> locations{ "door12", "door13", "door14", "door45", "door56", "room1", "room2", "room3", "room4", "room5", "room6" };
		auto nloc = locations.size();
		auto bloc = nloc - 6;
		auto& activeA = *_level2.front();	
		auto historyEventA = activeA.historyEvent ? activeA.historyEvent - 1 : activeA.historySize - 1;
		std::shared_ptr<HistoryRepa> hr = activeA.underlyingHistoryRepa.front();
		auto& hs = *activeA.historySparse;
		auto over = activeA.historyOverflow;
		auto& mm = _ur->mapVarSize();
		auto& mvv = hr->mapVarInt();
		auto location = mvv[mm[Variable("location")]];
		auto n = hr->dimension;
		auto z = hr->size;
		auto y = activeA.historyEvent;
		auto rr = hr->arr;	
		auto rs = hs.arr;
		auto sliceCount = activeA.historySlicesSetEvent.size();
		std::unordered_map<std::size_t, std::map<std::size_t, std::size_t>> slicesSliceSetNext;
		slicesSliceSetNext.reserve(sliceCount*nloc);
		{
			auto j = over ? y : z;	
			auto sliceLocB = rs[j%z]*nloc + rr[(j%z)*n+location];
			j++;
			while (j < y+z)
			{
				auto sliceLocC = rs[j%z]*nloc + rr[(j%z)*n+location];
				if (sliceLocC != sliceLocB)
				{
					slicesSliceSetNext[sliceLocB][sliceLocC]++;
					sliceLocB = sliceLocC;
				}
				j++;
			}					
		}
		_mode4SlicesSliceSetNext.reserve(slicesSliceSetNext.size());
		for (auto& p : slicesSliceSetNext)
			for (auto& q : p.second)
			{
				if (!_modeMultipleTransition || q.second > 1)
					_mode4SlicesSliceSetNext[p.first].insert(q.first);
			}
		if (_mode4Caching)
		{
			std::unordered_map<std::size_t, Alignment::SizeSet> slicesSliceSetPrev;
			slicesSliceSetPrev.reserve(sliceCount);
			for (auto& p : _mode4SlicesSliceSetNext)
				for (auto& q : p.second)
					slicesSliceSetPrev[q].insert(p.first);
			for (auto locA = bloc; locA < nloc; locA++)
			{
				auto& slicesStepCount = _mode4locationsSlicesStepCount[locA];
				slicesStepCount.reserve(sliceCount);
				SizeUSet sliceCurrents;
				sliceCurrents.reserve(sliceCount);		
				std::size_t steps = 0;				
				for (auto& p : slicesSliceSetPrev)
					if (p.first % nloc == locA)
					{
						slicesStepCount.insert_or_assign(p.first,steps);
						sliceCurrents.insert(p.first);
					}
				while (sliceCurrents.size())
				{
					steps++;
					SizeList sliceCurrentBs;
					sliceCurrentBs.reserve(sliceCurrents.size() * 20);
					for (auto sliceLocC : sliceCurrents)		
					{
						for (auto sliceLocD : slicesSliceSetPrev[sliceLocC])
						{					
							if (slicesStepCount.find(sliceLocD) == slicesStepCount.end())
							{
								sliceCurrentBs.push_back(sliceLocD);			
								slicesStepCount.insert_or_assign(sliceLocD,steps);
							}
						}
					}	
					sliceCurrents.clear();
					sliceCurrents.insert(sliceCurrentBs.begin(), sliceCurrentBs.end());
				}				
			}
		}
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
	}

	RCLCPP_INFO(this->get_logger(), "TBOT02 actor node has been initialised");
}

Actor::~Actor()
{
	_terminate = true;
	if (_system && (_struct=="struct001" || _struct=="struct002"))
	{
		for (auto activeA : _level1)
		{
			activeA->terminate = true;
			if ( _model!="")
			{
				ActiveIOParameters ppio;
				ppio.filename = activeA->name+".ac";
				activeA->logging = true;
				activeA->dump(ppio);		
			}			
		}	
		for (auto activeA : _level2)
		{
			activeA->terminate = true;
			if ( _model!="")
			{
				ActiveIOParameters ppio;
				ppio.filename = activeA->name+".ac";
				activeA->logging = true;
				activeA->dump(ppio);		
			}			
		}	
		for (auto activeA : _level3)
		{
			activeA->terminate = true;
			if ( _model!="")
			{
				ActiveIOParameters ppio;
				ppio.filename = activeA->name+".ac";
				activeA->logging = true;
				activeA->dump(ppio);		
			}			
		}	
		for (auto& t : _threads)
			t.join();	
	}
	RCLCPP_INFO(this->get_logger(), "TBOT02 actor node has been terminated");
}

void Actor::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
	if (!_crashed)
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
		if (msg->pose.pose.position.z >= 0.02)
		{
			_crashed = true;
			RCLCPP_INFO(this->get_logger(), "Crashed");		
		}			
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

void Actor::goal_callback(const std_msgs::msg::String::SharedPtr msg)
{
	_goal = msg->data;
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

