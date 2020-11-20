#include "dev.h"

#include <cmath>
#include <cstring>

using namespace Alignment;
using namespace TBOT02;
using namespace std;

typedef std::chrono::duration<double> sec; 
typedef std::chrono::high_resolution_clock clk;

#define ECHO(x) cout << #x << endl; x
#define EVAL(x) cout << #x << ": " << (x) << endl
#define EVALL(x) cout << #x << ": " << endl << (x) << endl
#define TRUTH(x) cout << #x << ": " << ((x) ? "true" : "false") << endl

int main(int argc, char **argv)
{
	if (argc >= 3 && string(argv[1]) == "fud_region")
	{
		auto uvars = systemsSetVar;
		auto hrsel = eventsHistoryRepasHistoryRepaSelection_u;
		auto hrred = setVarsHistoryRepasReduce_u;
		auto hrhrred = setVarsHistoryRepasHistoryRepaReduced_u;
		auto hrpr = setVarsHistoryRepasRed_u;
		auto prents = histogramRepaRedsListEntropy;
		auto hrconcat = vectorHistoryRepasConcat_u;
		auto hrshuffle = historyRepasShuffle_u;
		auto frvars = fudRepasSetVar;
		auto frder = fudRepasDerived;
		auto frund = fudRepasUnderlying;
		auto llfr = setVariablesListTransformRepasFudRepa_u;
		auto frmul = historyRepasFudRepasMultiply_up;
		auto frdep = fudRepasSetVarsDepends;
		auto layerer = parametersSystemsLayererMaxRollByMExcludedSelfHighestIORepa_up;
		
		string model = string(argv[2]);
		size_t size = argc >= 4 ? atoi(argv[3]) : 1000;
		EVAL(model);
		EVAL(size);
				
		std::unique_ptr<System> uu;
		std::unique_ptr<SystemRepa> ur;
		std::unique_ptr<HistoryRepa> hr;
		{
			HistoryRepaPtrList ll;
			int s = 17;
			std::ifstream in("data009.bin", std::ios::binary);
			auto qq = persistentsRecordList(in);
			in.close();
			for (int i = 0; i < 5; i++)
			{
				auto xx = recordListsHistoryRepaRegion(8, 60, s++, *qq);
				uu = std::move(std::get<0>(xx));
				ur = std::move(std::get<1>(xx));
				ll.push_back(std::move(std::get<2>(xx)));
			}
			hr = vectorHistoryRepasConcat_u(ll);
			if (size < hr->size)
			{
				SizeList ev;
				for (size_t i = 0; i < size; i++)
					ev.push_back(i);
				hr = hrsel(ev.size(), ev.data(), *hr);
			}
		}
		EVAL(hr->size);
		
		SizeList vv;		
		{
			auto vvk = *uvars(*uu);
			auto& vvi = ur->mapVarSize();
			auto vvk0 = sorted(vvk);
			for (auto& v : vvk0)
				vv.push_back(vvi[v]);			
		}
		EVAL(vv.size());

		std::unique_ptr<HistoryRepa> hrs;
		{
			size_t seed = 5;
			hrs = hrshuffle(*hr, (unsigned int)seed);	
		}

		int d = 0;
		std::size_t f = 1;
		size_t tint = 4;
		
		std::unique_ptr<FudRepa> fr;
		std::unique_ptr<DoubleSizeListPairList> mm;
		try
		{
			size_t wmax = 9;
			size_t lmax = 8;
			size_t xmax = 128;
			size_t omax = 10;
			size_t bmax = 10 * 3;
			size_t mmax = 3;
			size_t umax = 128;
			size_t pmax = 1;
			auto t = layerer(wmax, lmax, xmax, omax, bmax, mmax, umax, pmax, tint, vv, *hr, *hrs, f, *ur);
			fr = std::move(std::get<0>(t));
			mm = std::move(std::get<1>(t));
		}
		catch (const std::out_of_range& e)
		{
			std::cout << "out of range exception: " << e.what() << std::endl;
			return 1;
		}
		if (!mm || !mm->size())
		{
			std::cout << "no fud" << std::endl;
			return 1;
		}
		// EVAL(mm->size());			
		// EVAL(mm->back());
		// EVAL(*mm);
		auto& a = mm->back().first;
		auto& kk = mm->back().second;
		auto m = kk.size();
		auto z = hr->size;
		EVAL(m);
		EVAL(a);			
		EVAL(z);	
		EVAL(100.0*(exp(a/z/(m-1))-1.0));
		EVAL(fudRepasSize(*fr));
		EVAL(frvars(*fr)->size());
		EVAL(frder(*fr)->size());
		EVAL(frund(*fr)->size());
		EVAL(sorted(*frund(*fr)));
		
		auto dr = std::make_unique<ApplicationRepa>();
		{
			dr->substrate = vv;
			dr->fud = std::make_shared<FudRepa>();
			dr->slices = std::make_shared<SizeTree>();	
			auto vd = std::make_shared<Variable>(d);
			auto vl = std::make_shared<Variable>("s");
			auto vf = std::make_shared<Variable>((int)f);
			auto vdf = std::make_shared<Variable>(vd, vf);
			auto vfl = std::make_shared<Variable>(vdf, vl);
			SizeUSet kk1(kk.begin(), kk.end());
			SizeUSet vv1(vv.begin(), vv.end());
			auto gr = llfr(vv1, *frdep(*fr, kk1));
			auto ar = hrred(1.0, m, kk.data(), *frmul(tint, *hr, *gr));
			SizeList sl;
			TransformRepaPtrList ll;
			std::size_t sz = 1;
			auto skk = ar->shape;
			auto rr0 = ar->arr;
			for (std::size_t i = 0; i < m; i++)
				sz *= skk[i];
			sl.reserve(sz);
			ll.reserve(sz);
			bool remainder = false;
			std::size_t b = 1;
			auto& llu = ur->listVarSizePair;
			for (std::size_t i = 0; i < sz; i++)
			{
				if (rr0[i] <= 0.0)
				{
					remainder = true;
					continue;
				}
				auto tr = std::make_shared<TransformRepa>();
				tr->dimension = m;
				tr->vectorVar = new std::size_t[m];
				auto ww = tr->vectorVar;
				tr->shape = new std::size_t[m];
				auto sh = tr->shape;
				for (std::size_t j = 0; j < m; j++)
				{
					ww[j] = kk[j];
					sh[j] = skk[j];
				}
				tr->arr = new unsigned char[sz];
				auto rr = tr->arr;
				for (std::size_t j = 0; j < sz; j++)
					rr[j] = 0;
				rr[i] = 1;
				tr->valency = 2;
				auto vb = std::make_shared<Variable>((int)b++);
				auto vflb = std::make_shared<Variable>(vfl, vb);
				llu.push_back(VarSizePair(vflb, 2));
				auto w = llu.size() - 1;
				tr->derived = w;
				sl.push_back(w);
				ll.push_back(tr);
			}
			if (remainder)
			{
				auto tr = std::make_shared<TransformRepa>();
				tr->dimension = m;
				tr->vectorVar = new std::size_t[m];
				auto ww = tr->vectorVar;
				tr->shape = new std::size_t[m];
				auto sh = tr->shape;
				for (std::size_t j = 0; j < m; j++)
				{
					ww[j] = kk[j];
					sh[j] = skk[j];
				}
				tr->arr = new unsigned char[sz];
				auto rr = tr->arr;
				for (std::size_t j = 0; j < sz; j++)
					rr[j] = rr0[j] <= 0.0 ? 1 : 0;
				tr->valency = 2;
				auto vb = std::make_shared<Variable>((int)b++);
				auto vflb = std::make_shared<Variable>(vfl, vb);
				llu.push_back(VarSizePair(vflb, 2));
				auto w = llu.size() - 1;
				tr->derived = w;
				sl.push_back(w);
				ll.push_back(tr);
			}
			dr->fud->layers.insert(dr->fud->layers.end(), gr->layers.begin(), gr->layers.end());
			dr->fud->layers.push_back(ll);
			dr->slices->_list.reserve(sz);
			for (auto& s : sl)
				dr->slices->_list.push_back(SizeSizeTreePair(s, std::make_shared<SizeTree>()));			
		}
		{
			EVAL(model+".dr");
			std::ofstream out(model+".dr", std::ios::binary);
			systemRepasPersistent(*ur, out); cout << endl;
			applicationRepasPersistent(*dr, out); cout << endl;
			out.close();
		}
	}
	
	if (argc >= 3 && string(argv[1]) == "entropy_region")
	{
		auto uvars = systemsSetVar;
		auto uruu = systemsRepasSystem;
		auto aall = histogramsList;
		auto add = pairHistogramsAdd_u;
		auto ent = histogramsEntropy;
		auto araa = systemsHistogramRepasHistogram_u;
		auto hrred = [](const HistoryRepa& hr, const SystemRepa& ur, const VarList& kk)
		{
			auto& vvi = ur.mapVarSize();
			std::size_t m = kk.size();
			SizeList kk1;
			for (std::size_t i = 0; i < m; i++)
				kk1.push_back(vvi[kk[i]]);
			return setVarsHistoryRepasReduce_u(1.0, m, kk1.data(), hr);
		};
		auto hrconcat = vectorHistoryRepasConcat_u;
		auto hrshuffle = historyRepasShuffle_u;
		auto hrpart = systemsHistoryRepasApplicationsHistoryHistoryPartitionedRepa_u;
		auto frvars = fudRepasSetVar;
		auto frder = fudRepasDerived;
		auto frund = fudRepasUnderlying;
		
		string model = string(argv[2]);
		size_t mult = argc >= 4 ? atoi(argv[3]) : 1;
		string dataset = string(argc >= 5 ? argv[4] : "data009");
		size_t scale = argc >= 6 ? atoi(argv[5]) : 10;
		
		EVAL(model);
		EVAL(mult);
		EVAL(dataset);

		std::unique_ptr<System> uu;
		std::unique_ptr<SystemRepa> ur;
		std::unique_ptr<HistoryRepa> hr;
		{	
			HistoryRepaPtrList ll;
			int s = 17;
			std::ifstream in(dataset+".bin", std::ios::binary);
			auto qq = persistentsRecordList(in);
			in.close();
			for (int i = 0; i < scale; i++)
			{
				auto xx = recordListsHistoryRepaRegion(8, 60, s++, *qq);
				uu = std::move(std::get<0>(xx));
				ur = std::move(std::get<1>(xx));
				ll.push_back(std::move(std::get<2>(xx)));
			}
			hr = vectorHistoryRepasConcat_u(ll);
		}

		ECHO(auto z = hr->size);
		EVAL(z);
		ECHO(auto v = z * mult);
		EVAL(v);
		
		StrVarPtrMap m;
		std::ifstream in(model + ".dr", std::ios::binary);
		auto ur1 = persistentsSystemRepa(in, m);
		auto dr = persistentsApplicationRepa(in);
		in.close();

		EVAL(fudRepasSize(*dr->fud));
		EVAL(frder(*dr->fud)->size());
		EVAL(frund(*dr->fud)->size());
		EVAL(treesSize(*dr->slices));
		EVAL(treesLeafElements(*dr->slices)->size());

		auto hrp = hrpart(*hr, *dr, *ur);
		uruu(*ur, *uu);
		auto aa = araa(*uu, *ur, *hrred(*hrp, *ur, VarList{ Variable("partition0"), Variable("partition1") }));
		EVAL(ent(*aa) * z);
		
		HistoryRepaPtrList qq;
		qq.reserve(mult);
		for (std::size_t i = 1; i <= mult; i++)
			qq.push_back(hrshuffle(*hr, (unsigned int)(12345+i*z)));
		auto hrs = hrconcat(qq);
		
		auto hrsp = hrpart(*hrs, *dr, *ur);
		auto bb = araa(*uu, *ur, *hrred(*hrsp, *ur, VarList{ Variable("partition0"), Variable("partition1") }));
		EVAL(ent(*bb) * v);
		
		EVAL(ent(*add(*aa,*bb)) * (z+v));
		EVAL(ent(*add(*aa,*bb)) * (z+v) - ent(*aa) * z - ent(*bb) * v);
	}
	
	if (argc >= 3 && string(argv[1]) == "fud") 
	{
		auto uvars = systemsSetVar;
		auto hrsel = eventsHistoryRepasHistoryRepaSelection_u;
		auto hrred = setVarsHistoryRepasReduce_u;
		auto hrhrred = setVarsHistoryRepasHistoryRepaReduced_u;
		auto hrpr = setVarsHistoryRepasRed_u;
		auto prents = histogramRepaRedsListEntropy;
		auto hrconcat = vectorHistoryRepasConcat_u;
		auto hrshuffle = historyRepasShuffle_u;
		auto frvars = fudRepasSetVar;
		auto frder = fudRepasDerived;
		auto frund = fudRepasUnderlying;
		auto llfr = setVariablesListTransformRepasFudRepa_u;
		auto frmul = historyRepasFudRepasMultiply_up;
		auto frdep = fudRepasSetVarsDepends;
		auto drcopy = applicationRepasApplicationRepa_u;
		auto drjoin = applicationRepaPairsJoin_u;
		auto layerer = parametersSystemsLayererMaxRollByMExcludedSelfHighestLogIORepa_up;
		
		auto log = [](const std::string& str)
		{
			std::cout << str << std::endl;
			return;
		};

		string model = string(argv[2]);
		size_t size = argc >= 4 ? atoi(argv[3]) : 1000;
		EVAL(model);
		EVAL(size);
				
		std::unique_ptr<System> uu;
		std::unique_ptr<SystemRepa> ur;
		std::unique_ptr<HistoryRepa> hr;
		{
			std::ifstream in("data009.bin", std::ios::binary);
			auto qq = persistentsRecordList(in);
			in.close();
			auto xx = recordListsHistoryRepa_2(8, *qq);
			uu = std::move(std::get<0>(xx));
			ur = std::move(std::get<1>(xx));
			hr = std::move(std::get<2>(xx));
			if (size < hr->size)
			{
				SizeList ev;
				for (size_t i = 0; i < size; i++)
					ev.push_back(i);
				hr = hrsel(ev.size(), ev.data(), *hr);
			}
		}
		EVAL(hr->size);
		
		auto dr0 = std::make_unique<ApplicationRepa>();
		{
			auto dr = std::make_unique<ApplicationRepa>();
			StrVarPtrMap m;
			std::ifstream in("model026.dr", std::ios::binary);
			auto ur1 = persistentsSystemRepa(in, m);
			auto dr1 = persistentsApplicationRepa(in);
			in.close();
			auto& llu1 = ur1->listVarSizePair;
			VarSizeUMap ur0 = ur->mapVarSize();
			auto n = fudRepasSize(*dr1->fud);
			size_t a = 360;
			size_t b = 60;
			auto& llu = ur->listVarSizePair;
			llu.reserve(n*a/b + a);
			dr->slices = std::make_shared<SizeTree>();
			dr->slices->_list.reserve(dr1->slices->_list.size() * a / b);
			dr->fud = std::make_shared<FudRepa>();
			dr->fud->layers.reserve(dr1->fud->layers.size());
			dr->substrate.reserve(dr1->substrate.size() * a / b);
			for (int i = 0; i < a*2/b; i++)
			{
				auto dr2 = drcopy(*dr1);
				SizeSizeUMap nn;
				nn.reserve(n + b);
				for (auto x1 : dr1->substrate)
				{
					auto& p = llu1[x1];
					auto v1 = p.first->_var0;
					auto v2 = std::make_shared<Variable>(((int)(p.first->_var1->_int + i*b/2 - 1)) % a + 1);
					auto v = std::make_shared<Variable>(v1, v2);
					nn[x1] = ur0[*v];
				}
				auto vk = std::make_shared<Variable>((int)i + 1);
				for (auto& ll : dr1->fud->layers)
					for (auto& tr : ll)
					{
						auto x1 = tr->derived;
						auto& p = llu1[x1];
						auto v = std::make_shared<Variable>(p.first, vk);
						llu.push_back(VarSizePair(v, p.second));			
						nn[x1] = llu.size() - 1;
					}
				dr2->reframe_u(nn);
				dr->slices->_list.insert(dr->slices->_list.end(), dr2->slices->_list.begin(), dr2->slices->_list.end());
				for (std::size_t l = 0; l < dr2->fud->layers.size(); l++)
				{
					if (l < dr->fud->layers.size())
						dr->fud->layers[l].insert(dr->fud->layers[l].end(), dr2->fud->layers[l].begin(), dr2->fud->layers[l].end());
					else
						dr->fud->layers.push_back(dr2->fud->layers[l]);
				}
				dr->substrate.insert(dr->substrate.end(), dr2->substrate.begin(), dr2->substrate.end());
			}
			dr0 = std::move(dr);
		}

		SizeList vv = *treesElements(*dr0->slices);		
		EVAL(vv.size());
		
		size_t tint = 4;		
		size_t wmax = 18;
		size_t lmax = 8;
		size_t xmax = 128;
		double znnmax = 60000.0 * 2.0 * 100.0 * 100.0 * 32;
		size_t omax = 10;
		size_t bmax = 10 * 3;
		size_t mmax = 3;
		size_t umax = 128;
		size_t pmax = 1;
		size_t mult = 1;
		size_t seed = 5;
	
		std::unique_ptr<HistoryRepa> hrs;
		{
			std::unique_ptr<HistoryRepa> hr1 = hrhrred(vv.size(), vv.data(), *frmul(tint, *hr, *dr0->fud));
			auto z = hr->size;			
			auto nmax = (std::size_t)std::sqrt(znnmax / (double)(z + mult*z));	
			nmax = std::max(nmax, bmax);	
			EVAL(nmax);				
			if (nmax < vv.size())
			{
				auto ee = prents(*hrpr(vv.size(), vv.data(), *hr1));
				auto m = ee->size();
				SizeList vv0;		
				if (m > nmax)
				{
					std::sort(ee->begin(), ee->end());
					vv0.reserve(nmax);
					for (std::size_t i = m - nmax; i < m; i++)
						vv0.push_back((*ee)[i].second);
				}
				else
				{
					vv0.reserve(m);
					for (std::size_t i = 0; i < m; i++)
						vv0.push_back((*ee)[i].second);
				}
				if (vv0.size() < vv.size())
				{
					vv = vv0;
					SizeUSet vv00(hr->dimension);
					for (std::size_t i = 0; i < hr->dimension; i++)
						vv00.insert(hr->vectorVar[i]);
					SizeUSet vv01(vv0.begin(), vv0.end());
					auto er0 = llfr(vv00, *frdep(*dr0->fud, vv01));
					hrs = hrhrred(vv.size(), vv.data(), *frmul(tint, *hrshuffle(*hr, (unsigned int)seed), *er0));
					hr = hrhrred(vv.size(), vv.data(), *frmul(tint, *hr, *er0));					
				}
				else
				{
					hrs = hrhrred(vv.size(), vv.data(), *frmul(tint, *hrshuffle(*hr, (unsigned int)seed), *dr0->fud));
					hr = std::move(hr1);				
				}
			}	
			else
			{
				hrs = hrhrred(vv.size(), vv.data(), *frmul(tint, *hrshuffle(*hr, (unsigned int)seed), *dr0->fud));
				hr = std::move(hr1);				
			}
		}

		int d = 0;
		std::size_t f = 1;		
		std::unique_ptr<FudRepa> fr;
		std::unique_ptr<DoubleSizeListPairList> mm;
		try
		{

			auto t = layerer(wmax, lmax, xmax, omax, bmax, mmax, umax, pmax, tint, vv, *hr, *hrs, f, log, *ur);
			fr = std::move(std::get<0>(t));
			mm = std::move(std::get<1>(t));
		}
		catch (const std::out_of_range& e)
		{
			std::cout << "out of range exception: " << e.what() << std::endl;
			return 1;
		}
		if (!mm || !mm->size())
		{
			std::cout << "no fud" << std::endl;
			return 1;
		}
		// EVAL(mm->size());			
		// EVAL(mm->back());
		// EVAL(*mm);
		auto& a = mm->back().first;
		auto& kk = mm->back().second;
		auto m = kk.size();
		auto z = hr->size;
		EVAL(m);
		EVAL(a);			
		EVAL(z);	
		EVAL(100.0*(exp(a/z/(m-1))-1.0));
		EVAL(fudRepasSize(*fr));
		EVAL(frvars(*fr)->size());
		EVAL(frder(*fr)->size());
		EVAL(frund(*fr)->size());
		EVAL(sorted(*frund(*fr)));
		
		auto dr = std::make_unique<ApplicationRepa>();
		{
			dr->substrate = vv;
			dr->fud = std::make_shared<FudRepa>();
			dr->slices = std::make_shared<SizeTree>();	
			auto vd = std::make_shared<Variable>(d);
			auto vl = std::make_shared<Variable>("s");
			auto vf = std::make_shared<Variable>((int)f);
			auto vdf = std::make_shared<Variable>(vd, vf);
			auto vfl = std::make_shared<Variable>(vdf, vl);
			SizeUSet kk1(kk.begin(), kk.end());
			SizeUSet vv1(vv.begin(), vv.end());
			auto gr = llfr(vv1, *frdep(*fr, kk1));
			auto ar = hrred(1.0, m, kk.data(), *frmul(tint, *hr, *gr));
			SizeList sl;
			TransformRepaPtrList ll;
			std::size_t sz = 1;
			auto skk = ar->shape;
			auto rr0 = ar->arr;
			for (std::size_t i = 0; i < m; i++)
				sz *= skk[i];
			sl.reserve(sz);
			ll.reserve(sz);
			bool remainder = false;
			std::size_t b = 1;
			auto& llu = ur->listVarSizePair;
			for (std::size_t i = 0; i < sz; i++)
			{
				if (rr0[i] <= 0.0)
				{
					remainder = true;
					continue;
				}
				auto tr = std::make_shared<TransformRepa>();
				tr->dimension = m;
				tr->vectorVar = new std::size_t[m];
				auto ww = tr->vectorVar;
				tr->shape = new std::size_t[m];
				auto sh = tr->shape;
				for (std::size_t j = 0; j < m; j++)
				{
					ww[j] = kk[j];
					sh[j] = skk[j];
				}
				tr->arr = new unsigned char[sz];
				auto rr = tr->arr;
				for (std::size_t j = 0; j < sz; j++)
					rr[j] = 0;
				rr[i] = 1;
				tr->valency = 2;
				auto vb = std::make_shared<Variable>((int)b++);
				auto vflb = std::make_shared<Variable>(vfl, vb);
				llu.push_back(VarSizePair(vflb, 2));
				auto w = llu.size() - 1;
				tr->derived = w;
				sl.push_back(w);
				ll.push_back(tr);
			}
			if (remainder)
			{
				auto tr = std::make_shared<TransformRepa>();
				tr->dimension = m;
				tr->vectorVar = new std::size_t[m];
				auto ww = tr->vectorVar;
				tr->shape = new std::size_t[m];
				auto sh = tr->shape;
				for (std::size_t j = 0; j < m; j++)
				{
					ww[j] = kk[j];
					sh[j] = skk[j];
				}
				tr->arr = new unsigned char[sz];
				auto rr = tr->arr;
				for (std::size_t j = 0; j < sz; j++)
					rr[j] = rr0[j] <= 0.0 ? 1 : 0;
				tr->valency = 2;
				auto vb = std::make_shared<Variable>((int)b++);
				auto vflb = std::make_shared<Variable>(vfl, vb);
				llu.push_back(VarSizePair(vflb, 2));
				auto w = llu.size() - 1;
				tr->derived = w;
				sl.push_back(w);
				ll.push_back(tr);
			}
			dr->fud->layers.insert(dr->fud->layers.end(), gr->layers.begin(), gr->layers.end());
			dr->fud->layers.push_back(ll);
			dr->slices->_list.reserve(sz);
			for (auto& s : sl)
				dr->slices->_list.push_back(SizeSizeTreePair(s, std::make_shared<SizeTree>()));			
		}
		{
			auto dr1 = drjoin(*dr0, *dr);
			std::ofstream out(model+".dr", std::ios::binary);
			systemRepasPersistent(*ur, out); 
			applicationRepasPersistent(*dr1, out); 
			out.close();
			EVAL(model+".dr");
		}
	}

	if (argc >= 3 && string(argv[1]) == "fud2") 
	{
		auto uvars = systemsSetVar;
		auto hrsel = eventsHistoryRepasHistoryRepaSelection_u;
		auto hrred = setVarsHistoryRepasReduce_u;
		auto hrhrred = setVarsHistoryRepasHistoryRepaReduced_u;
		auto hrpr = setVarsHistoryRepasRed_u;
		auto prents = histogramRepaRedsListEntropy;
		auto hrconcat = vectorHistoryRepasConcat_u;
		auto hrshuffle = historyRepasShuffle_u;
		auto frvars = fudRepasSetVar;
		auto frder = fudRepasDerived;
		auto frund = fudRepasUnderlying;
		auto llfr = setVariablesListTransformRepasFudRepa_u;
		auto frmul = historyRepasFudRepasMultiply_up;
		auto frdep = fudRepasSetVarsDepends;
		auto drcopy = applicationRepasApplicationRepa_u;
		auto drjoin = applicationRepaPairsJoin_u;
		auto layerer = parametersLayererMaxRollByMExcludedSelfHighestLogIORepa_up;
		
		auto log = [](const std::string& str)
		{
			std::cout << str << std::endl;
			return;
		};

		string model = string(argv[2]);
		size_t size = argc >= 4 ? atoi(argv[3]) : 1000;
		bool logging = argc >= 5;
		EVAL(model);
		EVAL(size);
				
		std::unique_ptr<System> uu;
		std::unique_ptr<SystemRepa> ur;
		std::unique_ptr<HistoryRepa> hr;
		{
			std::ifstream in("data009.bin", std::ios::binary);
			auto qq = persistentsRecordList(in);
			in.close();
			auto xx = recordListsHistoryRepa_2(8, *qq);
			uu = std::move(std::get<0>(xx));
			ur = std::move(std::get<1>(xx));
			hr = std::move(std::get<2>(xx));
			if (size < hr->size)
			{
				SizeList ev;
				for (size_t i = 0; i < size; i++)
					ev.push_back(i);
				hr = hrsel(ev.size(), ev.data(), *hr);
			}
		}
		EVAL(hr->size);
		
		auto dr0 = std::make_unique<ApplicationRepa>();
		{
			auto dr = std::make_unique<ApplicationRepa>();
			StrVarPtrMap m;
			std::ifstream in("model026.dr", std::ios::binary);
			auto ur1 = persistentsSystemRepa(in, m);
			auto dr1 = persistentsApplicationRepa(in);
			in.close();
			auto& llu1 = ur1->listVarSizePair;
			VarSizeUMap ur0 = ur->mapVarSize();
			auto n = fudRepasSize(*dr1->fud);
			size_t a = 360;
			size_t b = 60;
			auto& llu = ur->listVarSizePair;
			llu.reserve(n*a/b + a);
			dr->slices = std::make_shared<SizeTree>();
			dr->slices->_list.reserve(dr1->slices->_list.size() * a / b);
			dr->fud = std::make_shared<FudRepa>();
			dr->fud->layers.reserve(dr1->fud->layers.size());
			dr->substrate.reserve(dr1->substrate.size() * a / b);
			for (int i = 0; i < a*2/b; i++)
			{
				auto dr2 = drcopy(*dr1);
				SizeSizeUMap nn;
				nn.reserve(n + b);
				for (auto x1 : dr1->substrate)
				{
					auto& p = llu1[x1];
					auto v1 = p.first->_var0;
					auto v2 = std::make_shared<Variable>(((int)(p.first->_var1->_int + i*b/2 - 1)) % a + 1);
					auto v = std::make_shared<Variable>(v1, v2);
					nn[x1] = ur0[*v];
				}
				auto vk = std::make_shared<Variable>((int)i + 1);
				for (auto& ll : dr1->fud->layers)
					for (auto& tr : ll)
					{
						auto x1 = tr->derived;
						auto& p = llu1[x1];
						auto v = std::make_shared<Variable>(p.first, vk);
						llu.push_back(VarSizePair(v, p.second));			
						nn[x1] = llu.size() - 1;
					}
				dr2->reframe_u(nn);
				dr->slices->_list.insert(dr->slices->_list.end(), dr2->slices->_list.begin(), dr2->slices->_list.end());
				for (std::size_t l = 0; l < dr2->fud->layers.size(); l++)
				{
					if (l < dr->fud->layers.size())
						dr->fud->layers[l].insert(dr->fud->layers[l].end(), dr2->fud->layers[l].begin(), dr2->fud->layers[l].end());
					else
						dr->fud->layers.push_back(dr2->fud->layers[l]);
				}
				dr->substrate.insert(dr->substrate.end(), dr2->substrate.begin(), dr2->substrate.end());
			}
			dr0 = std::move(dr);
		}

		SizeList vv = *treesElements(*dr0->slices);		
		EVAL(vv.size());
		
		size_t tint = 4;		
		size_t wmax = 18;
		size_t lmax = 8;
		size_t xmax = 128;
		double znnmax = 60000.0 * 2.0 * 100.0 * 100.0 * 32;
		size_t omax = 10;
		size_t bmax = 10 * 3;
		size_t mmax = 3;
		size_t umax = 128;
		size_t pmax = 1;
		size_t mult = 1;
		size_t seed = 5;
	
		std::unique_ptr<HistoryRepa> hrs;
		{
			std::unique_ptr<HistoryRepa> hr1 = hrhrred(vv.size(), vv.data(), *frmul(tint, *hr, *dr0->fud));
			auto z = hr->size;			
			auto nmax = (std::size_t)std::sqrt(znnmax / (double)(z + mult*z));	
			nmax = std::max(nmax, bmax);	
			EVAL(nmax);				
			if (nmax < vv.size())
			{
				auto ee = prents(*hrpr(vv.size(), vv.data(), *hr1));
				auto m = ee->size();
				SizeList vv0;		
				if (m > nmax)
				{
					std::sort(ee->begin(), ee->end());
					vv0.reserve(nmax);
					for (std::size_t i = m - nmax; i < m; i++)
						vv0.push_back((*ee)[i].second);
				}
				else
				{
					vv0.reserve(m);
					for (std::size_t i = 0; i < m; i++)
						vv0.push_back((*ee)[i].second);
				}
				if (vv0.size() < vv.size())
				{
					vv = vv0;
					SizeUSet vv00(hr->dimension);
					for (std::size_t i = 0; i < hr->dimension; i++)
						vv00.insert(hr->vectorVar[i]);
					SizeUSet vv01(vv0.begin(), vv0.end());
					auto er0 = llfr(vv00, *frdep(*dr0->fud, vv01));
					hrs = hrhrred(vv.size(), vv.data(), *frmul(tint, *hrshuffle(*hr, (unsigned int)seed), *er0));
					hr = hrhrred(vv.size(), vv.data(), *frmul(tint, *hr, *er0));					
				}
				else
				{
					hrs = hrhrred(vv.size(), vv.data(), *frmul(tint, *hrshuffle(*hr, (unsigned int)seed), *dr0->fud));
					hr = std::move(hr1);				
				}
			}	
			else
			{
				hrs = hrhrred(vv.size(), vv.data(), *frmul(tint, *hrshuffle(*hr, (unsigned int)seed), *dr0->fud));
				hr = std::move(hr1);				
			}
		}

		int d = 0;
		std::size_t f = 1;		
		std::unique_ptr<FudRepa> fr;
		std::unique_ptr<DoubleSizeListPairList> mm;
		try
		{
			auto urv = ur->listVarSizePair.size();

			auto t = layerer(wmax, lmax, xmax, omax, bmax, mmax, umax, pmax, tint, vv, *hr, *hrs, log, logging, urv);
			fr = std::move(std::get<0>(t));
			mm = std::move(std::get<1>(t));
		}
		catch (const std::out_of_range& e)
		{
			std::cout << "out of range exception: " << e.what() << std::endl;
			return 1;
		}
		if (!mm || !mm->size())
		{
			std::cout << "no fud" << std::endl;
			return 1;
		}
		// EVAL(mm->size());			
		// EVAL(mm->back());
		// EVAL(*mm);
		auto& a = mm->back().first;
		auto& kk = mm->back().second;
		auto m = kk.size();
		auto z = hr->size;
		EVAL(m);
		EVAL(a);			
		EVAL(z);	
		EVAL(100.0*(exp(a/z/(m-1))-1.0));
		EVAL(fudRepasSize(*fr));
		EVAL(frvars(*fr)->size());
		EVAL(frder(*fr)->size());
		EVAL(frund(*fr)->size());
		EVAL(sorted(*frund(*fr)));
		
		auto dr = std::make_unique<ApplicationRepa>();
		{
			dr->substrate = vv;
			dr->fud = std::make_shared<FudRepa>();
			dr->slices = std::make_shared<SizeTree>();	
			auto vd = std::make_shared<Variable>(d);
			auto vl = std::make_shared<Variable>("s");
			auto vf = std::make_shared<Variable>((int)f);
			auto vdf = std::make_shared<Variable>(vd, vf);
			auto vfl = std::make_shared<Variable>(vdf, vl);
			SizeUSet kk1(kk.begin(), kk.end());
			SizeUSet vv1(vv.begin(), vv.end());
			auto gr = llfr(vv1, *frdep(*fr, kk1));
			auto ar = hrred(1.0, m, kk.data(), *frmul(tint, *hr, *gr));
			SizeList sl;
			TransformRepaPtrList ll;
			std::size_t sz = 1;
			auto skk = ar->shape;
			auto rr0 = ar->arr;
			for (std::size_t i = 0; i < m; i++)
				sz *= skk[i];
			sl.reserve(sz);
			ll.reserve(sz);
			bool remainder = false;
			std::size_t b = 1;
			auto& llu = ur->listVarSizePair;
			for (std::size_t i = 0; i < sz; i++)
			{
				if (rr0[i] <= 0.0)
				{
					remainder = true;
					continue;
				}
				auto tr = std::make_shared<TransformRepa>();
				tr->dimension = m;
				tr->vectorVar = new std::size_t[m];
				auto ww = tr->vectorVar;
				tr->shape = new std::size_t[m];
				auto sh = tr->shape;
				for (std::size_t j = 0; j < m; j++)
				{
					ww[j] = kk[j];
					sh[j] = skk[j];
				}
				tr->arr = new unsigned char[sz];
				auto rr = tr->arr;
				for (std::size_t j = 0; j < sz; j++)
					rr[j] = 0;
				rr[i] = 1;
				tr->valency = 2;
				auto vb = std::make_shared<Variable>((int)b++);
				auto vflb = std::make_shared<Variable>(vfl, vb);
				llu.push_back(VarSizePair(vflb, 2));
				auto w = llu.size() - 1;
				tr->derived = w;
				sl.push_back(w);
				ll.push_back(tr);
			}
			if (remainder)
			{
				auto tr = std::make_shared<TransformRepa>();
				tr->dimension = m;
				tr->vectorVar = new std::size_t[m];
				auto ww = tr->vectorVar;
				tr->shape = new std::size_t[m];
				auto sh = tr->shape;
				for (std::size_t j = 0; j < m; j++)
				{
					ww[j] = kk[j];
					sh[j] = skk[j];
				}
				tr->arr = new unsigned char[sz];
				auto rr = tr->arr;
				for (std::size_t j = 0; j < sz; j++)
					rr[j] = rr0[j] <= 0.0 ? 1 : 0;
				tr->valency = 2;
				auto vb = std::make_shared<Variable>((int)b++);
				auto vflb = std::make_shared<Variable>(vfl, vb);
				llu.push_back(VarSizePair(vflb, 2));
				auto w = llu.size() - 1;
				tr->derived = w;
				sl.push_back(w);
				ll.push_back(tr);
			}
			dr->fud->layers.insert(dr->fud->layers.end(), gr->layers.begin(), gr->layers.end());
			dr->fud->layers.push_back(ll);
			dr->slices->_list.reserve(sz);
			for (auto& s : sl)
				dr->slices->_list.push_back(SizeSizeTreePair(s, std::make_shared<SizeTree>()));			
		}
		{
			auto dr1 = drjoin(*dr0, *dr);
			std::ofstream out(model+".dr", std::ios::binary);
			systemRepasPersistent(*ur, out); 
			applicationRepasPersistent(*dr1, out); 
			out.close();
			EVAL(model+".dr");
		}
	}

	if (argc >= 3 && string(argv[1]) == "entropy")
	{
		auto uvars = systemsSetVar;
		auto uruu = systemsRepasSystem;
		auto aall = histogramsList;
		auto add = pairHistogramsAdd_u;
		auto ent = histogramsEntropy;
		auto araa = systemsHistogramRepasHistogram_u;
		auto hrred = [](const HistoryRepa& hr, const SystemRepa& ur, const VarList& kk)
		{
			auto& vvi = ur.mapVarSize();
			std::size_t m = kk.size();
			SizeList kk1;
			for (std::size_t i = 0; i < m; i++)
				kk1.push_back(vvi[kk[i]]);
			return setVarsHistoryRepasReduce_u(1.0, m, kk1.data(), hr);
		};
		auto hrconcat = vectorHistoryRepasConcat_u;
		auto hrshuffle = historyRepasShuffle_u;
		auto hrpart = systemsHistoryRepasApplicationsHistoryHistoryPartitionedRepa_u;
		auto frvars = fudRepasSetVar;
		auto frder = fudRepasDerived;
		auto frund = fudRepasUnderlying;
		
		string model = string(argv[2]);
		size_t mult = argc >= 4 ? atoi(argv[3]) : 1;
		string dataset = string(argc >= 5 ? argv[4] : "data009");
		
		EVAL(model);
		EVAL(mult);
		EVAL(dataset);

		std::unique_ptr<System> uu;
		std::unique_ptr<SystemRepa> ur;
		std::unique_ptr<HistoryRepa> hr;
		{
			std::ifstream in(dataset+".bin", std::ios::binary);
			auto qq = persistentsRecordList(in);
			in.close();
			auto xx = recordListsHistoryRepa_2(8, *qq);
			uu = std::move(std::get<0>(xx));
			ur = std::move(std::get<1>(xx));
			hr = std::move(std::get<2>(xx));
		}

		ECHO(auto z = hr->size);
		EVAL(z);
		ECHO(auto v = z * mult);
		EVAL(v);
		
		StrVarPtrMap m;
		std::ifstream in(model + ".dr", std::ios::binary);
		auto ur1 = persistentsSystemRepa(in, m);
		auto dr = persistentsApplicationRepa(in);
		in.close();

		EVAL(fudRepasSize(*dr->fud));
		EVAL(frder(*dr->fud)->size());
		EVAL(frund(*dr->fud)->size());
		EVAL(treesSize(*dr->slices));
		EVAL(treesLeafElements(*dr->slices)->size());

		auto hrp = hrpart(*hr, *dr, *ur);
		uruu(*ur, *uu);
		auto aa = araa(*uu, *ur, *hrred(*hrp, *ur, VarList{ Variable("partition0"), Variable("partition1") }));
		EVAL(ent(*aa) * z);
		
		HistoryRepaPtrList qq;
		qq.reserve(mult);
		for (std::size_t i = 1; i <= mult; i++)
			qq.push_back(hrshuffle(*hr, (unsigned int)(12345+i*z)));
		auto hrs = hrconcat(qq);
		
		auto hrsp = hrpart(*hrs, *dr, *ur);
		auto bb = araa(*uu, *ur, *hrred(*hrsp, *ur, VarList{ Variable("partition0"), Variable("partition1") }));
		EVAL(ent(*bb) * v);
		
		EVAL(ent(*add(*aa,*bb)) * (z+v));
		EVAL(ent(*add(*aa,*bb)) * (z+v) - ent(*aa) * z - ent(*bb) * v);
	}
			
	if (argc >= 3 && string(argv[1]) == "drmul")
	{
		auto uvars = systemsSetVar;
		auto uruu = systemsRepasSystem;
		auto aall = histogramsList;
		auto add = pairHistogramsAdd_u;
		auto ent = histogramsEntropy;
		auto araa = systemsHistogramRepasHistogram_u;
		auto hrsel = eventsHistoryRepasHistoryRepaSelection_u;
		auto hrhrred = setVarsHistoryRepasHistoryRepaReduced_u;
		auto hrred = [](const HistoryRepa& hr, const SystemRepa& ur, const VarList& kk)
		{
			auto& vvi = ur.mapVarSize();
			std::size_t m = kk.size();
			SizeList kk1;
			for (std::size_t i = 0; i < m; i++)
				kk1.push_back(vvi[kk[i]]);
			return setVarsHistoryRepasReduce_u(1.0, m, kk1.data(), hr);
		};
		auto hrconcat = vectorHistoryRepasConcat_u;
		auto hrshuffle = historyRepasShuffle_u;
		auto hrpart = systemsHistoryRepasApplicationsHistoryHistoryPartitionedRepa_u;
		auto frmul = historyRepasFudRepasMultiply_u;
		auto frvars = fudRepasSetVar;
		auto frder = fudRepasDerived;
		auto frund = fudRepasUnderlying;
		auto hrhs = historyRepasHistorySparse;
		auto hshr = historySparsesHistoryRepa;
		auto llhs = listSetIntsHistorySparse;
		auto hsll = historySparsesListSetInt;
		auto hahs = historySparseArraysHistorySparse;
		auto hsha = historySparsesHistorySparseArray;
		auto erdr = applicationRepasDecompFudSlicedRepa_u;
		auto drmul = historyRepaPtrListsHistorySparseArrayPtrListsDecompFudSlicedRepasEventsPathSlice_u;
		
		string model = string(argv[2]);
		string dataset = string(argc >= 4 ? argv[3] : "data009");
		size_t size = argc >= 5 ? atoi(argv[4]) : 1;
		
		EVAL(model);
		EVAL(dataset);
		EVAL(size);

		std::unique_ptr<System> uu;
		std::unique_ptr<SystemRepa> ur;
		std::shared_ptr<HistoryRepa> hr;
		{
			std::ifstream in(dataset+".bin", std::ios::binary);
			auto qq = persistentsRecordList(in);
			in.close();
			auto xx = recordListsHistoryRepa_2(8, *qq);
			uu = std::move(std::get<0>(xx));
			ur = std::move(std::get<1>(xx));
			hr = std::move(std::get<2>(xx));
		}

		ECHO(auto z = hr->size > size ? size : hr->size);
		EVAL(z);
		
		StrVarPtrMap m;
		std::ifstream in(model + ".dr", std::ios::binary);
		auto ur1 = persistentsSystemRepa(in, m);
		auto er = persistentsApplicationRepa(in);
		in.close();

		EVAL(fudRepasSize(*er->fud));
		EVAL(frder(*er->fud)->size());
		EVAL(frund(*er->fud)->size());
		EVAL(treesSize(*er->slices));
		EVAL(treesLeafElements(*er->slices)->size());
		
		auto dr = erdr(*er);
		EVAL(dr->fuds.size());
		EVAL(dr->fudRepasSize);
		{
			std::set<std::size_t> derived;
			for (auto& fs : dr->fuds)
				for (auto& tr : fs.fud)
					derived.insert(tr->derived);
			EVAL(derived.size());				
		}
		
		for (std::size_t j = 0; z <= 1000 && j < z; j++)
		{
			auto t0 = clk::now();
			std::cout << "drmul: " << j 
				<< " " << *drmul(HistoryRepaPtrList{hr},HistorySparseArrayPtrList{},*dr,j,2)
				<< " " << ((sec)(clk::now() - t0)).count() << "s" << std::endl;		
		}		
		
		{
			SizeList ev;
			for (std::size_t j = 0; z <= 1000 && j < z; j++)
				ev.push_back(j);
			auto vv1 = treesLeafElements(*er->slices);
			auto hs1 = hrhs(*hrhrred(vv1->size(), vv1->data(), *frmul(*hrsel(ev.size(), ev.data(), *hr), *er->fud)));
			EVAL(*hs1);
		}
		std::shared_ptr<HistorySparseArray> ha = std::move(hsha(*hrhs(*hr)));

		for (std::size_t j = 0; z <= 1000 && j < z; j++)
		{
			auto t0 = clk::now();
			std::cout << "drmul: " << j 
				<< " " << *drmul(HistoryRepaPtrList{},HistorySparseArrayPtrList{ha},*dr,j,2)
				<< " " << ((sec)(clk::now() - t0)).count() << "s" << std::endl;		
		}	
		
		for (std::size_t m = 1; m <= 8; m++)
		{
			auto t1 = clk::now();
			for (std::size_t j = 0; j < z; j++)
			{
				drmul(HistoryRepaPtrList{hr},HistorySparseArrayPtrList{},*dr,j,m);		
			}		
			std::cout << "Average " << m << " " << ((sec)(clk::now() - t1)).count()/z << "s" << std::endl;				
		}
	}
	
	if (argc >= 3 && string(argv[1]) == "update_region")
	{
		auto hrsel = [](const HistoryRepa& hr, std::size_t ev)
		{
			SizeList ll {ev};
			return eventsHistoryRepasHistoryRepaSelection_u(ll.size(), (std::size_t*)ll.data(), hr);
		};
		auto erdr = applicationRepasDecompFudSlicedRepa_u;
		
		string model = string(argv[2]);
		string dataset = string(argc >= 4 ? argv[3] : "data009");
		size_t scale = argc >= 5 ? atoi(argv[4]) : 1;
		bool logging = argc >= 6;
		
		EVAL(model);
		EVAL(dataset);
		EVAL(scale);

		std::unique_ptr<HistoryRepa> hr;
		{	
			std::unique_ptr<System> uu;
			std::unique_ptr<SystemRepa> ur;

			HistoryRepaPtrList ll;
			int s = 17;
			std::ifstream in(dataset+".bin", std::ios::binary);
			auto qq = persistentsRecordList(in);
			in.close();
			for (int i = 0; i < scale; i++)
			{
				auto xx = recordListsHistoryRepaRegion(8, 60, s++, *qq);
				uu = std::move(std::get<0>(xx));
				ur = std::move(std::get<1>(xx));
				ll.push_back(std::move(std::get<2>(xx)));
			}
			hr = vectorHistoryRepasConcat_u(ll);
		}
		
		ActiveUpdateParameters pp;		
		{		
			EVAL(hr->size);
			Active active;
			active.logging = logging;
			active.historySize = 10;
			TRUTH(active.update(pp));
			EVAL(active.eventsUpdated);
			TRUTH(active.historyOverflow);
			EVAL(active.historyEvent);
			EVAL(active.historySize);
			EVAL(active.historySlicesSetEvent);
			EVAL(active.induceSlices);
						
			active.eventsSparse = std::make_shared<ActiveEventsArray>(1);
			EVAL(*active.eventsSparse);
			
			auto ev0 = std::make_shared<ActiveEventsRepa>(1);
			ev0->mapIdEvent[0] = HistoryRepaPtrSizePair(std::move(hrsel(*hr,0)),ev0->references);
			EVAL(*ev0);
			
			active.underlyingEventsRepa.push_back(ev0);
			
			auto hr0 = std::make_shared<HistoryRepa>();
			{
				auto n = hr->dimension;
				auto vv = hr->vectorVar;
				auto sh = hr->shape;
				hr0->dimension = n;
				hr0->vectorVar = new std::size_t[n];
				auto vv1 = hr0->vectorVar;
				hr0->shape = new std::size_t[n];
				auto sh1 = hr0->shape;
				for (std::size_t i = 0; i < n; i++)
				{
					vv1[i] = vv[i];
					sh1[i] = sh[i];
				}
				hr0->evient = true;
				hr0->size = active.historySize;
				auto z1 = hr0->size;
				hr0->arr = new unsigned char[z1*n];
				auto rr0 = hr0->arr;
				memset(rr0, 0, z1*n);
			}
			EVAL(*hr0);
			active.underlyingHistoryRepa.push_back(hr0);

			TRUTH(active.update(pp));
			EVAL(active.eventsUpdated);
			TRUTH(active.historyOverflow);
			EVAL(active.historyEvent);
			EVAL(active.historySize);
			EVAL(active.historySparse);
			EVAL(active.historySlicesSetEvent);
			EVAL(active.induceSlices);
			EVAL(*ev0);
			EVAL(*active.eventsSparse);
			std::cout << endl;
			
			active.decomp = std::make_shared<DecompFudSlicedRepa>();
			EVAL(*active.decomp);
			
			TRUTH(active.update(pp));
			EVAL(active.eventsUpdated);
			TRUTH(active.historyOverflow);
			EVAL(active.historyEvent);
			EVAL(active.historySize);
			EVAL(active.historySparse);
			EVAL(active.historySlicesSetEvent);
			EVAL(active.induceSlices);
			EVAL(*ev0);
			EVAL(*active.eventsSparse);
			std::cout << endl;

		}
		std::cout << endl;
		{		
			EVAL(hr->size);
			Active active;
			active.logging = logging;
			active.historySize = 10;
			active.induceThreshold = 2;
			TRUTH(active.update(pp));
			EVAL(active.eventsUpdated);
			TRUTH(active.historyOverflow);
			EVAL(active.historyEvent);
			EVAL(active.historySize);
			EVAL(active.historySparse);
			EVAL(active.historySlicesSetEvent);
			EVAL(active.induceSlices);
			std::cout << endl;
			
			{
				StrVarPtrMap m;
				std::ifstream in(model + ".dr", std::ios::binary);
				auto ur = persistentsSystemRepa(in, m);
				active.decomp = std::move(erdr(*persistentsApplicationRepa(in)));
				in.close();		
			}
			{			
				auto t0 = clk::now();
				TRUTH(active.update(pp));
				std::cout << ((sec)(clk::now() - t0)).count() << "s" << std::endl;
			}
			EVAL(active.eventsUpdated);
			TRUTH(active.historyOverflow);
			EVAL(active.historyEvent);
			EVAL(active.historySize);
			EVAL(active.historySparse);
			EVAL(active.historySlicesSetEvent);
			EVAL(active.induceSlices);
			std::cout << endl;
			
			active.eventsSparse = std::make_shared<ActiveEventsArray>(1);
			EVAL(*active.eventsSparse);
			
			auto ev0 = std::make_shared<ActiveEventsRepa>(1);
			ev0->mapIdEvent[0] = HistoryRepaPtrSizePair(std::move(hrsel(*hr,0)),ev0->references);
			EVAL(*ev0);
			
			active.underlyingEventsRepa.push_back(ev0);
			
			auto hr0 = std::make_shared<HistoryRepa>();
			{
				auto n = hr->dimension;
				auto vv = hr->vectorVar;
				auto sh = hr->shape;
				hr0->dimension = n;
				hr0->vectorVar = new std::size_t[n];
				auto vv1 = hr0->vectorVar;
				hr0->shape = new std::size_t[n];
				auto sh1 = hr0->shape;
				for (std::size_t i = 0; i < n; i++)
				{
					vv1[i] = vv[i];
					sh1[i] = sh[i];
				}
				hr0->evient = true;
				hr0->size = active.historySize;
				auto z1 = hr0->size;
				hr0->arr = new unsigned char[z1*n];
				auto rr0 = hr0->arr;
				memset(rr0, 0, z1*n);
			}
			EVAL(*hr0);
			active.underlyingHistoryRepa.push_back(hr0);

			{			
				auto t0 = clk::now();
				TRUTH(active.update(pp));
				std::cout << ((sec)(clk::now() - t0)).count() << "s" << std::endl;
			}
			EVAL(active.eventsUpdated);
			TRUTH(active.historyOverflow);
			EVAL(active.historyEvent);
			EVAL(active.historySize);
			EVAL(active.historySparse);
			EVAL(active.historySlicesSetEvent);
			EVAL(active.induceSlices);
			EVAL(*ev0);
			EVAL(*hr0);
			EVAL(*active.eventsSparse);
			std::cout << endl;
			
			for (std::size_t i = 1; i < 10; i++) 
				ev0->mapIdEvent[i] = HistoryRepaPtrSizePair(std::move(hrsel(*hr,i)),ev0->references);
			EVAL(*ev0);	

			{			
				auto t0 = clk::now();
				TRUTH(active.update(pp));
				std::cout << ((sec)(clk::now() - t0)).count() << "s" << std::endl;
			}
			EVAL(active.eventsUpdated);
			TRUTH(active.historyOverflow);
			EVAL(active.historyEvent);
			EVAL(active.historySize);
			EVAL(active.historySparse);
			EVAL(active.historySlicesSetEvent);
			EVAL(active.induceSlices);
			EVAL(*ev0);
			EVAL(*hr0);
			EVAL(*active.eventsSparse);
			std::cout << endl;
			
			for (std::size_t i = 10; i < 11; i++) 
				ev0->mapIdEvent[i] = HistoryRepaPtrSizePair(std::move(hrsel(*hr,i)),ev0->references);
			EVAL(*ev0);	

			{			
				auto t0 = clk::now();
				TRUTH(active.update(pp));
				std::cout << ((sec)(clk::now() - t0)).count() << "s" << std::endl;
			}
			EVAL(active.eventsUpdated);
			TRUTH(active.historyOverflow);
			EVAL(active.historyEvent);
			EVAL(active.historySize);
			EVAL(active.historySparse);
			EVAL(active.historySlicesSetEvent);
			EVAL(active.induceSlices);
			EVAL(*ev0);
			EVAL(*hr0);
			EVAL(*active.eventsSparse);
			std::cout << endl;
	
			for (std::size_t i = 11; i < 15; i++) 
				ev0->mapIdEvent[i] = HistoryRepaPtrSizePair(std::move(hrsel(*hr,i)),ev0->references);
			EVAL(*ev0);	

			{			
				auto t0 = clk::now();
				TRUTH(active.update(pp));
				std::cout << ((sec)(clk::now() - t0)).count() << "s" << std::endl;
			}
			EVAL(active.eventsUpdated);
			TRUTH(active.historyOverflow);
			EVAL(active.historyEvent);
			EVAL(active.historySize);
			EVAL(active.historySparse);
			EVAL(active.historySlicesSetEvent);
			EVAL(active.induceSlices);
			EVAL(*ev0);
			EVAL(*hr0);
			EVAL(*active.eventsSparse);
			std::cout << endl;		
				
			for (std::size_t i = 15; i < 18; i++) 
				ev0->mapIdEvent[i] = HistoryRepaPtrSizePair(std::move(hrsel(*hr,i)),ev0->references);
			EVAL(*ev0);	

			{			
				auto t0 = clk::now();
				TRUTH(active.update(pp));
				std::cout << ((sec)(clk::now() - t0)).count() << "s" << std::endl;
			}
			EVAL(active.eventsUpdated);
			TRUTH(active.historyOverflow);
			EVAL(active.historyEvent);
			EVAL(active.historySize);
			EVAL(active.historySparse);
			EVAL(active.historySlicesSetEvent);
			EVAL(active.induceSlices);
			EVAL(*ev0);
			EVAL(*hr0);
			EVAL(*active.eventsSparse);
			std::cout << endl;	
		}
		std::cout << endl;
		{		
			EVAL(hr->size);
			Active active;
			active.logging = logging;
			active.historySize = 10;
			active.induceThreshold = 2;
			TRUTH(active.update(pp));
			EVAL(active.eventsUpdated);
			TRUTH(active.historyOverflow);
			EVAL(active.historyEvent);
			EVAL(active.historySize);
			EVAL(active.historySparse);
			EVAL(active.historySlicesSetEvent);
			EVAL(active.induceSlices);
			std::cout << endl;
			
			{
				StrVarPtrMap m;
				std::ifstream in(model + ".dr", std::ios::binary);
				auto ur = persistentsSystemRepa(in, m);
				active.decomp = std::move(erdr(*persistentsApplicationRepa(in)));
				in.close();		
			}
			{			
				auto t0 = clk::now();
				TRUTH(active.update(pp));
				std::cout << ((sec)(clk::now() - t0)).count() << "s" << std::endl;
			}
			EVAL(active.eventsUpdated);
			TRUTH(active.historyOverflow);
			EVAL(active.historyEvent);
			EVAL(active.historySize);
			EVAL(active.historySparse);
			EVAL(active.historySlicesSetEvent);
			EVAL(active.induceSlices);
			std::cout << endl;
			
			active.eventsSparse = std::make_shared<ActiveEventsArray>(1);
			EVAL(*active.eventsSparse);
			
			auto ev0 = std::make_shared<ActiveEventsRepa>(2);
			ev0->mapIdEvent[0] = HistoryRepaPtrSizePair(std::move(hrsel(*hr,0)),ev0->references);
			EVAL(*ev0);
			
			active.underlyingEventsRepa.push_back(ev0);
			
			auto hr0 = std::make_shared<HistoryRepa>();
			{
				auto n = hr->dimension;
				auto vv = hr->vectorVar;
				auto sh = hr->shape;
				hr0->dimension = n;
				hr0->vectorVar = new std::size_t[n];
				auto vv1 = hr0->vectorVar;
				hr0->shape = new std::size_t[n];
				auto sh1 = hr0->shape;
				for (std::size_t i = 0; i < n; i++)
				{
					vv1[i] = vv[i];
					sh1[i] = sh[i];
				}
				hr0->evient = true;
				hr0->size = active.historySize;
				auto z1 = hr0->size;
				hr0->arr = new unsigned char[z1*n];
				auto rr0 = hr0->arr;
				memset(rr0, 0, z1*n);
			}
			EVAL(*hr0);
			active.underlyingHistoryRepa.push_back(hr0);

			{			
				auto t0 = clk::now();
				TRUTH(active.update(pp));
				std::cout << ((sec)(clk::now() - t0)).count() << "s" << std::endl;
			}
			EVAL(active.eventsUpdated);
			TRUTH(active.historyOverflow);
			EVAL(active.historyEvent);
			EVAL(active.historySize);
			EVAL(active.historySparse);
			EVAL(active.historySlicesSetEvent);
			EVAL(active.induceSlices);
			EVAL(*ev0);
			EVAL(*hr0);
			EVAL(*active.eventsSparse);
			std::cout << endl;
			
			for (std::size_t i = 1; i < 10; i++) 
				ev0->mapIdEvent[i] = HistoryRepaPtrSizePair(std::move(hrsel(*hr,i)),ev0->references);
			EVAL(*ev0);	

			{			
				auto t0 = clk::now();
				TRUTH(active.update(pp));
				std::cout << ((sec)(clk::now() - t0)).count() << "s" << std::endl;
			}
			EVAL(active.eventsUpdated);
			TRUTH(active.historyOverflow);
			EVAL(active.historyEvent);
			EVAL(active.historySize);
			EVAL(active.historySparse);
			EVAL(active.historySlicesSetEvent);
			EVAL(active.induceSlices);
			EVAL(*ev0);
			EVAL(*hr0);
			EVAL(*active.eventsSparse);
			std::cout << endl;
			
			for (std::size_t i = 10; i < 11; i++) 
				ev0->mapIdEvent[i] = HistoryRepaPtrSizePair(std::move(hrsel(*hr,i)),ev0->references);
			EVAL(*ev0);	

			{			
				auto t0 = clk::now();
				TRUTH(active.update(pp));
				std::cout << ((sec)(clk::now() - t0)).count() << "s" << std::endl;
			}
			EVAL(active.eventsUpdated);
			TRUTH(active.historyOverflow);
			EVAL(active.historyEvent);
			EVAL(active.historySize);
			EVAL(active.historySparse);
			EVAL(active.historySlicesSetEvent);
			EVAL(active.induceSlices);
			EVAL(*ev0);
			EVAL(*hr0);
			EVAL(*active.eventsSparse);
			std::cout << endl;
	
			for (std::size_t i = 11; i < 15; i++) 
				ev0->mapIdEvent[i] = HistoryRepaPtrSizePair(std::move(hrsel(*hr,i)),ev0->references);
			EVAL(*ev0);	

			{			
				auto t0 = clk::now();
				TRUTH(active.update(pp));
				std::cout << ((sec)(clk::now() - t0)).count() << "s" << std::endl;
			}
			EVAL(active.eventsUpdated);
			TRUTH(active.historyOverflow);
			EVAL(active.historyEvent);
			EVAL(active.historySize);
			EVAL(active.historySparse);
			EVAL(active.historySlicesSetEvent);
			EVAL(active.induceSlices);
			EVAL(*ev0);
			EVAL(*hr0);
			EVAL(*active.eventsSparse);
			std::cout << endl;		
			
			for (std::size_t i = 15; i < 18; i++) 
				ev0->mapIdEvent[i] = HistoryRepaPtrSizePair(std::move(hrsel(*hr,i)),1);
			EVAL(*ev0);	

			{			
				auto t0 = clk::now();
				TRUTH(active.update(pp));
				std::cout << ((sec)(clk::now() - t0)).count() << "s" << std::endl;
			}
			EVAL(active.eventsUpdated);
			TRUTH(active.historyOverflow);
			EVAL(active.historyEvent);
			EVAL(active.historySize);
			EVAL(active.historySparse);
			EVAL(active.historySlicesSetEvent);
			EVAL(active.induceSlices);
			EVAL(*ev0);
			EVAL(*hr0);
			EVAL(*active.eventsSparse);
			std::cout << endl;	
		}
	}
	
	if (argc >= 3 && string(argv[1]) == "update")
	{
		auto hrsel = [](const HistoryRepa& hr, std::size_t ev)
		{
			SizeList ll {ev};
			return eventsHistoryRepasHistoryRepaSelection_u(ll.size(), (std::size_t*)ll.data(), hr);
		};
		auto erdr = applicationRepasDecompFudSlicedRepa_u;
		
		string model = string(argv[2]);
		string dataset = string(argc >= 4 ? argv[3] : "data009");
		
		EVAL(model);
		EVAL(dataset);

		std::unique_ptr<System> uu;
		std::unique_ptr<SystemRepa> ur;
		std::unique_ptr<HistoryRepa> hr;
		{
			std::ifstream in(dataset+".bin", std::ios::binary);
			auto qq = persistentsRecordList(in);
			in.close();
			auto xx = recordListsHistoryRepa_2(8, *qq);
			uu = std::move(std::get<0>(xx));
			ur = std::move(std::get<1>(xx));
			hr = std::move(std::get<2>(xx));
		}
		
		ActiveUpdateParameters pp;
		
		{		
			EVAL(hr->size);
			Active active;
			active.historySize = 10;
			TRUTH(active.update(pp));
			EVAL(active.eventsUpdated);
			TRUTH(active.historyOverflow);
			EVAL(active.historyEvent);
			EVAL(active.historySize);
			EVAL(active.historySlicesSetEvent);
			EVAL(active.induceSlices);
						
			active.eventsSparse = std::make_shared<ActiveEventsArray>(1);
			EVAL(*active.eventsSparse);
			
			auto ev0 = std::make_shared<ActiveEventsRepa>(1);
			ev0->mapIdEvent[0] = HistoryRepaPtrSizePair(std::move(hrsel(*hr,0)),ev0->references);
			// EVAL(*ev0);
			
			active.underlyingEventsRepa.push_back(ev0);
			
			auto hr0 = std::make_shared<HistoryRepa>();
			{
				auto n = hr->dimension;
				auto vv = hr->vectorVar;
				auto sh = hr->shape;
				hr0->dimension = n;
				hr0->vectorVar = new std::size_t[n];
				auto vv1 = hr0->vectorVar;
				hr0->shape = new std::size_t[n];
				auto sh1 = hr0->shape;
				for (std::size_t i = 0; i < n; i++)
				{
					vv1[i] = vv[i];
					sh1[i] = sh[i];
				}
				hr0->evient = true;
				hr0->size = active.historySize;
				auto z1 = hr0->size;
				hr0->arr = new unsigned char[z1*n];
				auto rr0 = hr0->arr;
				memset(rr0, 0, z1*n);
			}
			// EVAL(*hr0);
			active.underlyingHistoryRepa.push_back(hr0);

			TRUTH(active.update(pp));
			EVAL(active.eventsUpdated);
			TRUTH(active.historyOverflow);
			EVAL(active.historyEvent);
			EVAL(active.historySize);
			EVAL(active.historySparse);
			EVAL(active.historySlicesSetEvent);
			EVAL(active.induceSlices);
			// EVAL(*ev0);
			EVAL(*active.eventsSparse);
			std::cout << endl;
			
			active.decomp = std::make_shared<DecompFudSlicedRepa>();
			EVAL(*active.decomp);
			
			TRUTH(active.update(pp));
			EVAL(active.eventsUpdated);
			TRUTH(active.historyOverflow);
			EVAL(active.historyEvent);
			EVAL(active.historySize);
			EVAL(active.historySparse);
			EVAL(active.historySlicesSetEvent);
			EVAL(active.induceSlices);
			// EVAL(*ev0);
			EVAL(*active.eventsSparse);
			std::cout << endl;

		}
		std::cout << endl;
		{		
			EVAL(hr->size);
			Active active;
			active.historySize = 10;
			active.induceThreshold = 2;
			TRUTH(active.update(pp));
			EVAL(active.eventsUpdated);
			TRUTH(active.historyOverflow);
			EVAL(active.historyEvent);
			EVAL(active.historySize);
			EVAL(active.historySparse);
			EVAL(active.historySlicesSetEvent);
			EVAL(active.induceSlices);
			std::cout << endl;
			
			{
				StrVarPtrMap m;
				std::ifstream in(model + ".dr", std::ios::binary);
				auto ur = persistentsSystemRepa(in, m);
				active.decomp = std::move(erdr(*persistentsApplicationRepa(in)));
				in.close();		
			}
			{			
				auto t0 = clk::now();
				TRUTH(active.update(pp));
				std::cout << ((sec)(clk::now() - t0)).count() << "s" << std::endl;
			}
			EVAL(active.eventsUpdated);
			TRUTH(active.historyOverflow);
			EVAL(active.historyEvent);
			EVAL(active.historySize);
			EVAL(active.historySparse);
			EVAL(active.historySlicesSetEvent);
			EVAL(active.induceSlices);
			std::cout << endl;
			
			active.eventsSparse = std::make_shared<ActiveEventsArray>(1);
			EVAL(*active.eventsSparse);
			
			auto ev0 = std::make_shared<ActiveEventsRepa>(1);
			ev0->mapIdEvent[0] = HistoryRepaPtrSizePair(std::move(hrsel(*hr,0)),ev0->references);
			// EVAL(*ev0);
			
			active.underlyingEventsRepa.push_back(ev0);
			
			auto hr0 = std::make_shared<HistoryRepa>();
			{
				auto n = hr->dimension;
				auto vv = hr->vectorVar;
				auto sh = hr->shape;
				hr0->dimension = n;
				hr0->vectorVar = new std::size_t[n];
				auto vv1 = hr0->vectorVar;
				hr0->shape = new std::size_t[n];
				auto sh1 = hr0->shape;
				for (std::size_t i = 0; i < n; i++)
				{
					vv1[i] = vv[i];
					sh1[i] = sh[i];
				}
				hr0->evient = true;
				hr0->size = active.historySize;
				auto z1 = hr0->size;
				hr0->arr = new unsigned char[z1*n];
				auto rr0 = hr0->arr;
				memset(rr0, 0, z1*n);
			}
			// EVAL(*hr0);
			active.underlyingHistoryRepa.push_back(hr0);

			{			
				auto t0 = clk::now();
				TRUTH(active.update(pp));
				std::cout << ((sec)(clk::now() - t0)).count() << "s" << std::endl;
			}
			EVAL(active.eventsUpdated);
			TRUTH(active.historyOverflow);
			EVAL(active.historyEvent);
			EVAL(active.historySize);
			EVAL(active.historySparse);
			EVAL(active.historySlicesSetEvent);
			EVAL(active.induceSlices);
			// EVAL(*ev0);
			// EVAL(*hr0);
			EVAL(*active.eventsSparse);
			std::cout << endl;
			
			for (std::size_t i = 1; i < 10; i++) 
				ev0->mapIdEvent[i] = HistoryRepaPtrSizePair(std::move(hrsel(*hr,i)),ev0->references);
			// EVAL(*ev0);	

			{			
				auto t0 = clk::now();
				TRUTH(active.update(pp));
				std::cout << ((sec)(clk::now() - t0)).count() << "s" << std::endl;
			}
			EVAL(active.eventsUpdated);
			TRUTH(active.historyOverflow);
			EVAL(active.historyEvent);
			EVAL(active.historySize);
			EVAL(active.historySparse);
			EVAL(active.historySlicesSetEvent);
			EVAL(active.induceSlices);
			// EVAL(*ev0);
			// EVAL(*hr0);
			EVAL(*active.eventsSparse);
			std::cout << endl;
			
			for (std::size_t i = 10; i < 11; i++) 
				ev0->mapIdEvent[i] = HistoryRepaPtrSizePair(std::move(hrsel(*hr,i)),ev0->references);
			// EVAL(*ev0);	

			{			
				auto t0 = clk::now();
				TRUTH(active.update(pp));
				std::cout << ((sec)(clk::now() - t0)).count() << "s" << std::endl;
			}
			EVAL(active.eventsUpdated);
			TRUTH(active.historyOverflow);
			EVAL(active.historyEvent);
			EVAL(active.historySize);
			EVAL(active.historySparse);
			EVAL(active.historySlicesSetEvent);
			EVAL(active.induceSlices);
			// EVAL(*ev0);
			// EVAL(*hr0);
			EVAL(*active.eventsSparse);
			std::cout << endl;
	
			for (std::size_t i = 11; i < 15; i++) 
				ev0->mapIdEvent[i] = HistoryRepaPtrSizePair(std::move(hrsel(*hr,i)),ev0->references);
			// EVAL(*ev0);	

			{			
				auto t0 = clk::now();
				TRUTH(active.update(pp));
				std::cout << ((sec)(clk::now() - t0)).count() << "s" << std::endl;
			}
			EVAL(active.eventsUpdated);
			TRUTH(active.historyOverflow);
			EVAL(active.historyEvent);
			EVAL(active.historySize);
			EVAL(active.historySparse);
			EVAL(active.historySlicesSetEvent);
			EVAL(active.induceSlices);
			// EVAL(*ev0);
			// EVAL(*hr0);
			EVAL(*active.eventsSparse);
			std::cout << endl;		
				
			for (std::size_t i = 15; i < 18; i++) 
				ev0->mapIdEvent[i] = HistoryRepaPtrSizePair(std::move(hrsel(*hr,i)),ev0->references);
			// EVAL(*ev0);	

			{			
				auto t0 = clk::now();
				TRUTH(active.update(pp));
				std::cout << ((sec)(clk::now() - t0)).count() << "s" << std::endl;
			}
			EVAL(active.eventsUpdated);
			TRUTH(active.historyOverflow);
			EVAL(active.historyEvent);
			EVAL(active.historySize);
			EVAL(active.historySparse);
			EVAL(active.historySlicesSetEvent);
			EVAL(active.induceSlices);
			// EVAL(*ev0);
			// EVAL(*hr0);
			EVAL(*active.eventsSparse);
			std::cout << endl;	
		}
		
		std::cout << endl;
		{		
			EVAL(hr->size);
			Active active;
			active.historySize = 10;
			active.induceThreshold = 2;
			TRUTH(active.update(pp));
			EVAL(active.eventsUpdated);
			TRUTH(active.historyOverflow);
			EVAL(active.historyEvent);
			EVAL(active.historySize);
			EVAL(active.historySparse);
			EVAL(active.historySlicesSetEvent);
			EVAL(active.induceSlices);
			std::cout << endl;
			
			{
				StrVarPtrMap m;
				std::ifstream in(model + ".dr", std::ios::binary);
				auto ur = persistentsSystemRepa(in, m);
				active.decomp = std::move(erdr(*persistentsApplicationRepa(in)));
				in.close();		
			}
			{			
				auto t0 = clk::now();
				TRUTH(active.update(pp));
				std::cout << ((sec)(clk::now() - t0)).count() << "s" << std::endl;
			}
			EVAL(active.eventsUpdated);
			TRUTH(active.historyOverflow);
			EVAL(active.historyEvent);
			EVAL(active.historySize);
			EVAL(active.historySparse);
			EVAL(active.historySlicesSetEvent);
			EVAL(active.induceSlices);
			std::cout << endl;
			
			active.eventsSparse = std::make_shared<ActiveEventsArray>(1);
			EVAL(*active.eventsSparse);
			
			auto ev0 = std::make_shared<ActiveEventsRepa>(2);
			ev0->mapIdEvent[0] = HistoryRepaPtrSizePair(std::move(hrsel(*hr,0)),ev0->references);
			// EVAL(*ev0);
			
			active.underlyingEventsRepa.push_back(ev0);
			
			auto hr0 = std::make_shared<HistoryRepa>();
			{
				auto n = hr->dimension;
				auto vv = hr->vectorVar;
				auto sh = hr->shape;
				hr0->dimension = n;
				hr0->vectorVar = new std::size_t[n];
				auto vv1 = hr0->vectorVar;
				hr0->shape = new std::size_t[n];
				auto sh1 = hr0->shape;
				for (std::size_t i = 0; i < n; i++)
				{
					vv1[i] = vv[i];
					sh1[i] = sh[i];
				}
				hr0->evient = true;
				hr0->size = active.historySize;
				auto z1 = hr0->size;
				hr0->arr = new unsigned char[z1*n];
				auto rr0 = hr0->arr;
				memset(rr0, 0, z1*n);
			}
			// EVAL(*hr0);
			active.underlyingHistoryRepa.push_back(hr0);

			{			
				auto t0 = clk::now();
				TRUTH(active.update(pp));
				std::cout << ((sec)(clk::now() - t0)).count() << "s" << std::endl;
			}
			EVAL(active.eventsUpdated);
			TRUTH(active.historyOverflow);
			EVAL(active.historyEvent);
			EVAL(active.historySize);
			EVAL(active.historySparse);
			EVAL(active.historySlicesSetEvent);
			EVAL(active.induceSlices);
			// EVAL(*ev0);
			// EVAL(*hr0);
			EVAL(*active.eventsSparse);
			std::cout << endl;
			
			for (std::size_t i = 1; i < 10; i++) 
				ev0->mapIdEvent[i] = HistoryRepaPtrSizePair(std::move(hrsel(*hr,i)),ev0->references);
			// EVAL(*ev0);	

			{			
				auto t0 = clk::now();
				TRUTH(active.update(pp));
				std::cout << ((sec)(clk::now() - t0)).count() << "s" << std::endl;
			}
			EVAL(active.eventsUpdated);
			TRUTH(active.historyOverflow);
			EVAL(active.historyEvent);
			EVAL(active.historySize);
			EVAL(active.historySparse);
			EVAL(active.historySlicesSetEvent);
			EVAL(active.induceSlices);
			// EVAL(*ev0);
			// EVAL(*hr0);
			EVAL(*active.eventsSparse);
			std::cout << endl;
			
			for (std::size_t i = 10; i < 11; i++) 
				ev0->mapIdEvent[i] = HistoryRepaPtrSizePair(std::move(hrsel(*hr,i)),ev0->references);
			// EVAL(*ev0);	

			{			
				auto t0 = clk::now();
				TRUTH(active.update(pp));
				std::cout << ((sec)(clk::now() - t0)).count() << "s" << std::endl;
			}
			EVAL(active.eventsUpdated);
			TRUTH(active.historyOverflow);
			EVAL(active.historyEvent);
			EVAL(active.historySize);
			EVAL(active.historySparse);
			EVAL(active.historySlicesSetEvent);
			EVAL(active.induceSlices);
			// EVAL(*ev0);
			// EVAL(*hr0);
			EVAL(*active.eventsSparse);
			std::cout << endl;
	
			for (std::size_t i = 11; i < 15; i++) 
				ev0->mapIdEvent[i] = HistoryRepaPtrSizePair(std::move(hrsel(*hr,i)),ev0->references);
			// EVAL(*ev0);	

			{			
				auto t0 = clk::now();
				TRUTH(active.update(pp));
				std::cout << ((sec)(clk::now() - t0)).count() << "s" << std::endl;
			}
			EVAL(active.eventsUpdated);
			TRUTH(active.historyOverflow);
			EVAL(active.historyEvent);
			EVAL(active.historySize);
			EVAL(active.historySparse);
			EVAL(active.historySlicesSetEvent);
			EVAL(active.induceSlices);
			// EVAL(*ev0);
			// EVAL(*hr0);
			EVAL(*active.eventsSparse);
			std::cout << endl;		
			
			for (std::size_t i = 15; i < 18; i++) 
				ev0->mapIdEvent[i] = HistoryRepaPtrSizePair(std::move(hrsel(*hr,i)),1);
			// EVAL(*ev0);	

			auto updatePost = [](const SizeSet& eventsA, std::size_t eventA, std::size_t historyEventA, std::size_t sliceA)
			{
				std::cout << "eventsA: " << eventsA << std::endl;
				std::cout << "eventA: " << eventA << std::endl;
				std::cout << "historyEventA: " << historyEventA << std::endl;
				std::cout << "sliceA: " << sliceA << std::endl;
				return true;
			};
			
			active.updateCallback = updatePost;
			{			
				auto t0 = clk::now();
				TRUTH(active.update(pp));
				std::cout << ((sec)(clk::now() - t0)).count() << "s" << std::endl;
			}
			EVAL(active.eventsUpdated);
			TRUTH(active.historyOverflow);
			EVAL(active.historyEvent);
			EVAL(active.historySize);
			EVAL(active.historySparse);
			EVAL(active.historySlicesSetEvent);
			EVAL(active.induceSlices);
			// EVAL(*ev0);
			// EVAL(*hr0);
			EVAL(*active.eventsSparse);
			std::cout << endl;	
		}
	}

	if (false)
	{
		auto hrsel = eventsHistoryRepasHistoryRepaSelection_u;
		auto hrpr = historyRepasRed;
		auto hrhs = historyRepasHistorySparse;
		auto hshr = historySparsesHistoryRepa;
		auto hahs = historySparseArraysHistorySparse;
		auto hsha = historySparsesHistorySparseArray;
		auto hrshuffle = historyRepasShuffle_u;
		auto hrshuffle2 = historyRepasShuffle_us;
		auto hashuffle2 = historySparseArrayShuffle_us;
		
		std::unique_ptr<HistoryRepa> hr;
		{
			std::unique_ptr<System> uu;
			std::unique_ptr<SystemRepa> ur;
			std::ifstream in("data009.bin", std::ios::binary);
			auto qq = persistentsRecordList(in);
			in.close();
			auto xx = recordListsHistoryRepaRegion(8, 10, 17, *qq);
			uu = std::move(std::get<0>(xx));
			ur = std::move(std::get<1>(xx));
			hr = std::move(std::get<2>(xx));
			
			SizeList ev;
			for (size_t i = 0; i < 20; i++)
				ev.push_back(i);
			hr = hrsel(ev.size(), ev.data(), *hr);
		}
		
		EVAL(*hr);				
		auto pr1 = hrpr(*hr);
		EVAL(*hrpr(*hr));
		{
			auto t0 = clk::now();
			ECHO(auto hrs1 = hrshuffle(*hr,17));
			std::cout << ((sec)(clk::now() - t0)).count() << "s" << std::endl;		
			EVAL(*hrs1);				
			EVAL(*hrpr(*hrs1));			
		}

		{
			auto t0 = clk::now();
			std::ranlux48_base gen(17);
			ECHO(auto hrs2 = hrshuffle2(*hr,gen));
			std::cout << ((sec)(clk::now() - t0)).count() << "s" << std::endl;		
			EVAL(*hrs2);				
			EVAL(*hrpr(*hrs2));
		}
	}
	
	if (false)
	{
		auto hrsel = eventsHistoryRepasHistoryRepaSelection_u;
		auto hrpr = historyRepasRed;
		auto hrhs = historyRepasHistorySparse;
		auto hshr = historySparsesHistoryRepa;
		auto hahs = historySparseArraysHistorySparse;
		auto hsha = historySparsesHistorySparseArray;
		auto hrshuffle = historyRepasShuffle_u;
		auto hrshuffle2 = historyRepasShuffle_us;
		auto hashuffle2 = historySparseArrayShuffle_us;
		
		std::unique_ptr<HistoryRepa> hr;
		{
			std::unique_ptr<System> uu;
			std::unique_ptr<SystemRepa> ur;
			std::ifstream in("data009.bin", std::ios::binary);
			auto qq = persistentsRecordList(in);
			in.close();
			auto xx = recordListsHistoryRepaRegion(8, 10, 17, *qq);
			uu = std::move(std::get<0>(xx));
			ur = std::move(std::get<1>(xx));
			hr = std::move(std::get<2>(xx));
			
			SizeList ev;
			for (size_t i = 0; i < 20; i++)
				ev.push_back(i);
			hr = hrsel(ev.size(), ev.data(), *hr);
			hr->transpose();		}
		
		EVAL(*hr);				
		auto pr1 = hrpr(*hr);
		EVAL(*hrpr(*hr));
		{
			auto t0 = clk::now();
			ECHO(auto hrs1 = hrshuffle(*hr,17));
			std::cout << ((sec)(clk::now() - t0)).count() << "s" << std::endl;		
			EVAL(*hrs1);				
			EVAL(*hrpr(*hrs1));			
		}

		{
			auto t0 = clk::now();
			std::ranlux48_base gen(17);
			ECHO(auto hrs2 = hrshuffle2(*hr,gen));
			std::cout << ((sec)(clk::now() - t0)).count() << "s" << std::endl;		
			EVAL(*hrs2);				
			EVAL(*hrpr(*hrs2));
		}
	}
	
	if (false)
	{
		auto hrsel = eventsHistoryRepasHistoryRepaSelection_u;
		auto hrpr = historyRepasRed;
		auto hrhs = historyRepasHistorySparse;
		auto hshr = historySparsesHistoryRepa;
		auto hahs = historySparseArraysHistorySparse;
		auto hsha = historySparsesHistorySparseArray;
		auto hrshuffle = historyRepasShuffle_u;
		auto hrshuffle2 = historyRepasShuffle_us;
		auto hashuffle2 = historySparseArrayShuffle_us;
		
		std::unique_ptr<HistoryRepa> hr;
		{
			std::unique_ptr<System> uu;
			std::unique_ptr<SystemRepa> ur;
			std::ifstream in("data009.bin", std::ios::binary);
			auto qq = persistentsRecordList(in);
			in.close();
			auto xx = recordListsHistoryRepaRegion(2, 1, 17, *qq);
			uu = std::move(std::get<0>(xx));
			ur = std::move(std::get<1>(xx));
			hr = std::move(std::get<2>(xx));
			
			SizeList ev;
			for (size_t i = 0; i < 200; i++)
				ev.push_back(i);
			hr = hrsel(ev.size(), ev.data(), *hr);
			hr->vectorVar[0] = 100;
		}
		
		EVAL(*hr);
		EVAL(*hrpr(*hr));
		
		{
			auto t0 = clk::now();
			std::ranlux48_base gen(17);
			ECHO(auto hrs2 = hrshuffle2(*hr,gen));
			std::cout << ((sec)(clk::now() - t0)).count() << "s" << std::endl;		
			EVAL(*hrs2);
			EVAL(*hrpr(*hrs2));
		}
		
		ECHO(auto ha = hsha(*hrhs(*hr)));
		// EVAL(*ha);
		// EVAL(*hahs(*ha));
		// EVAL(*hshr(*hahs(*ha)));
		EVAL(*hrpr(*hshr(*hahs(*ha))));
		
		{
			auto t0 = clk::now();
			std::ranlux48_base gen(17);
			ECHO(auto has2 = hashuffle2(*ha,gen));
			std::cout << ((sec)(clk::now() - t0)).count() << "s" << std::endl;		
			EVAL(*hshr(*hahs(*has2)));
			EVAL(*hrpr(*hshr(*hahs(*has2))));
		}
	}
	
	if (false)
	{
		auto hrsel = eventsHistoryRepasHistoryRepaSelection_u;
		auto hrpr = historyRepasRed;
		auto hrha = historyRepasHistorySparseArray;
		auto hahr = historySparseArraysHistoryRepa;
		auto hrshuffle = historyRepasShuffle_u;
		auto hrshuffle2 = historyRepasShuffle_us;
		auto hashuffle2 = historySparseArrayShuffle_us;
		
		std::unique_ptr<HistoryRepa> hr;
		{
			std::unique_ptr<System> uu;
			std::unique_ptr<SystemRepa> ur;
			std::ifstream in("data009.bin", std::ios::binary);
			auto qq = persistentsRecordList(in);
			in.close();
			auto xx = recordListsHistoryRepaRegion(2, 2, 17, *qq);
			uu = std::move(std::get<0>(xx));
			ur = std::move(std::get<1>(xx));
			hr = std::move(std::get<2>(xx));
			
			SizeList ev;
			for (size_t i = 0; i < 200; i++)
				ev.push_back(i);
			hr = hrsel(ev.size(), ev.data(), *hr);
			hr->vectorVar[0] = 100;
			hr->vectorVar[1] = 200;
		}
		
		EVAL(*hr);
		EVAL(*hrpr(*hr));
		// EVAL(*hrhs(*hr));
		
		{
			auto t0 = clk::now();
			std::ranlux48_base gen(17);
			ECHO(auto hrs2 = hrshuffle2(*hr,gen));
			std::cout << ((sec)(clk::now() - t0)).count() << "s" << std::endl;		
			EVAL(*hrs2);
			EVAL(*hrpr(*hrs2));
		}
		
		ECHO(auto ha = hrha(*hr));
		EVAL(*hrpr(*hahr(*ha)));
		
		{
			auto t0 = clk::now();
			std::ranlux48_base gen(17);
			ECHO(auto has2 = hashuffle2(*ha,gen));
			std::cout << ((sec)(clk::now() - t0)).count() << "s" << std::endl;		
			EVAL(*hahr(*has2));
			EVAL(*hrpr(*hahr(*has2)));
		}
	}
	
	if (false)
	{
		auto hrsel = eventsHistoryRepasHistoryRepaSelection_u;
		auto hrpr = historyRepasRed;
		auto hrha = historyRepasHistorySparseArray;
		auto hahr = historySparseArraysHistoryRepa;
		auto hrshuffle = historyRepasShuffle_u;
		auto hrshuffle2 = historyRepasShuffle_us;
		auto hashuffle2 = historySparseArrayShuffle_us;
		
		std::unique_ptr<HistoryRepa> hr;
		{
			std::unique_ptr<System> uu;
			std::unique_ptr<SystemRepa> ur;
			std::ifstream in("data009.bin", std::ios::binary);
			auto qq = persistentsRecordList(in);
			in.close();
			auto xx = recordListsHistoryRepaRegion(2, 2, 17, *qq);
			uu = std::move(std::get<0>(xx));
			ur = std::move(std::get<1>(xx));
			hr = std::move(std::get<2>(xx));
			
			SizeList ev;
			for (size_t i = 0; i < 200; i++)
				ev.push_back(i);
			hr = hrsel(ev.size(), ev.data(), *hr);
			hr->vectorVar[0] = 100;
			hr->vectorVar[1] = 200;
			hr->transpose();
		}
		
		EVAL(*hr);
		EVAL(*hrpr(*hr));
		// EVAL(*hrhs(*hr));
		
		{
			auto t0 = clk::now();
			std::ranlux48_base gen(17);
			ECHO(auto hrs2 = hrshuffle2(*hr,gen));
			std::cout << ((sec)(clk::now() - t0)).count() << "s" << std::endl;		
			EVAL(*hrs2);
			EVAL(*hrpr(*hrs2));
		}
		
		ECHO(auto ha = hrha(*hr));
		EVAL(*hrpr(*hahr(*ha)));
		
		{
			auto t0 = clk::now();
			std::ranlux48_base gen(17);
			ECHO(auto has2 = hashuffle2(*ha,gen));
			std::cout << ((sec)(clk::now() - t0)).count() << "s" << std::endl;		
			EVAL(*hahr(*has2));
			EVAL(*hrpr(*hahr(*has2)));
		}
	}
	
	if (argc >= 3 && string(argv[1]) == "induce01")
	{
		auto hrsel = [](const HistoryRepa& hr, std::size_t ev)
		{
			SizeList ll {ev};
			return eventsHistoryRepasHistoryRepaSelection_u(ll.size(), (std::size_t*)ll.data(), hr);
		};
		auto erdr = applicationRepasDecompFudSlicedRepa_u;
		
		string dataset = string(argc >= 3 ? argv[2] : "data009");
		
		EVAL(dataset);

		std::unique_ptr<System> uu;
		std::unique_ptr<SystemRepa> ur;
		std::unique_ptr<HistoryRepa> hr;
		{
			std::ifstream in(dataset+".bin", std::ios::binary);
			auto qq = persistentsRecordList(in);
			in.close();
			auto xx = recordListsHistoryRepa_4(8, *qq);
			uu = std::move(std::get<0>(xx));
			ur = std::move(std::get<1>(xx));
			hr = std::move(std::get<2>(xx));
		}
		EVAL(hr->size);
		EVAL(ur->listVarSizePair.size());
		
		{
			auto eventsA = std::make_shared<ActiveEventsRepa>(1);
			
			Active activeA;
			activeA.system = std::make_shared<ActiveSystem>();
			activeA.var = activeA.system->next(activeA.bits);
			EVAL(activeA.var);
			activeA.varSlice = activeA.system->next(activeA.bits);
			EVAL(activeA.varSlice);
			activeA.historySize = 10;
			activeA.induceThreshold = 5;
			activeA.logging = true;
			activeA.decomp = std::make_shared<DecompFudSlicedRepa>();
			activeA.underlyingEventsRepa.push_back(eventsA);
			{
				SizeList vv0;
				{
					auto& mm = ur->mapVarSize();
					auto vscan = std::make_shared<Variable>("scan");
					for (int i = 330; i < 390; i++)
						vv0.push_back(mm[Variable(vscan, std::make_shared<Variable>((i % 360) + 1))]);
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
					memset(rr1, 0, z1*n1);			
				}
				activeA.underlyingHistoryRepa.push_back(hr1);
			}
			activeA.eventsSparse = std::make_shared<ActiveEventsArray>(1);

			bool ok = true;
			
			EVAL(activeA.historyEvent);
			ECHO(ok = activeA.update(ActiveUpdateParameters()));
			TRUTH(ok);
			EVAL(activeA.historyEvent);

			EVAL(activeA.historySlicesSetEvent);
			EVAL(activeA.induceSlices);
			ECHO(ok = activeA.induce(ActiveInduceParameters()));
			TRUTH(ok);
			EVAL(activeA.historySlicesSetEvent);
			EVAL(activeA.induceSlices);
			
			for (std::size_t i = 0; i < 2; i++) 
				eventsA->mapIdEvent[i] = HistoryRepaPtrSizePair(std::move(hrsel(*hr,i)),eventsA->references);
	
			EVAL(activeA.historyEvent);
			ECHO(ok = activeA.update(ActiveUpdateParameters()));
			TRUTH(ok);
			EVAL(activeA.historyEvent);

			EVAL(activeA.historySlicesSetEvent);
			EVAL(activeA.induceSlices);
			ECHO(ok = activeA.induce(ActiveInduceParameters()));
			TRUTH(ok);
			EVAL(activeA.historySlicesSetEvent);
			EVAL(activeA.induceSlices);	
			
			for (std::size_t i = 2; i < 6; i++) 
				eventsA->mapIdEvent[i] = HistoryRepaPtrSizePair(std::move(hrsel(*hr,i)),eventsA->references);
	
			EVAL(activeA.historyEvent);
			ECHO(ok = activeA.update(ActiveUpdateParameters()));
			TRUTH(ok);
			EVAL(activeA.historyEvent);

			EVAL(activeA.historySlicesSetEvent);
			EVAL(activeA.induceSlices);
			ECHO(ok = activeA.induce(ActiveInduceParameters()));
			TRUTH(ok);
			EVAL(activeA.historySlicesSetEvent);
			EVAL(activeA.induceSlices);	
		}
		
	}
	
	if (argc >= 3 && string(argv[1]) == "induce02")
	{
		auto hrsel = [](const HistoryRepa& hr, std::size_t ev)
		{
			SizeList ll {ev};
			return eventsHistoryRepasHistoryRepaSelection_u(ll.size(), (std::size_t*)ll.data(), hr);
		};
		auto erdr = applicationRepasDecompFudSlicedRepa_u;
		
		string model = string(argv[2]);
		string dataset = string(argc >= 4 ? argv[3] : "data009");
		
		EVAL(model);
		EVAL(dataset);
		
		std::unique_ptr<System> uu;
		std::unique_ptr<SystemRepa> ur;
		std::unique_ptr<HistoryRepa> hr;
		{
			std::ifstream in(dataset+".bin", std::ios::binary);
			auto qq = persistentsRecordList(in);
			in.close();
			auto xx = recordListsHistoryRepa_4(8, *qq);
			uu = std::move(std::get<0>(xx));
			ur = std::move(std::get<1>(xx));
			hr = std::move(std::get<2>(xx));
		}
		EVAL(hr->size);
		EVAL(ur->listVarSizePair.size());
		
		{
			auto eventsA = std::make_shared<ActiveEventsRepa>(2);
			
			auto systemA = std::make_shared<ActiveSystem>();
			
			Active activeA;
			activeA.system = systemA;
			activeA.historySize = 10;
			activeA.logging = true;
			{
				StrVarPtrMap m;
				std::ifstream in(model + ".dr", std::ios::binary);
				auto ur = persistentsSystemRepa(in, m);
				activeA.decomp = std::move(erdr(*persistentsApplicationRepa(in)));
				in.close();		
				systemA->block = activeA.decomp->varMax() >> systemA->bits;
			}
			activeA.var = systemA->next(activeA.bits);
			EVAL(activeA.var);
			activeA.varSlice = systemA->next(activeA.bits);
			EVAL(activeA.varSlice);
			activeA.underlyingEventsRepa.push_back(eventsA);
			{
				SizeList vv0;
				{
					auto& mm = ur->mapVarSize();
					auto vscan = std::make_shared<Variable>("scan");
					for (int i = 0; i < 360; i++)
						vv0.push_back(mm[Variable(vscan, std::make_shared<Variable>((i % 360) + 1))]);
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
					memset(rr1, 0, z1*n1);			
				}
				activeA.underlyingHistoryRepa.push_back(hr1);
			}
			activeA.eventsSparse = std::make_shared<ActiveEventsArray>(1);

			Active activeB;
			activeB.system = systemA;
			activeB.var = systemA->next(activeB.bits);
			EVAL(activeB.var);
			activeB.varSlice = systemA->next(activeB.bits);
			EVAL(activeB.varSlice);
			activeB.historySize = 10;
			activeB.induceThreshold = 5;
			activeB.logging = true;
			activeB.decomp = std::make_shared<DecompFudSlicedRepa>();
			activeB.underlyingEventsRepa.push_back(eventsA);
			{
				SizeList vv0;
				{
					auto& mm = ur->mapVarSize();
					vv0.push_back(mm[Variable("motor")]);
					vv0.push_back(mm[Variable("location")]);
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
					memset(rr1, 0, z1*n1);			
				}
				activeB.underlyingHistoryRepa.push_back(hr1);
			}
			activeB.underlyingEventsSparse.push_back(activeA.eventsSparse);
			activeB.underlyingHistorySparse.push_back(std::make_shared<HistorySparseArray>(activeB.historySize,1));
			activeB.eventsSparse = std::make_shared<ActiveEventsArray>(1);
			
			bool ok = true;
						
			for (std::size_t i = 0; i < 2; i++) 
				eventsA->mapIdEvent[i] = HistoryRepaPtrSizePair(std::move(hrsel(*hr,i)),eventsA->references);
	
			ECHO(ok = activeA.update(ActiveUpdateParameters()));
			TRUTH(ok);

			ECHO(ok = activeB.update(ActiveUpdateParameters()));
			TRUTH(ok);

			EVAL(activeB.historySlicesSetEvent);
			EVAL(activeB.induceSlices);
			ECHO(ok = activeB.induce(ActiveInduceParameters()));
			TRUTH(ok);
			EVAL(activeB.historySlicesSetEvent);
			EVAL(activeB.induceSlices);	
			
			for (std::size_t i = 2; i < 6; i++) 
				eventsA->mapIdEvent[i] = HistoryRepaPtrSizePair(std::move(hrsel(*hr,i)),eventsA->references);
	
			ECHO(ok = activeA.update(ActiveUpdateParameters()));
			TRUTH(ok);

			ECHO(ok = activeB.update(ActiveUpdateParameters()));
			TRUTH(ok);
			
			{
				// for (auto& p : activeB.slicesPath)
				// {
					// EVAL(*p.second);
				// }
			}

			EVAL(activeB.historySlicesSetEvent);
			EVAL(activeB.induceSlices);
			ECHO(ok = activeB.induce(ActiveInduceParameters()));
			TRUTH(ok);
			EVAL(activeB.historySlicesSetEvent);
			EVAL(activeB.induceSlices);	
			
			{
				// EVAL(activeA.historySlicesSetEvent);
				// EVAL(activeB.induceSlices);
				// ActiveInduceParameters ppi;
				// ppi.znnmax = 0;
				// ppi.bmax = 2;
				// ECHO(ok = activeB.induce(ppi));
				// TRUTH(ok);
				// EVAL(activeA.historySlicesSetEvent);
				// EVAL(activeB.induceSlices);	
			}

		}
	}
		
	if (false)
	{			
		ActiveSystem systemA;
		systemA.bits = 4;
		EVAL(systemA.bits);
		EVAL(systemA.block);
		EVAL(systemA.next(4));
		EVAL(systemA.block);
		EVAL(systemA.next(6));
		EVAL(systemA.block);
		EVAL(systemA.next(4));
		EVAL(systemA.block);
		EVAL(systemA.next(4));
		EVAL(systemA.block);
		EVAL(systemA.next(3));
		EVAL(systemA.block);
		
		EVAL(std::size_t(-1));
		ECHO(systemA.block = std::size_t(-1) >> 4);
		try 
		{
			EVAL(systemA.next(4));
		} 
		catch (const std::out_of_range& e) 
		{
			EVAL(e.what());
		}		
		ECHO(systemA.block = std::size_t(-1) >> 4);
		try 
		{
			EVAL(systemA.next(6));
		} 
		catch (const std::out_of_range& e) 
		{
			EVAL(e.what());
		}		
	}

	return 0;
}
