#include "dev.h"

#include <cmath>

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
	if (true)
	{
		auto uvars = systemsSetVar;
		auto hrsel = eventsHistoryRepasHistoryRepaSelection_u;
		auto hrshuffle = historyRepasShuffle_u;
		auto frvars = fudRepasSetVar;
		auto frder = fudRepasDerived;
		auto frund = fudRepasUnderlying;
		auto layerer = parametersSystemsLayererMaxRollByMExcludedSelfHighestIORepa_up;
		
		size_t size = argc >= 2 ? atoi(argv[1]) : 1000;
				
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
			size_t tint = 4;
			auto t = layerer(wmax, lmax, xmax, omax, bmax, mmax, umax, pmax, tint, vv, *hr, *hrs, 0, *ur);
			fr = std::move(std::get<0>(t));
			mm = std::move(std::get<1>(t));
		}
		catch (const std::out_of_range& e)
		{
			std::cout << "out of range exception: " << e.what() << std::endl;
			fr.reset();
			mm.reset();
		}
		if (!mm || !mm->size())
		{
			std::cout << "no fud" << std::endl;
		}
		else
		{
			// EVAL(mm->size());			
			// EVAL(mm->back());
			// EVAL(*mm);
			auto& a = mm->back().first;
			auto& kk = mm->back().second;
			auto m = kk.size();
			auto z = (double)hr->size;
			EVAL(m);
			EVAL(a);			
			EVAL(z);	
			EVAL(100.0*(exp(a/z/(m-1))-1.0));
			EVAL(fudRepasSize(*fr));
			EVAL(frvars(*fr)->size());
			EVAL(frder(*fr)->size());
			EVAL(frund(*fr)->size());
			EVAL(sorted(*frund(*fr)));
		}
	}
	
	return 0;
}
