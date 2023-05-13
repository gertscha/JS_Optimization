#ifndef OPTIMIZERS_RANDOMSWAP_H_
#define OPTIMIZERS_RANDOMSWAP_H_

#include <string>
#include <vector>
#include <random>

#include "GlobalOrderRep.h"
#include "Solution.h"
#include "Problem.h"
#include "Heap.h"
#include "Wrapper.h"


namespace JSOptimizer {

	/*
	* This optimizer uses a list (length total number of Steps) as search space
	* entries in the list are task ids, which are then constructed into the exec order
	* per machine
	*/
	class RandomSwap : public GlobalOrderRep
	{
    friend struct Utility::Wrapper<GlobalOrderRep::InternalSolution>;

	public:

    RandomSwap(Problem* problem, Optimizer::TerminationCriteria& terminationCriteria, unsigned int seed, std::string namePrefix);

		~RandomSwap();

    // not copyable
    RandomSwap(const RandomSwap&) = delete;
    RandomSwap& operator=(const RandomSwap&) = delete;
    // not moveable
    RandomSwap(RandomSwap&&) = delete;
    RandomSwap& operator=(RandomSwap&&) = delete;


		// do multiple runs according to parameters, returns best solution
		const Solution& runOptimizer(unsigned int num_InternalSols);
		
		// initializes an optimization run
		virtual void Initialize();

		// performs an optimization iteration
		virtual void Iterate();

		// returns true if termination criteria reached
		virtual bool CheckTermination();
		
		virtual const Solution& getBestSolution();

		// returns sorted list (low to high) of the stored solutions
		std::vector<Solution> getAllStoredSolutions();

	private:

		std::string prefix_;
		unsigned int seed_;
		double temperature_; // simulated annealing temp
		unsigned int total_iterations_;
		unsigned int internal_sol_max_cnt_;

		std::mt19937 generator_;
		std::uniform_real_distribution<> zero_one_dist_;
		std::uniform_int_distribution<> zero_stepCnt_dist_;

		// representation of solution states
		std::vector<unsigned int> cur_sol_state_;
		std::vector<unsigned int> prev_sol_state_;

		// heap and vector access to the internal solutions
		Utility::Heap<Utility::Wrapper<GlobalOrderRep::InternalSolution>> solutions_heap_;

		// current minumum fitness internal solution
		// needs to be in m_solutionsHeap as well, to not leak memory
    GlobalOrderRep::InternalSolution* best_internal_solution_;

		// may have unitialized solution and problemRep members
		Solution best_solution_;

		/*
			requires:
			- sol was created with new (delete will be called on the pointer)
			- m_bestInternal is initialized (null-pointer dereference) and in the 
				heap (otherwise may cause a memory leak), => heap has at least one element
			does:
			- updates m_bestInternal if sol is better
			- adds the sol to the heap, if the total number of sols is below the limit(strict) it
			  gets added, if the limit is reached (inclusive) the worst solution will be discarded
			- never removes more than one element
		*/
		void updateInternalSols(GlobalOrderRep::InternalSolution* sol, unsigned int limit);

	};


  // specialization of Wrapper template to enable sorting of RandomSwapSolution's
  template<>
  inline bool Utility::Wrapper<GlobalOrderRep::InternalSolution>::operator<(const Utility::Wrapper<GlobalOrderRep::InternalSolution>& rhs)
  {
    return (this->pointer->getMakespan() < rhs.pointer->getMakespan());
  }


}

#endif  // OPTIMIZERS_RANDOMSWAP_H_
