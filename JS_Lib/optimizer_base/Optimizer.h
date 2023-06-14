#ifndef OPTIMIZER_BASE_OPTIMIZER_H_
#define OPTIMIZER_BASE_OPTIMIZER_H_

#include <memory>
#include <string>

#include "Problem.h"
#include "Solution.h"


namespace JSOptimizer {

  // base class for Optimizers
	class Optimizer
	{
	public:
		
		/*
		* defining -1 for the limits ignores them
		* if the found solution is within percentageThreshold of the lower bound, terminate
		*/
		struct TerminationCriteria {
			long iteration_limit;
			long restart_limit;
			double percentage_threshold;
		};
		
		// takes ownership of TerminationCriteria
		Optimizer(const Problem* const problem, const TerminationCriteria& criteria, std::string name_prefix)
			: problem_pointer_(problem), termination_criteria_(criteria),
        restart_count_(0), prefix_(name_prefix)
		{}

		virtual ~Optimizer() {}
		
		// calls initalize once and then iterate until a termination criteria is reached
		virtual void Run()
		{
			Initialize();
			while (!CheckTermination()) {
				Iterate();
			}
		}
		
		// initializes an optimization run
		virtual void Initialize() = 0;

		// performs an optimization iteration
		virtual void Iterate() = 0;

		// returns true if termination criteria reached
		virtual bool CheckTermination() = 0;

    // get current best solution, share ownership of the Solution
    virtual std::shared_ptr<Solution> getBestSolution() = 0;

    inline const Problem& getProblem() const { return *problem_pointer_; }
    inline const TerminationCriteria& getTerminationCriteria() const { return termination_criteria_; }
    inline unsigned int getRestartCount() const { return restart_count_; }
		
	protected:
		const Problem* const problem_pointer_;
		const TerminationCriteria& termination_criteria_;
    std::string prefix_;
		unsigned int restart_count_;

	};


}

#endif  // OPTIMIZER_BASE_OPTIMIZER_H_