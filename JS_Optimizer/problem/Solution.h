#ifndef PROBLEM_SOLUTION_H_
#define PROBLEM_SOLUTION_H_

#include <string>
#include <vector>

#include "Problem.h"


namespace JSOptimizer {

	/*
	* This class provides mainly two functionalities
	* 1. validateSolution, 2. saveToFile
	* a specific Optimizer should also implement a subclass that adds a constructor
	* for the internal represenation of the specific Optimizer
	*/
	class Solution
	{
	public:

		struct Step {
      unsigned int task_id;
      size_t step_index;
			unsigned int machine;
			long start_time;
      long end_time;

      Step()
        : task_id(0), step_index(0), machine(0), start_time(-1), end_time(-1)
      {}

      Step(unsigned int taskId, size_t stepIndex, unsigned int machine, long startTime, long endTime)
        :task_id(taskId), step_index(stepIndex), machine(machine), start_time(startTime), end_time(endTime)
      {}

			friend std::ostream& operator<<(std::ostream& os, const Step& ss);
		};

		/*
		* load Solution from file
		*/
		Solution(const std::string& filepath, const std::string& filename);

		// default constructor for uninitalized Solutions
		Solution()
			: initalized_(false), makespan_(-1), task_count_(0), machine_count_(0), name_("")
		{}

		/*
		* validate that the solution solves the problem correctly
		* sets completetionTime
		* returns false if Solution not valid
		*/
		virtual bool ValidateSolution(const Problem& problem) const final;

		/*
		* store this solution to a given file
		* returns true on success
		*/
		virtual bool SaveToFile(const std::string& filepath, const std::string& filename) const final;

    // false if the default constructor was used to create the solution
		bool isInitialized() { return initalized_; }

		// returns the m_completion time (calculates if not known)
		virtual long getMakespan() final;
		// returns the m_completion time (returns -1 if not known)
		virtual long getMakespan() const final { return makespan_; }

    
		friend std::ostream& operator<<(std::ostream& os, const Solution& sol);

	protected:
		bool initalized_;
		long makespan_;
		unsigned int task_count_;
		unsigned int machine_count_;
		std::string name_;
		/*
		  rows for machines, columns for Steps
		  represents the processing order
		*/
		std::vector<std::vector<Solution::Step>> solution_;
		/*
		  rows for tasks, columns for steps
		  represents the task order (same as Problem description)
      should not be used to modify the Steps
		*/
		mutable std::vector<std::vector<Solution::Step*>> problem_view_;

		// used in constructor (from file)
		void ParseFileAndInitSolution(std::ifstream& file);
		// creates problemRep from solution, needs the vector to be set to the correct size
		void FillProblemView() const;
		// used in validateSolution
		bool ValidateParametersMatch(const Problem& p) const;

	};


}

#endif // PROBLEM_SOLUTION_H_