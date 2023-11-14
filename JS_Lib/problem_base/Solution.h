#ifndef PROBLEM_BASE_SOLUTION_H_
#define PROBLEM_BASE_SOLUTION_H_

#include <string>
#include <vector>

#include "Problem.h"


namespace JSOptimizer {

	/*
	* This class provides mainly three functionalities
	* 1. validateSolution, 2. saveToFile, 3. loading Solutions from Files
	* a specific Optimizer should also implement a subclass that adds a constructor
	* for the internal represenation of the specific Optimizer
	*/
	class Solution
	{
	public:

		struct SolTask {
      unsigned int job_id;
      unsigned int task_index;
			unsigned int machine;
			long start_time;
      long end_time;

      SolTask()
        : job_id(0), task_index(0), machine(0), start_time(-1), end_time(-1)
      {}

      SolTask(unsigned int jobId, unsigned int taskIndex, unsigned int machine, long startTime, long endTime)
        : job_id(jobId), task_index(taskIndex), machine(machine), start_time(startTime), end_time(endTime)
      {}

			friend std::ostream& operator<<(std::ostream& os, const SolTask& ss);
		};

		// load Solution from file, filepath is an absolute path,
    // filename requires the file extension
		Solution(const std::string& filepath, const std::string& filename);

		// default constructor, marked as uninitalized
		Solution()
			: initialized_(false), makespan_(-1), job_count_(0), machine_count_(0), name_("")
    {
      solution_ = std::vector<std::vector<Solution::SolTask>>();
      problem_view_ = std::vector<std::vector<Solution::SolTask*>>();
    }

    Solution(const Solution& other) = default;
    Solution(Solution&& other) = default;

    virtual ~Solution() {}

		// validate that the solution solves the problem correctly
		// sets completetionTime
		// returns false if Solution not valid
		virtual bool ValidateSolution(const Problem& problem) const final;

		// store this solution to a given file, 'filepath' must already exist
    // 'filename' can extend a path and must define the filename
    // if the bool flag is set missing folders in 'filename' are created
		// returns true on success
		virtual bool SaveToFile(const std::string& filepath, const std::string& filename,
                            bool create_subfolders) const final;

    // false if the default constructor was used to create the solution
		inline bool isInitialized() const { return initialized_; }

		// returns the m_completion time (calculates if not known)
		virtual long getMakespan() final;
		// returns the m_completion time (returns -1 if not known)
		virtual long getMakespan() const final { return makespan_; }
    // get Solution name
    inline std::string getName() const { return name_; }
    // get number of Job's in this Problem
    inline unsigned int getJobCount() const { return job_count_; }
    // get number of Machines in this Problem
    inline unsigned int getMachineCount() const { return machine_count_; }
    // schedule matrix: rows for machines, columns for Steps
    // represents the processing order on each machine
    inline const std::vector<std::vector<Solution::SolTask>> getSchedule() const { return solution_; }

    inline bool operator< (const Solution& rhs) { return (this->makespan_ < rhs.makespan_); }
    inline bool operator> (const Solution& rhs) { return (this->makespan_ > rhs.makespan_); }
		friend std::ostream& operator<<(std::ostream& os, const Solution& sol);


	protected:
		bool initialized_;
		long makespan_;
		unsigned int job_count_;
		unsigned int machine_count_;
		std::string name_;
		
		// rows for machines, columns for Tasks
		// represents the processing order
		std::vector<std::vector<Solution::SolTask>> solution_;

		// rows for jobs, columns for tasks
		// represents the job order (same as Problem description)
    // should not be used to modify the tasks
		mutable std::vector<std::vector<Solution::SolTask*>> problem_view_;

		// used in constructor (from file)
		void ParseFileAndInitSolution(std::ifstream& file);
		// creates problemRep from solution, needs the vector to be set to the correct size
		void FillProblemView() const;
		// used in validateSolution
		bool ValidateParametersMatch(const Problem& problem) const;
    // given solution_ that has Solution::SolTask's that have the tid and index, machine filled
    // and start/endTime set to -1, fill in the start/endTime and set makespan_
    // may fail if Solution malformed, returns false in that case, true if successful
    bool CalculateTimings(const Problem& problem);

	};


}

#endif // PROBLEM_BASE_SOLUTION_H_