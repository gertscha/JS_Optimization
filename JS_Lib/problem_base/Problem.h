#ifndef PROBLEM_BASE_PROBLEM_H_
#define PROBLEM_BASE_PROBLEM_H_

#include <string>
#include <vector>
#include <memory>


namespace JSOptimizer {
	
	class Job;


	enum class SpecificationType { Standard, Detailed };
	/*
	* basic description of a Problem, essentially a list of Tasks
	* a specific Optimizer should use this class to construct its
	* internal representation of the problem
	*/
	class Problem {
    friend class Solution;
	public:


		class Bounds
		{
			friend Problem;
      struct Private_Tag { explicit Private_Tag() = default; };
		public:

			const unsigned int limiting_job_id;
			const unsigned int limiting_machine_id;
			const long job_lower_bound;
			const long machine_lower_bound;
			const long sequential_upper_bound;
			inline const std::vector<long>& getMachineLowerBounds() const { return machine_bounds_; }
			long getLowerBound() const;

      // this constructor is private, but needs to be accessible to std::make_unique,
      // to enforce privacy it uses a Private_Tag struct as an argument to the constructor
      // that is only accessible to Problem because it is a friend of this class
			explicit Bounds(unsigned int lJId, unsigned int lMId, long JlB, long MlB, long SuB,
				std::vector<long>&& machineBounds, Private_Tag p);
		private:
			std::vector<long> machine_bounds_;
		};

		// extract Problem description from file, requires absolute paths
		// problemName is optional, uses filename by default
		Problem(const std::string& filepath, const std::string& filename,
            SpecificationType type, std::string problemName = "");

    // create the Problem corresponding to a given Solution
    Problem(const Solution& solution);

    // disable the copying constructor
    Problem(const Problem&) = delete;

    // the move constructor is implemented
    Problem(Problem&& other) noexcept;

		
    // store this Problem to a given file, uses the SpecificationType to set
    // the format, 'filepath' must already exist (absolute path),
    // 'filename' may extend the path, 'filename' can create missing folders if
    // create_subfolders ist set to true
    // returns true on success
    virtual bool SaveToFile(const std::string& filepath, const std::string& filename,
      SpecificationType type, bool create_subfolders) const final;


		// get number of Job's in this Problem
    inline unsigned int getJobCount() const { return job_count_; }

		// get number of Machines in this Problem
    inline unsigned int getMachineCount() const { return machine_count_; }

		// the number of Tasks each Machine needs to process, indexed by machine id
		inline const std::vector<unsigned int>& getTaskCountForMachines() const { return machine_task_counts_; }


		// get list of Job's this Problem consists of
		inline const std::vector<Job>& getJobs() const { return jobs_; }

		inline const Problem::Bounds& getBounds() const { return *lower_bounds_; }

		inline const std::string& getName() const { return name_; }

    // is -1 if unknown
    inline long getKnownLowerBound() const { return known_lower_bound_; }

		friend std::ostream& operator<<(std::ostream& os, const Problem& dt);

    // Copy assignment operator.
    Problem& operator=(const Problem & other) = delete;

	private:
		unsigned int job_count_;
		unsigned int machine_count_;

		std::vector<Job> jobs_;
		std::vector<unsigned int> machine_task_counts_;
		std::unique_ptr<Problem::Bounds> lower_bounds_;
    long known_lower_bound_;
		std::string name_;

		// used in constructor
		void ParseDetailedFileAndInit(std::ifstream& file);
    void ParseStandardFileAndInit(std::ifstream& file);
    // init Problem::Bounds and init machine_task_counts_
    void CalculateAndSetBounds();

	};

}

#endif // PROBLEM_BASE_PROBLEM_H_