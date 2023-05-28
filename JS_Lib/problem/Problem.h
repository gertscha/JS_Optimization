#ifndef PROBLEM_PROBLEM_H_
#define PROBLEM_PROBLEM_H_

#include <string>
#include <vector>


namespace JSOptimizer {
	
	class Task;

	/*
	* basic description of a Problem, essentially a list of Tasks
	* a specific Optimizer should use this class to construct its
	* internal representation of the problem
	*/
	class Problem {
	public:

		enum SpecificationType { Standard, Detailed };

		class Bounds
		{
			friend Problem;
		public:

			const unsigned int limiting_task_id;
			const unsigned int limiting_machine_id;
			const long task_lower_bound;
			const long machine_lower_bound;
			const long sequential_upper_bound;
			inline const std::vector<long>& getMachineLowerBounds() const { return machine_bounds_; }
			inline const Problem& getProblem() const { return (*problem_pointer_); }
			long getLowerBound() const;

		private:
			Problem* const problem_pointer_;
			std::vector<long> machine_bounds_;

			Bounds(unsigned int lTId, unsigned int lMId, long TlB, long MlB, long SuB,
				std::vector<long>&& machineBounds, Problem* problem);
		};

		/*
		 * extract Problem description from file
		 * name is optional, takes filename by default
		 */
		Problem(const std::string& filepath, const std::string& filename,
            SpecificationType type, std::string problemName = "");

		/*
		* destructor
		*/
		~Problem();

		/*
		* get number of Task's in this Problem
		*/
    inline unsigned int getTaskCount() const { return task_count_; }

		/*
		* get number of Machines in this Problem
		*/
    inline unsigned int getMachineCount() const { return machine_count_; }

		/*
		* the number of Steps each Machine needs to process, indexed by machine id
		*/
		inline const std::vector<unsigned int>& getStepCountForMachines() const { return machine_step_counts_; }

		/*
		* get list of Task's this Problem consists of
		*/
		inline const std::vector<Task>& getTasks() const { return tasks_; }

		inline const Problem::Bounds& getBounds() const { return *lower_bounds_pointer_; }

		inline const std::string& getName() const { return name_; }

    // is -1 if unkown
    inline long getKnownLowerBound() const { return known_lowerBound_; }

		friend std::ostream& operator<<(std::ostream& os, const Problem& dt);

	private:
		unsigned int task_count_;
		unsigned int machine_count_;

		std::vector<Task> tasks_;
		std::vector<unsigned int> machine_step_counts_;
		Problem::Bounds* lower_bounds_pointer_;
    long known_lowerBound_;
		std::string name_;

		// used in constructor
		void ParseDetailedFileAndInit(std::ifstream& file);
    void ParseStandardFileAndInit(std::ifstream& file);

	};

}

#endif // PROBLEM_PROBLEM_H_