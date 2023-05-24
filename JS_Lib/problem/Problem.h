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
			const std::vector<long>& getMachineLowerBounds() const { return machine_bounds_; }
			const Problem& getProblem() const { return (*problem_pointer_); }
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
		size_t getTaskCount() const { return task_count_; }

		/*
		* get number of Machines in this Problem
		*/
		size_t getMachineCount() const { return machine_count_; }

		/*
		* the number of Steps each Machine needs to process, indexed by machine id
		*/
		const std::vector<size_t>& getStepCountForMachines() const { return machine_step_counts_; }

		/*
		* get list of Task's this Problem consists of
		*/
		const std::vector<Task>& getTasks() const { return tasks_; }

		const Problem::Bounds& getBounds() const { return *lower_bounds_pointer_; }

		const std::string& getName() const { return name_; }


		friend std::ostream& operator<<(std::ostream& os, const Problem& dt);

	private:
		size_t task_count_;
		size_t machine_count_;

		std::vector<Task> tasks_;
		std::vector<size_t> machine_step_counts_;
		Problem::Bounds* lower_bounds_pointer_;
    unsigned int known_lowerBound_;
		std::string name_;

		// used in constructor
		void ParseDetailedFileAndInit(std::ifstream& file);
    void ParseStandardFileAndInit(std::ifstream& file);

	};

}

#endif // PROBLEM_PROBLEM_H_