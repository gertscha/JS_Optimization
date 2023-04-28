#pragma once

#include <string>
#include <vector>



namespace JSOptimzer {
	class Task;

	/*
	* basic description of a Problem, essentially a list of Tasks
	* a specific Optimizer should use this class to construct its
	* internal representation of the problem
	*/
	class Problem {
	public:

		class Bounds
		{
			friend Problem;
		public:

			const unsigned int longestTaskId;
			const unsigned int longestMachineId;
			const long TaskLowerBound;
			const long MachineLowerBound;
			const long sequentialUpperBound;
			const std::vector<long>& getMachinelowerBounds() const { return *m_machineBounds; }
			const Problem& getProblem() const { return (*m_problem); }
			long getLowerBound() const;

		private:
			Problem* m_problem;
			std::vector<long>* m_machineBounds;

			Bounds(unsigned int lTId, unsigned int lMId, long TlB, long MlB, long SuB,
					std::vector<long>* machineBounds, Problem* problem);
			~Bounds();
		};

		/*
		 * extract Problem description from file
		 * name is optional, takes filename by default
		 */
		Problem(const std::string& filepath, const std::string& filename, std::string problemName = "");

		/*
		* destructor
		*/
		~Problem();

		/*
		* get number of Task's in this Problem
		*/
		size_t getTaskCnt() const { return m_numTasks; }

		/*
		* get number of Machines in this Problem
		*/
		size_t getMachineCnt() const { return m_numMachines; }

		/*
		* get list of Task's this Problem consists of
		*/
		const std::vector<Task>& getTasks() const { return m_tasks; }

		/*
		* get a list that contains the number of Task::Step's each Task has
		* using the task id as index gives the number of Task::Step's
		*/
		const std::vector<size_t>& getTaskStepCounts() const { return m_taskStepCounts; }

		const Problem::Bounds& getBounds() const { return *lowerBound; }

		const std::string& getName() const { return m_name; }


		friend std::ostream& operator<<(std::ostream& os, const Problem& dt);

	private:
		size_t m_numTasks;
		size_t m_numMachines;

		std::vector<Task> m_tasks;
		std::vector<size_t> m_taskStepCounts;
		Problem::Bounds* lowerBound;
		std::string m_name;

		// used in constructor
		void parseFileAndInitTaskVec(std::ifstream& file);

	};

}
