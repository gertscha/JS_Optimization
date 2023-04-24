#pragma once

#include <string>
#include <vector>


namespace JSOptimzer {
	class Problem;

	/*
	* This class provides mainly two functionalities
	* 1. validateSolution, 2. saveToFile
	* a specific Optimizer should also implement a subclass that adds a constructor
	* for the internal represenation of the specific Optimizer
	*/
	class Solution
	{
	public:

		struct SolStep {
			int taskId;
			int index;
			int machine;
			long startTime;
			long endTime;

			friend std::ostream& operator<<(std::ostream& os, const SolStep& ss);
		};

		/*
		* load Solution from file
		*/
		Solution(const std::string& filepath, const std::string& filename);

		/*
		* validate that the solution solves the problem correctly
		* sets completetionTime
		* returns false if Solution not valid
		*/
		virtual bool validateSolution(const Problem& problem) const final;

		/*
		* store this solution to a given file
		* returns true on success
		*/
		virtual bool saveToFile(const std::string& filepath, const std::string& filename) const final;


		// returns the m_completion time (calculates if not known)
		virtual long getCompletetionTime() final;
		// returns the m_completion time (returns -1 if not known)
		virtual long getCompletetionTime() const final { return m_completionTime; }

		/*
		* Utility
		*/

		friend std::ostream& operator<<(std::ostream& os, const Solution& dt);

	protected:
		long m_completionTime = -1;
		unsigned int m_taskCnt;
		unsigned int m_machineCnt;
		std::string m_name;
		/*
		* rows for machines, columns for Steps
		* represents the processing order
		*/
		std::vector<std::vector<SolStep>> solution;
		/*
		* rows for tasks, columns for steps
		* represents the task order (same as Problem description)
		*/
		std::vector<std::vector<SolStep*>> problemRep;

		// used in constructor (from file)
		void parseFileAndInitSolution(std::ifstream& file);
		// creates problemRep from solution, needs the vector to be set to the correct size
		void fillProblemRep();
		// used in validateSolution
		bool validateParametersMatch(const Problem& p) const;

	};

}
