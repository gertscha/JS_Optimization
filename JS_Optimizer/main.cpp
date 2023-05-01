#include "config.h" // generated by CMake, contains #define SOLUTION_DIR ...
// JS_Lib headers
#include "Problem.h"
#include "Task.h"
#include "Optimizer.h"
#include "Solution.h"
#include "Utility.h"
// JS_Lib Optimizers
#include "ShuffleStep.h"

#include "loguru.hpp"

#include <iostream>


/*
* Predefined Functions, Tests & Optimization runs
*/
namespace JSOptimzer {

	// global variables for filepaths
	std::string g_VSSolPath = std::string(SOLUTION_DIR);
	std::string g_problemsPath = g_VSSolPath + "/JobShopProblems/";
	std::string g_solutionsPath = g_VSSolPath + "/JobShopSolutions/";
	std::string g_visOutPath = g_VSSolPath + "/JobShopSolutions/visualizations/";
	std::string g_pythonPath = g_VSSolPath + "/pythonScripts/";
	
	
	void testingOnSmallProblem(bool printResults)
	{
		LOG_F(INFO, "running testingOnSmallProblem()");
		// check loading Problem from file
		Problem p_sb(g_problemsPath, "SmallTestingProblem.txt");
		
		// check loading Solution from file
		Solution s_sb(g_solutionsPath, "SmallTestingSolution.txt");
		Solution s_sb_inv(g_solutionsPath, "SmallTestingSolution_invalid.txt");
		// check saving Solution to file
		s_sb.saveToFile(g_solutionsPath, "SmallTestingSolution_saved.txt");
		// check that save is valid
		Solution s_sb_fromSave(g_solutionsPath, "SmallTestingSolution_saved.txt");
		
		// ensure validation works
		if (!(s_sb.validateSolution(p_sb))) {
			LOG_F(ERROR, "solution does not solve problem in testingOnSmallProblem()");
		}
		LOG_F(INFO, "Verifying that 'SmallTestingSolution_invalid.txt' is invalid:");
		if (s_sb_inv.validateSolution(p_sb)) {
			LOG_F(ERROR, "invalid test solution solves problem in testingOnSmallProblem()");
		}

		if (printResults)
		{
			LOG_F(INFO, "testingOnSmallProblem(), printing results to cout");
			long solTime = s_sb.getCompletetionTime();
			std::cout << "Completion time of solution is: " << solTime;
			std::cout << ", the lower bound is:" << p_sb.getBounds().getLowerBound() << std::endl;
			std::cout << "machine_bound " << p_sb.getBounds().MachineLowerBound << ", task_bound " << p_sb.getBounds().TaskLowerBound << std::endl;
			std::cout << p_sb;
			std::cout << s_sb;
			LOG_F(INFO, "testingOnSmallProblem(), finished printing results");
		}
		else {
			// access bounds anyway to check
			long lb = p_sb.getBounds().getLowerBound();
			long mlb = p_sb.getBounds().MachineLowerBound;
			long tlb = p_sb.getBounds().TaskLowerBound;
		}

		LOG_F(INFO, "Creating visualization...");
		std::string visName = "SmallTestingSolutionVis";
		Utility::visualize(g_solutionsPath, visName, g_visOutPath);
		LOG_F(INFO, "Visualization saved under %s", visName.c_str());
	}

	void runShuffleStep(const std::string ProblemFileName)
	{
		LOG_F(INFO, "running runShuffleStep()");
		Problem problem(g_problemsPath, ProblemFileName);

		Optimizer::TerminationCriteria tC = { 10000, 10, 0.01 };
		
		Optimizer* ssO = new ShuffleStep(&problem, tC, 1531321, "ShuffleRun");

		delete ssO;
	}

	// remember that only pointers/references of Optimizer may exits to prevent object slicing 
	void test(Optimizer& op)
	{
		op.getBestSolution();
	}

}


int main() {
	using namespace JSOptimzer;
	loguru::add_file("logs/latest.log", loguru::Truncate, loguru::Verbosity_INFO);
	loguru::add_file("logs/error.log", loguru::Truncate, loguru::Verbosity_ERROR);
	LOG_F(INFO, "Started Execution");


	testingOnSmallProblem(true);

	runShuffleStep("SmallTestingProblem.txt");


	LOG_F(INFO, "Finished Execution");
	return 0;
}
