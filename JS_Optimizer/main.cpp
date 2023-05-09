#include "config.h" // generated by CMake, contains #define SOLUTION_DIR ...
// JS_Lib headers
#include "Problem.h"
#include "Task.h"
#include "Optimizer.h"
#include "Solution.h"
#include "Utility.h"
// JS_Lib Optimizers
#include "RandomSwap.h"

#include "loguru.hpp"

#include <iostream>


/*
* Predefined Functions, Tests & Optimization runs
*/
namespace JSOptimizer {

	// global variables for filepaths
	std::string g_VSsol_path = std::string(SOLUTION_DIR);
	std::string g_problems_path = g_VSsol_path + "/JobShopProblems/";
	std::string g_solutions_path = g_VSsol_path + "/JobShopSolutions/";
	std::string g_visualizations_out_path = g_VSsol_path + "/JobShopSolutions/visualizations/";
	std::string g_python_path = g_VSsol_path + "/pythonScripts/";
	
	
	void testingOnSmallProblem(bool printResults)
	{
		LOG_F(INFO, "running testingOnSmallProblem()");
		// check loading Problem from file
		Problem p_sb(g_problems_path, "SmallTestingProblem.txt");
		
		// check loading Solution from file
    LOG_F(INFO, "constructing SmallTestingSolution.txt");
		Solution s_sb(g_solutions_path, "SmallTestingSolution.txt");
    LOG_F(INFO, "constructing SmallTestingSolution_invalid.txt");
		Solution s_sb_invalid(g_solutions_path, "SmallTestingSolution_invalid.txt");
		// check saving Solution to file
		s_sb.SaveToFile(g_solutions_path, "SmallTestingSolution_saved.txt");
		// check that save is valid
		Solution s_sb_from_save(g_solutions_path, "SmallTestingSolution_saved.txt");
		
		// ensure validation works
		if (!(s_sb.ValidateSolution(p_sb))) {
			LOG_F(ERROR, "solution does not solve problem in testingOnSmallProblem()");
		}
		LOG_F(INFO, "Verifying that 'SmallTestingSolution_invalid.txt' is invalid:");
		if (s_sb_invalid.ValidateSolution(p_sb)) {
			LOG_F(ERROR, "invalid test solution solves problem in testingOnSmallProblem()");
		}

		if (printResults)
		{
			LOG_F(INFO, "testingOnSmallProblem(), printing results to cout");
			long solTime = s_sb.getMakespan();
			std::cout << "Completion time of solution is: " << solTime;
			std::cout << ", the lower bound is:" << p_sb.getBounds().getLowerBound() << "\n";
			std::cout << "machine_bound " << p_sb.getBounds().machine_lower_bound << ", task_bound " << p_sb.getBounds().task_lower_bound << "\n";
			std::cout << p_sb;
			std::cout << s_sb;
			LOG_F(INFO, "testingOnSmallProblem(), finished printing results");
		}
		else {
			// access bounds anyway to check
			long lb = p_sb.getBounds().getLowerBound();
			long mlb = p_sb.getBounds().machine_lower_bound;
			long tlb = p_sb.getBounds().task_lower_bound;
		}

		LOG_F(INFO, "Creating visualization...");
		std::string visName = "SmallTestingSolutionVis";
		Utility::visualize(g_solutions_path, visName, g_visualizations_out_path);
		LOG_F(INFO, "Visualization saved under %s", visName.c_str());
	}

	void runRandomSwap(const std::string ProblemFileName)
	{
		LOG_F(INFO, "running runRandomSwap()");
		Problem problem(g_problems_path, ProblemFileName);

		Optimizer::TerminationCriteria tC = { 3000, 10, 0.0 };
		
		// 1531321, 89164, 6123
    RandomSwap ssO = RandomSwap(&problem, tC, 89164, "Seed_89164");

		Solution best_sol = ssO.runOptimizer(5);

		if (best_sol.ValidateSolution(problem)) {
			best_sol.SaveToFile(g_solutions_path, "ShuffleStepBestSol_saved.txt");
      LOG_F(INFO, "fitness of best solution is %i", best_sol.getMakespan());
		}
    else {
      LOG_F(ERROR, "solution found by RandomSwap is invalid");
    }

	}

	// remember that only pointers/references of Optimizer may exits to prevent object slicing 
	void test(Optimizer& op)
	{
		op.getBestSolution();
	}
	 
}


int main() {
	using namespace JSOptimizer;
	loguru::add_file("logs/latest.log", loguru::Truncate, loguru::Verbosity_INFO);
	loguru::add_file("logs/error.log", loguru::Truncate, loguru::Verbosity_ERROR);
	LOG_F(INFO, "Started Execution");


	testingOnSmallProblem(false);

  runRandomSwap("SmallTestingProblem.txt");


	LOG_F(INFO, "Finished Execution");
	return 0;
}
