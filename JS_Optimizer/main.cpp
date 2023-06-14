#include "config.h" // generated by CMake, contains #define SOLUTION_DIR ...

#include <iostream>
#include <chrono>
#include <memory>

#include "loguru.hpp"

#include "JS_Lib.h"


/*
* Predefined Functions, Tests & Optimization runs
*/
namespace JSOptimizer {

	// global variables for filepaths
	std::string g_VSsol_path = std::string(SOLUTION_DIR);
	std::string g_problems_path = g_VSsol_path + "/JobShopProblems/";
	std::string g_solutions_path = g_VSsol_path + "/JobShopSolutions/";
  std::string g_python_path = g_VSsol_path + "/PythonScripts/";
	std::string g_visualizations_out_path = g_VSsol_path + "/JobShopSolutions/visualizations/";
	// global ThreadManager for the visualization threads
  ThreadManager g_VisualizationManager;
	

	void testingOnSmallProblem(bool printResults)
	{
		LOG_F(INFO, "running testingOnSmallProblem()");
		// check loading Problem from file
		Problem p_sb(g_problems_path, "SmallTestingProblem.txt", Problem::Detailed);
		
		// check loading Solution from file
    LOG_F(INFO, "constructing SmallTestingSolution.txt");
		Solution s_sb(g_solutions_path, "SmallTestingSolution.txt");
    LOG_F(INFO, "constructing SmallTestingSolution_invalid.txt");
		Solution s_sb_invalid(g_solutions_path, "SmallTestingSolution_invalid.txt");
		// check saving Solution to file
    std::string sol_filename = "SmallTestingSolution_saved.txt";
		s_sb.SaveToFile(g_solutions_path, sol_filename);
		// check that save is valid
		Solution s_sb_from_save(g_solutions_path, sol_filename);
		
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
		Utility::visualize(g_solutions_path, sol_filename);
	}

  
	void runRandomSwap(const std::string& ProblemFileName)
	{
		LOG_F(INFO, "running runRandomSwap()");
		Problem problem(g_problems_path, ProblemFileName, Problem::Detailed, "SmallTestingProblem");

		Optimizer::TerminationCriteria tC = { 500, 10, 0.0 };
		
		// 1531321, 89164, 6123
    RandomSwap ssO = RandomSwap(&problem, tC, 89164, "Seed_89164");

    ssO.Run();
    std::shared_ptr<Solution> best_sol = ssO.getBestSolution();

		if (best_sol->ValidateSolution(problem)) {
			best_sol->SaveToFile(g_solutions_path, "RandomSwapBestSol_saved.txt");
      LOG_F(INFO, "fitness of best solution is %i", best_sol->getMakespan());
		}
    else {
      LOG_F(ERROR, "solution found by RandomSwap is invalid");
    }

	}

  void runRandomSearch(const std::string& ProblemFileName) {
    LOG_F(INFO, "running runRandomSearch()");
    
    Problem problem(g_problems_path, ProblemFileName, Problem::Detailed);

    Optimizer::TerminationCriteria tC = { 500, 0, 0.01 };

    RandomSearch rsO = RandomSearch(&problem, tC, 1531321, "Seed_1531321");

    rsO.Run();
    std::shared_ptr<Solution> best_sol = rsO.getBestSolution();

    if (best_sol->ValidateSolution(problem)) {
      best_sol->SaveToFile(g_solutions_path, "RandomSearchBestSol_saved.txt");
      LOG_F(INFO, "fitness of best solution is %i", best_sol->getMakespan());
    }
    else {
      LOG_F(ERROR, "solution found by RandomSearch is invalid");
    }

  }


	// remember that only pointers/references of Optimizer may exits to prevent object slicing 
	void test(Optimizer& op)
	{
		op.getBestSolution();
	}

  void abz5CompareRandomSwapAndRandomSearhc()
  {
    srand(time(0));
    unsigned int random_seed = static_cast<unsigned int>(rand());

    Problem abz5(g_problems_path, "Instances/abz/abz5.txt", Problem::Standard, "abz5");

    std::cout << "The known lower bound is:" << abz5.getKnownLowerBound() << "\n";
    std::cout << "The trivial lower bound is:" << abz5.getBounds().getLowerBound() << "\n";

    Optimizer::TerminationCriteria tC = { 30000, -1, -1 };

    std::string prefix = "Seed" + std::to_string(random_seed) + "-";

    RandomSwap rswO = RandomSwap(&abz5, tC, random_seed, prefix + "RandomSwap-");
    rswO.Run();
    std::shared_ptr<Solution> RSW_sol = rswO.getBestSolution();
    if (RSW_sol->ValidateSolution(abz5)) {
      RSW_sol->SaveToFile(g_solutions_path, "abz5_RSW_sol.txt");
      LOG_F(INFO, "fitness of best RandomSwap solution is %i", RSW_sol->getMakespan());
    }
    else
      LOG_F(INFO, "RandomSwap solution is invalid");

    RandomSearch rseO = RandomSearch(&abz5, tC, random_seed, prefix + "RandomSearch-");
    rseO.Run();
    std::shared_ptr<Solution> RSE_sol = rseO.getBestSolution();
    if (RSE_sol->ValidateSolution(abz5)) {
      RSE_sol->SaveToFile(g_solutions_path, "abz5_RSE_sol.txt");
      LOG_F(INFO, "fitness of best RandomSearch solution is %i", RSE_sol->getMakespan());
    }
    else
      LOG_F(INFO, "RandomSearch solution is invalid");

  }

  void ShiftingBottleneckTest(const std::string& problem_name, Problem::SpecificationType type) {

    Problem problem(g_problems_path, problem_name, type, problem_name);
    
    Optimizer::TerminationCriteria tc = { 500, -1, -1 };

    ShiftingBottleneck SBO = ShiftingBottleneck(&problem, tc, 1531321, "ShiftingBottle");

    SBO.Run();

    std::shared_ptr<Solution> sol = SBO.getBestSolution();

    if (sol->ValidateSolution(problem)) {
      sol->SaveToFile(g_solutions_path, "ShiftingBottleneckSol.txt");
      LOG_F(INFO, "fitness of best ShiftingBottleneck solution is %i", sol->getMakespan());
    }
    else
      LOG_F(WARNING, "ShiftingBottleneck solution is invalid");

  }


	 
}


int main() {
	using namespace JSOptimizer;
	loguru::add_file("logs/latest.log", loguru::Truncate, loguru::Verbosity_INFO);
	loguru::add_file("logs/error.log", loguru::Truncate, loguru::Verbosity_ERROR);

  // run scope
	LOG_F(INFO, "Started Execution");
  g_VisualizationManager = ThreadManager();
  auto start = std::chrono::steady_clock::now();
  {

	  //testingOnSmallProblem(false);

    //runRandomSwap("SmallTestingProblem.txt");

    //runRandomSearch("SmallTestingProblem.txt");

    //abz5CompareRandomSwapAndRandomSearhc();

    ShiftingBottleneckTest("Instances/ft/ft20.txt", Problem::Standard);
    //ShiftingBottleneckTest("SmallTestingProblem.txt", Problem::Detailed);

  }
  auto end = std::chrono::steady_clock::now();
  g_VisualizationManager.~ThreadManager();

  auto ms_int = duration_cast<std::chrono::milliseconds>(end - start);
  int64_t ms_printable = ms_int.count();

	LOG_F(INFO, "Finished Execution after %lld ms", ms_printable);

	return 0;
}
