#include "config.h" // generated by CMake, contains #define SOLUTION_DIR ...

#include <iostream>
#include <chrono>
#include <memory>

#include "loguru.hpp"

#include "JS_Lib.h"



namespace JSOptimizer {


	// global variables for filepaths
	std::string g_VSsol_path = std::string(SOLUTION_DIR);
	std::string g_problems_path = g_VSsol_path + "/JobShopProblems/";
	std::string g_solutions_path = g_VSsol_path + "/JobShopSolutions/";
  std::string g_python_path = g_VSsol_path + "/PythonScripts/";
	std::string g_visualizations_out_path = g_VSsol_path + "/JobShopSolutions/visualizations/";
  std::string g_evaluation_log_path = g_VSsol_path + "/JobShopEvaluationLog/";
  // global ThreadManager for the visualization threads
  ThreadManager g_VisualizationManager;


  // small sanity test to check if basic things still work
  void sanityTestOnSmallProblem(bool printResults)
  {
    LOG_F(INFO, "-------------------------------------------------");
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
    s_sb.SaveToFile(g_solutions_path, sol_filename, false);
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

    LOG_F(INFO, "Creating visualization...");
    Utility::visualize(g_solutions_path, sol_filename);
    LOG_F(INFO, "-------------------------------------------------");
  }


  // manually run a single optimizer, set termination criteria and seed manually in the body
  // expects the template type to match the signature of the base_optimizer's constructor
  template<typename T>
  void runOptimizer(const std::string& ProblemFilePath, Problem::SpecificationType type)
  {
    LOG_F(INFO, "-------------------------------------------------");
    // 1531321, 89164, 6123, 431899131
    unsigned int seed = 89164;
    // limits are: iteration_limit, restart_limit, percentage_threshold, -1 disables a limit
    Optimizer::TerminationCriteria tC = { 1000, -1, 0.0 };

    std::string prefix = std::string("seed_") + std::to_string(seed) + std::string("_");
    std::string problemName = Utility::getFilenameFromPathString(ProblemFilePath);

    LOG_F(INFO, "Loading problem %s", problemName.c_str());
    Problem problem(g_problems_path, ProblemFilePath, type, problemName);

    std::unique_ptr<Optimizer> opti = std::make_unique<T>(&problem, tC, prefix, seed);

    LOG_F(INFO, "Running a %s optimizer on %s", opti->getOptimizerName().c_str(), problemName.c_str());
    opti->Run();
    std::shared_ptr<Solution> best_sol = opti->getBestSolution();

    if (best_sol->ValidateSolution(problem)) {
      std::string solutionSaveName = opti->getOptimizerName() + std::string("_") + problemName + std::string("_sol.txt");

      best_sol->SaveToFile(g_solutions_path, solutionSaveName);
      LOG_F(INFO, "Fitness of best solution is %i", best_sol->getMakespan());
    }
    else {
      LOG_F(ERROR, "Solution is invalid");
    }
    LOG_F(INFO, "-------------------------------------------------");
  }



  void evaluateOptimizers() {

    auto seeds = std::vector<unsigned int>{ 1531321, 89164, 612376, 431899131, 9848646, 781249315, 3645762, 9746243 };
    auto test_seed = std::vector<unsigned int>{ 1329633 };
    // limits are: iteration_limit, restart_limit, percentage_threshold, -1 disables a limit
    Optimizer::TerminationCriteria TC = { 500, -1, 0.01 };

    Utility::StatsCollector eval = Utility::StatsCollector(g_evaluation_log_path, seeds, TC);

    //eval.RunAndLog<RandomSearch>("Instances", Problem::Standard);
    eval.RunAndLog<RandomSwap>("Instances", Problem::Standard);
    eval.RunAndLog<ShiftingBottleneck>("Instances", Problem::Standard);

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

    //sanityTestOnSmallProblem(false);

    evaluateOptimizers();

    //runOptimizer<JSOptimizer::RandomSwap>("Instances/abz/abz5.txt", Problem::Standard);

    //runOptimizer<JSOptimizer::ShiftingBottleneck>("Instances/abz/abz5.txt", Problem::Standard);

    //ShiftingBottleneckTest("Instances/abz/abz5.txt", Problem::Standard);
    // 
    //ShiftingBottleneckTest("SmallTestingProblem.txt", Problem::Detailed);

  }
  auto end = std::chrono::steady_clock::now();
  g_VisualizationManager.~ThreadManager();

  auto ms_int = duration_cast<std::chrono::milliseconds>(end - start);
  int64_t ms_printable = ms_int.count();

	LOG_F(INFO, "Finished Execution after %lld ms", ms_printable);

	return 0;
}
