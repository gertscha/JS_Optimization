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



  // small sanity test to check if basic things still work
  void sanityTestOnSmallProblem(bool printResults)
  {
    LOG_F(INFO, "-------------------------------------------------");
    LOG_F(INFO, "running testingOnSmallProblem()");
    // check loading Problem from file
    Problem p_sb(g_problems_path, "SmallTestingProblem.txt", SpecificationType::Detailed);
    
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

    LOG_F(INFO, "-------------------------------------------------");
  }



  // manually run a single optimizer, set termination criteria and seed manually in the body
  // expects the template type to match the signature of the base_optimizer's constructor
  template<typename T>
  void runOptimizer(const std::string& ProblemFilePath, SpecificationType type)
  {
    LOG_F(INFO, "-------------------------------------------------");
    // 1531321, 89164, 6123, 431899131, 981965720, 7703030
    // s_seed: 122064029, 318502452, 36191753, 3645762, 26047714
    unsigned int seed = 781249315;
    // limits are: iteration_limit, restart_limit, percentage_threshold, -1 disables a limit
    Optimizer::TerminationCriteria tC = { .iteration_limit = 200, .restart_limit = -1, .percentage_threshold = 0.0 };

    std::string prefix = std::string("seed_") + std::to_string(seed) + std::string("_");
    std::string problemName = Utility::getFilenameFromPathString(ProblemFilePath);

    LOG_F(INFO, "Loading problem %s", problemName.c_str());
    Problem problem(g_problems_path, ProblemFilePath, type, problemName);
    LOG_F(INFO, "Problem dimensions are %i jobs and %i machines", problem.getTaskCount(), problem.getMachineCount());

    // create the optimizer
    std::unique_ptr<Optimizer> opti = std::make_unique<T>(&problem, tC, prefix, seed);

    LOG_F(INFO, "Running a %s optimizer on %s", opti->getOptimizerName().c_str(), problemName.c_str());
    opti->Run();
    std::shared_ptr<Solution> best_sol = opti->getBestSolution();

    if (best_sol->ValidateSolution(problem)) {
      std::string solutionSaveName = opti->getOptimizerName() + std::string("_") + problemName + std::string("_sol.txt");
      best_sol->SaveToFile(g_solutions_path, solutionSaveName, false);

      if (type == SpecificationType::Standard) {
        LOG_F(INFO, "Fitness of best solution is %i, optimum is %i", best_sol->getMakespan(), problem.getKnownLowerBound());
      }
      else {
        LOG_F(INFO, "Fitness of best solution is %i", best_sol->getMakespan());
      }
    }
    else {
      LOG_F(ERROR, "Solution is invalid");
    }
    LOG_F(INFO, "-------------------------------------------------");
  }


  // this function runs the optimizer specified by the template argument once for each seed and problem
  // the problem selection occurs during the 'RunAndLog' call as the first argument
  // the seeds get selected during the StatsCollector object creation
  void evaluateOptimizers() {

    auto seeds = std::vector<unsigned int>{ 1531321, 9848646, 781249315, 3645762, 9746243, 89164, 612376, 431899131 };
    auto seeds_alt = std::vector<unsigned int>{ 2140240109, 312386259, 210327742, 122064029, 764303976, 981965720, 23933782, 21880244 };
    auto seeds_alt_alt = std::vector<unsigned int>{ 150066255, 206772536, 164491237, 109698136, 22292694, 36191753, 23933782, 117020778 };
    auto s_seeds = std::vector<unsigned int>{ 122064029, 318502452, 36191753, 3645762, 26047714 };
    auto s_seeds_alt = std::vector<unsigned int>{ 122364029, 318602452, 36196453, 3645492, 26015714 };
    auto ss_seeds = std::vector<unsigned int>{ 122064029, 318502452 };
    auto l_seeds = std::vector<unsigned int>{ 309597945, 264530771, 84911295, 26047714, 998505319, 48052834, 180929743,
                                             158729458, 2140240109, 263687153, 129894134, 313675223, 981965720, 314333760,
                                             150066255, 206772536, 164491237, 109698136, 22292694, 36191753, 23933782,
                                             21880244, 78503601, 312386259, 318502452, 210327742, 117020778, 266753747,
                                             19100590, 242396756, 125070240, 294238507, 138506392, 419205761, 94986800,
                                             234524097, 122064029, 7703030, 244925689, 764303976 };
    // negative value disables the criteria
    Optimizer::TerminationCriteria TC = { .iteration_limit = 2000, .restart_limit = -1, .percentage_threshold = -1.0 };

    // config of the run, sets the output location, the seeds and the termination criteria
    Utility::StatsCollector eval = Utility::StatsCollector(g_evaluation_log_path, s_seeds_alt, TC);

    //eval.RunAndLog<RandomSearch>("Instances", SpecificationType::Standard);
    //eval.RunAndLog<RandomSearchMachine>("Instances", SpecificationType::Standard);
    //eval.RunAndLog<RandomSwap>("Instances", SpecificationType::Standard);
    eval.RunAndLog<ShiftingBottleneck>("Instances", SpecificationType::Standard);

  }

	 
}


int main() {
	using namespace JSOptimizer;
  // setup loguru
	loguru::add_file("logs/latest.log", loguru::Truncate, loguru::Verbosity_INFO);
	loguru::add_file("logs/error.log", loguru::Truncate, loguru::Verbosity_ERROR);
  loguru::add_file("logs/complete.log", loguru::Truncate, loguru::Verbosity_MAX);

  // timed run scope
	LOG_F(INFO, "Started Execution");
  auto start = std::chrono::steady_clock::now();
  {

    //sanityTestOnSmallProblem(false);

    evaluateOptimizers();

    //runOptimizer<JSOptimizer::RandomSwap>("Instances/abz/abz5.txt", SpecificationType::Standard);
    //runOptimizer<JSOptimizer::RandomSwap>("Instances/dmu/dmu68.txt", SpecificationType::Standard);

    //runOptimizer<JSOptimizer::RandomSearch>("Instances/abz/abz5.txt", SpecificationType::Standard);
    //runOptimizer<JSOptimizer::RandomSearch>("Instances/dmu/dmu68.txt", SpecificationType::Standard);

    //runOptimizer<JSOptimizer::RandomSearchMachine>("Instances/dmu/dmu68.txt", SpecificationType::Standard);
    //runOptimizer<JSOptimizer::RandomSearchMachine>("Instances/abz/abz5.txt", SpecificationType::Standard);
    //runOptimizer<JSOptimizer::RandomSearchMachine>("SmallTestingProblem.txt", SpecificationType::Detailed);

    //runOptimizer<JSOptimizer::ShiftingBottleneck>("Instances/abz/abz5.txt", SpecificationType::Standard);
    //runOptimizer<JSOptimizer::ShiftingBottleneck>("Instances/swv/swv08.txt", SpecificationType::Standard);
    //runOptimizer<JSOptimizer::ShiftingBottleneck>("Instances/ft/ft06.txt", SpecificationType::Standard);
    //runOptimizer<JSOptimizer::ShiftingBottleneck>("Instances/dmu/dmu68.txt", SpecificationType::Standard);

  }
  auto end = std::chrono::steady_clock::now();
  auto ms_int = duration_cast<std::chrono::milliseconds>(end - start);
  int64_t ms_printable = ms_int.count();

	LOG_F(INFO, "Finished Execution after %lld ms", ms_printable);

	return 0;
}
