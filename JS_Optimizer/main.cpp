#include <chrono>
#include <memory>
#include <string>
#include <iostream>

#include "loguru.hpp"

#include "JS_Lib.h"



namespace JSOptimizer
{

  // global variables for filepaths
  std::string g_VSsol_path = std::string(SOLUTION_DIR); // SOLUTION_DIR is defined by CMake
  std::string g_problems_path = g_VSsol_path + "/JobShopProblems/";
  std::string g_solutions_path = g_VSsol_path + "/JobShopSolutions/";
  std::string g_evaluation_log_path = g_VSsol_path + "/JobShopEvaluationLog/";


  // small sanity test to check if basic things still work
  // setting the bool prints more information
  void sanityTestOnSmallProblem(bool printResults)
  {
    LOG_F(INFO, "--------- running testingOnSmallProblem ----------");
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
    if (!(s_sb.ValidateSolution(p_sb)))
    {
      LOG_F(ERROR, "solution does not solve problem in testingOnSmallProblem()");
    }
    LOG_F(INFO, "Verifying that 'SmallTestingSolution_invalid.txt' is invalid:");
    if (s_sb_invalid.ValidateSolution(p_sb))
    {
      LOG_F(ERROR, "invalid test solution solves problem in testingOnSmallProblem()");
    }
    else
    {
      LOG_F(INFO, "Success.");
    }

    if (printResults)
    {
      LOG_F(INFO, "testingOnSmallProblem(), printing results to cout");
      long solTime = s_sb.getMakespan();
      std::cout << "Completion time of solution is: " << solTime;
      std::cout << ", the lower bound is:" << p_sb.getBounds().getLowerBound() << "\n";
      std::cout << "machine_bound " << p_sb.getBounds().machine_lower_bound << ", job_bound " << p_sb.getBounds().job_lower_bound << "\n";
      std::cout << p_sb;
      std::cout << s_sb;
      LOG_F(INFO, "testingOnSmallProblem(), finished printing results");
    }

  }


  // perform a test of all functionality
  void completeFunctionalityTestRun()
  {
    LOG_F(INFO, "------ running completeFunctionalityTestRun ------");
    unsigned int seed = 3176987;
    std::string prefix = "test_";
    std::shared_ptr<Solution> curr(nullptr);
    Optimizer::TerminationCriteria tC = { .iteration_limit = 1000, .restart_limit = 500, .percentage_threshold = 0.01 };
    Solution smltestSolLoaded(g_solutions_path, "SmallTestingSolution.txt");
    Solution abz5SolLoaded(g_solutions_path, "Testing_abz5_sol.txt");
    LOG_F(INFO, "Loading of a Solution from a file succeeded");
    Problem sproblem(g_problems_path, "Instances/abz/abz5.txt", SpecificationType::Standard);
    Problem dproblem(g_problems_path, "SmallTestingProblem.txt", SpecificationType::Detailed);
    Problem savedproblem(abz5SolLoaded);
    LOG_F(INFO, "Loading of Problems from file and Solution succeeded");
    dproblem.SaveToFile(g_problems_path, "SmallTestingProblem_saved_Detailed.txt", SpecificationType::Detailed, false);
    sproblem.SaveToFile(g_problems_path, "abz5_saved_Standard.txt", SpecificationType::Standard, false);
    LOG_F(INFO, "Saving of Problems to file succeeded");
    if (abz5SolLoaded.ValidateSolution(sproblem))
      LOG_F(INFO, "The loaded Solution solves the loaded Problem");
    else
      LOG_F(ERROR, "The loaded Solution does NOT solve the loaded Problem");
    // testing RandomSearch
    std::unique_ptr<Optimizer> RSeopti = std::make_unique<RandomSearch>(&sproblem, tC, prefix, seed);
    RSeopti->Run();
    curr = RSeopti->getBestSolution();
    if (curr->ValidateSolution(sproblem))
      LOG_F(INFO, "RandomSearch's Solution solves the Problem");
    else
      LOG_F(ERROR, "RandomSearch's Solution does NOT solve the Problem");
    curr->SaveToFile(g_solutions_path, "Testing_output_sol.txt", false);
    // testing RandomSearchMachine
    std::unique_ptr<Optimizer> RSMopti = std::make_unique<RandomSearchMachine>(&sproblem, tC, prefix, seed);
    RSMopti->Run();
    curr = RSMopti->getBestSolution();
    if (curr->ValidateSolution(sproblem))
      LOG_F(INFO, "RandomSearchMachine's Solution solves the Problem");
    else
      LOG_F(ERROR, "RandomSearchMachine's Solution does NOT solve the Problem");
    // testing RandomSwap
    std::unique_ptr<Optimizer> RSwopti = std::make_unique<RandomSwap>(&sproblem, tC, prefix, seed);
    RSwopti->Run();
    curr = RSwopti->getBestSolution();
    if (curr->ValidateSolution(sproblem))
      LOG_F(INFO, "RandomSwap's Solution solves the Problem");
    else
      LOG_F(ERROR, "RandomSwap's Solution does NOT solve the Problem");
    // testing ShiftingBottleneck
    std::unique_ptr<Optimizer> SBopti = std::make_unique<ShiftingBottleneck>(&sproblem, tC, prefix, seed);
    SBopti->Run();
    curr = SBopti->getBestSolution();
    if (curr->ValidateSolution(sproblem))
      LOG_F(INFO, "ShiftingBottleneck's Solution solves the Problem");
    else
      LOG_F(ERROR, "ShiftingBottleneck's Solution does NOT solve the Problem");
    LOG_F(INFO, "Running optimizers and saving Solutions succeeded");
  }


  // manually run a single optimizer, set termination criteria and seed manually in the body
  // expects the template type to match the signature of the base_optimizer's constructor
  template<typename T>
  void runOptimizer(const std::string& ProblemFilePath, SpecificationType type)
  {
    LOG_F(INFO, "-------------- running runOptimizer --------------");
    // 1531321, 89164, 6123, 431899131, 7703030, 318502452, 3645762, 26047714
    unsigned int seed = 781249315;
    // limits are: iteration_limit, restart_limit, percentage_threshold, -1 disables a limit
    Optimizer::TerminationCriteria tC = { .iteration_limit = 500, .restart_limit = -1, .percentage_threshold = 0.0 };

    std::string prefix = std::string("seed_") + std::to_string(seed) + std::string("_");
    std::string problemName = Utility::getFilenameFromPathString(ProblemFilePath);

    LOG_F(INFO, "Loading problem %s", problemName.c_str());
    Problem problem(g_problems_path, ProblemFilePath, type, problemName);
    LOG_F(INFO, "Problem dimensions are %i jobs and %i machines", problem.getJobCount(), problem.getMachineCount());

    // create the optimizer
    std::unique_ptr<Optimizer> opti = std::make_unique<T>(&problem, tC, prefix, seed);

    LOG_F(INFO, "Running a %s optimizer on %s", opti->getOptimizerName().c_str(), problemName.c_str());
    opti->Run();
    std::shared_ptr<Solution> best_sol = opti->getBestSolution();

    // validate the produced solution
    if (best_sol->ValidateSolution(problem))
    {
      std::string solutionSaveName = opti->getOptimizerName() + std::string("_") + problemName + std::string("_sol.txt");
      best_sol->SaveToFile(g_solutions_path, solutionSaveName, false);
      if (type == SpecificationType::Standard)
      {
        LOG_F(INFO, "Fitness of best solution is %i, optimum is %i", best_sol->getMakespan(), problem.getKnownLowerBound());
      }
      else
      {
        LOG_F(INFO, "Fitness of best solution is %i", best_sol->getMakespan());
      }
    }
    else
    {
      LOG_F(ERROR, "Solution is invalid");
    }
  }


  // this function runs the optimizer specified by the template argument once for each seed and problem pair
  // the problem selection occurs during the 'RunAndLog' call as the first argument
  // the seeds get selected during the StatsCollector object creation
  // a log file will be created at the specified path, the problem path will be
  // mirrored into 'JS_Optimization/JobShopSolutions/' and the best solutions will be kept
  // successive runs of the executable with the same problem folder but different seeds
  // will overwrite the solutions even if they are worse
  void evaluateOptimizers() {
    LOG_F(INFO, "------------ start evaluateOptimizers ------------");
    // sample seed vectors
    auto ss_seeds = std::vector<unsigned int>{ 122064029, 318502452 };
    auto s_seeds = std::vector<unsigned int>{ 122064029, 318502452, 36191753, 3645762, 26047714 };
    auto m_seeds = std::vector<unsigned int>{ 1531321, 9848646, 781249315, 3645762, 9746243, 89164, 612376, 431899131 };
    auto m_seeds_alt = std::vector<unsigned int>{ 2140240109, 312386259, 210327742, 122064029, 764303976, 981965720, 23933782, 21880244 };

    // negative value disables the criteria
    Optimizer::TerminationCriteria TC = { .iteration_limit = 4000, .restart_limit = -1, .percentage_threshold = -1.0 };

    // config of the run, set the log file location, the seeds and the termination criteria
    // (base input location and output location are non-configurable setup in StatsCollector)
    Utility::StatsCollector eval = Utility::StatsCollector(g_evaluation_log_path, m_seeds, TC);

    // select the optimizer (template argument) and an input folder
    // all problems in the path are run (including subfolders)
    // the input folder is inside 'JS_Optimization/JobShopProblems/'
    eval.RunAndLog<ShiftingBottleneck>("TestSuite", SpecificationType::Standard);

    //eval.RunAndLog<RandomSwap>("Instances", SpecificationType::Standard);
    //eval.RunAndLog<ShiftingBottleneck>("Instances", SpecificationType::Standard);
    //eval.RunAndLog<RandomSearch>("Instances", SpecificationType::Standard);
    //eval.RunAndLog<RandomSearchMachine>("Instances", SpecificationType::Standard);

  }


}



int main()
{
  using namespace JSOptimizer;
  // setup loguru
  loguru::add_file("logs/latest.log", loguru::Truncate, loguru::Verbosity_INFO);
  loguru::add_file("logs/error.log", loguru::Truncate, loguru::Verbosity_ERROR);
  loguru::add_file("logs/complete.log", loguru::Truncate, loguru::Verbosity_MAX);

  // timed run scope
  LOG_F(INFO, "Started Execution");
  auto start = std::chrono::steady_clock::now();
  {
    /*
      Basic Testing
    */
    //sanityTestOnSmallProblem(false);
    //completeFunctionalityTestRun();

    /*
      Examples for loading Problems and Solutions from Files
      and for loading a problem from a solution (see 'gen_example')
    */
    //Problem example1_problem(g_problems_path, "SmallTestingProblem.txt", SpecificationType::Detailed);
    //Solution example1_solution(g_solutions_path, "SmallTestingSolution.txt");
    //Problem example2_problem(g_problems_path, "abz5.txt", SpecificationType::Standard);
    //Solution example2_solution(g_solutions_path, "Instances/abz/RandomSwap_abz5_sol.txt");
    //Problem gen_example(example2_solution);


    /*
      Run a test suite to run many optimizers with many seeds and log to a file
      configured in the 'evaluateOptimizers' function
    */
    evaluateOptimizers();


    /*
      Run a single optimizer on a single problem
      seed and termination must be configured in the 'runOptimizer' function itself
    */
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
  LOG_F(INFO, "--------------------------------------------------");
  LOG_F(INFO, "Finished Execution after %lld ms", ms_printable);
  LOG_F(INFO, "--------------------------------------------------");


  return 0;
}
