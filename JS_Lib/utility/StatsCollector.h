#ifndef UTILITY_STATSCOLLECTOR_H_
#define UTILITY_STATSCOLLECTOR_H_

#include <string>
#include <vector>
#include <chrono>
#include <fstream>
#include <filesystem>

#include "loguru.hpp"

#include "Optimizer.h"
#include "Solution.h"
#include "Problem.h"
#include "Utility.h"
#include "FileCollector.h"


namespace JSOptimizer::Utility
{

  /*
  * Run optimizers for many problems and seeds and log the results in a csv
  */
  class StatsCollector
  {
  public:

    StatsCollector(
      std::string log_file_path,
      const std::vector<unsigned int>& seeds,
      Optimizer::TerminationCriteria& TC
    )
      : seeds_(seeds), term_crit_(&TC)
    {
      auto now = std::chrono::system_clock::now();
      auto rounded_time = std::chrono::floor<std::chrono::seconds>(now);
      std::string time_string = std::format("{:%Y-%m-%d_%H-%M-%S}", rounded_time);
      std::string filename = "eval_log_" + time_string + "_UTC.csv";
      if (!std::filesystem::exists(log_file_path))
      {
        LOG_F(INFO, "Created folder: %s", log_file_path.c_str());
        std::filesystem::create_directories(log_file_path);
      }
      log_file_name_ = log_file_path + filename;
      log_file_ = std::ofstream(log_file_name_);
      if (!log_file_.good())
      {
        LOG_F(ERROR, "failed to create File '%s'", log_file_name_.c_str());
        ABORT_F("File IO Error");
      }
      log_file_ << "Seeds: ";
      for (unsigned int seed : seeds_)
        log_file_ << seed << ", ";
      log_file_ << "\n";
      log_file_ << "TC: " << TC.iteration_limit << ", " << TC.restart_limit << ", " << TC.percentage_threshold << "\n";
      log_file_ << "Format: problem, difference to optimum, optimizer, seed, makespan\n";
      log_file_ << "Results:\n";
      log_file_.close();
    }

    // run all problems in a folder using a specific optimizer, and all seeds in the seeds vector
    // expects the template types constructor to match the signature of the base_optimizer's constructor
    // saves best solutions in mirrored folder structure
    // logs the results in the a log file per instance of StatsCollector
    template<typename T>
    void RunAndLog(const std::string& folder, SpecificationType type)
    {
      log_file_ = std::ofstream(log_file_name_, std::ios::app);
      try
      {
        runProblemsInFolder<T>(folder, seeds_, *term_crit_, type);
      }
      catch (std::runtime_error e)
      {
        log_file_.close();
        LOG_F(ERROR, "RunAndLog: encountered a runtime error!");
        throw std::runtime_error(e.what());
      }
      log_file_.close();
      LOG_F(INFO, "Logged results to '%s'", log_file_name_.c_str());
      LOG_F(INFO, "Saved best solutions in Solutions folder with mirrored file structure to input Problems");
    }

  private:

    struct RunStat
    {
      std::string problem_name;
      std::string optimizer_name;
      unsigned int seed;
      long best_makespan;
      long diff_to_optima;

      friend std::ostream& operator<<(std::ostream& os, const RunStat& stat)
      {
        os << stat.problem_name << ", " << stat.diff_to_optima << ", ";
        os << stat.optimizer_name << ", " << stat.seed << ", " << stat.best_makespan;
        return os;
      }
    };

    std::string log_file_name_;
    const std::vector<unsigned int>& seeds_;
    Optimizer::TerminationCriteria* term_crit_;
    std::ofstream log_file_;

    // variables for filepaths
    const std::string VSsol_path_ = std::string(SOLUTION_DIR); // SOLUTION_DIR is defined by CMake
    const std::string problems_path_ = VSsol_path_ + "/JobShopProblems/";
    const std::string solutions_path_ = VSsol_path_ + "/JobShopSolutions/";

    template<typename T>
    void runProblemsInFolder(
      const std::string& folder,
      const std::vector<unsigned int>& seeds,
      const Optimizer::TerminationCriteria& TC,
      SpecificationType type
    )
    {
      Utility::FileCollector problem_files(problems_path_, folder);
      for (std::string& file : problem_files)
      {
        std::string problem_name = Utility::getFilenameFromPathString(file);
        RunStat stat = { .problem_name = problem_name, .optimizer_name = "undef",
                         .seed = 0, .best_makespan = -1, .diff_to_optima = -1 };
        Problem problem(problems_path_, file, type, problem_name);
        long known_lower_bound_info = problem.getKnownLowerBound();
        for (unsigned int seed : seeds)
        {
          if (stat.best_makespan != -1 && known_lower_bound_info != -1)
          {
            if (stat.best_makespan == known_lower_bound_info)
              break;
          }
          std::string prefix = std::string("seed_") + std::to_string(seed) + std::string("_");
          std::unique_ptr<Optimizer> opti = std::make_unique<T>(&problem, TC, prefix, seed);
          opti->Run();
          std::shared_ptr<Solution> best_sol = opti->getBestSolution();
#if _DEBUG
          if (best_sol->ValidateSolution(problem))
          {
#endif
            if (stat.best_makespan == -1)
            {
              stat.optimizer_name = opti->getOptimizerName();
              stat.best_makespan = best_sol->getMakespan() + 1;
            }
            if (best_sol->getMakespan() < stat.best_makespan)
            {
              stat.best_makespan = best_sol->getMakespan();
              stat.seed = seed;
              stat.diff_to_optima = stat.best_makespan - problem.getKnownLowerBound();
              std::string solutionSaveName = opti->getOptimizerName() + std::string("_") + problem_name + std::string("_sol.txt");
              std::string folder_path = Utility::getFilepathFromString(file);
              best_sol->SaveToFile(solutions_path_, folder_path + solutionSaveName, true);
            }
#if _DEBUG
          }
          else
          {
            LOG_F(ERROR, "Solution by %s for %s is invalid", opti->getOptimizerName().c_str(), (prefix + problem_name).c_str());
          }
#endif
        }
        log_file_ << stat << "\n";
      }
    }


  };

}

#endif // UTILITY_STATSCOLLECTOR_H_