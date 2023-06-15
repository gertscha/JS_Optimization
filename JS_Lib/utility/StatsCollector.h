#ifndef UTILITY_STATSCOLLECTOR_H_
#define UTILITY_STATSCOLLECTOR_H_

#include <string>
#include <vector>
#include <chrono>
#include <memory>
#include <fstream>
#include <sstream>

#include "loguru.hpp"

#include "Optimizer.h"
#include "Solution.h"
#include "Problem.h"
#include "Utility.h"
#include "FileCollector.h"


namespace JSOptimizer {

  extern std::string g_problems_path;
  extern std::string g_solutions_path;

  namespace Utility {


    class StatsCollector {
    public:

      StatsCollector(std::string log_file_path, const std::vector<unsigned int>& seeds,
                     Optimizer::TerminationCriteria& TC)
        : seeds_(seeds), term_crit_(&TC) {
        auto now = std::chrono::system_clock::now();
        auto rounded_time = std::chrono::floor<std::chrono::seconds>(now);
        std::string time_string = std::format("{:%Y-%m-%d_%H-%M-%S}", rounded_time);
        std::string filename = "eval_log_" + std::to_string(TC.iteration_limit) + "it_" + time_string + ".txt";
        log_file_ = std::ofstream(log_file_path + filename);
      }

      ~StatsCollector() {
        log_file_.close();
      }

      // run all problems in a folder using a specific optimizer, and all seeds in the seeds vector
      // expects the template types constructor to match the signature of the base_optimizer's constructor
      // saves best solutions in mirrored folder structure
      // logs the results in the a log file per instance of StatsCollector
      template<typename T>
      void RunAndLog(const std::string& folder, Problem::SpecificationType type) {
        try {
          runProblemsInFolder<T>(folder, seeds_, *term_crit_, type);
        }
        catch (std::runtime_error e) {
          log_file_.close();
          throw std::runtime_error(e.what());
        }
      }

    private:

      struct RunStat {
        std::string optimizer_name;
        std::string problem_name;
        std::string prefix;
        long best_makespan;

        friend std::ostream& operator<<(std::ostream& os, const RunStat& stat)
        {
          os << stat.problem_name << ", " << stat.best_makespan << ", ";
          os << stat.optimizer_name << "_" << stat.prefix;
          return os;
        }
      };

      const std::vector<unsigned int>& seeds_;
      Optimizer::TerminationCriteria* term_crit_;
      std::ofstream log_file_;

      template<typename T>
      void runProblemsInFolder(const std::string& folder, const std::vector<unsigned int>& seeds,
        const Optimizer::TerminationCriteria& TC, Problem::SpecificationType type)
      {
        Utility::FileCollector problem_files(g_problems_path, folder);
        for (std::string& file : problem_files) {
          std::string problemName = Utility::getFilenameFromPathString(file);
          RunStat stat = { "undef", problemName, "undef", -1 };
          Problem problem(g_problems_path, file, type, problemName);
          for (unsigned int seed : seeds)
          {
            std::string prefix = std::string("seed_") + std::to_string(seed) + std::string("_");
            std::unique_ptr<Optimizer> opti = std::make_unique<T>(&problem, TC, prefix, seed);
            opti->Run();
            std::shared_ptr<Solution> best_sol = opti->getBestSolution();
            if (best_sol->ValidateSolution(problem)) {
              if (stat.best_makespan == -1) {
                stat.optimizer_name = opti->getOptimizerName();
                stat.best_makespan = best_sol->getMakespan() + 1;
              }
              if (best_sol->getMakespan() < stat.best_makespan) {
                stat.best_makespan = best_sol->getMakespan();
                stat.prefix = prefix.substr(0, prefix.size() - 1);
                std::string solutionSaveName = opti->getOptimizerName() + std::string("_") + problemName + std::string("_sol.txt");
                std::string folder_path = Utility::getFilepathFromString(file);
                best_sol->SaveToFile(g_solutions_path, folder_path + solutionSaveName, true);
              }
            }
            else {
              LOG_F(ERROR, "Solution by %s for %s is invalid", opti->getOptimizerName().c_str(), (prefix + problemName).c_str());
            }
          }
          log_file_ << stat << ", " << problem.getKnownLowerBound() << "\n";
        }
      }


    };

  }
}

#endif // UTILITY_STATSCOLLECTOR_H_