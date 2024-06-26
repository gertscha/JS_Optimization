#include "Solution.h"

#include <filesystem>
#include <fstream>
#include <sstream>

#include "loguru.hpp"

#include "Job.h"
#include "Parsing.h"
#include "Utility.h"


namespace JSOptimizer
{

  /*////////////////////
     Member Functions
  ////////////////////*/

  void Solution::ParseFileAndInitSolution(std::ifstream& file)
  {
    std::string line; // current line of the file
    std::istringstream in_ss;
    long first = 0, second = 0;
    // allow comments
    unsigned int commentCount = 0; // enables accurate line information for error messages
    while (std::getline(file, line))
    {
      if (line[0] != '#')
        break;
      else
        commentCount++;
    }
    // first line is name
    name_ = line;
    // second line are problem parameters
    std::getline(file, line);
    in_ss = std::istringstream(line);
    in_ss >> first >> second;
    if (in_ss.good())
    {
      std::getline(in_ss, line);
      std::string error_msg = "on line " + std::to_string(commentCount + 2)
        + " trailing '" + line + "' is invalid";
      throw std::invalid_argument(error_msg);
    }
    if (first <= 0 || second <= 0)
    {
      std::string error_msg = "parameters on line " + std::to_string(commentCount + 2)
        + " must be greater zero";
      throw std::invalid_argument(error_msg);
    }
    // assign to members
    job_count_ = first;
    machine_count_ = second;

    // start init of solution_
    solution_ = std::vector<std::vector<Solution::SolTask>>(machine_count_);
    // track job sizes for problem_view_ 
    auto job_length = std::vector<size_t>(job_count_, 0);

    // read the remaining lines
    unsigned int machine_index = 0;
    unsigned int tuple_count = 0;
    unsigned int expected = 0;
    long jid = 0, tindex = 0, machine = 0, duration = 0, start = 0, end = 0;
    // iterate through lines
    while (std::getline(file, line))
    {
      if (machine_index >= machine_count_)
      {
        if (!line.empty())
        {
          std::string error_msg = "line " + std::to_string(commentCount + 3 + machine_index)
            + " is unexpected, contains '" + line + "'";
          throw std::invalid_argument(error_msg);
        }
        else
          break;
      }
      in_ss = std::istringstream(line);
      in_ss >> expected;
      if (in_ss.fail())
      {
        std::string error_msg = "expected to find values on line "
          + std::to_string(commentCount + 3 + machine_index);
        throw std::invalid_argument(error_msg);
      }
      in_ss.ignore(1, ',');
      // init inner vec
      solution_[machine_index] = std::vector<Solution::SolTask>(expected);
      // iterate through tuples
      while (in_ss >> jid) // read first value
      {
        if (in_ss.fail())
          break;
        if (tuple_count >= expected)
        {
          std::string error_msg = "on line " + std::to_string(commentCount + 3 + machine_index)
            + "there are more tuples than expected";
          throw std::invalid_argument(error_msg);
        }
        // read the other five values
        in_ss >> tindex >> machine >> duration >> start >> end;
        // check all valid
        if (in_ss.fail())
        {
          std::string error_msg = "on line " + std::to_string(commentCount + 3 + machine_index)
            + " tuple " + std::to_string(tuple_count + 1) + " is bad";
          throw std::invalid_argument(error_msg);
        }
        if (jid < 0 || tindex < 0 || machine < 0 || duration < 0 || start < 0 || end < 0)
        {
          std::string error_msg = "only positive numbers allowed in tuple "
            + std::to_string(tuple_count + 1) + " on line "
            + std::to_string(commentCount + 3 + machine_index);
          throw std::invalid_argument(error_msg);
        }
        if (static_cast<unsigned int>(jid) >= job_count_)
        {
          std::string error_msg = "invalid invalid job id on line " + std::to_string(commentCount + 3 + machine_index)
            + " tuple " + std::to_string(tuple_count + 1);
          throw std::invalid_argument(error_msg);
        }
        if (machine != static_cast<long>(machine_index))
        {
          std::string error_msg = "invalid machine on line " + std::to_string(commentCount + 3 + machine_index)
            + " tuple " + std::to_string(tuple_count + 1);
          throw std::invalid_argument(error_msg);
        }

        solution_[machine_index][tuple_count] = Solution::SolTask{ (unsigned int)jid, (size_t)tindex, (unsigned int)machine, start, end };

        ++job_length[jid];

        in_ss.ignore(1, ',');
        ++tuple_count;
      }
      if (tuple_count < expected)
      {
        std::string error_msg = "on line " + std::to_string(commentCount + 3 + machine_index)
          + " there are fewer tuples than expected";
        throw std::invalid_argument(error_msg);
      }

      tuple_count = 0;
      ++machine_index;
    }
    if (machine_index < machine_count_)
      throw std::invalid_argument("there are fewer lines than expected");

    // init problem_view with nullptr's
    problem_view_ = std::vector<std::vector<Solution::SolTask*>>(job_count_);
    for (unsigned int i = 0; i < job_count_; ++i)
    {
      problem_view_[i] = std::vector<Solution::SolTask*>(job_length[i], nullptr);
    }

  }


  void Solution::FillProblemView() const
  {
    unsigned int jid = 0;
    size_t tindex = 0;
    for (unsigned int i = 0; i < machine_count_; ++i)
    {
      for (unsigned int j = 0; j < solution_[i].size(); ++j)
      {
        const Solution::SolTask* s_task = &solution_[i][j];
        jid = s_task->job_id;
        tindex = s_task->task_index;
        if (jid >= job_count_ || tindex >= problem_view_[jid].size())
          ABORT_F("Tried to add invalid SolTask (id %i, ind %i) in fillProblemRep() for '%s'", jid, (unsigned int)tindex, name_.c_str());
        if (problem_view_[jid][tindex] != nullptr)
          LOG_F(ERROR, "There seems to be a duplicate SolTask (id %i, index %i) in %s", jid, (unsigned int)tindex, name_.c_str());
        problem_view_[jid][tindex] = const_cast<Solution::SolTask*>(s_task);
      }
    }
    for (unsigned int i = 0; i < job_count_; ++i)
    {
      for (unsigned int j = 0; j < problem_view_[i].size(); ++j)
      {
        if (problem_view_[i][j] == nullptr)
          LOG_F(ERROR, "There seems to a missing SolTask (id %i, index %i) in %s", i, j, name_.c_str());
      }
    }
  }


  Solution::Solution(const std::string& filepath, const std::string& filename)
    : initialized_(true), makespan_(-1)
  {
    // get data from input file
    std::ifstream file(filepath + filename);

    if (!file.good())
    {
      LOG_F(ERROR, "File %s is invalid", (filepath + filename).c_str());
      ABORT_F("File IO Error");
    }

    if (file.is_open())
    {
      try
      {
        ParseFileAndInitSolution(file);
      }
      catch (std::invalid_argument e)
      {
        std::string what = e.what();
        LOG_F(ERROR, "parsing failed: %s,", what.c_str());
        LOG_F(ERROR, "file is %s at %s", filename.c_str(), filepath.c_str());
        ABORT_F("Solution File Parsing Error");
      }

      file.close();
    }
    else
    {
      LOG_F(ERROR, "Could not open Solution file %s", (filepath + filename).c_str());
      ABORT_F("File IO Error");
    }
  }


  // SolTask file format: jobid, tind, taskm, taskdur, stime, etime
  bool Solution::SaveToFile(
    const std::string& filepath,
    const std::string& filename,
    bool create_subfolders
  ) const
  {
    if (!initialized_)
    {
      LOG_F(ERROR, "cannot save Solution that is uninitalized");
      return false;
    }
    // check the root is valid  
    if (!std::filesystem::exists(filepath))
    {
      LOG_F(ERROR, "SaveToFile: root filepath '%s' does not exist!", filepath.c_str());
      return false;
    }
    std::string folder_structure = Utility::getFilepathFromString(filename);
    // create folders if flag is set
    if (create_subfolders)
    {
      std::filesystem::create_directories(filepath + folder_structure);
    }
    else
    {
      if (!std::filesystem::exists(filepath + folder_structure))
      {
        LOG_F(ERROR, "SaveToFile: path '%s' does not exist!\n"
          "Set create_subfolders bool to allow it's creation.", filename.c_str());
        return false;
      }
    }
    // create file
    std::ofstream file(filepath + filename);

    if (!file.good())
    {
      LOG_F(ERROR, "Failed to create File, cannot save Solution");
      return false;
    }
    if (file.is_open())
    {
      // first line is name
      file << name_ << "\n";
      // second line, other members
      file << job_count_ << " " << machine_count_ << "\n";
      // output solution matrix
      for (unsigned int i = 0; i < machine_count_; ++i)
      {
        size_t len = solution_[i].size();
        file << len << ", ";
        for (unsigned int j = 0; j < len; ++j)
        {
          file << solution_[i][j] << ", ";
        }
        file << "\n";
      }

      file.close();
    }
    else
    {
      LOG_F(ERROR, "failed to open the file, cannot save");
      return false;
    }
    DLOG_F(5, "successfully saved solution %s", name_.c_str());
    return true;
  }


  // Helper function
  // checks that a given SolTask matches the one it represents in the Problem
  bool validateStepsMatch(const Solution::SolTask& ss, const Job::Task& ps, unsigned int i, unsigned int j)
  {
    if (ss.task_index != ps.index || ss.job_id != ps.job_id
      || (ss.end_time - ss.start_time) != static_cast<long>(ps.duration)
      || ss.machine != ps.machine
    )
      return false;
    if (ss.job_id != i || ss.task_index != j)
      return false;

    return true;
  }


  // Helper function
  // checks that a SolTask cur does not overlap with prev and that curs times are valid
  bool validateStepTiming(const Solution::SolTask& prev, const Solution::SolTask& cur)
  {
    if (cur.end_time - cur.start_time < 0 || prev.end_time > cur.start_time)
      return false;

    return true;
  }


  // Helper member function
  // check parameters match and actually represent the internal descriptions
  bool Solution::ValidateParametersMatch(const Problem& p) const
  {
    if (p.getMachineCount() != machine_count_ || p.getJobCount() != job_count_)
      return false;
    unsigned int mid = 0;
    for (size_t len : p.getTaskCountForMachines())
    {
      if (len != solution_[mid].size())
        return false;
      mid++;
    }

    return true;
  }


  bool Solution::ValidateSolution(const Problem& p) const
  {
    if (!initialized_)
    {
      LOG_F(ERROR, "cannot validate Solution that is uninitialized");
      return false;
    }
    // if the problemView is not filled, fill it
    if (problem_view_[0][0] == nullptr)
      FillProblemView();

    // check parameters match
    if (!ValidateParametersMatch(p))
    {
      DLOG_F(INFO, "Solution and Problem parameters (Job Count or Machine Count) do not match");
      return false;
    }
    // check no processing overlap on any machine
    const std::vector<std::vector<Solution::SolTask>>& sol = solution_; // alias with shorter name
    size_t taskCnt = 0;

    for (unsigned int i = 0; i < machine_count_; ++i)
    {
      taskCnt = sol[i].size();
      const Solution::SolTask& fst = sol[i][0];
      if (fst.start_time < 0)
      {
        DLOG_F(INFO, "Solution Step (id %i, index 0) has invalid timing", i);
        return false;
      }
      if (fst.machine != i)
      {
        DLOG_F(INFO, "Solution Step (id %i, index 0) is on the wrong machine", i);
        return false;
      }
      for (unsigned int j = 1; j < taskCnt; ++j)
      {
        if (!validateStepTiming(sol[i][j - 1], sol[i][j]))
        {
          DLOG_F(INFO, "Solution Step (id %i, index %i) has invalid timing", i, j);
          return false;
        }
        if (sol[i][j].machine != i)
        {
          DLOG_F(INFO, "Solution Step (id %i, index %i) is on the wrong machine", i, j);
          return false;
        }
      }
    }
    // check that Tasks match and that no processing overlap for a task
    const std::vector<Job>& pJobs = p.getJobs();

    for (unsigned int i = 0; i < job_count_; ++i)
    {
      const std::vector<Job::Task>& tlist = pJobs[i].getTasks();
      taskCnt = problem_view_[i].size();
      if (tlist.size() != taskCnt)
      {
        DLOG_F(INFO, "Solution and Problem Task count for Job (id %i) do not match", i);
        return false;
      }
      if (!validateStepsMatch(*problem_view_[i][0], tlist[0], i, 0))
      {
        DLOG_F(INFO, "Solution Step and Problem task do not match for: id %i, index 0", i);
        return false;
      }
      const Solution::SolTask& fst = *problem_view_[i][0];
      if (fst.start_time < 0 || fst.end_time - fst.start_time != static_cast<long>(tlist[0].duration))
      {
        DLOG_F(INFO, "Solution Step (id %i, index 0) has invalid timing", i);
        return false;
      }
      for (unsigned int j = 1; j < taskCnt; ++j)
      {
        if (!validateStepsMatch(*problem_view_[i][j], tlist[j], i, j))
        {
          DLOG_F(INFO, "Solution Step and Problem task do not match for: id %i, index %i", i, j);
          return false;
        }
        if (!validateStepTiming(*problem_view_[i][j - 1], *problem_view_[i][j]))
        {
          DLOG_F(INFO, "Solution Step (id %i, index %i) has invalid timing", i, j);
          return false;
        }
      }
    }
    return true;
  }


  long Solution::getMakespan()
  {
    if (!initialized_)
    {
      LOG_F(ERROR, "getCompletetionTime invoked on uninitialized Solution");
      return -1;
    }
    if (makespan_ != -1)
      return makespan_;

    long res = 0;
    size_t mCnt = solution_.size();
    for (unsigned int i = 0; i < mCnt; ++i)
    {
      long t = solution_[i].back().end_time;
      if (t > res)
        res = t;
    }
    makespan_ = res;
    return res;
  }


  bool Solution::CalculateTimings(const Problem& problem)
  {
    // given solution_ that has Solution::SolTask's that have the tid and index, machine filled out
    // and have startTimes's and endTime's set to -1, fill in timings and set makespan_

    const auto& problemJobs = problem.getJobs();
    // (index, endtime) pairs to check if a SolTask is the next and what the bound is (index refers to the next index)
    auto job_progress_info = std::vector<std::pair<unsigned int, long>>(job_count_, std::make_pair(0, 0));
    // tracks next index that has no endtime for each machine
    auto machine_progress_info = std::vector<size_t>(machine_count_, 0);
    // track progress, avoid endless loop for malformed solution
    bool overall_progress = false; // tracks if overall progress is made
    auto row_done = std::vector<bool>(machine_count_, false); // quickly skip rows that are done
    unsigned int row_done_count = 0; // termination criteria
    // iterate and cascade times
    while (row_done_count != machine_count_)
    {
      // cascade endtimes, for each machine
      overall_progress = false;
      for (unsigned int i = 0; i < machine_count_; ++i)
      {
        if (!row_done[i])
        {
          // check if row is done
          if (machine_progress_info[i] >= solution_[i].size())
          {
            row_done_count++;
            row_done[i] = true;
            continue;
          }
          // get current task for the machine
          Solution::SolTask& s_task = solution_[i][machine_progress_info[i]];
          // bind the (index, endTime) pair to references for easy access
          auto& [pInd, pEndT] = job_progress_info[s_task.job_id];
          // if this task is next (i.e. predecessor is done)
          if (s_task.task_index == pInd)
          {
            const Job::Task& p_task = problemJobs[s_task.job_id].getTasks()[s_task.task_index];
            // if it has no predecessor on the machine, only its task predecessor is relevant
            if (machine_progress_info[i] == 0)
            {
              s_task.start_time = pEndT;
              s_task.end_time = pEndT + p_task.duration;
            }
            // has a predecessor on machine and (maybe) on job (pEndT has the endTime or 0 if first task of a job)
            else
            {
              long predEndTime = std::max(pEndT, solution_[i][machine_progress_info[i] - 1].end_time);
              s_task.start_time = predEndTime;
              s_task.end_time = predEndTime + p_task.duration;
            }
            machine_progress_info[i]++;
            pEndT = s_task.end_time;
            pInd++;
            overall_progress = true;
          }
        }
      }
      if (!overall_progress)
        break;
    }
    if (row_done_count != machine_count_)
    {
      return false;
    }
    // set completion time
    makespan_ = -1;
    for (unsigned int i = 0; i < machine_count_; ++i)
    {
      long endT = solution_[i].back().end_time;
      if (endT > makespan_)
        makespan_ = endT;
    }
    return true;
  }



  /*//////////////////////
     Function Overloads
  //////////////////////*/

  std::ostream& operator<<(std::ostream& os, const Solution::SolTask& ss)
  {
    long dur = ss.end_time - ss.start_time;
    os << ss.job_id << " " << ss.task_index << " " << ss.machine << " " << dur << " " << ss.start_time << " " << ss.end_time;
    return os;
  }


  std::ostream& operator<<(std::ostream& os, const Solution& s)
  {
    os << "Solution '" << s.name_ << "'" << "\n";
    os << s.job_count_ << " Jobs on " << s.machine_count_ << " machines, completion time is " << s.makespan_ << "\n";
    os << "tuple format: (jobId, taskIndex, machine, duration, startTime, endTime)" << "\n";
    for (unsigned int i = 0; i < s.machine_count_; ++i)
    {
      os << "Machine " << i << " [";
      size_t size = s.solution_[i].size();
      for (int j = 0; j < size; ++j)
      {
        os << "(" << s.solution_[i][j] << "),";
      }
      os << "\b]" << "\n";
    }
    return os;
  }


}
