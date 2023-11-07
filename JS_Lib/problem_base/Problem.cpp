#include "Problem.h"

#include <stdexcept>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <tuple>

#include "loguru.hpp"

#include "Task.h"
#include "Parsing.h"
#include "Solution.h"
#include "Utility.h"


namespace JSOptimizer {
	
	long Problem::Bounds::getLowerBound() const
	{
		return (job_lower_bound > machine_lower_bound) ? job_lower_bound : machine_lower_bound;
	}

	/*
	* takes ownership of the machineBounds vector
	*/
	Problem::Bounds::Bounds(unsigned int lTId, unsigned int lMId, long TlB, long MlB,
                          long SuB, std::vector<long>&& machineBounds, Private_Tag tag)
	: limiting_job_id(lTId),
    limiting_machine_id(lMId),
    job_lower_bound(TlB),
    machine_lower_bound(MlB),
    sequential_upper_bound(SuB),
    machine_bounds_(std::move(machineBounds))
	{}


	void Problem::ParseDetailedFileAndInit(std::ifstream& file) {
    std::string line; // current line of the file
    std::istringstream in_ss;
    long first = 0, second = 0;
    // allow comments
    unsigned int commentCount = 0; // enables accurate line information for error messages
    while (std::getline(file, line)) {
      if (line[0] != '#')
        break;
      else
        commentCount++;
    }
    // first line are problem parameters
    in_ss = std::istringstream(line);
    in_ss >> first >> second;
    if (in_ss.good()) {
      std::getline(in_ss, line);
      std::string error_msg = "on line " + std::to_string(commentCount + 1)
                              + " trailing '" + line + "' is invalid";
      throw std::invalid_argument(error_msg);
    }
    if (first <= 0 || second <= 0) {
      std::string error_msg = "paramters on line " + std::to_string(commentCount + 1)
                              + " must be greater zero";
      throw std::invalid_argument(error_msg);
    }
    // assign to members
    job_count_ = first;
    machine_count_ = second;

    // start init of solution_
    jobs_ = std::vector<Job>();
    jobs_.reserve(job_count_);
    // track if all machines have at least one task
    auto machine_has_steps = std::vector<bool>(machine_count_, false);
    // read the remaining lines, each line is a job
    unsigned int job_index = 0;
    unsigned int tuple_count = 0;
    unsigned int expected = 0;
    long machine = 0, duration = 0;
    // iterate through lines
    while (std::getline(file, line)) {
      if (job_index >= job_count_) {
        if (!line.empty()) {
          std::string error_msg = "line " + std::to_string(commentCount + 2 + job_index)
                                   + " is unexpected, contains '" + line + "'";
          throw std::invalid_argument(error_msg);
        }
        else
          break;
      }
      in_ss = std::istringstream(line);
      in_ss >> expected;
      if (in_ss.fail()) {
        std::string error_msg = "expected to find values on line "
                                + std::to_string(commentCount + 2 + job_index);
        throw std::invalid_argument(error_msg);
      }
      in_ss.ignore(1, ',');
      // prepare Job
      jobs_.push_back(Job(job_index, expected));
      // iterate through tuples and add them to the Job
      while (in_ss >> machine) // read first value
      {
        if (in_ss.fail())
          break;
        if (tuple_count >= expected) {
          std::string error_msg = "on line " + std::to_string(commentCount + 2 + job_index)
                                  + "there are more pairs than expected";
          throw std::invalid_argument(error_msg);
        }
        // read the duration
        in_ss >> duration;
        // check all valid
        if (in_ss.fail()) {
          std::string error_msg = "on line " + std::to_string(commentCount + 2 + job_index)
                                  + " pair " + std::to_string(tuple_count + 1) + " is bad";
          throw std::invalid_argument(error_msg);
        }
        if (machine < 0 || duration < 0) {
          std::string error_msg = "only postive numbers allowed in pair "
                                  + std::to_string(tuple_count + 1) + " on line "
                                  + std::to_string(commentCount + 2 + job_index);
          throw std::invalid_argument(error_msg);
        }
        if (machine >= static_cast<long>(machine_count_)) {
          std::string error_msg = "invalid machine on line " + std::to_string(commentCount + 2 + job_index)
                                  + " pair " + std::to_string(tuple_count + 1);
          throw std::invalid_argument(error_msg);
        }

        jobs_.back().AppendTask(machine, duration);
        
        if (!machine_has_steps[machine])
          machine_has_steps[machine] = true;

        in_ss.ignore(1, ',');
        ++tuple_count;
      }
      if (tuple_count < expected) {
        std::string error_msg = "on line " + std::to_string(commentCount + 2 + job_index)
                                + " there are fewer pairs than expected";
        throw std::invalid_argument(error_msg);
      }
      tuple_count = 0;
      ++job_index;
    }
    if (job_index < job_count_)
      throw std::invalid_argument("there are fewer lines than expected");
    
    unsigned int tasks_on_all_machines = 0;
    for (bool b : machine_has_steps) {
      if (b == true)
        ++tasks_on_all_machines;
      else
        break;
    }
    if (tasks_on_all_machines != machine_count_) {
      LOG_F(WARNING, "machine with id %i in problem %s has no steps", tasks_on_all_machines, name_.c_str());
    }

	}

  void Problem::ParseStandardFileAndInit(std::ifstream& file) {
    std::string line;
    std::istringstream iss;

    std::getline(file, line);
    iss = std::istringstream(line);
    iss >> job_count_ >> machine_count_ >> known_lowerBound_;

    jobs_ = std::vector<Job>();
    jobs_.reserve(job_count_);
    unsigned int job_index = 0;

    auto pairs = std::vector<std::tuple<long, long>>();

    while (std::getline(file, line)) {
      iss = std::istringstream(line);

      unsigned int t_count = 0;
      try {
        t_count = Utility::parseTuples(iss, pairs);
      }
      catch (std::runtime_error) {
        throw std::invalid_argument("Reading Error on line " + std::to_string(job_index + 2));
      }

      jobs_.push_back(Job(job_index, t_count));

      for (auto& tuple : pairs) {
        // append the step
        jobs_.back().AppendTask(std::get<0>(tuple), std::get<1>(tuple));
      }

      ++job_index;
      pairs.clear();
    }

  }


	Problem::Problem(const std::string& filepath, const std::string& filename,
                   SpecificationType type, std::string problemName)
    : known_lowerBound_(-1)
	{
		if (problemName.empty())
      name_ = filename;
		else
      name_ = problemName;
		// get data from input file
		std::ifstream file(filepath + filename);

    if (!file.good()) {
      LOG_F(ERROR, "File %s is invalid", (filepath + filename).c_str());
      ABORT_F("File IO Error");
    }

		if (file.is_open()) {
      try {
        if (type == SpecificationType::Detailed)
          ParseDetailedFileAndInit(file);
        if (type == SpecificationType::Standard)
          ParseStandardFileAndInit(file);
      }
      catch (std::invalid_argument e) {
        std::string what = e.what();
        LOG_F(ERROR, "parsing failed: %s,", what.c_str());
        LOG_F(ERROR, "file is %s at %s", filename.c_str(), filepath.c_str());
        ABORT_F("Problem File Parsing Error");
      }
			file.close();
		}
    else {
      LOG_F(ERROR, "Could not open Problem file %s", (filepath + filename).c_str());
			ABORT_F("File IO Error");
    }
    // init lower_bounds_ (the Problem::Bounds)
    CalculateAndSetBounds();

		LOG_F(INFO, "successfully created Problem '%s'", name_.c_str());
	}


  void Problem::CalculateAndSetBounds()
  {
    // machine bounds 
    auto machineBounds = std::vector<long>(machine_count_, 0);
    // other bounds
    long jobDurationlB = 0;
    int lBjobId = 0;
    long machineDuationlB = 0;
    int lBmachineId = 0;
    long seqUpperBound = 0;
    // step counts from machine perspective
    machine_task_counts_ = std::vector<unsigned int>(machine_count_, 0);
    // step through all tasks to determine the values
    for (Job& t : jobs_) {
      // task bound calculation
      long JobMinDuration = t.getMinDuration();
      seqUpperBound += JobMinDuration;
      if (JobMinDuration > jobDurationlB) {
        jobDurationlB = JobMinDuration;
        lBjobId = t.getId();
      }
      // machine bound calc and counting steps per machine
      for (const Job::Task& s : t.getTasks()) {
        machineBounds[s.machine] += s.duration;
        machine_task_counts_[s.machine] += 1;
      }
    }
    // find biggest machine bound
    for (unsigned int i = 0; i < machine_count_; ++i) {
      if (machineBounds[i] > machineDuationlB) {
        machineDuationlB = machineBounds[i];
        lBmachineId = i;
      }
    }
    Problem::Bounds::Private_Tag tag = Problem::Bounds::Private_Tag();
    lower_bounds_ = std::make_unique<Problem::Bounds>(lBjobId, lBmachineId, jobDurationlB,
      machineDuationlB, seqUpperBound, std::move(machineBounds), tag);
  }


  Problem::Problem(const Solution& sol)
    : job_count_(sol.getJobCount()), machine_count_(sol.getMachineCount()),
      known_lowerBound_(-1)
  {
    if (!sol.isInitialized()) {
      ABORT_F("Problem Creation Error: Cannot create Problem from uninitialzed Solution");
    }
    name_ = std::string("Problem_from_") + sol.getName();

    auto job_counter = std::vector<unsigned int>(job_count_, 0);
    for (const auto& machine : sol.getSchedule()) {
      for (const auto& step : machine) {
        job_counter[step.job_id]++;
      }
    }
    jobs_ = std::vector<Job>();
    jobs_.reserve(job_count_);
    for (unsigned int i = 0; i < job_count_; ++i) {
      jobs_.push_back(Job(i, job_counter[i]));
    }
    for (const auto& machine : sol.getSchedule()) {
      for (const Solution::SolStep& solstep : machine) {
        unsigned int duration = solstep.end_time - solstep.start_time;
        Job::Task new_step(solstep.job_id, solstep.task_index, duration, solstep.machine);
        jobs_[solstep.job_id].SetTask(solstep.task_index, new_step);
      }
    }
    CalculateAndSetBounds();
    if (!sol.ValidateSolution(*this)) {
      LOG_F(ERROR, "Creation '%s' produced invalid result!", name_.c_str());
    }
  }

  
  Problem::Problem(Problem&& other) noexcept
    : job_count_(other.job_count_),
      machine_count_(other.machine_count_),
      jobs_(std::move(other.jobs_)),
      machine_task_counts_(std::move(other.machine_task_counts_)),
      lower_bounds_(std::move(other.lower_bounds_)),
      known_lowerBound_(other.known_lowerBound_),
      name_(std::move(other.name_))
  {}


  // SolStep file format: tid, tind, tm, td, st, et
  bool Problem::SaveToFile(const std::string& filepath, const std::string& filename,
    bool create_subfolders) const
  {
    // check the root is valid  
    if (!std::filesystem::exists(filepath)) {
        LOG_F(ERROR, "SaveToFile: root filepath '%s' does not exist!", filepath.c_str());
        return false;
    }
    std::string folder_structure = Utility::getFilepathFromString(filename);
    // create folders if flag is set
    if (create_subfolders) {
      std::filesystem::create_directories(filepath + folder_structure);
    }
    else {
      if (!std::filesystem::exists(filepath + folder_structure)) {
        LOG_F(ERROR, "SaveToFile: path '%s' does not exist! Set create_subfolders to allow creation", filename.c_str());
        return false;
      }
    }
    // create file
    std::ofstream file(filepath + filename);

    if (!file.good()) {
      LOG_F(ERROR, "Failed to create File, cannot save Problem");
      return false;
    }
    if (file.is_open()) {
      // first line problem size
      file << job_count_ << " " << machine_count_ << "\n";
      // output problem matrix
      for (const auto& job : jobs_) {
        size_t len = job.size();
        file << len;
        for (const auto& step : job.getTasks()) {
          file << ", " << step.machine << " " << step.duration;
        }
        file << "\n";
      }

      file.close();
    }
    else {
      LOG_F(ERROR, "failed to open the file, cannot save");
      return false;
    }
    DLOG_F(5, "successfully saved solution %s", name_.c_str());
    return true;
  }


	std::ostream& operator<<(std::ostream& os, const Problem& p)
	{
		os << "Problem has " << p.job_count_ << " Jobs and " << p.machine_count_ << " machines" << "\n";
		os << "Jobs in (machine, duration) format are:" << "\n";
		for (Job t : p.jobs_) {
			os << t << "\n";
		}
		return os;
	}

}
