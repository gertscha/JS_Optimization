#include "Problem.h"

#include <utility>
#include <fstream>
#include <sstream>
#include <tuple>

#include "loguru.hpp"

#include "Task.h"
#include "Parsing.h";


namespace JSOptimizer {
	
	long Problem::Bounds::getLowerBound() const
	{
		return (task_lower_bound > machine_lower_bound) ? task_lower_bound : machine_lower_bound;
	}

	/*
	* takes ownership of the machineBounds vector
	*/
	Problem::Bounds::Bounds(unsigned int lTId, unsigned int lMId, long TlB, long MlB,
                          long SuB, std::vector<long>&& machineBounds, Problem* problem)
	: limiting_task_id(lTId),
    limiting_machine_id(lMId),
    task_lower_bound(TlB),
    machine_lower_bound(MlB),
    sequential_upper_bound(SuB),
    problem_pointer_(problem),
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
      ABORT_F("on line %i trailing '%s' is invalid", (commentCount + 1), line.c_str());
    }
    if (first <= 0 || second <= 0)
      ABORT_F("parameters on line %i must be greater zero", (commentCount + 1));
    // assign to members
    task_count_ = first;
    machine_count_ = second;

    // start init of solution_
    tasks_ = std::vector<Task>();
    tasks_.reserve(task_count_);
    // track if all machines have at least one step
    auto machine_has_steps = std::vector<bool>(machine_count_, false);
    // read the remaining lines, each line is a task
    unsigned int task_index = 0;
    unsigned int tuple_count = 0;
    unsigned int expected = 0;
    long machine = 0, duration = 0;
    // iterate through lines
    while (std::getline(file, line)) {
      if (task_index >= task_count_) {
        if (!line.empty())
          ABORT_F("line %i is unexpected, contains '%s'", (commentCount + 2 + task_index), line.c_str());
        else
          break;
      }
      in_ss = std::istringstream(line);
      in_ss >> expected;
      if (in_ss.fail())
        ABORT_F("expected to find values on line %i", (commentCount + 2 + task_index));
      in_ss.ignore(1, ',');
      // prepare Task
      tasks_.push_back(Task(task_index, expected));
      // iterate through tuples and add them to the Task
      while (in_ss >> machine) // read first value
      {
        if (in_ss.fail())
          break;
        if (tuple_count >= expected)
          ABORT_F("on line %i there are more pairs than expected", (commentCount + 2 + task_index));
        // read the duration
        in_ss >> duration;
        // check all valid
        if (in_ss.fail())
          ABORT_F("on line %i pair %i is bad", (commentCount + 2 + task_index), (tuple_count + 1));
        if (machine < 0 || duration < 0)
          ABORT_F("only postive numbers allowed in pair %i on line %i", (tuple_count + 1), (commentCount + 2 + task_index));
        if (machine >= machine_count_)
          ABORT_F("invalid machine on line %i pair %i", (commentCount + 2 + task_index), (tuple_count + 1));

        tasks_.back().AppendStep(machine, duration);
        
        if (!machine_has_steps[machine])
          machine_has_steps[machine] = true;

        in_ss.ignore(1, ',');
        ++tuple_count;
      }
      if (tuple_count < expected)
        ABORT_F("on line %i there are fewer pairs than expected", (commentCount + 2 + task_index));
      tuple_count = 0;
      ++task_index;
    }
    if (task_index < task_count_)
      ABORT_F("there are fewer lines than expected");
    
    unsigned int steps_on_all_machines = 0;
    for (bool b : machine_has_steps) {
      if (b == true)
        ++steps_on_all_machines;
      else
        break;
    }
    if (steps_on_all_machines != machine_count_)
      LOG_F(WARNING, "machine with id %i in problem %s has no steps", steps_on_all_machines, name_.c_str());

	}

  void Problem::ParseStandardFileAndInit(std::ifstream& file) {
    std::string line;
    std::istringstream iss;
    long lowerBound = 0;

    std::getline(file, line);
    iss = std::istringstream(line);
    iss >> task_count_ >> machine_count_ >> lowerBound;

    tasks_ = std::vector<Task>();
    tasks_.reserve(task_count_);
    unsigned int task_index = 0;

    auto pairs = std::vector<std::tuple<long, long>>();

    while (std::getline(file, line)) {
      iss = std::istringstream(line);

      int t_count = 0;
      try {
        t_count = Utility::parseTuples(iss, pairs);
      }
      catch (std::runtime_error) {
        ABORT_F("Reading Error on line %i", (task_index + 1));
      }

      tasks_.push_back(Task(task_index, t_count));

      for (auto& tuple : pairs) {
        // append the step
        tasks_.back().AppendStep(std::get<0>(tuple), std::get<1>(tuple));
      }

      ++task_index;
      pairs.clear();
    }

  }


	Problem::Problem(const std::string& filepath, const std::string& filename, SpecificationType type, std::string problemName)
	{
		if (problemName.empty())
      name_ = filename;
		else
      name_ = problemName;
		// get data from input file
		std::ifstream file(filepath + filename);

		if (!file.good())
			ABORT_F("Invalid File, cannot create Problem Description");

		if (file.is_open()) {

      if (type == Detailed)
        ParseDetailedFileAndInit(file);
      if (type == Standard)
        ParseStandardFileAndInit(file);

			file.close();
		}
		else
			ABORT_F("failed to open the Problem description file");
		
		// machine bounds and give ownership to the Problem::Bounds class
		auto machineBounds = std::vector<long>(machine_count_, 0);
		// other bounds
		long taskDurationlB = 0;
		int lBtaskId = 0;
		long machineDuationlB = 0;
		int lBmachineId = 0;
		long seqUpperBound = 0;
		// step counts from machine perspective
    machine_step_counts_ = std::vector<size_t>(machine_count_, 0);
		// step through all tasks to determine the values
		for (Task& t : tasks_) {
			// task bound calculation
			long TaskMinDuration = t.getMinDuration();
			seqUpperBound += TaskMinDuration;
			if (TaskMinDuration > taskDurationlB) {
				taskDurationlB = t.getMinDuration();
				lBtaskId = t.getId();
			}
			// machine bound calc and counting steps per machine
			for (const Task::Step& s : t.getSteps()) {
				machineBounds[s.machine] += s.duration;
        machine_step_counts_[s.machine] += 1;
			}
		}
		// find biggest machine bound
		for (unsigned int i = 0; i < machine_count_; ++i) {
			if (machineBounds[i] > machineDuationlB) {
				machineDuationlB = machineBounds[i];
				lBmachineId = i;
			}
		}

    lower_bounds_pointer_ = new Problem::Bounds(lBtaskId, lBmachineId, taskDurationlB, machineDuationlB,
											seqUpperBound, std::move(machineBounds), this);
		DLOG_F(INFO, "successfully created Problem '%s'", name_.c_str());
	}


	Problem::~Problem()
	{
		delete lower_bounds_pointer_;
	}


	std::ostream& operator<<(std::ostream& os, const Problem& p)
	{
		os << "Problem has " << p.task_count_ << " Tasks and " << p.machine_count_ << " machines" << "\n";
		os << "Tasks in (machine, duration) format are:" << "\n";
		for (Task t : p.tasks_) {
			os << t << "\n";
		}
		return os;
	}

}
