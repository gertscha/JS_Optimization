#include "Problem.h"

#include <utility>

#include "loguru.hpp"

#include "Task.h"
#include "Utility.h"


namespace JSOptimizer {
	
	long Problem::Bounds::getLowerBound() const
	{
		return (task_lower_bound > machine_lower_bound) ? task_lower_bound : machine_lower_bound;
	}

	/*
	* takes ownership of the machineBounds vector
	*/
	Problem::Bounds::Bounds(unsigned int lTId, unsigned int lMId, long TlB, long MlB, long SuB,
							std::vector<long>&& machineBounds,  Problem* problem)
	: limiting_task_id(lTId),
    limiting_machine_id(lMId),
    task_lower_bound(TlB),
    machine_lower_bound(MlB),
    sequential_upper_bound(SuB),
    problem_pointer_(problem),
    machine_bounds_(std::move(machineBounds))
	{}


	void Problem::ParseFileAndInitTaskVectors(std::ifstream& file) {
		std::string line; // current line of the file
		unsigned int index = 0; // index into the current line, for parsing purposes
		long currentInt = 0; // last integer that was parsed
		unsigned int taskIndex = 0; // index of current Task, acts as Id
		unsigned int commentCount = 0; // allow accurate line information for error messages
		// allow comments
		while (std::getline(file, line)) {
			if (line[0] != '#')
				break;
			else
				commentCount++;
		}
		// get problem size variables
		long numTasks = 0;
		long numMachines = 0;
		// first line that does not start with '#', start of Problem description
		if (!(Utility::getNextInt(line, index, numTasks) && Utility::getNextInt(line, index, numMachines)))
			ABORT_F("parsing error, first non-comment line needs to be two interges for 'task_count machine_count'");
		if (numTasks < 1 || numMachines < 1)
			ABORT_F("Problem invalid, it needs at least one machine and one task");
    num_tasks_ = numTasks;
    num_machines_ = numMachines;
		if (Utility::getNextInt(line, index, currentInt))
			ABORT_F("parsing error, expected newline after task and machine count on file line %i", (taskIndex + commentCount + 1));
		// prepare vector for tasks
    tasks_ = std::vector<Task>();
    tasks_.reserve(num_tasks_);
		// create indiviual tasks (each task is a line)
		while (std::getline(file, line)) {
			// ensure length is correct
			if (taskIndex >= num_tasks_) {
				if (!line.empty()) // allow trailing empty lines
					ABORT_F("parsing error, found more Tasks than declared");
				break;
			}
			currentInt = 0;
			index = 0;
			// get number of subtasks
			if (!Utility::getNextInt(line, index, currentInt) || currentInt < 1) // uses eval order to check if current is positive 
				ABORT_F("parsing error, expected a Task description on file line %i", (taskIndex + commentCount + 2));
			index += 2; // move past '-' in the problem format
			long taskSize = currentInt;
			// create the task
      tasks_.push_back(Task(taskIndex, taskSize));
			// parse the Steps of this Task
			while (Utility::getNextInt(line, index, currentInt)) {
				long m = currentInt;
				if ((m < 0 || m >= num_machines_) || !Utility::getNextInt(line, index, currentInt)) // use eval order to check that m is not too large
					ABORT_F("parsing error, invalid step tuple on file line %i at character %i", (taskIndex + commentCount + 2), (index - 1));
				// create the step (i.e. subtask)
				bool exceeded = tasks_[taskIndex].AppendStep(m, currentInt); // checks for postive values in constructor
				CHECK_F(exceeded, "On file line %i the number of Steps exceeded declared number", (taskIndex + commentCount + 2));
				index++; // move past ',' in the problem format
			}
			// ensure that subtask size is correct
			CHECK_F(tasks_[taskIndex].isFinal(), "On file line %i the number of Steps was lower than declared", (taskIndex + commentCount + 2));
			taskIndex++;
		}
	}


	Problem::Problem(const std::string& filepath, const std::string& filename, std::string problemName)
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

      ParseFileAndInitTaskVectors(file);

			file.close();
		}
		else
			ABORT_F("failed to open the Problem description file");
		
		// machine bounds and give ownership to the Problem::Bounds class
		auto machineBounds = std::vector<long>(num_machines_, 0);
		// other bounds
		long taskDurationlB = 0;
		int lBtaskId = 0;
		long machineDuationlB = 0;
		int lBmachineId = 0;
		long seqUpperBound = 0;
		// step counts from machine perspective
    machine_step_counts_ = std::vector<size_t>(num_machines_, 0);
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
		for (unsigned int i = 0; i < num_machines_; ++i) {
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
		os << "Problem has " << p.num_tasks_ << " Tasks and " << p.num_machines_ << " machines" << "\n";
		os << "Tasks in (machine, duration) format are:" << "\n";
		for (Task t : p.tasks_) {
			os << t << "\n";
		}
		return os;
	}

}
