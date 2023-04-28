#include "Problem.h"

#include "Task.h"
#include "Utility.h"

#include "loguru.hpp"

#include <fstream>


namespace JSOptimzer {
	
	long Problem::Bounds::getLowerBound() const
	{
		return (TaskLowerBound > MachineLowerBound) ? TaskLowerBound : MachineLowerBound;
	}

	/*
	* takes ownership of the machineBounds vector
	*/
	Problem::Bounds::Bounds(unsigned int lTId, unsigned int lMId, long TlB, long MlB, long SuB,
							std::vector<long>* machineBounds,  Problem* problem)
	: longestTaskId(lTId),
	  longestMachineId(lMId),
	  TaskLowerBound(TlB),
	  MachineLowerBound(MlB),
	  sequentialUpperBound(SuB),
	  m_problem(problem)
	{
		m_machineBounds = machineBounds;
	}

	Problem::Bounds::~Bounds()
	{
		delete m_machineBounds;
	}


	void Problem::parseFileAndInitTaskVec(std::ifstream& file) {
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
		m_numTasks = numTasks;
		m_numMachines = numMachines;
		if (Utility::getNextInt(line, index, currentInt))
			ABORT_F("parsing error, expected newline after task and machine count on file line %i", (taskIndex + commentCount + 1));
		// prepare vector for tasks
		m_tasks = std::vector<Task>();
		m_tasks.reserve(m_numTasks);
		// create indiviual tasks (each task is a line)
		while (std::getline(file, line)) {
			// ensure length is correct
			if (taskIndex >= m_numTasks) {
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
			int taskSize = currentInt;
			// create the task
			m_tasks.push_back(Task(taskIndex, taskSize));
			// parse the Steps of this Task
			while (Utility::getNextInt(line, index, currentInt)) {
				int m = currentInt;
				if ((m > 0 && (unsigned int)m >= m_numMachines) || !Utility::getNextInt(line, index, currentInt)) // use eval order to check that m is not too large
					ABORT_F("parsing error, invalid step tuple on file line %i at character %i", (taskIndex + commentCount + 2), (index - 1));
				// create the step (i.e. subtask)
				bool exceeded = m_tasks[taskIndex].appendStep(m, currentInt); // checks for postive values in constructor
				CHECK_F(exceeded, "On file line %i the number of Steps exceeded declared number", (taskIndex + commentCount + 2));
				index++; // move past ',' in the problem format
			}
			// ensure that subtask size is correct
			CHECK_F(m_tasks[taskIndex].isFinal(), "On file line %i the number of Steps was lower than declared", (taskIndex + commentCount + 2));
			taskIndex++;
		}
	}


	Problem::Problem(const std::string& filepath, const std::string& filename, std::string problemName)
	{
		if (problemName.empty())
			m_name = filename;
		else
			m_name = problemName;
		// get data from input file
		std::ifstream file(filepath + filename);

		if (!file.good())
			ABORT_F("Invalid File, cannot create Problem Description");

		if (file.is_open()) {
			DLOG_F(INFO, "parsing Problem description file");

			parseFileAndInitTaskVec(file);

			file.close();
			DLOG_F(INFO, "finished parsing the Problem description file");
		}
		else
			ABORT_F("failed to open the Problem description file");
		// finalize Tasks & get lower bound
		long taskDurationlB = 0;
		int lBtaskId = 0;
		long machineDuationlB = 0;
		int lBmachineId = 0;
		long seqUpperBound = 0;
		// create with new and hand over ownership to the Problem::Bounds class
		std::vector<long>* machineBounds = new std::vector<long>(m_numMachines, 0);
		// task length member also setup in this loop
		m_taskStepCounts = std::vector<size_t>(m_tasks.size());
		for (Task& t : m_tasks) {
			// set task length
			m_taskStepCounts[t.getId()] = t.getSteps().size();
			// task bound calculation
			long TaskMinDuration = t.getMinDuration();
			seqUpperBound += TaskMinDuration;
			if (TaskMinDuration > taskDurationlB) {
				taskDurationlB = t.getMinDuration();
				lBtaskId = t.getId();
			}
			// machine bound calc
			for (const Task::Step& s : t.getSteps()) {
				(*machineBounds)[s.machine] += s.duration;
			}
		}

		for (unsigned int i = 0; i < m_numMachines; ++i) {
			if ((*machineBounds)[i] > machineDuationlB) {
				machineDuationlB = (*machineBounds)[i];
				lBmachineId = i;
			}
		}
		lowerBound = new Problem::Bounds(lBtaskId, lBmachineId, taskDurationlB, machineDuationlB,
											seqUpperBound, machineBounds, this);

	}


	Problem::~Problem()
	{
		delete lowerBound;
	}


	std::ostream& operator<<(std::ostream& os, const Problem& p)
	{
		os << "Problem has " << p.m_numTasks << " Tasks and " << p.m_numMachines << " machines" << std::endl;
		os << "Tasks in (machine, duration) format are:" << std::endl;
		for (Task t : p.m_tasks) {
			os << t << std::endl;
		}
		return os;
	}

}
