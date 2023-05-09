#include "Solution.h"

#include <fstream>

#include "loguru.hpp"

#include "Task.h"
#include "Utility.h"


namespace JSOptimizer {

	/*////////////////////
	   Member Functions
	////////////////////*/

	void Solution::ParseFileAndInitSolution(std::ifstream& file) {
		std::string line; // current line of the file
		unsigned int lineIndex = 0; // index into the current line, for parsing purposes
		long currentInt = 0; // last integer that was parsed
		unsigned int commentCount = 0; // enables accurate line information for error messages
		unsigned int machineIndex = 0; // index of current machine, for this->solution
		unsigned int stepIndex = 0;
		// allow comments
		while (std::getline(file, line)) {
			if (line[0] != '#')
				break;
			else
				commentCount++;
		}
		// first line is name
    name_ = line;
		std::getline(file, line);
		// get problem size variables, started on first line that does not start with '#'
		long numTasks = 0;
		long numMachines = 0;
		if (!(Utility::getNextInt(line, lineIndex, numTasks) && Utility::getNextInt(line, lineIndex, numMachines)))
		{
			ABORT_F("parsing error, first non-comment line needs to be two interges for 'task_cnt machine_cnt'");
		}
		if (numTasks < 1 || numMachines < 1)
			ABORT_F("Solution invalid, it needs at least one machine and one task");
    task_count_ = numTasks;
    machine_count_ = numMachines;
		// check no trailing integers on this line
		if (Utility::getNextInt(line, lineIndex, currentInt))
			ABORT_F("parsing error, expected line to end after task and machine count on file line %i", (machineIndex + commentCount + 1));
		// init outer vector
		solution_ = std::vector<std::vector<Solution::Step>>(machine_count_);
		// prepare problemRep, need to track sizes to finish
		problem_view_ = std::vector<std::vector<Solution::Step*>>(numTasks);
		std::vector<long> taskIndCnt = std::vector<long>(numTasks, 0);
		// read the solution
		while (std::getline(file, line)) {
			// ensure declared length is not exceeded
			if (machineIndex >= machine_count_) {
				if (!line.empty()) // allow trailing empty lines
					ABORT_F("parsing error, found more lines in the solution than declared");
				break;
			}
			currentInt = 0;
			lineIndex = 0;
			// get number of entries for the machine
			if (!Utility::getNextInt(line, lineIndex, currentInt) || currentInt < 1) // uses eval order to check if currentInt is positive 
				ABORT_F("parsing error, expected a SolStep description on file line %i", (machineIndex + commentCount + 3));
			lineIndex += 2; // move past '-' in the problem format
			unsigned int curSolStepCnt = currentInt;
			// create the a inner vector
			solution_[machineIndex] = std::vector<Solution::Step>(curSolStepCnt);
			// parse the SolSteps, format: tid, tind, tm, td, st, et
			long tid = 0, tindex = 0, machine = 0, duration = 0, start = 0, end = 0;
			stepIndex = 0;
			while (Utility::getNextInt(line, lineIndex, currentInt)) {
				tid = currentInt;
				if (stepIndex >= curSolStepCnt)
					ABORT_F("parsing error, more SolSteps than declared on line %i", (machineIndex + commentCount + 3));
				if (!(Utility::getNextInt(line, lineIndex, tindex) && Utility::getNextInt(line, lineIndex, machine)
					&& Utility::getNextInt(line, lineIndex, duration) && Utility::getNextInt(line, lineIndex, start)
					&& Utility::getNextInt(line, lineIndex, end))) // use eval order to carrie lineIndex through all calls
				{
					ABORT_F("parsing error, invalid SolStep on line %i around character %i", (machineIndex + commentCount + 3), lineIndex);
				}
				lineIndex++; // move past ',' in the problem format
				if (tid < 0 || tindex < 0 || machine < 0 || duration < 0 || start < 0 || end < 0)
					ABORT_F("parsing error, invalid SolStep (negative value) on line %i, tuple %i", (machineIndex + commentCount + 3), (stepIndex + 1));
				if ((unsigned int)machine != machineIndex || (unsigned int)tid >= task_count_)
					ABORT_F("parsing error, invalid machine, index or task_id on line %i, tuple %i", (machineIndex + commentCount + 3), (stepIndex + 1));
				// create SolStep
				size_t solSize = solution_.size();
				size_t solMachineSize = solution_[machineIndex].size();
				if (!(machineIndex < solSize && stepIndex < solMachineSize))
					ABORT_F("parsing error, indices out of bound on line %i, tuple %i", (machineIndex + commentCount + 3), (stepIndex + 1));
				solution_[machineIndex][stepIndex] = Solution::Step{ (unsigned int)tid, (size_t)tindex, (unsigned int)machine, start, end };
				stepIndex++;
				if (tindex > taskIndCnt[tid])
					taskIndCnt[tid] = tindex;
			}
			if (stepIndex < curSolStepCnt)
				ABORT_F("parsing error, found %i SolSteps on file line %i, but %i were declared", stepIndex, (machineIndex + commentCount + 3), curSolStepCnt);
			machineIndex++;
		}
		// complete init of problemRep vectors
		for (int i = 0; i < numTasks; ++i) {
			problem_view_[i] = std::vector<Solution::Step*>(taskIndCnt[i] + 1, nullptr);
		}
	}


	void Solution::FillProblemView() const
	{
		unsigned int tid = 0;
		size_t tindex = 0;
		for (unsigned int i = 0; i < machine_count_; ++i) {
			for (unsigned int j = 0; j < solution_[i].size(); ++j) {
			  const Solution::Step* sstep = &solution_[i][j];
				tid = sstep->task_id;
				tindex = sstep->step_index;
				if (tid >= task_count_ || tindex >= problem_view_[tid].size())
					ABORT_F("Tried to add invalid SolStep (id %i, ind %i) in fillProblemRep() for '%s'", tid, (unsigned int)tindex, name_.c_str());
				if (problem_view_[tid][tindex] != nullptr)
					LOG_F(ERROR, "There seems to be a duplicate SolStep (id %i, index %i) in %s", tid, (unsigned int)tindex, name_.c_str());
        problem_view_[tid][tindex] = const_cast<Solution::Step*>(sstep);
			}
		}
		for (unsigned int i = 0; i < task_count_; ++i) {
			for (unsigned int j = 0; j < problem_view_[i].size(); ++j) {
				if (problem_view_[i][j] == nullptr)
					LOG_F(ERROR, "There seems to a missing SolStep (id %i, index %i) in %s", i, j, name_.c_str());
			}
		}
	}


	Solution::Solution(const std::string& filepath, const std::string& filename)
		: initalized_(true), makespan_(-1)
	{
		// get data from input file
		std::ifstream file(filepath + filename);

		if (!file.good())
			ABORT_F("Invalid File, cannot create Solution");

		if (file.is_open()) {

      ParseFileAndInitSolution(file);

			file.close();
		}
		else
			ABORT_F("failed to open the Solution file");
	}


	// SolStep file format: tid, tind, tm, td, st, et
	bool Solution::SaveToFile(const std::string& filepath, const std::string& filename) const
	{
		if (!initalized_) {
			LOG_F(ERROR, "cannot save Solution that is uninitalized");
			return false;
		}
		// create file
		std::ofstream file(filepath + filename);

		if (!file.good()) {
			LOG_F(ERROR, "Failed to create File, cannot save Solution");
			return false;
		}
		if (file.is_open()) {
			// first line is name
			file << name_ << "\n";
			// second line, other memebers
			file << task_count_ << " " << machine_count_ << "\n";
			// output solution matrix
			for (unsigned int i = 0; i < machine_count_; ++i) {
				size_t len = solution_[i].size();
				file << len << " - ";
				for (unsigned int j = 0; j < len; ++j) {
					file << solution_[i][j] << ", ";
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


	// Helper function
	// checks that a given SolStep matches the one it represents in the Problem
	bool validateStepsMatch(const Solution::Step& ss, const Task::Step& ps, unsigned int i, unsigned int j)
	{
		if (ss.step_index != ps.index || ss.task_id != ps.task_id || (ss.end_time - ss.start_time) != ps.duration
			|| ss.machine != ps.machine)
			return false;
		if (ss.task_id != i || ss.step_index != j)
			return false;

		return true;
	}

	// Helper function
	// checks that a SolStep cur does not overlap with prev and that curs times are valid
	bool validateStepTiming(const Solution::Step& prev, const Solution::Step& cur)
	{
		if (cur.end_time - cur.start_time < 0 || prev.end_time > cur.start_time)
			return false;

		return true;
	}

	// Helper member function
	// check paramters match and actually represent the internal descriptions
	bool Solution::ValidateParametersMatch(const Problem& p) const {
		if (p.getMachineCount() != machine_count_ || p.getTaskCount() != task_count_)
			return false;
		unsigned int mid = 0;
		for (size_t len : p.getStepCountForMachines()) {
			if (len != solution_[mid].size())
				return false;
			mid++;
		}

		return true;
	}


	bool Solution::ValidateSolution(const Problem& p) const
	{
		if (!initalized_) {
			LOG_F(ERROR, "cannot validate Solution that is uninitalized");
			return false;
		}
    // if the problemView is not filled, fill it
    if (problem_view_[0][0] == nullptr)
      FillProblemView();

		// check parameters match
		if (!ValidateParametersMatch(p)) {
			DLOG_F(INFO, "Solution and Problem parameters (Task Count or Machine Count) do not match");
			return false;
		}
		// check no processing overlap on any machine
		const std::vector<std::vector<Solution::Step>>& sol = solution_; // alias with shorter name
		size_t stepCnt = 0;

		for (unsigned int i = 0; i < machine_count_; ++i) {
			stepCnt = sol[i].size();
			const Solution::Step& fst = sol[i][0];
			if (fst.start_time < 0) {
				DLOG_F(INFO, "Solution Step (id %i, index 0) has invalid timing", i);
				return false;
			}
			if (fst.machine != i) {
				DLOG_F(INFO, "Solution Step (id %i, index 0) is on the wrong machine", i);
				return false;
			}
			for (unsigned int j = 1; j < stepCnt; ++j) {
				if (!validateStepTiming(sol[i][j - 1], sol[i][j])) {
					DLOG_F(INFO, "Solution Step (id %i, index %i) has invalid timing", i, j);
					return false;
				}
				if (sol[i][j].machine != i) {
					DLOG_F(INFO, "Solution Step (id %i, index %i) is on the wrong machine", i, j);
					return false;
				}
			}
		}
		// check that Steps match and that no processing overlap for a step
		const std::vector<Task>& pTasks = p.getTasks();

		for (unsigned int i = 0; i < task_count_; ++i) {
			const std::vector<Task::Step>& tlist = pTasks[i].getSteps();
			stepCnt = problem_view_[i].size();
			if (tlist.size() != stepCnt) {
				DLOG_F(INFO, "Solution and Problem Step count for Task (id %i) do not match", i);
				return false;
			}
			if (!validateStepsMatch(*problem_view_[i][0], tlist[0], i, 0)) {
				DLOG_F(INFO, "Solution Step and Problem Step do not match for: id %i, index 0", i);
				return false;
			}
			const Solution::Step& fst = *problem_view_[i][0];
			if (fst.start_time < 0 || fst.end_time - fst.start_time != tlist[0].duration) {
				DLOG_F(INFO, "Solution Step (id %i, index 0) has invalid timing", i);
				return false;
			}
			for (unsigned int j = 1; j < stepCnt; ++j) {
				if (!validateStepsMatch(*problem_view_[i][j], tlist[j], i, j)) {
					DLOG_F(INFO, "Solution Step and Problem Step do not match for: id %i, index %i", i, j);
					return false;
				}
				if (!validateStepTiming(*problem_view_[i][j - 1], *problem_view_[i][j])) {
					DLOG_F(INFO, "Solution Step (id %i, index %i) has invalid timing", i, j);
					return false;
				}
			}
		}
		return true;
	}


	long Solution::getMakespan()
	{
		if (!initalized_) {
			LOG_F(ERROR, "getCompletetionTime invoked on uninitalized Solution");
			return -1;
		}
		if (makespan_ != -1)
			return makespan_;

		long res = 0;
		size_t mCnt = solution_.size();
		for (unsigned int i = 0; i < mCnt; ++i) {
			long t = solution_[i].back().end_time;
			if (t > res)
				res = t;
		}
		makespan_ = res;
		return res;
	}


	/*//////////////////////
	   Function Overloads
	//////////////////////*/

	std::ostream& operator<<(std::ostream& os, const Solution::Step& ss)
	{
		long dur = ss.end_time - ss.start_time;
		os << ss.task_id << " " << ss.step_index << " " << ss.machine << " " << dur << " " << ss.start_time << " " << ss.end_time;
		return os;
	}


	std::ostream& operator<<(std::ostream& os, const Solution& s)
	{
		os << "Solution '" << s.name_ << "'" << "\n";
		os << s.task_count_ << " Tasks on " << s.machine_count_ << " machines, completion time is " << s.makespan_ << "\n";
		os << "tuple format: (taskId, taskIndex, machine, duration, startTime, endTime)" << "\n";
		for (unsigned int i = 0; i < s.machine_count_; ++i) {
			os << "Machine " << i << " [";
			size_t size = s.solution_[i].size();
			for (int j = 0; j < size; ++j) {
				os << "(" << s.solution_[i][j] << "),";
			}
			os << "\b]" << "\n";
		}
		return os;
	}


}
