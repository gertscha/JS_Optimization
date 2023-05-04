#include "Solution.h"

#include "Problem.h"
#include "Task.h"
#include "Utility.h"

#include "loguru.hpp"

#include <fstream>


namespace JSOptimzer {

	/*////////////////////
	   Member Functions
	////////////////////*/

	void Solution::parseFileAndInitSolution(std::ifstream& file) {
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
		m_name = line;
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
		m_taskCnt = numTasks;
		m_machineCnt = numMachines;
		// check no trailing integers on this line
		if (Utility::getNextInt(line, lineIndex, currentInt))
			ABORT_F("parsing error, expected line to end after task and machine count on file line %i", (machineIndex + commentCount + 1));
		// init outer vector
		m_solution = std::vector<std::vector<SolStep>>(m_machineCnt);
		// prepare problemRep, need to track sizes to finish
		m_problemRep = std::vector<std::vector<SolStep*>>(numTasks);
		std::vector<long> taskIndCnt = std::vector<long>(numTasks, 0);
		// read the solution
		while (std::getline(file, line)) {
			// ensure declared length is not exceeded
			if (machineIndex >= m_machineCnt) {
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
			m_solution[machineIndex] = std::vector<SolStep>(curSolStepCnt);
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
				if ((unsigned int)machine != machineIndex || (unsigned int)tid >= m_taskCnt)
					ABORT_F("parsing error, invalid machine, index or task_id on line %i, tuple %i", (machineIndex + commentCount + 3), (stepIndex + 1));
				// create SolStep
				size_t solSize = m_solution.size();
				size_t solMachineSize = m_solution[machineIndex].size();
				if (!(machineIndex < solSize && stepIndex < solMachineSize))
					ABORT_F("parsing error, indices out of bound on line %i, tuple %i", (machineIndex + commentCount + 3), (stepIndex + 1));
				m_solution[machineIndex][stepIndex] = SolStep{ (unsigned int)tid, (size_t)tindex, (unsigned int)machine, start, end };
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
			m_problemRep[i] = std::vector<SolStep*>(taskIndCnt[i] + 1, nullptr);
		}
	}


	void Solution::fillProblemRep()
	{
		unsigned int tid = 0;
		size_t tindex = 0;
		for (unsigned int i = 0; i < m_machineCnt; ++i) {
			for (unsigned int j = 0; j < m_solution[i].size(); ++j) {
				SolStep* sstep = &m_solution[i][j];
				tid = sstep->taskId;
				tindex = sstep->index;
				if (tid >= m_taskCnt || tindex >= m_problemRep[tid].size())
					ABORT_F("Tried to add invalid SolStep (id %i, ind %i) in fillProblemRep() for '%s'", tid, (unsigned int)tindex, m_name.c_str());
				if (m_problemRep[tid][tindex] != nullptr)
					LOG_F(ERROR, "There seems to be a duplicate SolStep (id %i, index %i) in %s", tid, (unsigned int)tindex, m_name.c_str());
				m_problemRep[tid][tindex] = sstep;
			}
		}
		for (unsigned int i = 0; i < m_taskCnt; ++i) {
			for (unsigned int j = 0; j < m_problemRep[i].size(); ++j) {
				if (m_problemRep[i][j] == nullptr)
					LOG_F(ERROR, "There seems to a missing SolStep (id %i, index %i) in %s", i, j, m_name.c_str());
			}
		}
	}


	Solution::Solution(const std::string& filepath, const std::string& filename)
		: m_initalized(true), m_completionTime(-1)
	{
		// get data from input file
		std::ifstream file(filepath + filename);

		if (!file.good())
			ABORT_F("Invalid File, cannot create Solution");

		if (file.is_open()) {

			parseFileAndInitSolution(file);

			file.close();
		}
		else
			ABORT_F("failed to open the Solution file");

		fillProblemRep();
		DLOG_F(INFO, "successfully created Solution '%s'", m_name.c_str());
	}


	// SolStep file format: tid, tind, tm, td, st, et
	bool Solution::saveToFile(const std::string& filepath, const std::string& filename) const
	{
		if (!m_initalized) {
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
			file << m_name << "\n";
			// second line, other memebers
			file << m_taskCnt << " " << m_machineCnt << "\n";
			// output solution matrix
			for (unsigned int i = 0; i < m_machineCnt; ++i) {
				size_t len = m_solution[i].size();
				file << len << " - ";
				for (unsigned int j = 0; j < len; ++j) {
					file << m_solution[i][j] << ", ";
				}
				file << "\n";
			}

			file.close();
		}
		else {
			LOG_F(ERROR, "failed to open the file, cannot save");
			return false;
		}
		LOG_F(INFO, "successfully saved solution %s", m_name.c_str());
		return true;
	}


	// Helper function
	// checks that a given SolStep matches the one it represents in the Problem
	bool validateStepsMatch(const Solution::SolStep& ss, const Task::Step& ps, unsigned int i, unsigned int j)
	{
		if (ss.index != ps.index || ss.taskId != ps.taskId || (ss.endTime - ss.startTime) != ps.duration
			|| ss.machine != ps.machine)
			return false;
		if (ss.taskId != i || ss.index != j)
			return false;

		return true;
	}

	// Helper function
	// checks that a SolStep cur does not overlap with prev and that curs times are valid
	bool validateStepTiming(const Solution::SolStep& prev, const Solution::SolStep& cur)
	{
		if (cur.endTime - cur.startTime < 0 || prev.endTime > cur.startTime)
			return false;

		return true;
	}

	// Helper member function
	// check paramters match and actually represent the internal descriptions
	bool Solution::validateParametersMatch(const Problem& p) const {
		if (p.getMachineCnt() != m_machineCnt || p.getTaskCnt() != m_taskCnt)
			return false;
		unsigned int mid = 0;
		for (size_t len : p.getStepCountForMachines()) {
			if (len != m_solution[mid].size())
				return false;
			mid++;
		}

		return true;
	}


	bool Solution::validateSolution(const Problem& p) const
	{
		if (!m_initalized) {
			LOG_F(ERROR, "cannot validate Solution that is uninitalized");
			return false;
		}
		// check parameters match
		if (!validateParametersMatch(p)) {
			DLOG_F(INFO, "Solution and Problem parameters (Task Count or Machine Count) do not match");
			return false;
		}
		// check no processing overlap on any machine
		const std::vector<std::vector<SolStep>>& sol = m_solution; // alias with shorter name
		size_t stepCnt = 0;

		for (unsigned int i = 0; i < m_machineCnt; ++i) {
			stepCnt = sol[i].size();
			const SolStep& fst = sol[i][0];
			if (fst.startTime < 0) {
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

		for (unsigned int i = 0; i < m_taskCnt; ++i) {
			const std::vector<Task::Step>& tlist = pTasks[i].getSteps();
			stepCnt = m_problemRep[i].size();
			if (tlist.size() != stepCnt) {
				DLOG_F(INFO, "Solution and Problem Step count for Task (id %i) do not match", i);
				return false;
			}
			if (!validateStepsMatch(*m_problemRep[i][0], tlist[0], i, 0)) {
				DLOG_F(INFO, "Solution Step and Problem Step do not match for: id %i, index 0", i);
				return false;
			}
			const SolStep& fst = *m_problemRep[i][0];
			if (fst.startTime < 0 || fst.endTime - fst.startTime != tlist[0].duration) {
				DLOG_F(INFO, "Solution Step (id %i, index 0) has invalid timing", i);
				return false;
			}
			for (unsigned int j = 1; j < stepCnt; ++j) {
				if (!validateStepsMatch(*m_problemRep[i][j], tlist[j], i, j)) {
					DLOG_F(INFO, "Solution Step and Problem Step do not match for: id %i, index %i", i, j);
					return false;
				}
				if (!validateStepTiming(*m_problemRep[i][j - 1], *m_problemRep[i][j])) {
					DLOG_F(INFO, "Solution Step (id %i, index %i) has invalid timing", i, j);
					return false;
				}
			}
		}
		return true;
	}


	long Solution::getCompletetionTime()
	{
		if (!m_initalized) {
			LOG_F(ERROR, "getCompletetionTime invoked on uninitalized Solution");
			return -1;
		}
		if (m_completionTime != -1)
			return m_completionTime;

		long res = 0;
		size_t mCnt = m_solution.size();
		for (unsigned int i = 0; i < mCnt; ++i) {
			long t = m_solution[i][m_solution[i].size() - 1].endTime;
			if (t > res)
				res = t;
		}
		m_completionTime = res;
		return res;
	}


	/*//////////////////////
	   Function Overloads
	//////////////////////*/

	std::ostream& operator<<(std::ostream& os, const Solution::SolStep& ss)
	{
		long dur = ss.endTime - ss.startTime;
		os << ss.taskId << " " << ss.index << " " << ss.machine << " " << dur << " " << ss.startTime << " " << ss.endTime;
		return os;
	}


	std::ostream& operator<<(std::ostream& os, const Solution& s)
	{
		os << "Solution '" << s.m_name << "'" << "\n";
		os << s.m_taskCnt << " Tasks on " << s.m_machineCnt << " machines, completion time is " << s.m_completionTime << "\n";
		os << "tuple format: (taskId, taskIndex, machine, duration, startTime, endTime)" << "\n";
		for (unsigned int i = 0; i < s.m_machineCnt; ++i) {
			os << "Machine " << i << " [";
			size_t size = s.m_solution[i].size();
			for (int j = 0; j < size; ++j) {
				os << "(" << s.m_solution[i][j] << "),";
			}
			os << "\b]" << "\n";
		}
		return os;
	}


}
