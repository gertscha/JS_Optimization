#include "Task.h"

#include "loguru.hpp"


/*
* all the DCHECK_F's in this file should be checked even in Release, but are not for performance reasons
* (and because they are correctly initialized with the Problem class)
*/

namespace SimAnn {

	Task::Step::Step(int id, int index, int duration, int machine)
		:taskId(id), index(index), duration(duration), machine(machine)
	{
		DCHECK_F(id >= 0);
		DCHECK_F(index >= 0);
		CHECK_F(duration >= 0, "Creation of Step with negative duration");
		CHECK_F(machine >= 0, "Creation of Step with negative machine id");
	}


	Task::Task(int id, int StepCount)
		:m_final(false), m_id(id), m_targetStepCnt(StepCount), m_StepCnt(0), m_minDuration(0)
	{
		m_steps = std::vector<Step>();
		m_steps.reserve(m_targetStepCnt);
		m_machineSeq = std::vector<int>();
		m_machineSeq.reserve(m_targetStepCnt);
		m_taskDurationSeq = std::vector<int>();
		m_taskDurationSeq.reserve(m_targetStepCnt);
		m_usedMachinePool = std::set<int>();
	}


	bool Task::appendStep(int machine, int duration)
	{
		if (!m_final) {
			m_steps.push_back(Step(m_id, m_StepCnt, duration, machine));
			m_minDuration += duration;
			m_machineSeq.push_back(machine);
			m_taskDurationSeq.push_back(duration);
			if (!(m_usedMachinePool.contains(machine)))
				m_usedMachinePool.insert(machine);
			m_StepCnt++;
			if (m_StepCnt == m_targetStepCnt)
				m_final = true;
			return true;
		}
		else {
			LOG_F(WARNING, "Trying to append a Step to a final Task (task id: %i)", m_id);
			return false;
		}
	}


	std::ostream& operator<<(std::ostream& os, const Task& t)
	{
		os << "Task " << t.m_id << ": [";
		for (const Task::Step& s : t.m_steps) {
			os << "(" << s.machine << "," << s.duration << "),";
		}
		os << "\b]";
		return os;
	}


}