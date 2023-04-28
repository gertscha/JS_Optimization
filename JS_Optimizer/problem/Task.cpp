#include "Task.h"

#include "loguru.hpp"



namespace JSOptimzer {

	Task::Step::Step(unsigned int id, size_t index, unsigned int duration, unsigned int machine)
		:taskId(id), index(index), duration(duration), machine(machine)
	{}


	Task::Task(unsigned int id, size_t StepCount)
		:m_final(false), m_id(id), m_targetStepCnt(StepCount), m_StepCnt(0), m_minDuration(0)
	{
		m_steps = std::vector<Step>();
		m_steps.reserve(m_targetStepCnt);
		m_usedMachinePool = std::set<unsigned int>();
	}


	bool Task::appendStep(unsigned int machine, unsigned int duration)
	{
		if (!m_final) {
			m_steps.push_back(Step(m_id, m_StepCnt, duration, machine));
			m_minDuration += duration;
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