#include "Task.h"

#include "loguru.hpp"


namespace JSOptimizer {

	Task::Step::Step(unsigned int id, unsigned int index, unsigned int duration, unsigned int machine)
		:task_id(id), index(index), duration(duration), machine(machine)
	{}


	Task::Task(unsigned int id, unsigned int StepCount)
		:m_final(false), id_(id), target_step_count_(StepCount), step_count_(0), min_duration_(0)
	{
    steps_ = std::vector<Step>();
    steps_.reserve(target_step_count_);
	}


	bool Task::AppendStep(unsigned int machine, unsigned int duration)
	{
		if (!m_final) {
      steps_.push_back(Step(id_, step_count_, duration, machine));
      min_duration_ += duration;
      step_count_++;
			if (step_count_ == target_step_count_)
				m_final = true;
			return true;
		}
		else {
			LOG_F(WARNING, "Trying to append a Step to a final Task (task id: %i)", id_);
			return false;
		}
	}


	std::ostream& operator<<(std::ostream& os, const Task& t)
	{
		os << "Task " << t.id_ << ": [";
		for (const Task::Step& s : t.steps_) {
			os << "(" << s.machine << "," << s.duration << "),";
		}
		os << "\b]";
		return os;
	}


}