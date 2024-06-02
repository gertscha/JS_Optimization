#include "Job.h"

#include "loguru.hpp"


namespace JSOptimizer
{

  Job::Task::Task(unsigned int id, unsigned int index, unsigned int duration, unsigned int machine)
    :job_id(id), index(index), duration(duration), machine(machine)
  {}


  Job::Job(unsigned int id, unsigned int StepCount)
    :m_final(false), id_(id), target_task_count_(StepCount), task_count_(0), min_duration_(0)
  {
    tasks_ = std::vector<Task>();
    tasks_.reserve(target_task_count_);
  }


  bool Job::AppendTask(unsigned int machine, unsigned int duration)
  {
    if (!m_final)
    {
      tasks_.push_back(Task(id_, task_count_, duration, machine));
      min_duration_ += duration;
      task_count_++;
      if (task_count_ == target_task_count_)
        m_final = true;
      return true;
    }
    else
    {
      LOG_F(WARNING, "Trying to append a Task to a final Job (task id: %i)", id_);
      return false;
    }
  }

  bool Job::SetTask(unsigned int index, Job::Task task)
  {
    if (!m_final) {
      if (tasks_.empty())
        tasks_ = std::vector<Task>(target_task_count_, Task(0, 0, 0, 0));

      tasks_[index] = task;
      min_duration_ += task.duration;
      task_count_++;

      if (task_count_ == target_task_count_)
        m_final = true;
      return true;
    }
    else
    {
      LOG_F(WARNING, "Trying to set a Task to a final Job (task id: %i)", id_);
      return false;
    }
  }


  std::ostream& operator<<(std::ostream& os, const Job& t)
  {
    os << "Job " << t.id_ << ": [";
    for (const Job::Task& s : t.tasks_)
    {
      os << "(" << s.machine << "," << s.duration << "),";
    }
    os << "\b]";
    return os;
  }


}