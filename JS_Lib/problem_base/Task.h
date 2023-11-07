#ifndef PROBLEM_BASE_JOB_H_
#define PROBLEM_BASE_JOB_H_

#include <iostream>
#include <vector>


namespace JSOptimizer {

	class Job
	{
	public:

		class Task {
		public:
			unsigned int task_id;
			unsigned int index;
			unsigned int duration;
			unsigned int machine;

      Task() = delete;
			// create a subtask, only positive arguments allowed
			Task(unsigned int id, unsigned int index, unsigned int duration, unsigned int machine);

			bool operator< (const Task& rhs) { return (this->task_id <= rhs.task_id) && (this->index < rhs.index); }
			bool operator> (const Task& rhs) { return (this->task_id >= rhs.task_id) && (this->index > rhs.index); }
			bool operator==(const Task& rhs) { return (this->task_id == rhs.task_id) && (this->index == rhs.index)
														                      && (this->machine == rhs.machine); }
			bool operator!=(const Task& rhs) { return !(*this == rhs); }
		};

    Job() = delete;

		// create a new Job, set unique id and max Task count
		// usually only called from the Problem constructor
		Job(unsigned int id, unsigned int StepCount);

		// append a Task to this task
		// returns false if max Task count exceeded
		bool AppendTask(unsigned int machine, unsigned int duration);

    // set the Task at index, intended for a one time setup
    // will not modify the Job more than target_task_count_ times
    // returns false if this limit has been reached
    bool SetTask(unsigned int index, Job::Task step);

		// get Job Id
		inline unsigned int getId() const { return id_; }

		// get the list of Task's this Job consists of
		inline const std::vector<Task>& getTasks() const { return tasks_; }

    // get the number of Task's this Job has
		inline size_t size() const { return task_count_; }

		// the minimum time it takes to complete all the Tasks in this Job
		inline long getMinDuration() const { return min_duration_; }

		// mainly used to check if minDuration was calculated and if Task count
		// matches the declared maximum
		inline bool isFinal() const { return m_final; }

		// Utility
		friend std::ostream& operator<<(std::ostream& os, const Job& dt);

	private:
		bool m_final;
		unsigned int id_;
		unsigned int target_task_count_;
		unsigned int task_count_;
		long min_duration_;

		std::vector<Task> tasks_;

	};

}

#endif // PROBLEM_BASE_JOB_H_