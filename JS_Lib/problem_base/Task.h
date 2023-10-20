#ifndef PROBLEM_BASE_TASK_H_
#define PROBLEM_BASE_TASK_H_

#include <iostream>
#include <vector>


namespace JSOptimizer {

	class Task
	{
	public:

		class Step {
		public:
			unsigned int task_id;
			unsigned int index;
			unsigned int duration;
			unsigned int machine;

			// create a subtask, only positive arguments allowed
			Step(unsigned int id, unsigned int index, unsigned int duration, unsigned int machine);

			bool operator< (const Step& rhs) { return (this->task_id <= rhs.task_id) && (this->index < rhs.index); }
			bool operator> (const Step& rhs) { return (this->task_id >= rhs.task_id) && (this->index > rhs.index); }
			bool operator==(const Step& rhs) { return (this->task_id == rhs.task_id) && (this->index == rhs.index)
														                      && (this->machine == rhs.machine); }
			bool operator!=(const Step& rhs) { return !(*this == rhs); }
		};

		// create a new Task, set unique id and max Step count
		// usually only called from the Problem constructor
		Task(unsigned int id, unsigned int StepCount);

		// append a Step to this task
		// returns false if max Step count exceeded
		bool AppendStep(unsigned int machine, unsigned int duration);

    // set the step at index, intended for a one time setup
    // will not modify the Task more than target_step_count_ times
    // returns false if this limit has been reached
    bool SetStep(unsigned int index, Task::Step step);

		// get Task Id
		inline unsigned int getId() const { return id_; }

		// get the list of Step's this Task consists of
		inline const std::vector<Step>& getSteps() const { return steps_; }

    // get the number of Step's this Task has
		inline size_t size() const { return step_count_; }

		// the minimum time it takes to complete all the Steps in this Task
		inline long getMinDuration() const { return min_duration_; }

		// mainly used to check if minDuration was calculated and if Step count
		// matches the declared maximum
		inline bool isFinal() const { return m_final; }

		// Utility
		friend std::ostream& operator<<(std::ostream& os, const Task& dt);

	private:
		bool m_final;
		unsigned int id_;
		unsigned int target_step_count_;
		unsigned int step_count_;
		long min_duration_;

		std::vector<Step> steps_;

	};

}

#endif // PROBLEM_BASE_TASK_H_