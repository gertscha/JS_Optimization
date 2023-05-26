#ifndef PROBLEM_TASK_H_
#define PROBLEM_TASK_H_

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

			/*
			* create a subtask, only positive arguments allowed
			*/
			Step(unsigned int id, size_t index, unsigned int duration, unsigned int machine);

			bool operator< (const Step& rhs) { return (this->task_id <= rhs.task_id) && (this->index < rhs.index); }
			bool operator> (const Step& rhs) { return (this->task_id >= rhs.task_id) && (this->index > rhs.index); }
			bool operator==(const Step& rhs) { return (this->task_id == rhs.task_id) && (this->index == rhs.index)
														                      && (this->machine == rhs.machine); }
			bool operator!=(const Step& rhs) { return !(*this == rhs); }
		};

		/*
		  create a new Task, set unique id and max Step count
		  usually only called from the Problem constructor
		*/
		Task(unsigned int id, size_t StepCount);

		/*
		  append a Step to this task
		  returns false if max Step count exceeded
		*/
		bool AppendStep(unsigned int machine, unsigned int duration);

		/*
		  get Task Id
		*/
		unsigned int getId() const { return id_; }

		/*
		  get the list of Step's this Task consists of
		*/
		const std::vector<Step>& getSteps() const { return steps_; }

		/*
		  get the number of Step's this Task has
		*/
		size_t size() const { return step_count_; }

		/*
		  the minimum time it takes to complete all the Steps in this Task
		*/
		long getMinDuration() const { return min_duration_; }

		/*
		  mainly used to check if minDuration was calculated and if Step count
		  matches the declared maximum
		*/
		bool isFinal() const { return m_final; }

		/*
		  Utility
		*/
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

#endif // PROBLEM_TASK_H_