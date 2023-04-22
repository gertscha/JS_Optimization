#pragma once

#include <iostream>
#include <vector>
#include <set>


namespace SimAnn {

	class Task
	{
	public:

		class Step {
		public:
			int taskId;
			int index;
			int duration;
			int machine;

			/*
			* create a subtask, only positive arguments allowed
			*/
			Step(int id, int index, int duration, int machine);
		};

		/*
		* create a new Task, set unique id and max Step count
		* usually only called from the Problem constructor
		*/
		Task(int id, int StepCount);

		/*
		* append a Step to this task
		* returns false if max Step count exceeded
		*/
		bool appendStep(int machine, int duration);

		/*
		* Getters
		*/

		int getId() const { return m_id; }

		const std::vector<Step>& getSteps() const { return m_steps; }

		/*
		* returns the number of Step this Task has
		*/
		int size() const { return m_StepCnt; }

		/*
		* a vector containing the Machines of the Steps, in the order that they need to be processed
		*/
		const std::vector<int>& getMseq() const { return m_machineSeq; }

		/*
		* a vector containing the Durations of all Steps, in the order that they need to be processed
		*/
		const std::vector<int>& getDseq() const { return m_taskDurationSeq; }

		/*
		* a set containing all machines that need to process at least one Step from this Task
		*/
		const std::set<int>& getMPool() const { return m_usedMachinePool; }

		/*
		* the minimum time it takes to complete all the Steps in this Task
		*/
		int getMinDuration() const { return m_minDuration; }

		/*
		* mainly used to check if minDuration was calculated and if Step count
		* matches the declared maximum
		*/
		bool isFinal() const { return m_final; }

		/*
		* Utility
		*/
		friend std::ostream& operator<<(std::ostream& os, const Task& dt);

	private:
		bool m_final;
		int m_id;
		int m_targetStepCnt;
		int m_StepCnt;
		int m_minDuration;

		std::vector<Step> m_steps;
		std::vector<int> m_machineSeq;
		std::vector<int> m_taskDurationSeq;
		std::set<int> m_usedMachinePool;

	};

}


/*
#pragma once

#include "../libs/loguru.hpp"

class Step
{
public:
	int taskId;
	int index;
	int duration;
	int machine;

	
	//* create a subtask, only positive arguments allowed

Step(int id, int index, int duration, int machine)
	:taskId(id), index(index), duration(duration), machine(machine)
{
	DCHECK_F(id >= 0);
	DCHECK_F(index >= 0);
	CHECK_F(duration >= 0, "Creation of Step with negative duration");
	CHECK_F(machine >= 0, "Creation of Step with negative machine id");
}
};

*/