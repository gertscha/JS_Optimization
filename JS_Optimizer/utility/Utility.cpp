#include "Utility.h"

#include "loguru.hpp"

#include <algorithm>

namespace JSOptimzer {

	bool Utility::getNextInt(std::string line, unsigned int& index, long& ret) {
		if (index >= line.size())
			return false;
		std::string string = line.substr(index, line.size());
		if (string.size() < 1)
			return false;
		const char* str = string.c_str();
		char* offsetMes;
		long res = strtol(str, &offsetMes, 10);
		int offset = (int)(offsetMes - str);
		index += offset;
		ret = res;
		if (offset == 0)
			return false;
		else
			return true;
	}


	/*///////////
		Heap
	///////////*/

	template<typename T>
	Utility::Heap<T>::Heap()
	{
		m_heap = std::vector<T>();
	}

	template<typename T>
	void Utility::Heap<T>::add(T element)
	{
		if (!std::is_heap(m_heap.begin(), m_heap.end()))
			std::make_heap(m_heap.begin(), m_heap.end());

		m_heap.push_back(element);

		std::push_heap(m_heap.begin(), m_heap.end());
	}

	template<typename T>
	T Utility::Heap<T>::pop()
	{
		std::pop_heap(m_heap.begin(), m_heap.end()); // moves the largest to the end

		T largest = m_heap.back();

		m_heap.pop_back(); // actually removes the largest element
		return largest;
	}

	template<typename T>
	T Utility::Heap<T>::peek()
	{
		std::pop_heap(m_heap.begin(), m_heap.end()); // moves the largest to the end

		T largest = m_heap.back();
		
		std::make_heap(m_heap.begin(), m_heap.end());

		return largest;
	}



	/*////////////////////
		Visualization
	////////////////////*/


	void run_python_script(const std::string& filepath) {
		// called with: python38 createGnatt.py "../JobShopSolutions/small_basic_sampleSol_testing.txt"
	}


	// inspired by https://towardsdatascience.com/gantt-charts-with-pythons-matplotlib-395b7af72d72
	void Utility::visualize(const std::string& sourceFolder, const std::string& sourceName,
		const std::string& outputFolder)
	{
		DLOG_F(WARNING, "called visualize, not implemented");

		// create thread for the visualization
		/*
		std::thread t(run_python_script, filepath);

		// Detach the thread so it can run independently
		t.detach();
		*/

	}


	/*
	#include <iostream>
	void printVecOfVec() {

		for (unsigned int i = 0; i < machineCnt; ++i) {
			unsigned int length = m_machineStepLists[i].size();
			for (unsigned int j = 0; j < length; ++j) {
				StepIdentifier c = m_machineStepLists[i][j];
				std::cout << "(" << c.taskId << ", " << c.stepIndex << "), ";
			}
			std::cout << std::endl;
		}
	}
	*/

}
