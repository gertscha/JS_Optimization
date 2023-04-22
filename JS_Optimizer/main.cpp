#include "Problem.h"
#include "Task.h"
#include "Optimizer.h"
#include "Solution.h"

#include "loguru.hpp"

#include <iostream>


/*
* helper functions
*/
namespace SimAnn {

	void test() {
		std::cout << "test" << std::endl;;
	}

}


int main() {
	using namespace SimAnn;
	loguru::add_file("logs/latest.log", loguru::Truncate, loguru::Verbosity_INFO);
	loguru::add_file("logs/error.log", loguru::Truncate, loguru::Verbosity_ERROR);
	LOG_F(INFO, "Started Execution");
	
	std::string problemPath = "D:/Dev/C++/SimulatedAnnealing/SimAnn/Optimizer/Problems/";
	std::string solutionPath = "D:/Dev/C++/SimulatedAnnealing/SimAnn/Optimizer/Solutions/";
	std::string visPath = "D:/Dev/C++/SimulatedAnnealing/SimAnn/Optimizer/Visualizations/";

	Problem p_sb(problemPath, "small_basic.txt");

	Solution s_sb(solutionPath, "small_basic_SampleSol_testing_improved.txt");
	
	long solTime = s_sb.getCompletetionTime();

	s_sb.saveToFile(solutionPath, "small_basic_SampleSol_saved.txt");

	if (s_sb.validateSolution(p_sb)) {
		LOG_F(INFO, "p_sb is solved by s_sb");
	}
	else {
		LOG_F(INFO, "p_sb is *not* solved by s_sb");
	}
	
	std::cout << "Completion time of solution is: " << solTime;
	std::cout << ", the lower bound is:" << p_sb.getBounds().getLowerBound() << std::endl;
	std::cout << "mbound " << p_sb.getBounds().MachineLowerBound << ", tbound " << p_sb.getBounds().TaskLowerBound << std::endl;

	std::cout << p_sb;
	//std::cout << std::endl;
	std::cout << s_sb;


	LOG_F(INFO, "Creating visualization...");

	s_sb.visualize(visPath);

	test();

	LOG_F(INFO, "Finished Execution");
	return 0;
}
