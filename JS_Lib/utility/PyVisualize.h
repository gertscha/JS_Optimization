#ifndef UTILITY_PYVISUALIZE_H_
#define UTILITY_PYVISUALIZE_H_

#include <string>

#include "ThreadManager.h"


namespace JSOptimizer {
  
  extern std::string g_python_path;
  extern std::string g_visualizations_out_path;
  extern ThreadManager g_VisualizationManager;

  class Solution;

  namespace Utility {

    // runs createGnattFromFile.py on the file
    // if new_thread is false, this call will block until the visualization window is closed
    // uses g_VisualizationManager to manage new threads
    void visualize(const std::string& path_to_solution, const std::string& filename, bool new_thread);

    // saves the solution to a temporary file, and runs createGnattFromFile.py on it
    // if new_thread is false, this call will block until the visualization window is closed
    // uses g_VisualizationManager to manage new threads
    void visualize(const Solution& solution, bool new_thread);

  }

}

#endif // UTILITY_PYVISUALIZE_H_
