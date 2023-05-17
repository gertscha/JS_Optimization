#ifndef UTILITY_PYVISUALIZE_H_
#define UTILITY_PYVISUALIZE_H_

#include <string>

#include "ThreadManager.h"

namespace JSOptimizer {
  
  extern std::string g_VSsol_path;
  extern std::string g_python_path;
  extern ThreadManager g_VisualizationManager;

  class Solution;

  namespace Utility {


    /*////////////////////
        Visualization
    ////////////////////*/


    // inspired by https://towardsdatascience.com/gantt-charts-with-pythons-matplotlib-395b7af72d72
    void visualize(const std::string& path_to_solution, const std::string& file_name);

    void visualize(const Solution& solution);


  }

}

#endif // UTILITY_PYVISUALIZE_H_
