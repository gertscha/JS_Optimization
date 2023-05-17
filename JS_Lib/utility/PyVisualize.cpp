#include "PyVisualize.h"

#include <string>
#include <thread>

#include "Python.h"
#include "loguru.hpp"

#include "Solution.h"


namespace JSOptimizer {

  namespace Utility {


    /*////////////////////
        Visualization
    ////////////////////*/


    void run_python_script(const std::vector<std::string>& args) {
      // called with: python38 createGnatt.py "../JobShopSolutions/small_basic_sampleSol_testing.txt"

      // initialize the python instance
      Py_Initialize();

      // Set command-line arguments
      int argc = static_cast<int>(args.size());
      wchar_t** argv = new wchar_t* [argc];
      for (int i = 0; i < argc; ++i) {
        argv[i] = Py_DecodeLocale(args[i].c_str(), nullptr);
      }
      PySys_SetArgvEx(argc, argv, 0);

      // Run the script
      std::string scriptPath = g_python_path + "createGnattFromFile.py";
      FILE* script_file = fopen(scriptPath.c_str(), "r");
      if (script_file) {
        PyRun_AnyFile(script_file, scriptPath.c_str());
        fclose(script_file);
      }

      // Cleanup command-line arguments
      for (int i = 0; i < argc; ++i) {
        PyMem_RawFree(argv[i]);
      }
      delete[] argv;

      Py_Finalize();
    }

    
    void Utility::visualize(const std::string& path, const std::string& file_name)
    {
      
#ifdef _DEBUG
      LOG_F(WARNING, "visualize only executed if in Release mode");
#else
      std::string solutionPath = path + file_name;
      std::vector<std::string> args = { "--arg1", solutionPath };

      g_VisualizationManager.addThread(new std::thread(run_python_script, args));
      
      //std::thread pythonThread(run_python_script, args);
      //pythonThread.join();
#endif
    }

    void visualize(const Solution& solution)
    {
      
      LOG_F(WARNING, "visualize not implemented for solutions, save it first and visualize from file");

    }




  }

}
