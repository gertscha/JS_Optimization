#include "PyVisualize.h"

#ifdef _DEBUG
#define Py_DEBUG
#endif

#define PY_SSIZE_T_CLEAN
#include <errno.h>
#include "Python.h"

#include <string>
#include <thread>

#include "loguru.hpp"

#include "Solution.h"


namespace JSOptimizer {

  namespace Utility {


    /*/
     *  Visualization only works in Release mode because there are some
     *  linking problems in Debug mode. The debug libs for numpy seem to
     *  be inaccessible with the current project setup
     *  This is something that I would like to fix, but it is a low priority
    /*/


    // function to run in a new thread
    void run_python_script(const std::vector<std::string>& args) {
      // called with: python createGnattFromFile.py "../JobShopSolutions/SmallTestingSolution.txt"

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
      FILE* script_file;
      errno_t error = fopen_s(&script_file, scriptPath.c_str(), "r");
      if (error == 0) {
        PyRun_AnyFile(script_file, scriptPath.c_str());
        fclose(script_file);
      }
      else {
        char buf[100];
        strerror_s(buf, 100, error);
        ABORT_F("Failed to open 'createGnattFromFile.py': %s", buf);
      }

      // Cleanup command-line arguments
      for (int i = 0; i < argc; ++i) {
        PyMem_RawFree(argv[i]);
      }
      delete[] argv;

      Py_Finalize();
    }

    
    void visualize(const std::string& path, const std::string& filename, bool new_thread)
    {
#ifdef _DEBUG
      LOG_F(WARNING, "visualize only available in Release build");
#else
      std::string solutionPath = path + filename;
      std::vector<std::string> args = { "--arg1", solutionPath };
      if (new_thread) {
        g_VisualizationManager.addThread(new std::thread(run_python_script, args));;
      }
      else {
        run_python_script(args);
      }
#endif
    }


    void visualize(const Solution& solution, bool new_thread)
    {
#ifdef _DEBUG
      LOG_F(WARNING, "visualize only available in Release build");
#else
      std::string filename = "tmp/vis_sol.txt";
      solution.SaveToFile(g_visualizations_out_path, filename, true);
      std::string sol_path = g_visualizations_out_path + filename;
      std::vector<std::string> args = { "--arg1", sol_path };
      if (new_thread) {
        g_VisualizationManager.addThread(new std::thread(run_python_script, args));
      }
      else {
        run_python_script(args);
      }
#endif
    }


  }

}
