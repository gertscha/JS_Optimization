#include "FileCollector.h"

#include <filesystem>


namespace JSOptimizer::Utility
{

  FileCollector::FileCollector(std::string root_path, std::string folder_name)
  {
    namespace fs = std::filesystem;
    files_ = std::vector<std::string>();
    if (fs::is_directory(root_path + folder_name))
    {
      for (const auto& entry : fs::recursive_directory_iterator(root_path + folder_name))
      {
        if (entry.is_regular_file() && !entry.is_symlink()
          && entry.path().extension() == ".txt"
        )
        {
          std::string full_path = entry.path().string();
          size_t pos = full_path.find(folder_name);
          full_path.erase(0, pos);
          for (char& c : full_path)
          {
            if (c == '\\')
              c = '/';
          }
          files_.push_back(full_path);
        }
      }
    }
  }


}
