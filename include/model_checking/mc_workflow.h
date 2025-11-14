//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2025 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file
#pragma once

#include "failable.h"
#include <filesystem>

namespace vfm
{
static const std::string PROSE_DESC_NAME{ "prose_scenario_description.txt" };

namespace mc{

class McWorkflow : public Failable
{
public:
   McWorkflow();

   std::vector<std::string> runMCJobs(
      const std::filesystem::path& working_dir, 
      const std::function<bool(const std::string& folder)> job_selector, 
      const std::string& path_template);

   void runMCJob(
      const std::string& path_generated_raw, 
      const std::string& config_name, 
      const std::string& path_template);

   void deleteMCOutputFromFolder(const std::string& path_generated, const bool actually_delete_gif); // ...or otherwise copy "waiting for" image.

private:
   std::mutex main_file_mutex_{};

};

} // mc
} // vfm