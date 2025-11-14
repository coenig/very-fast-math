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
static const std::string FILE_NAME_JSON{ "envmodel_config.json" };

namespace mc{

class McWorkflow : public Failable
{
public:
   McWorkflow(
      const std::shared_ptr<DataPack> data_,
      const std::shared_ptr<FormulaParser> parser);

   std::vector<std::string> runMCJobs(
      const std::filesystem::path& working_dir, 
      const std::function<bool(const std::string& folder)> job_selector, 
      const std::string& path_template, 
      const std::string& json_tpl_filename);

   void runMCJob(
      const std::string& path_generated_raw, 
      const std::string& config_name, 
      const std::string& path_template, 
      const std::string& json_tpl_filename);

   void deleteMCOutputFromFolder(const std::string& path_generated, const bool actually_delete_gif); // ...or otherwise copy "waiting for" image.
   void preprocessAndRewriteJSONTemplate(const std::string& path_template, const std::string& json_tpl_filename);
   bool putJSONIntoDataPack(const std::string& path_template, const std::string& json_config = JSON_TEMPLATE_DENOTER, const std::string& json_template_input = );

private:
   std::mutex main_file_mutex_{};
   std::shared_ptr<DataPack> data_{};
   std::shared_ptr<FormulaParser> parser_{};

};

} // mc
} // vfm