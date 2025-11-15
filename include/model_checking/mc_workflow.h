//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2025 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file
#pragma once

#include "failable.h"
#include "parser.h"
#include "data_pack.h"
#include "static_helper.h"
#include "json_parsing/json.hpp"

#include <filesystem>

namespace vfm
{
static const std::string PROSE_DESC_NAME{ "prose_scenario_description.txt" };
static const std::string FILE_NAME_JSON{ "envmodel_config.json" };
static const std::string FILE_NAME_JSON_TEMPLATE { "envmodel_config.tpl.json" };

namespace mc{

class McWorkflow : public Failable
{
public:
   McWorkflow();

   std::vector<std::string> runMCJobs(
      const std::filesystem::path& working_dir, 
      const std::function<bool(const std::string& folder)> job_selector, 
      const std::string& path_template,
      const std::string& path_cached,
      const std::string& path_external,
      const std::string& json_tpl_filename,
      std::filesystem::file_time_type& previous_write_time,
      std::shared_ptr<std::mutex> formula_evaluation_mutex
   );

   void runMCJob(
      const std::string& path_generated_raw, 
      const std::string& config_name, 
      const std::string& path_template,
      const std::string& path_cached,
      const std::string& path_external,
      const std::string& json_tpl_filename,
      std::filesystem::file_time_type& previous_write_time,
      std::shared_ptr<std::mutex> formula_evaluation_mutex
   );

   nlohmann::json instanceFromTemplate(
      const nlohmann::json& j_template,
      const std::vector<std::pair<std::string, std::vector<float>>>& ranges,
      const std::vector<int>& counter_vec,
      const bool is_ltl);

   void deleteMCOutputFromFolder(
      const std::string& path_generated, 
      const bool actually_delete_gif,
      const std::string& path_template,
      std::filesystem::file_time_type& previous_write_time
   ); // ...or otherwise copy "waiting for" image.

   void preprocessAndRewriteJSONTemplate(
      const std::string& path_template, 
      const std::string& json_tpl_filename, 
      std::shared_ptr<std::mutex> formula_evaluation_mutex);

   bool putJSONIntoDataPack(const std::string& path_template, const std::string& config_name = JSON_TEMPLATE_DENOTER);
   void generatePreview(const std::string& path_generated, const int cex_num);
   void evaluateFormulasInJSON(const nlohmann::json j_template, std::shared_ptr<std::mutex> formula_evaluation_mutex);
   nlohmann::json getJSON(const std::string& path) const;

   void copyWaitingForPreviewGIF(
      const std::string& path_template,
      const std::string& path_generated,
      std::filesystem::file_time_type& previous_write_time
      );

   void resetParserAndData();

   std::shared_ptr<DataPack> data_{}; // TODO: make private again.
   std::shared_ptr<FormulaParser> parser_{}; // TODO: make private again.

private:
   std::mutex main_file_mutex_{};
};

} // mc
} // vfm