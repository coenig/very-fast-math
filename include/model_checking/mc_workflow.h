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
static const std::string DEFAULT_FILE_NAME_JSON_TEMPLATE { "envmodel_config.tpl.json" };
static const std::string FILE_NAME_ENVMODEL_ENTRANCE{ "EnvModel.tpl" };
static const std::string GUI_NAME{ "M²oRTy" };

static std::string getJsonFileNameFromJsonTemplateFileName(const std::string& json_tpl_filename)
{
   return StaticHelper::replaceAll(json_tpl_filename, ".tpl.json", ".json");
}

namespace mc{

class McWorkflow : public Failable
{
public:
   McWorkflow(const std::shared_ptr<DataPack> data, const std::shared_ptr<FormulaParser> parser);
   McWorkflow();

   void generateEnvmodels(
      const std::string& path_template,
      const std::string& json_tpl_filename,
      const std::string& envmodel_entrance_filename,
      const std::shared_ptr<std::mutex> formula_evaluation_mutex
      );

   std::vector<std::string> runMCJobs(
      const std::filesystem::path& path_generated_config_level,
      const std::function<bool(const std::string& folder)> job_selector,
      const std::string& path_template,
      const std::string& json_tpl_filename,
      const int num_threads
   );

   std::vector<std::string> runMCJobs(
      const std::filesystem::path& path_generated_config_level,
      const std::function<bool(const std::string& folder)> job_selector,
      const std::string& path_template,
      const std::string& json_tpl_filename,
      std::filesystem::file_time_type& previous_write_time,
      const std::shared_ptr<std::mutex> formula_evaluation_mutex,
      const int num_threads
   );

   void runMCJob(
      const std::filesystem::path& path_generated_config_level,
      const std::string& config_name,
      const std::string& path_template,
      const std::string& json_tpl_filename);

   void runMCJob(
      const std::filesystem::path& path_generated_config_level,
      const std::string& config_name,
      const std::string& path_template,
      const std::string& json_tpl_filename,
      std::filesystem::file_time_type& previous_write_time,
      const std::shared_ptr<std::mutex> formula_evaluation_mutex
   );

   static void createTestCase(
      const std::string& generated_parent_dir,
      const std::string& id,
      const std::map<std::string, std::string>& modes);

   void createTestCases(
      const std::map<std::string, std::string>& modes, 
      const std::string& path_template,
      const std::string& json_tpl_filename,
      const std::vector<std::string>& sec_ids);

   nlohmann::json instanceFromTemplate(
      const nlohmann::json& j_template,
      const std::vector<std::pair<std::string, std::vector<float>>>& ranges,
      const std::vector<int>& counter_vec,
      const bool is_ltl);

   void deleteMCOutputFromFolder(
      const std::filesystem::path& path_generated,
      const bool actually_delete_gif,
      const std::string& path_template,
      std::filesystem::file_time_type& previous_write_time
   ); // ...or otherwise copy "waiting for" image.

   void preprocessAndRewriteJSONTemplate(
      const std::string& path_template, 
      const std::string& json_tpl_filename, 
      const std::shared_ptr<std::mutex> formula_evaluation_mutex);

   bool putJSONIntoDataPack(
      const std::string& path_template, 
      const std::string& filename_json_template, // Resist the urge to provide the plain json path. It's always the tpl path which is automatically tranferred if necessary.
      const std::string& config_name = JSON_TEMPLATE_DENOTER);

   void generatePreview(const std::filesystem::path& path_generated_config_level, const int cex_num);
   void evaluateFormulasInJSON(const nlohmann::json j_template, const std::shared_ptr<std::mutex> formula_evaluation_mutex);
   nlohmann::json getJSON(const std::string& path) const;

   void copyWaitingForPreviewGIF(
      const std::filesystem::path& path_generated,
      const std::filesystem::path& path_template,
      std::filesystem::file_time_type& previous_write_time
      );

   void resetParserAndData();
   void resetParserAndData(const std::shared_ptr<DataPack> data, const std::shared_ptr<FormulaParser> parser);
   
   std::string getValueForJSONKeyAsString(
      const std::string& key_to_find, 
      const std::string& path_template, 
      const std::string& filename_json_template, // Resist the urge to provide the plain json path. It's always the tpl path which is automatically tranferred if necessary.
      const std::string& config_name) const;

   std::string getValueForJSONKeyAsStringPlain(const std::string& key_to_find, const nlohmann::json& json, const std::string& config_name) const;
   bool isLTL(const std::string& config, const std::string& path_template, const std::string& filename_json_template);
   std::filesystem::path getCachedDir(const std::string& path_template, const std::string& filename_json_template) const;
   std::filesystem::path getBPIncludesFileDir(const std::string& path_template, const std::string& filename_json_template) const;
   std::filesystem::path getGeneratedDir(const std::string& path_template, const std::string& filename_json_template) const;
   std::filesystem::path getExternalDir(const std::string& path_template, const std::string& filename_json_template) const;
   std::filesystem::path getGeneratedParentDir(const std::string& path_template, const std::string& filename_json_template) const;

   std::shared_ptr<DataPack> data_{}; // TODO: make private again.
   std::shared_ptr<FormulaParser> parser_{}; // TODO: make private again.

private:
   std::mutex main_file_mutex_{};
};

} // mc
} // vfm