//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2025 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file

#include "model_checking/mc_workflow.h"
#include "static_helper.h"
#include "testing/interactive_testing.h"
#include "vfmacro/script.h"

using namespace vfm;
using namespace mc;

McWorkflow::McWorkflow(
   const std::shared_ptr<DataPack> data,
   const std::shared_ptr<FormulaParser> parser) : Failable("McWorkflow"), data_{ data }, parser_{ parser }
{
}

std::vector<std::string> McWorkflow::runMCJobs(
   const std::filesystem::path& working_dir, 
   const std::function<bool(const std::string& folder)> job_selector, 
   const std::string& path_template)
{
   std::vector<std::string> possibles{};
   std::string prefix{ working_dir.filename().string() };

   for (const auto& entry : std::filesystem::directory_iterator(working_dir)) {
      std::string possible{ entry.path().filename().string() };
      if (std::filesystem::is_directory(entry) && possible != prefix && StaticHelper::stringStartsWith(possible, prefix)) {
         possibles.push_back(entry.path().string());
      }
   }

   { // Scope which makes us wait for finishing the jobs before entering the next line.
      ThreadPool pool(test::MAX_THREADS);

      for (const auto& folder : possibles) {
         std::this_thread::sleep_for(std::chrono::seconds(1));

         const std::string config_name{ std::filesystem::path(folder).filename().string() };

         if (job_selector(config_name)) {
            pool.enqueue([this, folder, working_dir, prefix, path_template] {
               runMCJob(folder, folder.substr(working_dir.string().size() + prefix.size() + 1), path_template);
            });
         }
      }
   }

   return possibles;
}

void McWorkflow::runMCJob(const std::string& path_generated_raw, const std::string& config_name, const std::string& path_template, const std::string& json_tpl_filename)
{
   std::string path_generated{ StaticHelper::replaceAll(path_generated_raw, "\\", "/") };

   {
      std::lock_guard<std::mutex> lock{ main_file_mutex_ };

      addNote("Running model checker and creating preview for folder '" + path_generated + "' (config: '" + config_name + "').");
      deleteMCOutputFromFolder(path_generated, true);
      preprocessAndRewriteJSONTemplate(path_template, json_tpl_filename);

      if (!mc_scene->putJSONIntoDataPack()) return;
      if (!mc_scene->putJSONIntoDataPack(config_name)) return;

      std::string main_smv{ StaticHelper::readFile(path_generated + "/main.smv") };

      std::string script_template{ StaticHelper::readFile(mc_scene->getTemplateDir() + "/script.tpl") };
      std::string main_template{ StaticHelper::readFile(mc_scene->getTemplateDir() + "/main.tpl") };
      std::string generated_script{ CppParser::generateScript(script_template, mc_scene->data_, mc_scene->parser_) };
      StaticHelper::writeTextToFile(generated_script, path_generated + "/script.cmd");

      addNote("Created script.cmd with the following content:\n" + StaticHelper::readFile(path_generated + "/script.cmd") + "<EOF>");

      static const std::string SPEC_BEGIN{ "--SPEC-STUFF" };
      static const std::string SPEC_END{ "--EO-SPEC-STUFF" };
      static const std::string ADDONS_BEGIN{ "--ADDONS" };
      static const std::string ADDONS_END{ "--EO-ADDONS" };
      data_->addStringToDataPack(mc_scene->getTemplateDir(), macro::MY_PATH_VARNAME); // Set the script processors home path (for the case it's not already been set during EnvModel generation).

      // Re-generate SPEC stuff.
      main_smv = StaticHelper::removeMultiLineComments(main_smv, SPEC_BEGIN, SPEC_END);
      main_smv += SPEC_BEGIN + "\n";

      auto spec_part = StaticHelper::removePartsOutsideOf(main_template, SPEC_BEGIN, SPEC_END);
      data_->addStringToDataPack(path_generated, "FULL_GEN_PATH");
      spec_part = vfm::macro::Script::processScript(spec_part, macro::Script::DataPreparation::both, mc_scene->data_, mc_scene->parser_);
      main_smv += spec_part + "\n";
      main_smv += SPEC_END + "\n";

      StaticHelper::writeTextToFile(main_smv, path_generated + "/main.smv");
   }

   test::convenienceArtifactRunHardcoded(test::MCExecutionType::mc, path_generated, "FAKE_PATH_NOT_USED", path_template, "FAKE_PATH_NOT_USED", mc_scene->getCachedDir(), mc_scene->path_to_external_folder_);
   mc_scene->generatePreview(path_generated, 0);

   addNote("Model checker run finished for folder '" + path_generated + "'.");
}

void McWorkflow::deleteMCOutputFromFolder(const std::string& path_generated, const bool actually_delete_gif)
{
   //StaticHelper::removeFileSafe(path_generated + "/" + "main.smv"); // Do NOT remove main.smv. It only gets changed at this point.

   std::string path_result{ path_generated + "/debug_trace_array.txt" };
   std::string path_runtimes{ path_generated + "/mc_runtimes.txt" };
   std::string path_script{ path_generated + "/" + "script.cmd" };

   std::string path_prose_description{ path_generated + "/" + PROSE_DESC_NAME };
   std::string path_planner_smv{ path_generated + "/planner.cpp_combined.k2.smv" };
   std::string path_generated_preview{ path_generated + "/preview" };

   if (actually_delete_gif) {
      for (int i = 0; i < 100; i++) StaticHelper::removeAllFilesSafe(path_generated + "/" + std::to_string(i));
      addNote("Deleting folder '" + path_generated_preview + "'.");
      StaticHelper::removeAllFilesSafe(path_generated_preview);
   }
   else {
      addNote("Overwriting folder '" + path_generated_preview + "'.");
      copyWaitingForPreviewGIF();
   }

   if (std::filesystem::exists(path_prose_description)) {
      addNote("Deleting file '" + path_prose_description + "'.");
      StaticHelper::removeFileSafe(path_prose_description);
   }

   if (std::filesystem::exists(path_planner_smv)) {
      addNote("Deleting file '" + path_planner_smv + "'.");
      StaticHelper::removeFileSafe(path_planner_smv);
   }

   if (std::filesystem::exists(path_result)) {
      addNote("Deleting file '" + path_result + "'.");
      StaticHelper::removeFileSafe(path_result);
   }

   if (std::filesystem::exists(path_runtimes)) {
      addNote("Deleting file '" + path_runtimes + "'.");
      StaticHelper::removeFileSafe(path_runtimes);
   }

   if (std::filesystem::exists(path_script)) {
      addNote("Deleting file '" + path_script + "'.");
      StaticHelper::removeFileSafe(path_script);
   }

   for (auto& sec : se_controllers_) {
      sec.selected_ = false;
   }
}

void vfm::mc::McWorkflow::preprocessAndRewriteJSONTemplate(const std::string& path_template, const std::string& json_tpl_filename)
{
   const std::string path_json_template{ path_template + "/" + json_tpl_filename };
   const std::string path_json_plain{ path_template + "/" + FILE_NAME_JSON };

   std::string s{};

   nlohmann::json j_template = getJSON();
   nlohmann::json j_out = {};
   const bool is_ltl{ isLTL(JSON_TEMPLATE_DENOTER) };

   if (j_template.empty() || j_template.items().begin().key() != JSON_TEMPLATE_DENOTER) {
      addError("JSON parsing failed.");
   }

   evaluateFormulasInJSON(j_template);
   //addNote("This is the vfm heap after all the operations:\n" + data_->toStringHeap());

   addNote("Processing nuxmv formulas in JSON template: looking for variables and ranges.");
   std::set<std::string> variables{};

   for (auto& [key_config, value_config] : j_template.items()) {
      if (key_config == JSON_TEMPLATE_DENOTER) {
         for (auto& [key, value] : value_config.items()) {
            if (value.type() == nlohmann::detail::value_t::string) {
               std::string val_str{ StaticHelper::replaceAll(nlohmann::to_string(value), "\"", "") };

               if (StaticHelper::stringContains(val_str, JSON_NUXMV_FORMULA_BEGIN)) {
                  int begin{ StaticHelper::indexOfFirstInnermostBeginBracket(val_str, JSON_NUXMV_FORMULA_BEGIN, JSON_NUXMV_FORMULA_END) };
                  int end{ StaticHelper::findMatchingEndTagLevelwise(val_str, begin, JSON_NUXMV_FORMULA_BEGIN, JSON_NUXMV_FORMULA_END) };
                  std::string formula_str{ val_str };

                  StaticHelper::distributeGetOnlyInner(formula_str, begin + JSON_NUXMV_FORMULA_BEGIN.size(), end);
                  auto formula{ MathStruct::parseMathStruct(formula_str, parser_, data_, DotTreatment::as_operator) };

                  formula->applyToMeAndMyChildrenIterative([&variables, this](const MathStructPtr m)
                     {
                        auto m_var{ m->toVariableIfApplicable() };
                        if (m_var) {
                           auto m_var_name{ m_var->getVariableName() };
                           if (data_->isDeclared(m_var_name)) {
                              addNote("Found variable in nuXmv formula: " + m->serializeWithinSurroundingFormula());
                              variables.insert(m_var_name);
                           }
                        }
                     }, TraverseCompoundsType::avoid_compound_structures);
               }
            }
         }
      }
   }

   if (variables.empty()) { // Mock some range to get exactly one config.
      static const std::string DUMMY_RANGE_VAR_NAME{ "DUMMYVAR" };
      variables.insert(DUMMY_RANGE_VAR_NAME);
      int address{ data_->reserveHeap(2) };
      data_->addOrSetSingleVal(DUMMY_RANGE_VAR_NAME, address);
      data_->setHeapLocation(address, 0);
      data_->setHeapLocation(address + 1, std::numeric_limits<float>::quiet_NaN());
   }

   std::vector<std::pair<std::string, std::vector<float>>> ranges{};
   addNote("Recovering ranges from vfm heap.");
   for (const auto& var : variables) {
      addNote("Range for '" + var + "' is: ", true, "");
      ranges.push_back({ var, {} });
      int address{ (int)data_->getSingleVal(var) };

      while (!std::isnan(data_->getHeapLocation(address))) {
         ranges.back().second.push_back(data_->getHeapLocation(address));
         address++;
         addNotePlain(std::to_string(ranges.back().second.back()), " ");
      }

      addNotePlain("");
   }

   std::vector<int> arity_vec{};
   std::vector<int> counter_vec{};
   for (const auto& range : ranges) {
      arity_vec.push_back(range.second.size());
      counter_vec.push_back(range.second.size() - 1);
   }
   counter_vec.front()++;

   while (!VECTOR_COUNTER_IS_ALL_ZERO(counter_vec)) {
      VECTOR_COUNTER_DECREMENT(arity_vec, counter_vec);
      addNote("Working on ", true, "");
      for (int i = 0; i < counter_vec.size(); i++) {
         addNotePlain(std::to_string(ranges[i].second[counter_vec[i]]), " ");
      }
      addNotePlain("");

      nlohmann::json json_instance = instanceFromTemplate(j_template, ranges, counter_vec, is_ltl);

      j_out[json_instance.items().begin().key()] = json_instance.items().begin().value();
   }

   //std::cout << j_out.dump(3) << std::endl;

   std::ofstream json_file{};
   json_file.open(path_json_plain);
   json_file << j_out.dump(3);
   json_file.close();
}