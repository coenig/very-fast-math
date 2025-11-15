//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2025 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file

#include "model_checking/mc_workflow.h"
#include "testing/interactive_testing.h"
#include "vfmacro/script.h"

using namespace vfm;
using namespace mc;

McWorkflow::McWorkflow() : McWorkflow(nullptr, nullptr)
{}

McWorkflow::McWorkflow(const std::shared_ptr<DataPack> data, const std::shared_ptr<FormulaParser> parser) : Failable("McWorkflow")
{
   resetParserAndData(data, parser);
}

void vfm::mc::McWorkflow::resetParserAndData()
{
   resetParserAndData(nullptr, nullptr);
}

void McWorkflow::resetParserAndData(const std::shared_ptr<DataPack> data, const std::shared_ptr<FormulaParser> parser)
{
   if (data) {
      data_ = data;
   }
   else {
      data_ = std::make_shared<DataPack>();
   }

   if (parser) {
      parser_ = parser;
   } else {
      parser_ = std::make_shared<FormulaParser>();
      parser_->addDefaultDynamicTerms();
      parser_->addnuSMVOperators(TLType::LTL);
   }
}

std::vector<std::string> vfm::mc::McWorkflow::runMCJobs(
   const std::filesystem::path& path_generated, 
   const std::function<bool(const std::string& folder)> job_selector, 
   const std::string& path_template, 
   const std::string& path_cached, 
   const std::string& path_external, 
   const std::string& json_tpl_filename, 
   const int num_threads)
{
   return runMCJobs(
      path_generated, 
      job_selector, 
      path_template, 
      path_cached, 
      path_external, 
      json_tpl_filename,
      std::filesystem::file_time_type{},
      nullptr, num_threads);
}

std::vector<std::string> McWorkflow::runMCJobs(
   const std::filesystem::path& path_generated, 
   const std::function<bool(const std::string& folder)> job_selector, 
   const std::string& path_template,
   const std::string& path_cached,
   const std::string& path_external,
   const std::string& json_tpl_filename,
   std::filesystem::file_time_type& previous_write_time,
   std::shared_ptr<std::mutex> formula_evaluation_mutex,
   const int num_threads)
{
   std::filesystem::path path_generated_parent = path_generated.parent_path();
   std::vector<std::string> possibles{};
   std::string prefix{ path_generated.filename().string() };

   for (const auto& entry : std::filesystem::directory_iterator(path_generated_parent)) {
      std::string possible{ entry.path().filename().string() };
      if (std::filesystem::is_directory(entry) && possible != prefix && StaticHelper::stringStartsWith(possible, prefix)) {
         possibles.push_back(entry.path().string());
      }
   }

   { // Scope which makes us wait for finishing the jobs before entering the next line.
      ThreadPool pool(num_threads);

      for (const auto& folder : possibles) {
         std::this_thread::sleep_for(std::chrono::seconds(1));

         const std::string config_name{ std::filesystem::path(folder).filename().string() };

         if (job_selector(config_name)) {
            pool.enqueue([this, &folder, &path_generated_parent, &prefix, &path_template, &json_tpl_filename, &path_cached, &path_external, &previous_write_time, formula_evaluation_mutex] {
               runMCJob(folder, folder.substr(path_generated_parent.string().size() + prefix.size() + 1), path_template, path_cached, path_external, json_tpl_filename, previous_write_time, formula_evaluation_mutex);
            });
         }
      }
   }

   return possibles;
}

void McWorkflow::runMCJob(
   const std::string& path_generated_raw, 
   const std::string& config_name, 
   const std::string& path_template, 
   const std::string& path_cached, 
   const std::string& path_external, 
   const std::string& json_tpl_filename)
{
   runMCJob(
      path_generated_raw,
      config_name,
      path_template,
      path_cached,
      path_external,
      json_tpl_filename,
      std::filesystem::file_time_type{},
      nullptr);
}

void McWorkflow::runMCJob(
   const std::string& path_generated_raw, 
   const std::string& config_name, 
   const std::string& path_template,
   const std::string& path_cached,
   const std::string& path_external,
   const std::string& json_tpl_filename,
   std::filesystem::file_time_type& previous_write_time, 
   std::shared_ptr<std::mutex> formula_evaluation_mutex
)
{
   std::string path_generated{ StaticHelper::replaceAll(path_generated_raw, "\\", "/") };

   {
      std::lock_guard<std::mutex> lock{ main_file_mutex_ };

      addNote("Running model checker and creating preview for folder '" + path_generated + "' (config: '" + config_name + "').");
      deleteMCOutputFromFolder(path_generated, true, path_template, previous_write_time);
      preprocessAndRewriteJSONTemplate(path_template, json_tpl_filename, formula_evaluation_mutex);

      if (!putJSONIntoDataPack(path_template)) return;
      if (!putJSONIntoDataPack(path_template, config_name)) return;

      std::string main_smv{ StaticHelper::readFile(path_generated + "/main.smv") };

      std::string script_template{ StaticHelper::readFile(path_template + "/script.tpl") };
      std::string main_template{ StaticHelper::readFile(path_template + "/main.tpl") };
      std::string generated_script{ CppParser::generateScript(script_template, data_, parser_) };
      StaticHelper::writeTextToFile(generated_script, path_generated + "/script.cmd");

      addNote("Created script.cmd with the following content:\n" + StaticHelper::readFile(path_generated + "/script.cmd") + "<EOF>");

      static const std::string SPEC_BEGIN{ "--SPEC-STUFF" };
      static const std::string SPEC_END{ "--EO-SPEC-STUFF" };
      static const std::string ADDONS_BEGIN{ "--ADDONS" };
      static const std::string ADDONS_END{ "--EO-ADDONS" };
      data_->addStringToDataPack(path_template, macro::MY_PATH_VARNAME); // Set the script processors home path (for the case it's not already been set during EnvModel generation).

      // Re-generate SPEC stuff.
      main_smv = StaticHelper::removeMultiLineComments(main_smv, SPEC_BEGIN, SPEC_END);
      main_smv += SPEC_BEGIN + "\n";

      auto spec_part = StaticHelper::removePartsOutsideOf(main_template, SPEC_BEGIN, SPEC_END);
      data_->addStringToDataPack(path_generated, "FULL_GEN_PATH");
      spec_part = vfm::macro::Script::processScript(spec_part, macro::Script::DataPreparation::both, data_, parser_);
      main_smv += spec_part + "\n";
      main_smv += SPEC_END + "\n";

      StaticHelper::writeTextToFile(main_smv, path_generated + "/main.smv");
   }

   test::convenienceArtifactRunHardcoded(test::MCExecutionType::mc, path_generated, "FAKE_PATH_NOT_USED", path_template, "FAKE_PATH_NOT_USED", path_cached, path_external);
   generatePreview(path_generated, 0);

   addNote("Model checker run finished for folder '" + path_generated + "'.");
}

void McWorkflow::generatePreview(const std::string& path_generated, const int cex_num)
{
   addNote("Generating preview for folder '" + path_generated + "'.");

   mc::trajectory_generator::VisualizationLaunchers::quickGeneratePreviewGIFs(
      { cex_num },
      path_generated,
      "debug_trace_array",
      mc::trajectory_generator::CexTypeEnum::smv);

   //refreshPreview(); // TODO: Function not taken over. Do we need it here?
}

void McWorkflow::deleteMCOutputFromFolder(
   const std::string& path_generated, 
   const bool actually_delete_gif,
   const std::string& path_template,
   std::filesystem::file_time_type& previous_write_time
)
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
      copyWaitingForPreviewGIF(path_template, path_generated, previous_write_time);
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
}

nlohmann::json McWorkflow::getJSON(const std::string& path) const
{
   std::string json_text{ StaticHelper::readFile(path) };
   nlohmann::json json{};

   try {
      json = nlohmann::json::parse(json_text);
   }
   catch (const nlohmann::json::parse_error& e) {
      addError("Json file not loaded from '" + path + "'.");
   }

   return json;
}

void McWorkflow::preprocessAndRewriteJSONTemplate(
   const std::string& path_template, 
   const std::string& json_tpl_filename,
   std::shared_ptr<std::mutex> formula_evaluation_mutex)
{
   const std::string path_json_template{ path_template + "/" + json_tpl_filename };
   const std::string path_json_plain{ path_template + "/" + FILE_NAME_JSON };

   std::string s{};

   nlohmann::json j_template = getJSON(path_json_template);
   nlohmann::json j_out = {};
   const bool is_ltl{ isLTL(JSON_TEMPLATE_DENOTER, path_template) };

   if (j_template.empty() || j_template.items().begin().key() != JSON_TEMPLATE_DENOTER) {
      addError("JSON parsing failed.");
   }

   evaluateFormulasInJSON(j_template, formula_evaluation_mutex);
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
      //addNote("Working on ", true, "");
      //for (int i = 0; i < counter_vec.size(); i++) {
      //   addNotePlain(std::to_string(ranges[i].second[counter_vec[i]]), " ");
      //}
      //addNotePlain("");

      nlohmann::json json_instance = instanceFromTemplate(j_template, ranges, counter_vec, is_ltl);

      j_out[json_instance.items().begin().key()] = json_instance.items().begin().value();
   }

   //std::cout << j_out.dump(3) << std::endl;

   std::ofstream json_file{};
   json_file.open(path_json_plain);
   json_file << j_out.dump(3);
   json_file.close();
}


void McWorkflow::evaluateFormulasInJSON(const nlohmann::json j_template, std::shared_ptr<std::mutex> formula_evaluation_mutex)
{
   auto lock = (formula_evaluation_mutex == nullptr) ?
      std::unique_lock<std::mutex>()
      : std::unique_lock<std::mutex>(*formula_evaluation_mutex);

   parser_ = std::make_shared<FormulaParser>();
   parser_->addDefaultDynamicTerms();

   for (auto& [key_config, value_config] : j_template.items()) {
      if (key_config == JSON_TEMPLATE_DENOTER) {
         for (auto& [key, value] : value_config.items()) {
            if (StaticHelper::stringStartsWith(key, JSON_VFM_FORMULA_PREFIX)) {
               std::string val_str{ StaticHelper::replaceAll(nlohmann::to_string(value), "\"", "") };
               parser_->resetErrors(ErrorLevelEnum::error);
               TermPtr formula{ _val0() };

               try {
                  formula = MathStruct::parseMathStruct(val_str, parser_, data_, DotTreatment::as_operator)->toTermIfApplicable();
               }
               catch (const std::exception& e) {
                  parser_->addError("Parsing of formula '" + val_str + "' raised an unhandled error.");
               }

               if (parser_->hasErrorOccurred()) {
                  if (StaticHelper::stringContains(val_str, "@f")) {
                     int fct_name_begin_index{ StaticHelper::indexOfFirstInnermostBeginBracket(val_str, "(", ")") };
                     int fct_name_end_index{ StaticHelper::findMatchingEndTagLevelwise(val_str, fct_name_begin_index, "(", ")") };
                     int fct_pars_begin_index{ StaticHelper::indexOfFirstInnermostBeginBracket(val_str, "{", "}") };
                     int fct_pars_end_index{ StaticHelper::findMatchingEndTagLevelwise(val_str, fct_pars_begin_index, "{", "}") };
                     std::string fct_name{ val_str.substr(fct_name_begin_index + 1, fct_name_end_index - fct_name_begin_index - 1) };
                     std::string fct_pars{ val_str.substr(fct_pars_begin_index + 1, fct_pars_end_index - fct_pars_begin_index - 1) };

                     StaticHelper::trim(fct_pars);
                     int arg_num{ fct_pars.empty() ? 0 : (int)(fct_pars.size() - StaticHelper::replaceAll(fct_pars, ",", "").size() + 1) };

                     parser_->addDynamicTerm(_var("#ERROR"), fct_name, false, arg_num);
                  }
               }

               float result{ formula->eval(data_, parser_) };
            }
         }
      }
   }
}

bool McWorkflow::putJSONIntoDataPack(const std::string& path_template, const std::string& config_name)
{
   bool from_template{ config_name == JSON_TEMPLATE_DENOTER };
   bool config_valid{ false };

   try {
      nlohmann::json j = nlohmann::json::parse(StaticHelper::readFile(path_template + "/" + (from_template ? FILE_NAME_JSON_TEMPLATE : FILE_NAME_JSON)));

      for (auto& [key_config, value_config] : j.items()) {
         if (key_config == config_name) {
            config_valid = true;

            for (auto& [key, value] : value_config.items()) {
               if (value.is_string()) {
                  std::string val_str = StaticHelper::replaceAll(nlohmann::to_string(value), "\"", "");
                  data_->addStringToDataPack(val_str, key);
               }
               else {
                  data_->addOrSetSingleVal(key, value);
               }
            }
         }
      }
   }
   catch (const nlohmann::json::parse_error& e) {
      //addError("#JSON Error in 'getValueForJSONKeyAsString' (key: '" + key_to_find + "', config: '" + config_name + "', from_template: " + std::to_string(from_template) + ").");
   }
   catch (...) {
      //addError("#JSON Error in 'getValueForJSONKeyAsString' (key: '" + key_to_find + "', config: '" + config_name + "', from_template: " + std::to_string(from_template) + ").");
   }

   if (!config_valid) addError("Config '" + config_name + "' not found in JSON. (Did you rename the folders by hand? You're not supposed to do that.)");

   return config_valid;
}


nlohmann::json McWorkflow::instanceFromTemplate(
   const nlohmann::json& j_template,
   const std::vector<std::pair<std::string, std::vector<float>>>& ranges,
   const std::vector<int>& counter_vec,
   const bool is_ltl)
{
   std::string name{ "_config" };

   for (int i = 0; i < counter_vec.size(); i++) {
      name += "_" + ranges[i].first + "=" + StaticHelper::floatToStringNoTrailingZeros(ranges[i].second[counter_vec[i]]);
   }

   nlohmann::json res = { { name.c_str(), {} } };

   for (auto& [key_config, value_config] : j_template.items()) {
      if (key_config == JSON_TEMPLATE_DENOTER) {
         for (auto& [key, value] : value_config.items()) {
            if (!StaticHelper::stringStartsWith(key, JSON_VFM_FORMULA_PREFIX)) {
               if (value.type() == nlohmann::detail::value_t::string) {
                  std::string val_str{ StaticHelper::replaceAll(nlohmann::to_string(value), "\"", "") };

                  if (StaticHelper::stringContains(val_str, JSON_NUXMV_FORMULA_BEGIN)) {
                     int begin{ StaticHelper::indexOfFirstInnermostBeginBracket(val_str, JSON_NUXMV_FORMULA_BEGIN, JSON_NUXMV_FORMULA_END) };
                     int end{ StaticHelper::findMatchingEndTagLevelwise(val_str, begin, JSON_NUXMV_FORMULA_BEGIN, JSON_NUXMV_FORMULA_END) };
                     std::string formula_str{ val_str };
                     std::string before{};
                     std::string after{};

                     StaticHelper::distributeIntoBeforeInnerAfter(before, formula_str, after, begin + JSON_NUXMV_FORMULA_BEGIN.size(), end);
                     auto formula{ _id(MathStruct::parseMathStruct(formula_str, parser_, data_, DotTreatment::as_operator)->toTermIfApplicable()) };

                     formula->applyToMeAndMyChildrenIterative([&ranges, &counter_vec, this](const MathStructPtr m)
                        {
                           auto m_var{ m->toVariableIfApplicable() };
                           if (m_var) {
                              auto m_var_name{ m_var->getVariableName() };

                              for (int i = 0; i < ranges.size(); i++) {
                                 if (ranges[i].first == m_var_name) {
                                    m_var->replaceJumpOverCompounds(_val(ranges[i].second[counter_vec[i]]));
                                 }
                              }
                           }
                        }, TraverseCompoundsType::avoid_compound_structures);

                     const std::string before_without_bracket{ before.substr(0, before.size() - JSON_NUXMV_FORMULA_BEGIN.size()) };
                     const std::string after_without_bracket{ after.substr(JSON_NUXMV_FORMULA_END.size()) };
                     const std::string new_inner{ formula->child0()->isTermVal()
                        ? formula->child0()->serializePlainOldVFMStyle(MathStruct::SerializationSpecial::enforce_square_array_brackets)    // If it's only a number, use regular serialization since the nusmv one casts to int.
                        : formula->child0()->serializeNuSMV(data_, parser_) // For larger formulas, print in nusmv style since it goes directly to the model checker.
                     };

                     val_str =
                        before_without_bracket
                        + new_inner
                        + after_without_bracket;

                     if (StaticHelper::stringStartsWith(key, "SPEC")) {
                        val_str = std::string(is_ltl ? "LTLSPEC " : "INVARSPEC ") + val_str + ";";
                     }
                  }

                  // Special treatment: check if format is "-(NUM)" for some float NUM and replace with "-NUM".
                  // TODO: Should vfm serialize negative numbers - or all negative terms - to "-NUM" right away?
                  std::string val_str_tmp{ StaticHelper::removeWhiteSpace(val_str) };
                  if (StaticHelper::stringStartsWith(val_str_tmp, "-(") && StaticHelper::stringEndsWith(val_str_tmp, ")")) {
                     std::string middle_part = val_str_tmp.substr(2);
                     middle_part = middle_part.substr(0, middle_part.size() - 1);
                     if (StaticHelper::isParsableAsFloat(middle_part)) {
                        val_str = "-" + middle_part;
                     }
                  }

                  if (StaticHelper::isParsableAsFloat(val_str)) {
                     float num{ std::stof(val_str) };
                     if (StaticHelper::isFloatInteger(num)) {
                        res.begin().value()[key] = (int)num;
                     }
                     else {
                        res.begin().value()[key] = num;
                     }
                  }
                  else {
                     res.begin().value()[key] = val_str;
                  }
               }
               else {
                  res.begin().value()[key] = value;
               }
            }
         }
      }
   }

   return res;
}


void McWorkflow::copyWaitingForPreviewGIF(
   const std::string& path_template,
   const std::string& path_generated,
   std::filesystem::file_time_type& previous_write_time
) {
   std::string path_preview_template{ path_template + "/preview.img" };
   std::string path_preview_target{ path_generated + "/preview/preview.gif" };
   StaticHelper::createDirectoriesSafe(path_generated + "/preview", false);

   if (StaticHelper::existsFileSafe(path_preview_target, false)) {
      StaticHelper::removeFileSafe(path_preview_target, false);
   }

   try {
      std::filesystem::copy(path_preview_template, path_preview_target);
   }
   catch (const std::exception& e) {
      addWarning("Directory '" + StaticHelper::absPath(path_preview_template) + "' not copied to '" + StaticHelper::absPath(path_preview_target) + "'. Error: " + e.what());
   }

   previous_write_time = StaticHelper::lastWritetimeOfFileSafe(path_preview_target, false);
}

std::string McWorkflow::getValueForJSONKeyAsString(const std::string& key_to_find, const nlohmann::json& json, const std::string& config_name) const
{
   for (auto& [key_config, value_config] : json.items()) {
      if (key_config == config_name) {
         for (auto& [key, value] : value_config.items()) {
            if (key == key_to_find) {
               return StaticHelper::replaceAll(nlohmann::to_string(value), "\"", "");
            }
         }
      }
   }

   addError("#KEY-NOT-FOUND in 'getValueForJSONKeyAsString' (key: '" + key_to_find + "', config: '" + config_name + "').");
   return "#KEY-NOT-FOUND";
}

bool McWorkflow::isLTL(const std::string& config, const std::string& path_template)
{
   std::string json_file_name{ config == JSON_TEMPLATE_DENOTER ? FILE_NAME_JSON_TEMPLATE : FILE_NAME_JSON };
   nlohmann::json json = getJSON(path_template + "/" + json_file_name);
   auto val = getValueForJSONKeyAsString("LTL_MODE", json, config);
   return StaticHelper::isBooleanTrue(val);
}
