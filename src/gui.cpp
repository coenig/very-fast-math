//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2024 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file

#include "gui/gui.h"
#include "gui/process_helper.h"
#include "model_checking/results_analysis.h"

#ifdef _WIN32
   #include <windows.h>
#elif __linux__
   #include <cstdlib>
#endif

void openExplorerWindow(const char* path) {
#ifdef _WIN32
   ShellExecuteA(NULL, "open", path, NULL, NULL, SW_SHOWNORMAL);
#elif __linux__
   std::string command = "xdg-open " + std::string(path);
   std::system(command.c_str());
#endif
}

using namespace vfm;

std::map<std::string, std::pair<std::string, std::string>> MCScene::loadNewBBsFromJson()
{
   std::string json_string{ json_input_->buffer()->text() };
   if (last_json_string_ == json_string) {
      return last_bb_stuff_;
   }
   else {
      last_json_string_ = json_string;
   }

   last_bb_stuff_.clear();

   auto json = getJSON();
   evaluateFormulasInJSON(json);

   auto bb_pair = getSpec(JSON_TEMPLATE_DENOTER);
   std::string bb{ bb_pair.second };
   std::string bb_str{};

   // SPEC
   bb_str = 
      std::string(isLTL(JSON_TEMPLATE_DENOTER) ? "LTLSPEC" : "INVARSPEC")
      + " = " + bb.substr(JSON_NUXMV_FORMULA_BEGIN.size(), bb.size() - JSON_NUXMV_FORMULA_BEGIN.size() - JSON_NUXMV_FORMULA_END.size() - 1);

   parser_->resetErrors(ErrorLevelEnum::error);
   MathStructPtr fmla = MathStruct::parseMathStruct(bb_str, parser_, data_, DotTreatment::as_operator);

   if (parser_->hasErrorOccurred()) {
      last_bb_stuff_.insert({ StaticHelper::removeWhiteSpace(StaticHelper::split(bb_str, "=").at(0)), { bb_str, bb_pair.first } });
   }
   else {
      if (fmla->getTermsJumpIntoCompounds().size() == 2) {
         std::string fmla_str_name{ fmla->child0JumpIntoCompounds()->serializePlainOldVFMStyle(MathStruct::SerializationSpecial::enforce_square_array_brackets) }; // Only the name of the BB.
         std::string fmla_str{ fmla->child1JumpIntoCompounds()->serializePlainOldVFMStyle(MathStruct::SerializationSpecial::enforce_square_array_brackets) }; // Actual formula.

         if (StaticHelper::stringContains(fmla_str, "#")) { // Error has occurred.
            last_bb_stuff_.insert({ fmla_str_name, { bb_str, bb_pair.first } });
         }
         else {
            last_bb_stuff_.insert({ fmla_str_name, { fmla_str, bb_pair.first } });
         }
      }
   }

   // BBs
   for (const auto& dynamic_terms : parser_->getDynamicTermMetas()) {
      const std::string name{ dynamic_terms.first };
      
      for (const auto& dynamic_term : dynamic_terms.second) {
         const int num_params{ dynamic_term.first };
         const TermPtr fmla{ dynamic_term.second->copy() };
         std::string actual_fmla_json_text{};

         if (StaticHelper::startsWithUppercase(name)) {
            std::string fmla_id{};

            for (const auto& formula_pair : getAllFormulasFromJSON(json)) {
               const std::string id{ formula_pair.first };
               const std::string definition{ formula_pair.second };

               if (StaticHelper::stringContains(definition, "@f(" + name + ")")) { // We found a function definition of the same name.
                  int begin{ StaticHelper::indexOfFirstInnermostBeginBracket(definition, "{", "}") },
                     end{ StaticHelper::findMatchingEndTagLevelwise(definition, begin, "{", "}") };
                  std::string inner{ definition };

                  StaticHelper::distributeGetOnlyInner(inner, begin + 1, end);
                  int num_params_json{ (int) (inner.size() - StaticHelper::replaceAll(inner, ",", "").size() + 1) }; // number of commas + 1

                  if (StaticHelper::isEmptyExceptWhiteSpaces(inner)) { // Special case: Just "{}".
                     num_params_json = 0;
                  }

                  if (num_params_json == num_params) { // We found the right overloaded function.
                     fmla_id = id;
                     int definition_end_index = definition.find_last_of('}');
                     int definition_begin_index{ StaticHelper::findMatchingBegTagLevelwise(definition, "{", "}", definition_end_index) };
                     actual_fmla_json_text = definition.substr(definition_begin_index + 1, definition_end_index - definition_begin_index - 1);

                     StaticHelper::trim(actual_fmla_json_text);

                     break;
                  }
               }
            }

            if (!fmla_id.empty()) {
               std::string params{};
               const auto& par_names_map = parser_->getParameterNamesFor(name, num_params);

               for (const auto& param : par_names_map) {
                  params += ", " + param.second;
               }

               if (!params.empty()) params = params.substr(2);

               fmla->applyToMeAndMyChildren([par_names_map](const MathStructPtr m) {
                  if (m->isMetaCompound()) {
                     m->replaceJumpOverCompounds(_var(par_names_map.at(m->child0()->constEval())));
                  }
               });

               std::string fmla_serialized_str{ fmla->serializePlainOldVFMStyle(MathStruct::SerializationSpecial::enforce_square_array_brackets) };

               std::string final_fmla_str{};
               if (StaticHelper::stringContains(fmla_serialized_str, "#")) { // Parsing error in formula.
                  final_fmla_str = actual_fmla_json_text;
               } else {
                  final_fmla_str = fmla_serialized_str;
               }

               last_bb_stuff_.insert({ name + "(" + params + ")", 
                  { final_fmla_str, fmla_id } });
            }
         }
      }
   }

   return last_bb_stuff_;
}

std::pair<std::string, std::string> vfm::MCScene::getBBNameAndFormulaByJsonName(const std::string& json_name)
{
   for (const auto& el : loadNewBBsFromJson()) {
      if (el.second.second == json_name) {
         return { el.first, el.second.first };
      }
   }

   return { "", "" };
}

float calculateScaleFactor(int screenNumber)
{
   constexpr int referenceWidth = 1920;
   constexpr int referenceHeight = 1080;
   int dummy, screenWidth, screenHeight;

   Fl::screen_xywh(dummy, dummy, screenWidth, screenHeight, screenNumber);

   return (std::min)(static_cast<float>(screenWidth) / referenceWidth, static_cast<float>(screenHeight) / referenceHeight);
}

MCScene::MCScene(const InputParser& inputs) : Failable(GUI_NAME + "-GUI")
{
   inputs.printArgumentsForMC();

   path_to_template_dir_ = inputs.getCmdOption(vfm::test::CMD_TEMPLATE_DIR_PATH);
   json_tpl_filename_ = inputs.getCmdOption(vfm::test::CMD_JSON_TEMPLATE_FILE_NAME);
   auto path_to_nuxmv = inputs.getCmdOption(vfm::test::CMD_NUXMV_EXEC);

   path_to_external_folder_ = std::filesystem::path(path_to_nuxmv).parent_path().parent_path().generic_string();

   for (int screenNumber = 0; screenNumber < Fl::screen_count(); ++screenNumber) {
      Fl::screen_scale(screenNumber, calculateScaleFactor(screenNumber));
   }

   window_->color(FL_RED);
   main_group_->end();

   resetParserAndData();

   ADDITIONAL_LOGGING_PIPE = new std::ostringstream{};
   addOrChangeErrorOrOutputStream(*ADDITIONAL_LOGGING_PIPE, true);
   addOrChangeErrorOrOutputStream(*ADDITIONAL_LOGGING_PIPE, false);
   addFailableChild(Failable::getSingleton(GUI_NAME + "_Related"), "");

   logging_output_and_interpreter_ = new InterpreterTerminal(data_, parser_, 10, 300, window_->w(), 90);
   addNote("Terminal initialized.");

   window_->resizable(window_);
   Fl_PNG_Image icon((path_to_template_dir_ + "/logo-plain.png").c_str());
   window_->icon(&icon);

   button_reload_json_->callback(buttonReloadJSON, this);
   button_save_json_->callback(buttonSaveJSON, this);
   button_check_json_->callback(buttonCheckJSON, this);
   button_delete_generated_->callback(buttonDeleteGenerated, this);
   button_delete_testcases_->callback(buttonDeleteTestCases, this);
   button_delete_current_preview_->callback(buttonDeleteCurrentPreview, this);
   button_delete_cached_->callback(buttonDeleteCached, this);
   button_delete_cached_->tooltip("Use right click if you really want to delete cache");
   button_run_mc_and_preview_->callback(buttonRunMCAndPreview, this);
   button_run_parser_->callback(buttonReparse, this);
   button_run_cex_->callback(buttonCEX, this);
   button_runtime_analysis_->callback(buttonRuntimeAnalysis, this);
   checkbox_json_visible_->callback(checkboxJSONVisibleCallback, this);
   checkbox_json_visible_->when(FL_WHEN_RELEASE_ALWAYS);

   json_input_->buffer(json_buffer_);
   json_input_->textfont(FL_COURIER);
   json_input_->textsize(12);

   json_input_->callback(jsonChangedCallback, this);
   json_input_->when(FL_WHEN_CHANGED);
   loadJsonText();

   json_input_->color(FL_YELLOW);
   //button_run_cex_->color(FL_YELLOW);
   button_reload_json_->color(FL_YELLOW);
   button_save_json_->color(FL_YELLOW);
   button_check_json_->color(FL_YELLOW);

   window_->end();
   window_->show();

   Fl::add_timeout(TIMEOUT_FREQUENT, refreshFrequently, this);
   Fl::add_timeout(TIMEOUT_RARE, refreshRarely, this);
   Fl::add_timeout(TIMEOUT_SOMETIMES, refreshSometimes, this);

   copyWaitingForPreviewGIF();

   scene_description_->textfont(FL_COURIER_BOLD);
   scene_description_->textsize(16);
   scene_description_->wrap(Fl_Text_Display::WRAP_AT_BOUNDS);
   scene_description_->readonly(1);
   scene_description_->align(FL_ALIGN_TOP);
   scene_description_->position(window_->w() - scene_description_->w());
   setTitle();

   main_group_->position(50, 200);

   runtime_global_options_ = std::make_shared<OptionsGlobal>(getGeneratedDir() + "/runtime_options.morty");
   runtime_global_options_->loadOptions();
   runtime_global_options_->saveOptions(); // In case of missing file, create it with default values.

   bool json_visible_checked{ StaticHelper::isBooleanTrue(runtime_global_options_->getOptionValue(SecOptionGlobalItemEnum::json_visible)) };
   checkbox_json_visible_->value(json_visible_checked);

   variables_list_->align(FL_ALIGN_TOP);

   addNote(GUI_NAME + " started.");

   //window_->fullscreen();

   fl_run_info_ = Fl::run();
}

int vfm::MCScene::getFlRunInfo() const
{
   return fl_run_info_;
}

SingleExpController vfm::MCScene::getSECFromConfig(const std::string& name) const
{
   for (const auto& sec : se_controllers_) {
      if (sec.getMyId() == name) {
         return sec;
      }
   }

   return SingleExpController("", "", "", "", nullptr);
}

std::string vfm::MCScene::getOptionFromSECConfig(const std::string& config_name, const SecOptionLocalItemEnum option_name) const
{
   return getSECFromConfig(config_name).runtime_local_options_.getOptionValue(option_name);
}

std::string vfm::MCScene::getValueForJSONKeyAsString(const std::string& key_to_find, const std::string& config_name) const
{
   const bool from_template{ config_name == JSON_TEMPLATE_DENOTER };

   try {
      nlohmann::json j = nlohmann::json::parse(from_template ? json_input_->buffer()->text() : StaticHelper::readFile(getTemplateDir() + "/" + FILE_NAME_JSON));

      for (auto& [key_config, value_config] : j.items()) {
         if (key_config == config_name) {
            for (auto& [key, value] : value_config.items()) {
               if (key == key_to_find) {
                  return StaticHelper::replaceAll(nlohmann::to_string(value), "\"", "");
               }
            }
         }
      }
   }
   catch (const nlohmann::json::parse_error& e) {
      addError("#JSON Error in 'getValueForJSONKeyAsString' (key: '" + key_to_find + "', config: '" + config_name + "', from_template: " + std::to_string(from_template) + ").");
      return "#JSON Error"; // TODO: Return more specific error?
   }
   catch (...) {
      addError("#JSON Error in 'getValueForJSONKeyAsString' (key: '" + key_to_find + "', config: '" + config_name + "', from_template: " + std::to_string(from_template) + ").");
      return "#JSON Error";
   }

   addError("#KEY-NOT-FOUND in 'getValueForJSONKeyAsString' (key: '" + key_to_find + "', config: '" + config_name + "', from_template: " + std::to_string(from_template) + ").");
   return "#KEY-NOT-FOUND";
}

void vfm::MCScene::setValueForJSONKeyFromBool(const std::string& key_to_find, const std::string& config_name, const bool from_template, const bool value_to_set) const
{ // TODO: Double code
   try {
      nlohmann::json j = nlohmann::json::parse(from_template ? json_input_->buffer()->text() : StaticHelper::readFile(getTemplateDir() + "/" + FILE_NAME_JSON));

      for (auto& [key_config, value_config] : j.items()) {
         if (key_config == config_name) {
            for (auto& [key, value] : value_config.items()) {
               if (key == key_to_find) {
                  value = value_to_set;
                  json_input_->buffer()->text(j.dump(3).c_str());
                  return;
               }
            }
         }
      }
   }
   catch (const nlohmann::json::parse_error& e) {
      addError("#JSON Error in 'setValueForJSONKeyFromString' (key: '" + key_to_find + "', value: '" + std::to_string(value_to_set) + "', config: '" + config_name + "', from_template: " + std::to_string(from_template) + ").");
      return;
   }
   catch (...) {
      addError("#JSON Error in 'setValueForJSONKeyFromString' (key: '" + key_to_find + "', value: '" + std::to_string(value_to_set) + "', config: '" + config_name + "', from_template: " + std::to_string(from_template) + ").");
      return;
   }

   addError("#KEY-NOT-FOUND in 'setValueForJSONKeyFromString' (key: '" + key_to_find + "', value: '" + std::to_string(value_to_set) + "', config: '" + config_name + "', from_template: " + std::to_string(from_template) + ").");
}

void vfm::MCScene::setValueForJSONKeyFromString(const std::string& key_to_find, const std::string& config_name, const bool from_template, const std::string& value_to_set) const
{ // TODO: Double code
   try {
      nlohmann::json j = nlohmann::json::parse(from_template ? json_input_->buffer()->text() : StaticHelper::readFile(getTemplateDir() + "/" + FILE_NAME_JSON));

      for (auto& [key_config, value_config] : j.items()) {
         if (key_config == config_name) {
            for (auto& [key, value] : value_config.items()) {
               if (key == key_to_find) {
                  value = value_to_set;
                  json_input_->buffer()->text(j.dump(3).c_str());
                  return;
               }
            }
         }
      }
   }
   catch (const nlohmann::json::parse_error& e) {
      addError("#JSON Error in 'setValueForJSONKeyFromString' (key: '" + key_to_find + "', value: '" + value_to_set + "', config: '" + config_name + "', from_template: " + std::to_string(from_template) + ").");
      return;
   }
   catch (...) {
      addError("#JSON Error in 'setValueForJSONKeyFromString' (key: '" + key_to_find + "', value: '" + value_to_set + "', config: '" + config_name + "', from_template: " + std::to_string(from_template) + ").");
      return;
   }

   addError("#KEY-NOT-FOUND in 'setValueForJSONKeyFromString' (key: '" + key_to_find + "', value: '" + value_to_set + "', config: '" + config_name + "', from_template: " + std::to_string(from_template) + ").");
}

std::string MCScene::getTemplateDir() const
{
   return path_to_template_dir_;
}

std::string MCScene::getCachedDir() const
{
   if (cached_dir_.empty()) {
      cached_dir_ = getValueForJSONKeyAsString("_CACHED_PATH", JSON_TEMPLATE_DENOTER);
   }

   return cached_dir_;
}

std::string MCScene::getBPIncludesFileDir() const
{
   if (includes_file_dir_.empty()) {
      includes_file_dir_ = getValueForJSONKeyAsString("_BP_INCLUDES_FILE_PATH", JSON_TEMPLATE_DENOTER);
   }

   return includes_file_dir_;
}

std::string MCScene::getGeneratedDir() const
{
   if (cached_dir_.empty()) {
      generated_dir_ = getValueForJSONKeyAsString("_GENERATED_PATH", JSON_TEMPLATE_DENOTER);
   }

   return generated_dir_;
}

std::string vfm::MCScene::getGeneratedParentDir() const
{
   return std::filesystem::path(getGeneratedDir()).parent_path().string();
}

int vfm::MCScene::getActualJSONWidth() const
{
   return json_input_->visible() ? json_input_->w() : 0;
}

void vfm::MCScene::resetCachedVariables() const
{
   cached_dir_ = "";
   generated_dir_ = "";
}

void MCScene::copyWaitingForPreviewGIF() {
   std::string path_preview_template{ getTemplateDir() + "/preview.img" };
   std::string path_preview_target{ getGeneratedDir() + "/preview/preview.gif" };
   StaticHelper::createDirectoriesSafe(getGeneratedDir() + "/preview", false);

   if (StaticHelper::existsFileSafe(path_preview_target, false)) {
      StaticHelper::removeFileSafe(path_preview_target, false);
   }

   try {
      std::filesystem::copy(path_preview_template, path_preview_target);
   }
   catch (const std::exception& e) {
      addWarning("Directory '" + StaticHelper::absPath(path_preview_template) + "' not copied to '" + StaticHelper::absPath(path_preview_target) + "'. Error: " + e.what());
   }

   previous_write_time_ = StaticHelper::lastWritetimeOfFileSafe(path_preview_target, false);

   refreshPreview(); // TODO: HERE IT HAPPENS THAT THE RED BACKGORUND IS NOT SHOWN.
}

void MCScene::refreshPreview()
{
   std::lock_guard<std::mutex> lock{ refresh_mutex_ };
   std::string path_preview_target{ getGeneratedDir() + "/preview/preview.gif" };

   if (!StaticHelper::existsFileSafe(getGeneratedDir()) || !StaticHelper::existsFileSafe(path_preview_target)) {
      return;
   }

   box_->image(nullptr);
   gif_->release(); // Here the gif image is deleted.
   try {
      gif_ = new Fl_Anim_GIF_Image(path_preview_target.c_str());
   }
   catch (std::exception& e) {

   }

   gif_->delay(50); // Set a delay of 50 milliseconds between frames
   box_->image(*gif_);
   gif_->scale(window_->w(), window_->h(), 1, 1);

   box_->box(FL_BORDER_BOX);
   box_->align(FL_ALIGN_INSIDE | FL_ALIGN_TOP);
   box_->when(FL_WHEN_CHANGED);
}

void MCScene::activateMCButtons(const bool active, const ButtonClass which)
{
   if (active) {
      if (which == ButtonClass::RunButtons || which == ButtonClass::All) {
         button_run_parser_->activate();
         button_run_cex_->activate();
         button_run_mc_and_preview_->activate();
         button_runtime_analysis_->activate();
      }

      if (which == ButtonClass::DeleteButtons || which == ButtonClass::All) {
         button_delete_generated_->activate();
         button_delete_testcases_->activate();
         button_delete_current_preview_->activate();
         button_delete_cached_->activate();
      }
   }
   else {
      if (which == ButtonClass::RunButtons || which == ButtonClass::All) {
         button_run_parser_->deactivate();
         button_run_cex_->deactivate();
         button_run_mc_and_preview_->deactivate();
         button_runtime_analysis_->deactivate();
      }

      if (which == ButtonClass::DeleteButtons || which == ButtonClass::All) {
         button_delete_generated_->deactivate();
         button_delete_testcases_->deactivate();
         button_delete_current_preview_->deactivate();
         button_delete_cached_->deactivate();
      }
   }
}

bool MCScene::isLTL(const std::string& config)
{
   auto val = getValueForJSONKeyAsString("LTL_MODE", config);
   return StaticHelper::isBooleanTrue(val);
}

int MCScene::bmcDepth(const std::string& config)
{
   auto val = getValueForJSONKeyAsString("BMC_CNT", config);
   return std::stoi(val);
}

std::vector<std::pair<std::string, std::string>> MCScene::getAllFormulasFromJSON(const nlohmann::json json)
{
   std::vector<std::pair<std::string, std::string>> result{};

   try {
      for (auto& [key_config, value_config] : json.items()) {
         if (key_config == JSON_TEMPLATE_DENOTER) {
            for (auto& [key, value] : value_config.items()) {
               if (StaticHelper::stringStartsWith(key, JSON_VFM_FORMULA_PREFIX)) {
                  result.push_back({ key, value });
               }
            }
         }
      }
   }
   catch (const nlohmann::json::parse_error& e) {
      result.push_back({ "#NOKEY", "#JSON Error" }); // TODO: Return more specific error?
   }
   catch (...) {
      result.push_back({ "#NOKEY", "#JSON Error" }); // TODO: Return more specific error?
   }

   return result;
}

std::pair<std::string, std::string> MCScene::getSpec(const std::string& config)
{
   const std::string file_name{ config == JSON_TEMPLATE_DENOTER ? json_tpl_filename_ : FILE_NAME_JSON };

   try {
      std::string json_text{ StaticHelper::readFile(getTemplateDir() + "/" + file_name) };
      nlohmann::json j = nlohmann::json::parse(json_text);

      for (auto& [key_config, value_config] : j.items()) {
         if (key_config == config) {
            for (auto& [key, value] : value_config.items()) {
               if (key == "SPEC" || StaticHelper::stringStartsWith(key, "BB")) {
                  return { key, value };
               }
            }
         }
      }
   }
   catch (const nlohmann::json::parse_error& e) {
      return { "#NOKEY", "#JSON Error" }; // TODO: Return more specific error?
   }
   catch (...) {
      return { "#NOKEY", "#JSON Error" }; // TODO: Return more specific error?
   }

   return {};
}

void MCScene::loadJsonText()
{
   std::string path{ getTemplateDir() };
   path += "/" + json_tpl_filename_;
   json_input_->buffer()->text(StaticHelper::readFile(path).c_str());
   //spec_input->value(getSpecAndBBs().back().c_str());
}

void MCScene::saveJsonText()
{
   std::string path{ getTemplateDir() };
   path += "/" + json_tpl_filename_;
   StaticHelper::writeTextToFile(json_input_->buffer()->text(), path);

   // Check json syntax.
   try {
      nlohmann::json json = nlohmann::json::parse(json_input_->buffer()->text());
      resetCachedVariables();
      getGeneratedDir();
      getCachedDir();
   }
   catch (const nlohmann::json::parse_error& e) {
   }
}

void MCScene::setTitle()
{
   window_->label((GUI_NAME + std::string("         ")
      + json_tpl_filename_ + " | "
      + StaticHelper::absPath(getTemplateDir()) + " ==> " 
      + StaticHelper::absPath(getGeneratedDir())
      + " [[" + StaticHelper::absPath(getCachedDir()) + "]]").c_str());
}

std::shared_ptr<OptionsGlobal> vfm::MCScene::getRuntimeGlobalOptions() const
{
   return runtime_global_options_;
}

void vfm::MCScene::putJSONIntoDataPack(const std::string& json_config)
{
   const bool from_template{ json_config == JSON_TEMPLATE_DENOTER };

   //data_->reset();

   try {
      nlohmann::json j = nlohmann::json::parse(from_template ? json_input_->buffer()->text() : StaticHelper::readFile(getTemplateDir() + "/" + FILE_NAME_JSON));

      for (auto& [key_config, value_config] : j.items()) {
         if (key_config == json_config) {
            for (auto& [key, value] : value_config.items()) {
               if (value.is_string()) {
                  std::string val_str = StaticHelper::replaceAll(nlohmann::to_string(value), "\"", "");
                  //data_->addStringToDataPack(val_str, key); // TODO: Currently leads to heap overflow. Need to overwrite instead of append.
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

   //addError("#KEY-NOT-FOUND in 'getValueForJSONKeyAsString' (key: '" + key_to_find + "', config: '" + config_name + "', from_template: " + std::to_string(from_template) + ").");
}

void MCScene::showAllBBGroups(const bool show)
{
   for (const auto& group : bb_groups_) {
      envmodel_variables_.insert(group.first);

      const std::string fmla_str{ group.second.second->getFormula() };
      parser_->resetErrors(ErrorLevelEnum::error);
      const TermPtr fmla{ MathStruct::parseMathStruct(fmla_str, parser_, data_, DotTreatment::as_operator)->toTermIfApplicable() };

      if (parser_->hasErrorOccurred()) {
         group.second.second->color(FL_RED);
      }
      else {
         if (group.second.second->getJsonName() == "SPEC") {
            group.second.second->color(fl_rgb_color(222, 210, 150));
         }
         else if (fmla->hasLTLOperators()) {
            group.second.second->color(fl_rgb_color(255, 229, 204));
         }
         else {
            group.second.second->color(FL_WHITE);
         }
      }

      group.second.second->getEditor().showOnScreen(show);
   }
}

void MCScene::resetAllBBGroups()
{
   const auto json_bbs{ loadNewBBsFromJson() };

   for (const auto& group : bb_groups_) {
      if (json_bbs.count(group.first)) {
         const auto data{ json_bbs.at(group.first) };

         group.second.second->setName(group.first);
         group.second.second->setValue(data.first);
      }
   }
}

void showBoxWithPreviews(const MCScene* mc_scene, const DragGroup* drag_group)
{
   const std::filesystem::path previews_path{ mc_scene->getGeneratedParentDir() + "/previews" };
   StaticHelper::createDirectoriesSafe(previews_path);
   float current_best{ std::numeric_limits<float>::infinity() };
   std::filesystem::directory_entry best{};
   std::string best_spec{};

   try {
      for (const auto& entry : std::filesystem::directory_iterator(previews_path)) { // Find all previews
         const std::string spec_raw{ StaticHelper::readFile(entry.path().string() + "/spec.txt") };

         //if (StaticHelper::stringContains(spec_raw, "LTLSPEC")) continue; // TODO: Display message on how to "read" LTL-based previews.

         std::string spec_str{ 
            StaticHelper::replaceAllRegex(
               StaticHelper::replaceAll(
                  StaticHelper::replaceAll( // Replace the toplevel "G" in LTL specs. 
                     spec_raw, 
                     "INVARSPEC", ""),
                  "LTLSPEC", ""), 
               R"(^\s*G\s*([\s\+\-\!\(]|$))", "$1") };

         TermPtr spec{ _not(MathStruct::parseMathStruct(spec_str, nullptr, mc_scene->getData(), DotTreatment::as_operator)->toTermIfApplicable()) }; // Don't use mc_scene->parser_ since BBs are functions there.
         spec = _id(mc::simplification::simplifyFast(spec));

         const std::set<std::string> BOOL_OPERATORS{
            SYMB_AND, SYMB_OR, SYMB_LTL_IFF, SYMB_LTL_IMPLICATION, SYMB_NOT, SYMB_LTL_XOR, SYMB_LTL_XNOR };
         const std::string VAR_NAME_PREFIX{ "__v" };
         std::map<std::string, std::string> var_to_block{};
         int cnt{ 0 };
         auto abandon_children{ std::make_shared<bool>(false) };

         spec->child0()->applyToMeAndMyChildren([&var_to_block, &cnt, &BOOL_OPERATORS, &VAR_NAME_PREFIX, abandon_children](const MathStructPtr m) {
            if (!BOOL_OPERATORS.count(m->getOptorJumpIntoCompound())) {
               std::string var_name{ VAR_NAME_PREFIX + std::to_string(cnt++) };
               var_to_block[var_name] = m->serializePlainOldVFMStyle(MathStruct::SerializationSpecial::enforce_square_array_brackets);
               m->replaceJumpOverCompounds(_var(var_name));
               *abandon_children = true;
               //std::cout << var_name << " is " << var_to_block.at(var_name) << std::endl;
            }
            }, TraverseCompoundsType::avoid_compound_structures, abandon_children);

         std::vector<int> arity_vec{};
         std::vector<int> counter_vec{};
         for (const auto& var_block : var_to_block) {
            arity_vec.push_back(2);
            counter_vec.push_back(1);
         }
         counter_vec.front()++;

         // Calculate score based on truth table.
         bool is_valid_for_bb{ true };
         double counter_of_ones_in_truth_table{ 0 };
         const auto local_data{ std::make_shared<DataPack>() };

         std::set<std::string> all_bbs{};

         for (const auto& bbg : mc_scene->getBBGroups()) {
            std::string bb_name{ DragGroup::removeFunctionStuffFromString(bbg.second.second->getName()) };
            all_bbs.insert(StaticHelper::trimAndReturn(bb_name));
         }

         std::string drag_group_name{ DragGroup::removeFunctionStuffFromString(drag_group->getName()) };

         while (!VECTOR_COUNTER_IS_ALL_ZERO(counter_vec) && is_valid_for_bb) {
            VECTOR_COUNTER_DECREMENT(arity_vec, counter_vec);
            int var_cnt{ 0 };
            int my_bb_num{ -1 };
            float number_of_zeroes_apart_from_this_bb{ 0 }; // These count as bonus to award this bb to be sole 1 in the row.

            for (const auto& var_block : var_to_block) {
               local_data->addOrSetSingleVal(var_block.first, counter_vec.at(var_cnt));

               if (var_block.second == drag_group_name) {
                  my_bb_num = var_cnt;
               }
               else if (all_bbs.count(var_block.second)) {
                  number_of_zeroes_apart_from_this_bb += !counter_vec.at(var_cnt);
               }

               var_cnt++;
            }

            float tt_value = spec->eval(local_data);
            number_of_zeroes_apart_from_this_bb *= tt_value; // Only count these if the spec is actually 1 at this position.

            if (my_bb_num < 0 || (tt_value && !counter_vec.at(my_bb_num))) { // If the BB is part of the formula...
               is_valid_for_bb = false; // it needs to be true any time the full !(spec) is true.
            }

            counter_of_ones_in_truth_table += tt_value; // Add each one in the TT (which is also a one for this BB, if is_valid_for_bb).
            counter_of_ones_in_truth_table -= number_of_zeroes_apart_from_this_bb / counter_vec.size();
         }

         double score{ is_valid_for_bb
            ? counter_of_ones_in_truth_table
            : std::numeric_limits<float>::infinity() };

         if (score < current_best) {
            current_best = score;
            best = entry;
            best_spec = spec_raw;
         }
      }
   }
   catch (const std::exception& e) {
      mc_scene->addError("Error loading preview. " + std::string(e.what()));
   }

   drag_group->getEditor().preview_scroll_->clear();

   if (!std::isinf(current_best) && best.exists()) { // Do we have a valid preview?
      std::vector<std::filesystem::directory_entry> previews{}; // All available previews in the best-matching preview group.
      try {
         for (const auto& entry : std::filesystem::directory_iterator(best.path())) {
            if (entry.is_directory() && StaticHelper::existsFileSafe(entry.path() / "preview2_0.png")) {
               previews.push_back(entry);
            }
         }
      }
      catch (const std::exception& e) {}

      if (!previews.empty()) { // This can happen in case of a broken preview, e.g., after premature cancellation.
         drag_group->getEditor().preview_scroll_->show();
         drag_group->getEditor().preview_description_->show();
         drag_group->getEditor().slider_->show();

         drag_group->getEditor().slider_->range(0, previews.size() - 1);
         const int preview_num{ (int)drag_group->getEditor().slider_->value() };

         int i{ 0 };
         try {
            for (int img_num{ 0 };; img_num++) {
               std::filesystem::path img_path{ previews.at(preview_num) / std::filesystem::path("preview2_" + std::to_string(img_num) + ".png") };
               std::filesystem::path img_next_path{ previews.at(preview_num) / std::filesystem::path("preview2_" + std::to_string(img_num + 1) + ".png") };

               if (StaticHelper::existsFileSafe(img_path.string()) && StaticHelper::existsFileSafe(img_next_path.string())) {
                  Fl_PNG_Image* image = new Fl_PNG_Image(img_path.string().c_str());
                  Fl_Image* image_scaled{ image->copy(image->w() / 2, image->h() / 2) };
                  Fl_Box* imageBox = new Fl_Box(
                     drag_group->getEditor().preview_scroll_->x(),
                     drag_group->getEditor().preview_scroll_->y() + i * (image_scaled->h() + 1),
                     image_scaled->w(),
                     image_scaled->h() + 1);
                  imageBox->image(image_scaled);
                  drag_group->getEditor().preview_scroll_->add(imageBox);
                  i++;
               }
               else {
                  break;
               }
            }
         }
         catch (const std::exception& e) {}

         drag_group->getEditor().preview_scroll_->type(Fl_Scroll::VERTICAL);
         drag_group->getEditor().preview_scroll_->size(
            drag_group->getEditor().preview_scroll_->child(0)->w(),
            drag_group->getEditor().preview_scroll_->child(0)->h());
         drag_group->getEditor().preview_description_->resize(
            drag_group->getEditor().preview_scroll_->x(),
            drag_group->getEditor().preview_scroll_->y() + drag_group->getEditor().preview_scroll_->h() + 2,
            2 * drag_group->getEditor().preview_scroll_->w() / 3,
            25);
         drag_group->getEditor().slider_->resize(
            drag_group->getEditor().preview_scroll_->x() + 2 * drag_group->getEditor().preview_scroll_->w() / 3 + 50,
            drag_group->getEditor().preview_scroll_->y() + drag_group->getEditor().preview_scroll_->h() + 2,
            150,
            25);

         drag_group->getEditor().slider_->type(FL_HORIZONTAL);
         drag_group->getEditor().slider_->color(fl_rgb_color(255, 230, 173));
         drag_group->getEditor().slider_->precision(0);

         drag_group->getEditor().preview_description_->color(fl_rgb_color(255, 230, 173));
         drag_group->getEditor().preview_description_->color2(FL_BACKGROUND_COLOR);
         drag_group->getEditor().preview_description_->readonly(1);

         auto editor{ drag_group->getEditor() };
         editor.setPreviewScrollHeight(editor.preview_scroll_->child(0)->h() * (i - 1));

         if (StaticHelper::stringContains(best_spec, "LTLSPEC")) {
            size_t pos = drag_group->getName().find('('); // Remove parameters, only name counts.
            std::string bb_simple_name{ pos == std::string::npos ? drag_group->getName() : drag_group->getName().substr(0, pos) };

            if (!mc_scene->getRuntimeGlobalOptions()->containsSetOption(SecOptionGlobalItemEnum::no_show_ltl_note, bb_simple_name)) {
               int choice = fl_choice(
                  ("NOTE: The preview for this building block is based on an LTL specification.\n\nThe property '" + std::string(drag_group->getName()) + "' might not be visible in the LAST frame (as is the case for INVAR specs), but somewhere in between.").c_str(),
                  "Understood", "Understood / don't show again", 0);

               if (choice == 1) { // Don't show again.
                  mc_scene->getRuntimeGlobalOptions()->addToSetOption(SecOptionGlobalItemEnum::no_show_ltl_note, bb_simple_name);
                  mc_scene->getRuntimeGlobalOptions()->saveOptions();
               }
            }
         } else {
            editor.preview_scroll_->scroll_to( // Scroll to the bottom.
               0,
               editor.getPreviewScrollHeight()
               * (1 + (editor.preview_scroll_->yposition() != 0)) // TODO: Alternating factor "2" is to compensate for some weird toggling in fltk, which is not yet fully understood.
            );
         }

         drag_group->getEditor().preview_description_->value(
            ("Preview based on CEX for '" + best_spec + "'. Found " + std::to_string(previews.size()) + " previews, showing #" + std::to_string(preview_num) + ".").c_str());
      }
   }
}

std::map<std::string, std::pair<std::string, DragGroup*>> vfm::MCScene::getBBGroups() const
{
   return bb_groups_;
}

// Callbacks

void MCScene::bbBoxSliderCallback(Fl_Widget* widget, void* data)
{
   auto drag_group{ static_cast<DragGroup*>(data) };
   auto mc_scene{ drag_group->getMCScene() };

   showBoxWithPreviews(mc_scene, drag_group);
}

void MCScene::bbBoxCallback(Fl_Widget* widget, void* data)
{
   auto drag_group{ static_cast<DragGroup*>(widget) };
   auto mc_scene{ static_cast<MCScene*>(data) };
   mc_scene->showAllBBGroups(false);

   if (Fl::event_clicks()) { // Double click: Just hide all groups and reset to current JSON value.
      mc_scene->resetAllBBGroups();
      return;
   }

   drag_group->getEditor().showOnScreen(true);

   //  --------------------------   ---------------   ---------------        --------------- 
   // |       name_editor        | |    button     | |    button     | ...  |    button     |
   //  --------------------------   ---------------   ---------------        --------------- 
   // |                                                                                     |
   // |                                     formula_editor                                  |
   // |                                                                                     |
   //  -------------------------------------------------------------------------------------

   static constexpr int fmla_editor_width{ 850 };
   static constexpr int fmla_editor_height{ 75 };
   static constexpr int num_buttons{ 3 };
   static constexpr int button_width{ 140 };
   static constexpr int button_pad{ 2 };
   static constexpr int width_of_all_buttons{ num_buttons * (button_width + button_pad) };

   drag_group->getEditor().formula_editor_->resize(drag_group->x(), drag_group->y() + drag_group->h() + 40, fmla_editor_width, fmla_editor_height);
   drag_group->getEditor().formula_name_editor_->resize(drag_group->x(), drag_group->y() + drag_group->h() + 10, fmla_editor_width - width_of_all_buttons, 30);
   drag_group->getEditor().formula_apply_formula_button_->resize(drag_group->getEditor().formula_name_editor_->x() + drag_group->getEditor().formula_name_editor_->w() + button_pad, drag_group->y() + drag_group->h() + 10, button_width, 30);
   drag_group->getEditor().formula_simplify_button_->resize(drag_group->getEditor().formula_apply_formula_button_->x() + drag_group->getEditor().formula_apply_formula_button_->w() + button_pad, drag_group->y() + drag_group->h() + 10, button_width, 30);
   drag_group->getEditor().formula_flatten_button_->resize(drag_group->getEditor().formula_simplify_button_->x() + drag_group->getEditor().formula_simplify_button_->w() + button_pad, drag_group->y() + drag_group->h() + 10, button_width, 30);
   
   drag_group->getEditor().formula_editor_->textfont(FL_COURIER);
   drag_group->getEditor().formula_editor_->textsize(16);
   
   drag_group->getEditor().preview_scroll_->resize(
      200,
      drag_group->getEditor().formula_editor_->y() + drag_group->getEditor().formula_editor_->h() + 10,
      0,
      0);

   drag_group->getEditor().formula_editor_->wrap(1);
   drag_group->getEditor().formula_editor_->color(fl_rgb_color(255, 236, 158));
   drag_group->getEditor().formula_name_editor_->color(fl_rgb_color(255, 236, 158));

   mc_scene->window_->remove(drag_group->getEditor().slider_);
   mc_scene->window_->add(drag_group->getEditor().slider_);
   mc_scene->window_->remove(drag_group->getEditor().preview_scroll_);
   mc_scene->window_->add(drag_group->getEditor().preview_scroll_);
   mc_scene->window_->remove(drag_group->getEditor().preview_description_);
   mc_scene->window_->add(drag_group->getEditor().preview_description_);
   mc_scene->window_->remove(drag_group->getEditor().formula_apply_formula_button_);
   mc_scene->window_->add(drag_group->getEditor().formula_apply_formula_button_);
   mc_scene->window_->remove(drag_group->getEditor().formula_simplify_button_);
   mc_scene->window_->add(drag_group->getEditor().formula_simplify_button_);
   mc_scene->window_->remove(drag_group->getEditor().formula_flatten_button_);
   mc_scene->window_->add(drag_group->getEditor().formula_flatten_button_);
   mc_scene->window_->remove(drag_group->getEditor().formula_editor_);      // bring
   mc_scene->window_->add(drag_group->getEditor().formula_editor_);         // to
   mc_scene->window_->remove(drag_group->getEditor().formula_name_editor_); // the
   mc_scene->window_->add(drag_group->getEditor().formula_name_editor_);    // top

   drag_group->getEditor().preview_scroll_->hide();
   drag_group->getEditor().preview_description_->hide();
   drag_group->getEditor().slider_->hide();

   //drag_group->getEditor().slider_->callback(ss);

   drag_group->color(fl_rgb_color(245, 184, 78));

   showBoxWithPreviews(mc_scene, drag_group);
}

void MCScene::checkboxJSONVisibleCallback(Fl_Widget* widget, void* data)
{
   Fl_Check_Button* checkbox{ (Fl_Check_Button*)widget };
   auto mc_scene{ static_cast<MCScene*>(data) };

   for (const auto& bb_el : mc_scene->bb_groups_) {
      bb_el.second.second->hide();
      bb_el.second.second->getEditor().showOnScreen(false);
      if (bb_el.second.second->getEditor().formula_apply_formula_button_) delete bb_el.second.second->getEditor().formula_apply_formula_button_;
      if (bb_el.second.second->getEditor().formula_simplify_button_) delete bb_el.second.second->getEditor().formula_simplify_button_;
      if (bb_el.second.second->getEditor().formula_flatten_button_) delete bb_el.second.second->getEditor().formula_flatten_button_;
      if (bb_el.second.second->getEditor().formula_name_editor_) delete bb_el.second.second->getEditor().formula_name_editor_;
      if (bb_el.second.second->getEditor().preview_scroll_) delete bb_el.second.second->getEditor().preview_scroll_;
      //if (bb_el.second.second->getEditor().preview_description_) delete bb_el.second.second->getEditor().preview_description_;
      if (bb_el.second.second->getEditor().formula_editor_) delete bb_el.second.second->getEditor().formula_editor_;
      //if (bb_el.second.second->getEditor().slider_) delete bb_el.second.second->getEditor().slider_;
      if (bb_el.second.second) delete bb_el.second.second;
   }

   mc_scene->bb_groups_.clear();

   mc_scene->showAllBBGroups(false);
}

void MCScene::refreshRarely(void* data)
{
   auto mc_scene{ static_cast<MCScene*>(data) };
   mc_scene->putJSONIntoDataPack();

   std::string path_generated_base_str{ mc_scene->getGeneratedDir() };

   if (!StaticHelper::existsFileSafe(path_generated_base_str)) {
      StaticHelper::createDirectoriesSafe(path_generated_base_str);
   }

   std::string path_prose_description{ path_generated_base_str + "/" + PROSE_DESC_NAME };
   std::filesystem::path path_generated_base(path_generated_base_str);
   std::filesystem::path path_generated_base_parent = path_generated_base.parent_path();
   std::string path_template{ mc_scene->getTemplateDir() };
   std::string prefix{ path_generated_base.filename().string() };
   std::set<std::string> packages{};

   if (StaticHelper::stringContains(StaticHelper::toLowerCase(path_generated_base_str), "error")) {
      return;
   }

   // Populate list of variables.
   mc_scene->variables_list_->position(mc_scene->json_input_->x() + mc_scene->getActualJSONWidth() + 10, mc_scene->json_input_->y());
   mc_scene->variables_list_->size(mc_scene->scene_description_->x() - mc_scene->json_input_->x() - mc_scene->getActualJSONWidth() - 20, 100);

   mc_scene->runtime_global_options_->loadOptions();
   if (mc_scene->runtime_global_options_->hasOptionChanged(SecOptionGlobalItemEnum::envmodel_variables_general)
      || mc_scene->runtime_global_options_->hasOptionChanged(SecOptionGlobalItemEnum::envmodel_variables_specific)
      || mc_scene->runtime_global_options_->hasOptionChanged(SecOptionGlobalItemEnum::planner_variables)) {
      std::string vars_str_general{ mc_scene->runtime_global_options_->getOptionValue(SecOptionGlobalItemEnum::envmodel_variables_general) };
      std::string vars_str_specific{ mc_scene->runtime_global_options_->getOptionValue(SecOptionGlobalItemEnum::envmodel_variables_specific) };
      std::string vars_str_planner{ mc_scene->runtime_global_options_->getOptionValue(SecOptionGlobalItemEnum::planner_variables) };
      mc_scene->variables_list_->clear();
      mc_scene->envmodel_variables_.clear();

      auto general_vars = StaticHelper::split(vars_str_general, ",");
      auto specific_vars = StaticHelper::split(vars_str_specific, ",");
      auto planner_vars = StaticHelper::split(vars_str_planner, ",");

      for (const auto& envmodel_general_var : general_vars) {
         mc_scene->variables_list_->add(envmodel_general_var.c_str());
         mc_scene->envmodel_variables_.insert(envmodel_general_var);
      }

      SyntaxHighlightMultilineInput::setKeywords(&mc_scene->envmodel_variables_);

      for (const auto& envmodel_specific_var : specific_vars) {
         mc_scene->variables_list_->add((std::string("*") + envmodel_specific_var).c_str());
      }

      for (const auto& planner_var : planner_vars) {
         mc_scene->variables_list_->add(planner_var.c_str());
         mc_scene->envmodel_variables_.insert(planner_var);
      }

      std::string label_vars{ "Variables  ---  EnvModel: " + std::to_string(general_vars.size()) + " universal ("
         + std::to_string(specific_vars.size()) + " partial)  ---  Planner: " + std::to_string(planner_vars.size()) };

      mc_scene->variables_list_->copy_label(label_vars.c_str());
   }

   if (StaticHelper::existsFileSafe(path_prose_description)) {
      mc_scene->scene_description_->value(StaticHelper::readFile(path_prose_description).c_str());
   }
   else {
      mc_scene->scene_description_->value("Waiting for data...");
   }

   std::string path_preview_target{ std::string(mc_scene->getGeneratedDir()) + "/preview/preview.gif" };

   if (!mc_scene->mc_running_internal_ && StaticHelper::existsFileSafe(mc_scene->getGeneratedDir()) && StaticHelper::existsFileSafe(path_preview_target)) {
      auto currentWriteTime = StaticHelper::lastWritetimeOfFileSafe(path_preview_target);

      // Compare the current write time with the previous write time
      if (currentWriteTime != mc_scene->previous_write_time_) {
         // Update the previous write time with the current write time
         mc_scene->previous_write_time_ = currentWriteTime;
         mc_scene->refreshPreview();
      }
   }

   // Spawn Single Experiment Controllers
   try {
      for (const auto& entry : std::filesystem::directory_iterator(path_generated_base_parent)) { // Find current packages.
         std::string possible{ entry.path().filename().string() };
         if (std::filesystem::is_directory(entry) && possible != prefix && StaticHelper::stringStartsWith(possible, prefix)) {
            packages.insert(possible);
         }
      }
   }
   catch (const std::exception& e) {
      // If an exception occurs, just work with however many packages are collected at that point.
   }

   std::set<SingleExpController> to_delete{};
   bool changed{ false };
   for (const auto& el : mc_scene->se_controllers_) { // Find old ones that are not present anymore on the filesystem...
      if (!packages.count(el.getMyId())) {
         to_delete.insert(el);
      }
   }

   for (const auto& del : to_delete) { // ...and delete those.
      mc_scene->window_->remove(del.sec_group_);
      mc_scene->se_controllers_.erase(del);
      Fl::delete_widget(del.sec_group_);
      changed = true;
   }

   for (const auto& el : packages) { // Insert new ones that were not there before.
      bool found{ false };
      for (const auto& el2 : mc_scene->se_controllers_) {
         if (el == el2.getMyId()) {
            found = true;
            break;
         }
      }
      if (!found) {
         mc_scene->se_controllers_.insert(SingleExpController(
            path_template,
            path_generated_base_str,
            el,
            StaticHelper::replaceAll(el, prefix + "_config_", ""),
            mc_scene));
         changed = true;
      }
   }

   if (changed || mc_scene->window_width_archived_ != mc_scene->window_->w()) { // If something changed, redraw.
      static const int MIN_WIDTH{ 100 };
      mc_scene->window_width_archived_ = mc_scene->window_->w();

      if (!mc_scene->sec_scroll_) {
         mc_scene->sec_scroll_ = new Fl_Scroll(10, 220, mc_scene->window_->w() - 20, 120);
         mc_scene->sec_scroll_->type(Fl_Scroll::HORIZONTAL);
         mc_scene->window_->add(mc_scene->sec_scroll_);
      }

      mc_scene->sec_scroll_->resize(10, 220, mc_scene->window_->w() - 20, 120);

      int cnt{ 0 };
      int max{ static_cast<int>(mc_scene->se_controllers_.size()) };
      for (auto& controller : mc_scene->se_controllers_) {
         auto width{ (std::max)(mc_scene->window_->w() / max - 10, MIN_WIDTH) };
         controller.sec_group_->size(width - 5, 70);
         mc_scene->sec_scroll_->add(controller.sec_group_);
         controller.sec_group_->position(width * cnt + 20, 240);
         controller.sec_group_->copy_label(controller.getAbbreviatedShortId().c_str());
         controller.invisible_widget_->copy_tooltip(controller.getShortId().c_str());
         controller.invisible_widget_->callback(MCScene::onGroupClickBM, (void*)&controller);
         cnt++;
      }
   }

   mc_scene->progress_detector_.placeProgressImage(path_template, packages, mc_scene->se_controllers_, path_generated_base_parent);

   // Auto-select if only one run available.
   int num{ 0 };

   for (auto& sec : mc_scene->se_controllers_) {
      if (sec.hasPreview() && sec.hasEnvmodel()) {
         num++;
      }
   }

   if (num == 1) {
      for (auto& sec : mc_scene->se_controllers_) {
         if (sec.hasPreview() && sec.hasEnvmodel()) {
            sec.tryToSelectController();
         }
      }
   }

   for (auto& sec : mc_scene->se_controllers_) {
      if (sec.hasPreview()) {
         if (sec.selected_) {
            sec.sec_group_->color(fl_rgb_color(255, 255, 102));
         }
         else {
            sec.sec_group_->color(fl_rgb_color(255, 255, 204));
         }
      }
      else {
         sec.sec_group_->color(fl_rgb_color(204, 255, 229));
      }

      sec.adjustAbbreviatedShortID();
      sec.registerCallbacks();
      sec.adjustWidgetsAppearances();
   }

   mc_scene->checkbox_json_visible_->position(mc_scene->box_->x(), mc_scene->window_->h() - 120);

   // Make sure we don't override a recent user click on the checkbox.
   // TODO: Just realized that this breaks the idea of only reflecting the stored state on disc. 
   // But it works, and it's only a minor functionality, so let's leave it as is for now.
   // (Just don't use this as a blueprint for the MC run checkboxes.)
   mc_scene->runtime_global_options_->setOptionValue(SecOptionGlobalItemEnum::json_visible, std::to_string(mc_scene->checkbox_json_visible_->value()));
   mc_scene->runtime_global_options_->saveOptions();

   // Actually load the value.
   if (StaticHelper::isBooleanTrue(mc_scene->runtime_global_options_->getOptionValue(SecOptionGlobalItemEnum::json_visible))) {
      mc_scene->checkbox_json_visible_->value(1);
      mc_scene->json_input_->show();
      mc_scene->button_reload_json_->show();
      mc_scene->button_save_json_->show();
      mc_scene->button_check_json_->show();
   }
   else {
      mc_scene->checkbox_json_visible_->value(0);
      mc_scene->json_input_->hide();
      mc_scene->button_reload_json_->hide();
      mc_scene->button_save_json_->hide();
      mc_scene->button_check_json_->hide();
   }

   mc_scene->logging_output_and_interpreter_->position(mc_scene->box_->x(), mc_scene->window_->h() - 100);

   if (mc_scene->getValueForJSONKeyAsString("ShowLOG", JSON_TEMPLATE_DENOTER) == "true")
   {
      mc_scene->logging_output_and_interpreter_->show();
   }
   else {
      mc_scene->logging_output_and_interpreter_->hide();
   }

   // Show building blocks as draggable boxes.
   if (mc_scene->button_run_parser_->active()) { // Only if the buttons are active, i.e., if no jobs are running in the background.
      std::lock_guard<std::mutex> lock{ mc_scene->parser_mutex_ };
      std::map<std::string, std::pair<std::string, std::string>> new_bbs{ mc_scene->loadNewBBsFromJson() };
      std::set<std::string> to_delete_bbs{};
      std::set<std::string> to_add_bbs{};

      for (const auto& bb_old : mc_scene->bb_groups_) {
         if (!new_bbs.count(bb_old.first)) {
            to_delete_bbs.insert(bb_old.first);
         }
      }

      for (const auto& bb_new : new_bbs) {
         if (!mc_scene->bb_groups_.count(bb_new.first)) {
            to_add_bbs.insert(bb_new.first);
         }
      }

      for (const auto& del_el : to_delete_bbs) {
         delete mc_scene->bb_groups_.at(del_el).second;
         mc_scene->bb_groups_.erase(del_el);
      }

      int offset_x{ 0 };
      int offset_y{ mc_scene->variables_list_->h() + 10 };
      std::string spec{};

      int row_width_limit =
         mc_scene->scene_description_->x()
         - mc_scene->json_input_->x() 
         - mc_scene->getActualJSONWidth()
         - 20;
      
      for (const auto& fmla_str_name : to_add_bbs) {
         if (!StaticHelper::stringContains(fmla_str_name, "SPEC")) {
            int box_size{ (int)fmla_str_name.size() * 7 };
            
            // Calculate the total width of the boxes in the current row
            auto bb_group{ new DragGroup(0, 0, box_size + 20, 30, new_bbs.at(fmla_str_name).second, fmla_str_name, new_bbs.at(fmla_str_name).first, mc_scene) };
            Fl_Box* box1 = new Fl_Box(2, 2, box_size + 10, 30);
            bb_group->add(box1);
            box1->copy_label(fmla_str_name.c_str());
            int center_x = (bb_group->w() - box1->w()) / 2;
            int center_y = (bb_group->h() - box1->h()) / 2;
            box1->position(center_x, center_y);
            bb_group->callback(bbBoxCallback, mc_scene);
            bb_group->getEditor().slider_->callback(bbBoxSliderCallback, bb_group);
            bb_group->end();
            bb_group->copy_tooltip((new_bbs.at(fmla_str_name).second + " " + new_bbs.at(fmla_str_name).first).c_str());

            // Position the bb_group below each other in a row-wise manner
            int x{}, y{};

            // Check if the current row width exceeds the limit
            if (offset_x + bb_group->w() > row_width_limit) {
               offset_y = offset_y + bb_group->h() + 5;
               offset_x = 0; // Reset the current row width

               x = mc_scene->json_input_->x() + mc_scene->getActualJSONWidth() + 10 + offset_x;
               y = mc_scene->json_input_->y() + offset_y;
            }
            else {
               x = mc_scene->json_input_->x() + mc_scene->getActualJSONWidth() + 10 + offset_x;
               y = mc_scene->json_input_->y() + offset_y;

            }
            
            offset_x += bb_group->w() + 5; // Update the current row width

            bb_group->position(x, y);
            mc_scene->window_->add(bb_group);
            mc_scene->bb_groups_.insert({ fmla_str_name, { new_bbs.at(fmla_str_name).second, bb_group } });
         }
         else {
            spec = fmla_str_name;
         }
      }

      if (to_add_bbs.count(spec)) {
         int box_size{ (int)spec.size() * 10 };

         auto bb_group{ new DragGroup{0, 0, box_size + 20, 30, new_bbs.at(spec).second, spec, new_bbs.at(spec).first, mc_scene} }; // TODO: DBL CODE
         Fl_Box* box1 = new Fl_Box(2, 2, box_size + 10, 400);
         bb_group->add(box1);
         box1->copy_label(spec.c_str());
         int center_x = (bb_group->w() - box1->w()) / 2;
         int center_y = (bb_group->h() - box1->h()) / 2;
         box1->position(center_x, center_y);
         bb_group->callback(bbBoxCallback, mc_scene);
         bb_group->end();
         bb_group->copy_tooltip(new_bbs.at(spec).first.c_str());
         bb_group->position(mc_scene->json_input_->x() + mc_scene->getActualJSONWidth() + 10, mc_scene->json_input_->y() + offset_y + bb_group->h() + 5);
         mc_scene->window_->add(bb_group);
         mc_scene->bb_groups_.insert({ spec, { new_bbs.at(spec).second, bb_group } });
      }

      if (!to_add_bbs.empty()) mc_scene->showAllBBGroups(false);
      mc_scene->window_->redraw();
   }

   Fl::repeat_timeout(TIMEOUT_RARE, refreshRarely, data);
}

std::pair<std::vector<std::string>, std::vector<std::string>> getCommonAndUniqueStrings(
   const std::vector<std::set<std::string>>& sets) {
   std::vector<std::string> commonStrings{};
   std::vector<std::string> uniqueStrings{};

   if (sets.empty()) return { {}, {} };

   // Check if a string is present in all sets
   for (const std::string& str : sets.at(0)) {
      bool isCommon = true;
      for (const auto& set2 : sets) {
         if (!set2.count(str)) {
            isCommon = false;
            break;
         }
      }
      if (isCommon) {
         commonStrings.push_back(str);
      }
   }

   // Check if a string is present in at least one set, but not in all sets
   for (const auto& set : sets) {
      for (const std::string& str : set) {
         bool isCommon = true;
         for (const auto& set2 : sets) {
            if (!set2.count(str)) {
               isCommon = false;
            }
         }
         if (!isCommon) {
            uniqueStrings.push_back(str);
         }
      }
   }

   return { commonStrings, uniqueStrings };
}

void MCScene::refreshSometimes(void* data)
{
   auto mc_scene{ static_cast<MCScene*>(data) };

   if (mc_scene->button_run_parser_->active()) { // Don't display this warning when jobs are running.
      if (!test::isCacheUpToDateWithTemplates(mc_scene->getCachedDir(), mc_scene->getTemplateDir(), GUI_NAME + "_Related")) {
         mc_scene->addWarning("Cache in '" + mc_scene->getCachedDir() + "' is outdated w.r.t. the template files in '" + mc_scene->getTemplateDir() + "'.\n"
            + "All cached entries will be deleted on next click on 'Create EnvModels...'. <Delete cache with right-click on the button to stop this message.>");
      }
   }

   mc_scene->window_->redraw();
   Fl::repeat_timeout(TIMEOUT_SOMETIMES, refreshSometimes, data);
   std::vector<std::set<std::string>> envmodel_sets{};
   std::set<std::string> planner_set{};
   bool planner_done{ false };

   for (const auto& sec : mc_scene->se_controllers_) {
      std::filesystem::path path_envmodel_vars{ (sec.getPathOnDisc() / "envmodel-variables.txt") };

      if (StaticHelper::existsFileSafe(path_envmodel_vars)) { // Collect EnvModel variables.
         try {
            std::string vars_str{ StaticHelper::readFile(path_envmodel_vars.string()) };
            std::vector<std::string> vars{ StaticHelper::split(vars_str, "\n") };
            std::set<std::string> set{ vars.begin(), vars.end() };
            envmodel_sets.push_back(set);
         }
         catch (const std::exception& e) {}
      }

      std::filesystem::path path_planner{ (sec.getPathOnDisc() / "planner-variables.txt") };
      if (!planner_done && StaticHelper::existsFileSafe(path_planner)) { // Collect Planner variables.
         try {
            std::string vars_str{ StaticHelper::readFile(path_planner.string()) };
            std::vector<std::string> vars{ StaticHelper::split(vars_str, "\n") };
            planner_set.insert(vars.begin(), vars.end());
         }
         catch (const std::exception& e) {}

         planner_done = true; // Do only once since planner is always equal.
      }
   }

   auto pair = getCommonAndUniqueStrings(envmodel_sets);
   auto common = pair.first;
   auto unique = pair.second;
   std::string common_str{};
   std::string unique_str{};
   std::string planner_str{};

   for (const auto& str : common) {
      if (!str.empty()) common_str += "env." + str + ",";
   }

   for (const auto& str : unique) {
      if (!str.empty()) unique_str += "env." + str + ",";
   }

   for (const auto& str : planner_set) {
      if (!str.empty()) planner_str += "planner.:::" + str + ":::,";
   }

   if (!unique_str.empty()) unique_str.pop_back();
   if (!common_str.empty()) common_str.pop_back();
   if (!planner_str.empty()) planner_str.pop_back();

   mc_scene->runtime_global_options_->setOptionValue(SecOptionGlobalItemEnum::envmodel_variables_general, common_str);
   mc_scene->runtime_global_options_->setOptionValue(SecOptionGlobalItemEnum::envmodel_variables_specific, unique_str);
   mc_scene->runtime_global_options_->setOptionValue(SecOptionGlobalItemEnum::planner_variables, planner_str);
   mc_scene->runtime_global_options_->saveOptions();
}

void MCScene::refreshFrequently(void* data)
{
   auto mc_scene{ static_cast<MCScene*>(data) };

   mc_scene->window_->redraw();
   Fl::repeat_timeout(TIMEOUT_FREQUENT, refreshFrequently, data);

   std::string current_content = StaticHelper::trimAndReturn(static_cast<std::ostringstream*>(ADDITIONAL_LOGGING_PIPE)->str());
   if (!current_content.empty()) {
      mc_scene->logging_output_and_interpreter_->append((current_content + "\n").c_str());
      static_cast<std::ostringstream*>(ADDITIONAL_LOGGING_PIPE)->str("");
   }
}

void MCScene::jsonChangedCallback(Fl_Widget* widget, void* data) {
   auto mc_scene{ static_cast<MCScene*>(data) };
   mc_scene->button_check_json_->color(FL_YELLOW);
}



void MCScene::runMCJob(MCScene* mc_scene, const std::string& path_generated_raw, const std::string config_name)
{
   std::string path_generated{ StaticHelper::replaceAll(path_generated_raw, "\\", "/") };
   std::string path_template{ mc_scene->getTemplateDir() };

   mc_scene->addNote("Running model checker and creating preview for folder '" + path_generated + "' (config: '" + config_name + "').");

   StaticHelper::removeFileSafe(path_generated + "/" + "debug_trace_array.txt");
   StaticHelper::removeFileSafe(path_generated + "/" + "debug_trace_array.smv");
   StaticHelper::removeFileSafe(path_generated + "/" + "debug_trace_array_unscaled.smv");
   StaticHelper::removeFileSafe(path_generated + "/" + "mc_runtimes.txt");
   StaticHelper::removeFileSafe(path_generated + "/" + "prose_scenario_description.txt");
   StaticHelper::removeAllFilesSafe(path_generated + "/" + "preview");
   StaticHelper::removeAllFilesSafe(path_generated + "/" + "preview2");

   mc_scene->preprocessAndRewriteJSONTemplate();

   std::string main_smv{ StaticHelper::readFile(path_generated + "/main.smv") };

   mc_scene->putJSONIntoDataPack();
   std::string script_template{ StaticHelper::readFile(mc_scene->getTemplateDir() + "/script.tpl") };
   std::string generated_script{ CppParser::generateScript(script_template, mc_scene->data_, mc_scene->parser_) };
   StaticHelper::writeTextToFile(generated_script, path_generated + "/script.cmd");

   mc_scene->addNote("Created script.cmd with the following content:\n" + StaticHelper::readFile(path_generated + "/script.cmd") + "<EOF>");

   static const std::string SPEC_BEGIN{ "--SPEC-STUFF" };
   static const std::string SPEC_END{ "--EO-SPEC-STUFF" };

   main_smv = StaticHelper::removeMultiLineComments(main_smv, SPEC_BEGIN, SPEC_END);
   main_smv += SPEC_BEGIN + "\n";
   
   auto spec_pair = mc_scene->getSpec(config_name);
   main_smv += spec_pair.second + "\n";
   main_smv += SPEC_END + "\n";

   StaticHelper::writeTextToFile(main_smv, path_generated + "/main.smv");

   test::convenienceArtifactRunHardcoded(test::MCExecutionType::mc, path_generated, "FAKE_PATH_NOT_USED", path_template, "FAKE_PATH_NOT_USED", mc_scene->getCachedDir(), mc_scene->path_to_external_folder_);
   mc_scene->generatePreview(path_generated, 0);

   mc_scene->addNote("Model checker run finished for folder '" + path_generated + "'.");
}

void vfm::MCScene::generatePreview(const std::string& path_generated, const int cex_num)
{
   addNote("Generating preview for folder '" + path_generated + "'.");

   mc::trajectory_generator::VisualizationLaunchers::quickGeneratePreviewGIFs(
      { cex_num },
      path_generated,
      "debug_trace_array",
      mc::trajectory_generator::CexTypeEnum::smv);

   refreshPreview();
}

void vfm::MCScene::runMCJobs(MCScene* mc_scene)
{
   const std::string path_generated_base_str{ mc_scene->getGeneratedDir() };
   const std::filesystem::path path_generated_base(path_generated_base_str);
   std::filesystem::path path_generated_base_parent = path_generated_base.parent_path();
   std::string prefix{ path_generated_base.filename().string() };

   mc_scene->deletePreview(path_generated_base_str, false);
   std::vector<std::string> possibles{};

   for (const auto& entry : std::filesystem::directory_iterator(path_generated_base_parent)) {
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

         if (StaticHelper::isBooleanTrue(mc_scene->getOptionFromSECConfig(config_name, SecOptionLocalItemEnum::selected_job))) {
            pool.enqueue([mc_scene, folder, path_generated_base_parent, prefix] {
               MCScene::runMCJob(mc_scene, folder, folder.substr(path_generated_base_parent.string().size() + prefix.size() + 1));
            });
         }
      }
   }

   std::filesystem::path path_preview_bb{};
   for (const auto& folder : possibles) { // Prepare directories for previews.
      path_preview_bb = path_generated_base;
      
      std::string spec{ mc_scene->getValueForJSONKeyAsString("SPEC", JSON_TEMPLATE_DENOTER) };
      bool is_ltl{ mc_scene->isLTL(JSON_TEMPLATE_DENOTER) };
      spec = StaticHelper::replaceAll(spec, JSON_NUXMV_FORMULA_BEGIN, "");
      spec = StaticHelper::replaceAll(spec, JSON_NUXMV_FORMULA_END, "");
      spec = StaticHelper::replaceAll(spec, ";", "");
      std::string type_str{ is_ltl ? "LTLSPEC" : "INVARSPEC" };

      spec = StaticHelper::trimAndReturn(StaticHelper::replaceAll(StaticHelper::replaceAll(spec, "LTLSPEC", ""), "INVARSPEC", ""));
      auto spec_fmla{ MathStruct::parseMathStruct(spec, mc_scene->parser_, mc_scene->data_, DotTreatment::as_operator)->toTermIfApplicable() };
      spec_fmla = _id(mc::simplification::simplifyFast(spec_fmla, mc_scene->parser_));

      spec_fmla->applyToMeAndMyChildren([](const MathStructPtr m) { // Replace function stuff with variables: "GapLeft(20, 20) ==> GapLeft"
         if (m->isTermCompound() && !MathStruct::DEFAULT_EXCEPTIONS_FOR_FLATTENING.count(m->getOptorJumpIntoCompound())) {
            m->replaceJumpOverCompounds(_var(m->getOptorJumpIntoCompound()));
         }
      });

      spec = type_str + " " + spec_fmla->child0()->serializePlainOldVFMStyle(MathStruct::SerializationSpecial::enforce_square_array_brackets);

      auto previews_parent_path{ path_preview_bb.parent_path() / "previews" };
      int cnt{ 0 };

      for (;; cnt++) {
         if (StaticHelper::existsFileSafe(previews_parent_path / ("spec_" + std::to_string(cnt)))) {
            std::string stored_spec{ StaticHelper::readFile((previews_parent_path / ("spec_" + std::to_string(cnt)) / "spec.txt").string())};
            if (stored_spec == spec) {
               break;
            }
         }
         else {
            break;
         }
      }

      path_preview_bb = previews_parent_path / ("spec_" + std::to_string(cnt));
      StaticHelper::createDirectoriesSafe(path_preview_bb);

      StaticHelper::writeTextToFile(spec, (path_preview_bb / "spec.txt").string());

      for (int cnt{ 0 }; ; cnt++) {
         if (!StaticHelper::existsFileSafe((path_preview_bb / std::to_string(cnt)).string())) {
            path_preview_bb /= std::to_string(cnt);
            break;
         }
      }

      StaticHelper::createDirectoriesSafe(path_preview_bb);

      // Copy bb previews to resp. folders.
      try {
         for (const auto& entry : std::filesystem::directory_iterator(folder + "/preview2")) {
            try {
               if (!std::filesystem::is_directory(entry)) {
                  std::filesystem::copy(entry, path_preview_bb);
               }
            }
            catch (const std::exception& e) {
               // ...
            }
         }
      }
      catch (const std::exception& e) {
         // ...
      }
   }

   mc_scene->mc_running_internal_ = false;
   mc_scene->activateMCButtons(true, ButtonClass::All);
}

void MCScene::deletePreview(const std::string& path_generated, const bool actually_delete_gif) 
{
   std::string path_generated_preview{ path_generated + "/preview" };
   std::string path_generated_preview2{ path_generated + "/preview2" };
   std::string path_prose_description{ path_generated + "/" + PROSE_DESC_NAME };
   std::string path_planner_smv{ path_generated + "/planner.cpp_combined.k2.smv" };
   std::string path_result{ path_generated + "/debug_trace_array.txt" };

   if (actually_delete_gif) {
      addNote("Deleting folder '" + path_generated_preview + "'.");
      StaticHelper::removeAllFilesSafe(path_generated_preview);
      addNote("Deleting folder '" + path_generated_preview2 + "'.");
      StaticHelper::removeAllFilesSafe(path_generated_preview2);
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

   for (auto& sec : se_controllers_) {
      sec.selected_ = false;
   }
}

nlohmann::json MCScene::getJSON() const
{
   std::string json_text{ json_input_->buffer()->text() };
   nlohmann::json json{};

   try {
      json = nlohmann::json::parse(json_text);
   }
   catch (const nlohmann::json::parse_error& e) {
      addError("JSON parsing error.");
      button_check_json_->color(FL_DARK_RED);
   }

   return json;
}

nlohmann::json MCScene::instanceFromTemplate(
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

                     if (key == "SPEC") {
                        val_str = std::string(is_ltl ? "LTLSPEC " : "INVARSPEC ") + val_str + ";";
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

void vfm::MCScene::evaluateFormulasInJSON(const nlohmann::json j_template)
{
   std::lock_guard<std::mutex> lock{ formula_evaluation_mutex_ };
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
                     int fct_name_begin_index{ StaticHelper::indexOfFirstInnermostBeginBracket(val_str, "(", ")")};
                     int fct_name_end_index{ StaticHelper::findMatchingEndTagLevelwise(val_str, fct_name_begin_index, "(", ")") };
                     int fct_pars_begin_index{ StaticHelper::indexOfFirstInnermostBeginBracket(val_str, "{", "}")};
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

void vfm::MCScene::preprocessAndRewriteJSONTemplate()
{
   std::lock_guard<std::mutex> lock(parser_mutex_);
   const std::string path_template{ getTemplateDir() };
   const std::string path_json_template{ path_template + "/" + json_tpl_filename_ };
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
      int address{ (int) data_->getSingleVal(var) };

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

   loadJsonText();
   button_check_json_->color(FL_YELLOW);
   setTitle();
}

void MCScene::buttonSaveJSON(Fl_Widget* widget, void* data) {
   auto mc_scene{ static_cast<MCScene*>(data) };
   mc_scene->showAllBBGroups(false);
   mc_scene->saveJsonText();

}

void MCScene::buttonReloadJSON(Fl_Widget* widget, void* data) {
   auto mc_scene{ static_cast<MCScene*>(data) };
   mc_scene->evaluateFormulasInJSON(mc_scene->getJSON());

   mc_scene->showAllBBGroups(false);
   mc_scene->loadJsonText();
   mc_scene->button_check_json_->color(FL_YELLOW);
   mc_scene->setTitle();

   for (const auto& bb_el : mc_scene->bb_groups_) {
      delete bb_el.second.second;
   }

   mc_scene->bb_groups_.clear();
}

void MCScene::buttonCheckJSON(Fl_Widget* widget, void* data) {
   auto mc_scene{ static_cast<MCScene*>(data) };
   std::string json_text{ mc_scene->json_input_->buffer()->text() };

   try {
      nlohmann::json json = nlohmann::json::parse(json_text);
      mc_scene->button_check_json_->color(FL_GREEN);
      //mc_scene->spec_input->value(mc_scene->getSpecAndBBs().back().c_str());
   }
   catch (const nlohmann::json::parse_error& e) {
      mc_scene->button_check_json_->color(FL_DARK_RED);
   }
}

void vfm::MCScene::resetParserAndData()
{
   std::lock_guard<std::mutex> lock{ parser_mutex_ };

   parser_ = std::make_shared<FormulaParser>();
   data_ = std::make_shared<DataPack>();
   parser_->addDefaultDynamicTerms();
   parser_->addnuSMVOperators(TLType::LTL);
}

void deleteCurrentPreview(MCScene* mc_scene) // Free function that actually does it, ran in a thread from the callback.
{
   mc_scene->activateMCButtons(false, ButtonClass::RunButtons);
   std::string path_generated_base_str{ mc_scene->getGeneratedDir() };
   std::filesystem::path path_generated_base(path_generated_base_str);
   std::filesystem::path path_generated_base_parent = path_generated_base.parent_path();
   std::string prefix{ path_generated_base.filename().string() };

   for (const auto& entry : std::filesystem::directory_iterator(path_generated_base_parent)) {
      std::string possible{ entry.path().filename().string() };
      if (std::filesystem::is_directory(entry) && possible != prefix && StaticHelper::stringStartsWith(possible, prefix)) {
         if (!StaticHelper::isBooleanTrue(mc_scene->getOptionFromSECConfig(possible, SecOptionLocalItemEnum::selected_job))) {
            mc_scene->deletePreview(entry.path().string(), true);
         }
      }
   }

   mc_scene->deletePreview(path_generated_base_str, false);
   mc_scene->resetParserAndData();
   mc_scene->activateMCButtons(true, ButtonClass::All);
}

void deleteTestCases(MCScene* mc_scene) // Free function that actually does it, ran in a thread from the callback.
{
   mc_scene->activateMCButtons(false, ButtonClass::RunButtons);
   std::string path_generated_base_str{ mc_scene->getGeneratedDir() };
   std::filesystem::path path_generated_base(path_generated_base_str);
   std::filesystem::path path_generated_base_parent = path_generated_base.parent_path();
   std::string prefix{ path_generated_base.filename().string() };

   for (const auto& entry : std::filesystem::directory_iterator(path_generated_base_parent)) {
      std::string possible{ entry.path().filename().string() };
      if (std::filesystem::is_directory(entry) && possible != prefix && StaticHelper::stringStartsWith(possible, prefix)) {
         if (!StaticHelper::isBooleanTrue(mc_scene->getOptionFromSECConfig(possible, SecOptionLocalItemEnum::selected_job))) { // Delete only unselected.
            std::vector<std::string> folders{
               entry.path().string() + "/cex-birdseye",
               entry.path().string() + "/cex-cockpit-only",
               entry.path().string() + "/cex-full",
               entry.path().string() + "/cex-smooth-birdseye",
               entry.path().string() + "/cex-smooth-full",
               entry.path().string() + "/cex-smooth-with-arrows-birdseye",
               entry.path().string() + "/cex-smooth-with-arrows-full",
            };

            for (const auto& folder : folders) {
               if (StaticHelper::existsFileSafe(folder)) {
                  mc_scene->addNote("Deleting folder '" + folder + "'.");
                  StaticHelper::removeAllFilesSafe(folder);
               }
            }
         }
      }
   }

   mc_scene->activateMCButtons(true, ButtonClass::All);
}

void deleteFolders(MCScene* mc_scene) // Free function that actually does it, ran in a thread from the callback.
{
   mc_scene->activateMCButtons(false, ButtonClass::RunButtons);
   std::string path_generated_base_str{ mc_scene->getGeneratedDir() };
   std::filesystem::path path_generated_base(path_generated_base_str);
   std::filesystem::path path_generated_base_parent = path_generated_base.parent_path();
   std::string prefix{ path_generated_base.filename().string() };

   try {
      for (const auto& entry : std::filesystem::directory_iterator(path_generated_base_parent)) {
         std::string possibly_to_delete{ entry.path().filename().string() };
         //std::cout << possibly_to_delete << std::endl;

         try {
            if (std::filesystem::is_directory(entry) && StaticHelper::stringStartsWith(possibly_to_delete, prefix)) {
               std::string path_generated{ entry.path().string() };

               if (possibly_to_delete == prefix) { // It's the main folder, NOT one of the configs.)
                  StaticHelper::removeAllFilesSafe(path_generated + "/preview");
                  StaticHelper::removeAllFilesSafe(path_generated + "/prose_scenario_description.txt");
               }
               else if (!StaticHelper::isBooleanTrue(mc_scene->getOptionFromSECConfig(possibly_to_delete, SecOptionLocalItemEnum::selected_job))) {
                  mc_scene->addNote("Deleting folder '" + path_generated + "'.");

                  StaticHelper::removeAllFilesSafe(path_generated, false);
               }
            }
         }
         catch (const std::exception& e) {
            mc_scene->addWarning("Deleting possible folder '" + possibly_to_delete + "' not possible due to error: " + std::string(e.what()));
         }
      }
   } catch (const std::exception& e) {
      mc_scene->addWarning("Deleting folders not possible due to error: " + std::string(e.what()));
   }

   mc_scene->copyWaitingForPreviewGIF();
   mc_scene->activateMCButtons(true, ButtonClass::All);
}

void deleteCached(MCScene* mc_scene) // Free function that actually does it, ran in a thread from the callback.
{
   mc_scene->activateMCButtons(false, ButtonClass::RunButtons);
   std::string path_cached{ mc_scene->getCachedDir() };
   mc_scene->addNote("Deleting folder '" + path_cached + "'.");
   StaticHelper::removeAllFilesSafe(path_cached);
   mc_scene->activateMCButtons(true, ButtonClass::All);
}

void MCScene::buttonDeleteCurrentPreview(Fl_Widget* widget, void* data)
{
   auto mc_scene{ static_cast<MCScene*>(data) };
   mc_scene->showAllBBGroups(false);
   std::thread t{ deleteCurrentPreview, mc_scene };
   t.detach();
}

void MCScene::buttonDeleteTestCases(Fl_Widget* widget, void* data)
{
   auto mc_scene{ static_cast<MCScene*>(data) };
   mc_scene->showAllBBGroups(false);
   std::thread t{ deleteTestCases, mc_scene };
   t.detach();
}

void MCScene::buttonDeleteGenerated(Fl_Widget* widget, void* data)
{
   const auto mc_scene{ static_cast<MCScene*>(data) };
   mc_scene->showAllBBGroups(false);
   std::thread t{ deleteFolders, mc_scene };
   t.detach();
}

void vfm::MCScene::buttonDeleteCached(Fl_Widget* widget, void* data)
{
   auto mc_scene{ static_cast<MCScene*>(data) };
   mc_scene->showAllBBGroups(false);
   if (Fl::event_button() == FL_RIGHT_MOUSE) { // Only delete cache on right-click.
      const auto mc_scene{ static_cast<MCScene*>(data) };
      std::thread t{ deleteCached, mc_scene };
      t.detach();
   }
   else {
      mc_scene->addNote("DOING NOTHING! Use right button if you really want to delete the cache.");
   }
}

void MCScene::buttonRunMCAndPreview(Fl_Widget* widget, void* data) {
   auto mc_scene{ static_cast<MCScene*>(data) };
   mc_scene->showAllBBGroups(false);
   mc_scene->activateMCButtons(false, ButtonClass::RunButtons);
   mc_scene->mc_running_internal_ = true;

   //generatePreviews(mc_scene);
   std::thread t{ runMCJobs, mc_scene };
   t.detach();
}

void MCScene::doAllEnvModelGenerations(MCScene* mc_scene)
{
   mc_scene->activateMCButtons(false, ButtonClass::RunButtons);
   mc_scene->showAllBBGroups(false);

   std::string path{ mc_scene->getTemplateDir() };
   std::string generated_dir{ mc_scene->getGeneratedDir() };
   std::string cached_dir{ mc_scene->getCachedDir() };
   std::string path_planner{ mc_scene->getBPIncludesFileDir() };
   std::string path_json{ path + "/" + FILE_NAME_JSON };
   std::string path_envmodel{ path + "/EnvModel.tpl" };
   std::string template_dir{ mc_scene->getTemplateDir() };

   mc_scene->saveJsonText();
   mc_scene->resetParserAndData();
   mc_scene->preprocessAndRewriteJSONTemplate();

   auto envmodeldefs{ test::retrieveEnvModelDefinitionFromJSON(path_json, test::EnvModelCachedMode::always_regenerate) };

   std::map<test::EnvModelConfig, std::string> env_model_configs{};
   std::set<std::string> relevant_variables{};
   int max{ (int) envmodeldefs.size() };
   int cnt{ 1 };

   for (const auto& envmodeldef : envmodeldefs) { // Create empty dirs to make visualization nicer.
      if (envmodeldef.first != JSON_TEMPLATE_DENOTER) {
         StaticHelper::createDirectoriesSafe(generated_dir + envmodeldef.first);
      }
   }

   for (const auto& envmodeldef : envmodeldefs) {
      mc_scene->addNote("Generating EnvModel " + std::to_string(cnt++) + "/" + std::to_string(max) + ".");

      if (envmodeldef.first != JSON_TEMPLATE_DENOTER) {
         test::doParsingRun( // TODO: go over command line!
            envmodeldef,
            ".",
            path_envmodel,
            path_planner,
            generated_dir,
            cached_dir,
            GUI_NAME + "_Related");
      }
   }

   mc_scene->activateMCButtons(true, ButtonClass::All);
}

void MCScene::onGroupClickBM(Fl_Widget* widget, void* data)
{
   const auto controller{ static_cast<SingleExpController*>(data) };
   const std::string path_generated_base_str{ controller->generated_path_ };
   const std::filesystem::path path_generated_base(path_generated_base_str);
   const std::string generated_basename{ path_generated_base.filename().string() };
   const std::filesystem::path path_generated_base_parent = path_generated_base.parent_path();
   const std::filesystem::path source_path{ path_generated_base_parent / controller->getMyId() };

   controller->mc_scene_->showAllBBGroups(false);

   if (Fl::event_button() == FL_RIGHT_MOUSE) {
      openExplorerWindow(source_path.string().c_str());
      return;
   }

   if (Fl::event_clicks()) { // Double-click.
      for (const auto& other : controller->mc_scene_->se_controllers_) {
         other.selected_ = false;
      }

      controller->mc_scene_->deletePreview(controller->mc_scene_->getGeneratedDir(), false);

      return;
   }

   controller->tryToSelectController();
}

void MCScene::buttonReparse(Fl_Widget* widget, void* data)
{
   auto mc_scene{ static_cast<MCScene*>(data) };
   mc_scene->showAllBBGroups(false);
   std::thread t{ doAllEnvModelGenerations, mc_scene };
   t.detach();
}

void vfm::MCScene::createTestCase(const MCScene* mc_scene, const std::string& generated_parent_dir, const int cnt, const int max, const std::string& id, const int cex_num)
{
   mc_scene->addNote("Running test case generation " + std::to_string(cnt) + "/" + std::to_string(max) + " for '" + id + "'.");

   mc::trajectory_generator::VisualizationLaunchers::quickGenerateGIFs(
      { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9 }, // TODO: for now only first CEX is processed if several given.
      generated_parent_dir + "/" + id,
      "debug_trace_array",
      mc::trajectory_generator::CexTypeEnum::smv,
      true, // export basics
      true, // export with smooth arrows
      false, // export without smooth arrows
      { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10 });

   mc_scene->addNote("Finished test case generation for '" + id + "'.");
}

void MCScene::createTestCases(MCScene* mc_scene)
{
   mc_scene->activateMCButtons(false, ButtonClass::RunButtons);

   int cnt{ 1 };
   int max{ (int)mc_scene->se_controllers_.size() };
   std::vector<std::thread> threads{};
   int numThreads{ 0 };

   for (const auto& sec : mc_scene->se_controllers_) {
      if (StaticHelper::isBooleanTrue(sec.runtime_local_options_.getOptionValue(SecOptionLocalItemEnum::selected_job))) {
         bool spawned{ false };

         while (!spawned) {
            if (numThreads < test::MAX_THREADS) {
               threads.emplace_back(std::thread(
                  MCScene::createTestCase, mc_scene, mc_scene->getGeneratedParentDir(), cnt++, max, sec.getMyId(), sec.slider_->value()));
               spawned = true;
               numThreads++;
            }
            else if (!threads.empty()) {
               threads.front().join();
               threads.erase(threads.begin());
               numThreads--;
            }
         }
      }
   }

   for (auto& t : threads) t.join();

   mc_scene->activateMCButtons(true, ButtonClass::All);
}

void MCScene::buttonCEX(Fl_Widget* widget, void* data)
{
   auto mc_scene{ static_cast<MCScene*>(data) };
   mc_scene->showAllBBGroups(false);
   std::thread t{ createTestCases, mc_scene };
   t.detach();
}

std::string getElapsedTimeNuxmv(const std::string& runtime_file_content)
{
   std::regex regex(R"(.*(\d{2}:\d{2}:\d{2}\.\d{3}).*)");
   std::smatch match{};

   std::string lastElapsedTime{};

   // Iterate over each line and update the lastElapsedTime if a match is found
   std::istringstream iss{ runtime_file_content };
   std::string line{};
   while (std::getline(iss, line)) {
      bool res{ std::regex_match(line, match, regex) };
      res = res && StaticHelper::stringContains(line, "script.cmd"); // Identify nuXmv line (using "script", because "nuXmv" is also present in kratos lines).

      if (res) {
         lastElapsedTime = match[1];
      }
   }

   return lastElapsedTime; // Returns empty string if none matched.
}

void vfm::MCScene::buttonRuntimeAnalysis(Fl_Widget* widget, void* data)
{
   auto mc_scene{ static_cast<MCScene*>(data) };
   mc_scene->showAllBBGroups(false);
   std::string res{};

   if (mc_scene->se_controllers_.empty()) return;

   for (const auto& varname : mc_scene->se_controllers_.begin()->getVariableNames()) {
      res += varname + ";";
   }
   res += "runtime_ms;runtime_str\n";

   for (const auto& sec : mc_scene->se_controllers_) {
      std::filesystem::path runtime_sec_path{ sec.getPathOnDisc() / "mc_runtimes.txt" };
      long long time_ms{ -100000 };
      std::string time_str{};

      try { // Skip over missing "runtime" files and files containing no runtime for nuXmv. I.e., time_ms stays inf.
         if (StaticHelper::existsFileSafe(runtime_sec_path)) {
            std::string file_content{ StaticHelper::readFile(runtime_sec_path.string()) };
            time_str = getElapsedTimeNuxmv(file_content);

            if (!time_str.empty()) {
               time_ms = StaticHelper::convertTimeStampToMilliseconds(time_str);
            }
         }
      }
      catch (const std::exception& e) {}

      for (const auto& varval : sec.getVariableValues()) {
         res += StaticHelper::floatToStringNoTrailingZeros(varval) + ";";
      }

      res += std::to_string(time_ms) + ";" + time_str + "\n";
   }

   const std::string runtime_analysis_path{ mc_scene->getGeneratedDir() + "/runtime_summary.csv" };
   const std::string abs_path{ StaticHelper::absPath(runtime_analysis_path) };
   const std::string tikz_path{ StaticHelper::removeLastFileExtension(abs_path) + ".tex" };

   mc_scene->addNote("Storing runtime analysis in '" + abs_path + "'.");
   StaticHelper::writeTextToFile(res, runtime_analysis_path);

   mc_scene->addNote("Creating tikz plot in '" + abs_path + "'.");
   mc::Analyzor a{};
   a.createTikzPlots(abs_path, tikz_path, false);
}
