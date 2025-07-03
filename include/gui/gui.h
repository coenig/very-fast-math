//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file
#pragma once

#include "testing/interactive_testing.h"
#include "static_helper.h"
#include "json_parsing/json.hpp"
#include "failable.h"
#include "gui/interpreter_terminal.h"
#include "gui/gui_options.h"
#include "gui/custom_widgets.h"

#include <FL/Fl.H>
#include <FL/Fl_Window.H>
#include <FL/Fl_Double_Window.H>
#include <FL/Fl_Button.H>
#include <FL/Fl_Anim_GIF_Image.H>
#include <FL/Fl_Multiline_Input.H>
#include <FL/Fl_Multiline_Output.H>
#include <FL/Fl_Native_File_Chooser.H>
#include <FL/Fl_Group.H>
#include <FL/Fl_Output.H>
#include <FL/Fl_PNG_Image.H>
#include <FL/fl_draw.H>

#include <filesystem>
#include <iostream>
#include <vector>
#include <string>
#include <thread>
#include <mutex>

namespace vfm {

static const std::string GUI_NAME{ "M²oRTy" };
static const std::string PROSE_DESC_NAME{ "prose_scenario_description.txt" };
static const std::string FILE_NAME_JSON{ "envmodel_config.json" };
static float TIMEOUT_FREQUENT{ 0.05 };
static float TIMEOUT_RARE{ 0.5 };
static float TIMEOUT_SOMETIMES{ 2 };
static std::string REGEX_PATTERN_BB_NAMES                      { "[a-zA-Z0-9_]+" };
static std::string REGEX_PATTERN_BB_PARAMETERS                 { "[a-zA-Z0-9_,\\s]*" };
static std::string REGEX_PATTERN_BB_DECLARATION_WITH_PARAMETERS{ "[a-zA-Z0-9_,\\s()]+" };
static constexpr int INTERPRETER_TERMINAL_HEIGHT{ 150 };

static bool detached_run_running{ false };

/// 
/// PHILOSOPHY 
///
/// The overall philosophy behind the morty GUI is data-driven, meaning, we keep the GUI logic as simple as possible.
/// 
/// This means concretely that the GUI should be seen -- as far as possible -- as a mere visualization of a system state
/// stored on disc. Operations should be kept simple and their result immediately stored on disc, such that the GUI can react to it.
/// Thus, terminating the program (possibly due to a crash) should leave the user with as little data loss as possible.
/// 
/// For example, there is no internal mechanism to exchange progress states between the GUI and subsequent processes. Rather
/// information such as "env models have been generated", "model checker run completed" etc. are derived from the file structure
/// in the respective directories.
/// 

enum class ButtonClass {
   RunButtons,
   DeleteButtons,
   All,
   None
};

class MCScene : public Failable {
public:
   MCScene(const InputParser& inputs);
   void copyWaitingForPreviewGIF();
   void refreshPreview();

   void activateMCButtons(const bool active, const ButtonClass which);
   bool isLTL(const std::string& config);
   int bmcDepth(const std::string& config);

   /// <summary>
   /// Get all formulas from the template json. 
   /// Works only for the template json since the actual instances don't have formulas in them anymore.
   /// </summary>
   /// <param name="json">THE json.</param>
   /// <returns>The result.</returns>
   std::vector<std::pair<std::string, std::string>> getAllFormulasFromJSON(const nlohmann::json json);

   /// <summary>
   /// Retrieves the SPEC from a given config. 
   /// </summary>
   /// <param name="config">The config, which can be JSON_TEMPLATE_DENOTER.</param>
   /// <param name="any_non_template">If config != JSON_TEMPLATE_DENOTER, retrieves just any SPEC, not necessarily the one from the specified config.</param>
   /// <returns></returns>
   std::pair<std::string, std::string> getSpec(const std::string& config, const bool any_non_template = false);
   void loadJsonText();
   void saveJsonText();
   void setTitle();
   void deleteMCOutputFromFolder(const std::string& path_generated, const bool actually_delete_gif); // ...or otherwise copy "waiting for" image.
   void evaluateFormulasInJSON(const nlohmann::json j_template);
   void preprocessAndRewriteJSONTemplate();
   nlohmann::json getJSON() const;

   void generatePreview(const std::string& path_generated, const int cex_num);
   
   std::map<std::string, std::pair<std::string, std::string>> loadNewBBsFromJson();
   std::pair<std::string, std::string> getBBNameAndFormulaByJsonName(const std::string& json_name);

   std::string getTemplateDir() const;
   std::string getCachedDir() const;
   std::string getBPIncludesFileDir() const;
   std::string getGeneratedDir() const;
   std::string getGeneratedParentDir() const;
   int getActualJSONWidth() const;

   void resetCachedVariables() const;

   int getFlRunInfo() const;
   SingleExpController getSECFromConfig(const std::string& name) const;
   std::string getOptionFromSECConfig(const std::string& config_name, const SecOptionLocalItemEnum option) const;
   void resetParserAndData();

   inline std::shared_ptr<FormulaParser> getParser() const { return parser_; }
   inline std::shared_ptr<DataPack> getData() const { return data_; }
   std::string getValueForJSONKeyAsString(const std::string& key_to_find, const std::string& config_name) const;
   void setValueForJSONKeyFromString(const std::string& key_to_find, const std::string& config_name, const bool from_template, const std::string& value_to_set) const;
   void setValueForJSONKeyFromBool(const std::string& key_to_find, const std::string& config_name, const bool from_template, const bool value_to_set) const;

   std::shared_ptr<OptionsGlobal> getRuntimeGlobalOptions() const;

   void putJSONIntoDataPack(const std::string& json_config = JSON_TEMPLATE_DENOTER);

   static void runMCJob(MCScene* mc_scene, const std::string& path_generated, const std::string config_name);
   static void runMCJobs(MCScene* mc_scene);
   static void createTestCase(const MCScene* mc_scene, const std::string& generated_parent_dir, const int cnt, const int max, const std::string& id, const int cex_num); // cex_num as given by slider position for resp. run.
   static void createTestCases(MCScene* mc_scene);

   // Callbacks:
   void showAllBBGroups(const bool show);
   void resetAllBBGroups();
   std::map<std::string, std::pair<std::string, DragGroup*>> getBBGroups() const;
   Fl_Double_Window* getWindow() const;

   static void bbBoxSliderCallback(Fl_Widget* widget, void* data);
   static void bbBoxCallback(Fl_Widget* widget, void* data);
   static void checkboxJSONVisibleCallback(Fl_Widget* widget, void* data);
   static void refreshFrequently(void* data);
   static void refreshRarely(void* data);
   static void refreshSometimes(void* data);
   static void jsonChangedCallback(Fl_Widget* widget, void* data);
   static void buttonSaveJSON(Fl_Widget* widget, void* data);
   static void buttonReloadJSON(Fl_Widget* widget, void* data);
   static void buttonCheckJSON(Fl_Widget* widget, void* data);
   static void buttonDeleteMCOutput(Fl_Widget* widget, void* data);
   static void buttonDeleteGenerated(Fl_Widget* widget, void* data);
   static void buttonDeleteTestCases(Fl_Widget* widget, void* data);
   static void buttonDeleteCached(Fl_Widget* widget, void* data);
   static void buttonRunMCAndPreview(Fl_Widget* widget, void* data);
   static void onGroupClickBM(Fl_Widget* widget, void* data);
   static void buttonReparse(Fl_Widget* widget, void* data);
   static void buttonCEX(Fl_Widget* widget, void* data);
   static void buttonRuntimeAnalysis(Fl_Widget* widget, void* data);

   std::set<SingleExpController> se_controllers_;

private:
   Fl_Double_Window* window_ = new Fl_Double_Window(0, 0, 1500, 900, GUI_NAME.c_str());
   Fl_Box* box_ = new Fl_Box(0, 0, window_->w(), window_->h());
   InterpreterTerminal* logging_output_and_interpreter_{ nullptr };
   //Fl_Text_Buffer* logging_buffer_ = new Fl_Text_Buffer();
   Fl_Check_Button* checkbox_json_visible_ = new Fl_Check_Button(100, 180, 100, 15, "Show JSON");

   Fl_Anim_GIF_Image* gif_ = new Fl_Anim_GIF_Image();

   Fl_Group* main_group_ = new Fl_Group(0, 95, window_->w(), window_->h());
   Fl_Scroll* sec_scroll_ = nullptr;
   Fl_Button* button_run_parser_ = new Fl_Button(0, 250, 200, 30, "Create EnvModels...");
   Fl_Button* button_run_mc_and_preview_ = new Fl_Button(0, 300, 200, 30, "Run Model Checker...");
   Fl_Button* button_run_cex_ = new Fl_Button(0, 350, 200, 30, "Generate test cases...");
   Fl_Button* button_runtime_analysis_ = new Fl_Button(0, 400, 200, 30, "Runtime analysis");
   Fl_Button* button_delete_mc_output_ = new Fl_Button(0, 490, 200, 30, "Delete selected MC output! <RM>");
   Fl_Button* button_delete_testcases_ = new Fl_Button(0, 520, 200, 30, "Delete selected test cases! <RM>");
   Fl_Button* button_delete_generated_ = new Fl_Button(0, 550, 200, 30, "Delete selected folders! <RM>");
   Fl_Button* button_delete_cached_ = new Fl_Button(0, 600, 200, 30, "Delete cache! <RM>");
   Fl_Text_Buffer* json_buffer_ = new Fl_Text_Buffer();;
   Fl_Text_Editor* json_input_ = new Fl_Text_Editor(230, 250, 410, 380);
   Fl_Button* button_reload_json_ = new Fl_Button(230, 630, 130, 30, "Reload JSON config");
   Fl_Button* button_save_json_ = new Fl_Button(370, 630, 130, 30, "Save JSON config");
   Fl_Button* button_check_json_ = new Fl_Button(510, 630, 130, 30, "Check JSON syntax");
   CustomMultiBrowser* variables_list_ = new CustomMultiBrowser(10, 10, 380, 280);
   Fl_Multiline_Input* scene_description_ = new Fl_Multiline_Input(1000, 250, 400, 300, "Scene description:");
   std::mutex parser_mutex_{};
   std::mutex refresh_mutex_{};
   std::mutex formula_evaluation_mutex_{};
   std::mutex preview_filesystem_mutex_{};

   //Fl_Input* spec_input = new Fl_Input(750, 610, 560, 30, "SPEC:");

   static void doAllEnvModelGenerations(MCScene* mc_scene);
   
   int fl_run_info_{};
   std::filesystem::file_time_type previous_write_time_{};

   std::shared_ptr<DataPack> data_{};
   std::shared_ptr<FormulaParser> parser_{};

   std::map<std::string, std::pair<std::string, DragGroup*>> bb_groups_{};

   int window_width_archived_{ 0 };
   
   std::set<std::string> envmodel_variables_{};

   mutable std::string generated_dir_{};
   mutable std::string cached_dir_{};
   mutable std::string includes_file_dir_{};
   std::shared_ptr<OptionsGlobal> runtime_global_options_{};

   nlohmann::json instanceFromTemplate(
      const nlohmann::json& j_template,
      const std::vector<std::pair<std::string, std::vector<float>>>& ranges,
      const std::vector<int>& counter_vec,
      const bool is_ltl);

   std::string last_json_string_{};
   std::map<std::string, std::pair<std::string, std::string>> last_bb_stuff_;

   bool mc_running_internal_{ false };

   ProgressDetector progress_detector_{};

   std::string json_tpl_filename_{};
   std::string path_to_template_dir_{};
   std::string path_to_external_folder_{};
};

} // vfm