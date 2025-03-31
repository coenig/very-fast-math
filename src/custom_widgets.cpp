//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file

#include "gui/custom_widgets.h"
#include "gui/gui.h"
#include "gui/process_helper.h"

#include <filesystem>

using namespace vfm;

std::set<std::string>* SyntaxHighlightMultilineInput::keywords_{};

vfm::SingleExpController::SingleExpController(
   const std::string& template_path,
   const std::string& generated_path,
   const std::string& my_id,
   const std::string& short_id,
   MCScene* mc_scene)
   : template_path_{ template_path },
   generated_path_{ generated_path },
   my_id_{ my_id },
   my_short_id_{ short_id },
   my_abbr_short_id_{ StaticHelper::shortenInTheMiddle(short_id, 15, 0.5, "...") },
   mc_scene_{ mc_scene },
   runtime_local_options_{ OptionsLocal{ std::filesystem::path(generated_path).parent_path().string() + "/" + my_id + "/runtime_options.morty"} }
{
   image_box_envmodel_->show();
   image_box_envmodel_->align(FL_ALIGN_RIGHT);
   labelBox_envmodel_->align(FL_ALIGN_RIGHT);

   image_box_mcrun_->show();
   image_box_mcrun_->align(FL_ALIGN_RIGHT);
   labelBox_mcrun_->align(FL_ALIGN_RIGHT);

   image_box_testcase_->show();
   image_box_testcase_->align(FL_ALIGN_RIGHT);
   labelBox_testcase_->align(FL_ALIGN_RIGHT);
   slider_->hide();
   slider_->value(std::stoi(runtime_local_options_.getOptionValue(SecOptionLocalItemEnum::selected_cex_num)));

   const int checkboxWidth = 50;
   const int checkboxHeight = 100;
   const int checkboxX = sec_group_->w() - checkboxWidth - 0;
   const int checkboxY = sec_group_->h() - checkboxHeight - 10;
   checkbox_selected_job_->resize(checkboxX, checkboxY, checkboxWidth, checkboxHeight);
   const int cancel_button_width = 30;
   const int cancel_button_height = 60;
   cancel_button_->size(cancel_button_width, cancel_button_height);
   cancel_button_->position(checkboxX, cancel_button_->y());
   cancel_button_->color(FL_BLACK);
   cancel_button_->tooltip("Cancel running MC process.");
   checkbox_selected_job_->tooltip("(De-)activate job; right click to (de-)activate all");

   sec_group_->box(FL_BORDER_BOX); // Set the box type to a border box

   progress_bar_sec_->position(0, 100);
   progress_bar_sec_->minimum(0);
   progress_bar_sec_->maximum(100);
   progress_bar_sec_->color(FL_WHITE);
   progress_bar_sec_->color2(FL_DARK_CYAN);
   progress_bar_sec_->value(30);
   progress_bar_sec_->show();

   progress_description_->label("");
   progress_description_->align(FL_ALIGN_LEFT | FL_ALIGN_INSIDE); // Align text to the left
   progress_description_->labelsize(10);

   removeProgressFile();

   sec_group_->end();
}

void vfm::SingleExpController::registerCallbacks() const
{
   checkbox_selected_job_->callback(checkboxJobSelectedCallback, const_cast<SingleExpController*>(this));
   cancel_button_->callback(cancelButtonCallback, const_cast<SingleExpController*>(this));
   slider_->callback(sliderCallback, const_cast<SingleExpController*>(this));
}

std::map<ProgressElement, std::string> vfm::SingleExpController::getProgress() const
{
   std::filesystem::path morty_progress_path{ generated_path_ };
   morty_progress_path = morty_progress_path.parent_path();
   morty_progress_path /= my_id_;
   morty_progress_path /= "progress.morty";

   if (StaticHelper::existsFileSafe(morty_progress_path)) {
      std::string progress_string{ StaticHelper::readFile(morty_progress_path.string()) };
      std::vector<std::string> triple{ StaticHelper::split(progress_string, "#") };

      if (triple.size() == 3 && StaticHelper::isParsableAsFloat(triple[0]) && StaticHelper::isParsableAsFloat(triple[1])) {
         return std::map<ProgressElement, std::string>{
            { ProgressElement::value, triple[0] },
            { ProgressElement::maximum, triple[1] },
            { ProgressElement::stage, triple[2] }
         };
      }
   }

   return {};
}

std::string vfm::SingleExpController::getProgressStage() const
{
   const std::map<ProgressElement, std::string> progress{ getProgress() };
   return progress.empty() ? "" : progress.at(ProgressElement::stage);
}

void vfm::SingleExpController::removeProgressFile() const
{
   std::filesystem::path morty_progress_path{ generated_path_ };
   morty_progress_path = morty_progress_path.parent_path();
   morty_progress_path /= my_id_;
   morty_progress_path /= "progress.morty";

   if (StaticHelper::existsFileSafe(morty_progress_path)) {
      StaticHelper::removeFileSafe(morty_progress_path);
   }
}

void vfm::SingleExpController::adjustWidgetsAppearances() const
{
   progress_bar_sec_->size(sec_group_->w(), sec_group_->h() / 6);
   progress_bar_sec_->position(sec_group_->x(), sec_group_->y() + sec_group_->h());
   progress_description_->position(sec_group_->x(), sec_group_->y() + sec_group_->h());

   auto progress_item{ getProgress() };

   if (!progress_item.empty()) {
      progress_bar_sec_->maximum(std::stof(progress_item.at(ProgressElement::maximum)));
      progress_bar_sec_->value(std::stof(progress_item.at(ProgressElement::value)));
      progress_description_->copy_label(StaticHelper::shortenInTheMiddle(progress_item.at(ProgressElement::stage), 20, 0.5, "...").c_str());
      progress_bar_sec_->show();
      progress_description_->show();
   }
   else {
      progress_bar_sec_->value(0);
      progress_bar_sec_->hide();
      progress_description_->hide();
   }

   runtime_local_options_.loadOptions();
   if (StaticHelper::isBooleanTrue(runtime_local_options_.getOptionValue(SecOptionLocalItemEnum::selected_job))) {
      checkbox_selected_job_->value(1);
   }
   else {
      checkbox_selected_job_->value(0);
   }
}

std::string vfm::SingleExpController::getMyId() const
{
   return my_id_;
}

std::string vfm::SingleExpController::getShortId() const
{
   return my_short_id_;
}

std::string vfm::SingleExpController::getAbbreviatedShortId() const
{
   return my_abbr_short_id_;
}

std::filesystem::path vfm::SingleExpController::getPathOnDisc() const
{
   return std::filesystem::path(generated_path_).parent_path() / my_id_;
}

void vfm::SingleExpController::adjustAbbreviatedShortID() const
{
   const_cast<SingleExpController*>(this)->my_abbr_short_id_ = StaticHelper::shortenInTheMiddle(my_short_id_, sec_group_->w() / 8, 1, "...");
   sec_group_->copy_label(getAbbreviatedShortId().c_str());
}

bool vfm::SingleExpController::hasEnvmodel() const
{
   return has_envmodel_;
}

bool vfm::SingleExpController::hasPreview() const
{
   return has_preview_;
}

bool vfm::SingleExpController::hasTestcase() const
{
   return has_testcase_;
}

bool vfm::SingleExpController::hasSlider() const
{
   return slider_->minimum() != slider_->maximum();
}

std::vector<std::string> vfm::SingleExpController::getVariableNames() const
{
   std::vector<std::string> vec{};
   std::vector<std::string> varvals{ StaticHelper::split(getShortId(), "_") };

   for (const auto& varval : varvals) {
      vec.push_back(StaticHelper::split(varval, "=").at(0)); // Just do it, no check.
   }

   return vec;
}

std::vector<float> vfm::SingleExpController::getVariableValues() const
{
   std::vector<float> vec{};
   std::vector<std::string> varvals{ StaticHelper::split(getShortId(), "_") };

   for (const auto& varval : varvals) {
      vec.push_back(std::stof(StaticHelper::split(varval, "=").at(1))); // Just do it, no check.
   }

   return vec;
}

void vfm::SingleExpController::setHasEnvmodel(const bool value)
{
   has_envmodel_ = value;
}

void vfm::SingleExpController::setHasTestcase(const bool value)
{
   has_testcase_ = value;
}

void vfm::SingleExpController::setHasPreview(const bool value)
{
   has_preview_ = value;
}

void DragGroup::initializeEditor(const std::string& name, const std::string& formula)
{
   Failable::getSingleton()->addNote("Initialized building block '" + name + "' with value '" + formula + "' (json name: '" + bb_json_name + "').");

   if (editor_.formula_name_editor_) delete editor_.formula_name_editor_;
   if (editor_.formula_editor_) delete editor_.formula_editor_;
   if (editor_.formula_apply_formula_button_) delete editor_.formula_apply_formula_button_;
   if (editor_.formula_simplify_button_) delete editor_.formula_simplify_button_;
   if (editor_.formula_flatten_button_) delete editor_.formula_flatten_button_;
   if (editor_.preview_scroll_) delete editor_.preview_scroll_;
   if (editor_.preview_description_) delete editor_.preview_description_;
   if (editor_.slider_) delete editor_.slider_;

   editor_.formula_name_editor_ = new Fl_Input(500, 500, 100, 30);
   editor_.formula_editor_ = new SyntaxHighlightMultilineInput(500, 520, 400, 300);
   editor_.formula_apply_formula_button_ = new Fl_Button(500, 520, 400, 300, "Apply");
   editor_.formula_simplify_button_ = new Fl_Button(500, 520, 400, 300, "Simplify");
   editor_.formula_flatten_button_ = new Fl_Button(500, 520, 400, 300, "Flatten");
   editor_.preview_scroll_ = new Fl_Scroll(0, 0, 0, 0);
   editor_.preview_description_ = new Fl_Input(500, 500, 100, 30);
   editor_.slider_ = new Fl_Value_Slider(100, 100, 100, 30);
   editor_.showOnScreen(false);

   editor_.formula_name_editor_->value(name.c_str());
   editor_.formula_editor_->value(formula.c_str());

   editor_.formula_apply_formula_button_->callback(buttonApplyCallback, this);
   editor_.formula_simplify_button_->callback(buttonSimplifyCallback, this);
   editor_.formula_flatten_button_->callback(buttonFlattenCallback, this);
   editor_.formula_name_editor_->callback(FormulaEditor::callbackNameChanged, this);
   editor_.formula_name_editor_->when(FL_WHEN_CHANGED);

   editor_.formula_apply_formula_button_->tooltip(APPLY_BUTTON_TOOLTIP.c_str());
}

void refreshApplyButton(const DragGroup* group, const bool apply_error = false)
{
   auto mc_scene{ group->getMCScene() };

   std::string group_name{ group->getName() };
   std::string group_json_name{ group->getJsonName() };
   std::string original_name{ mc_scene->getBBNameAndFormulaByJsonName(group_json_name).first };

   if (group_name == original_name) {
      group->getEditor().formula_apply_formula_button_->label("Apply");
      group->getEditor().formula_apply_formula_button_->color(apply_error ? FL_RED : FL_GRAY);
      group->getEditor().formula_apply_formula_button_->tooltip(APPLY_BUTTON_TOOLTIP.c_str());
      group->getEditor().formula_apply_formula_button_->activate();
   }
   else if (original_name == "LTLSPEC" || original_name == "INVARSPEC") {
      group->getEditor().formula_apply_formula_button_->label("CHANGE!");
      group->getEditor().formula_apply_formula_button_->color(fl_rgb_color(255, 145, 107));
      group->getEditor().formula_apply_formula_button_->tooltip("Change the type of the spec between 'LTLSPEC' and 'INVARSPEC'. The MC mode will be set accordingly.");

      if (group_name == "LTLSPEC" || group_name == "INVARSPEC") {
         group->getEditor().formula_apply_formula_button_->activate();
      }
      else {
         group->getEditor().formula_apply_formula_button_->deactivate();
      }
   }
   else {
      group->getEditor().formula_apply_formula_button_->label("RE-DECLARE!");
      group->getEditor().formula_apply_formula_button_->color(fl_rgb_color(255, 145, 107));
      group->getEditor().formula_apply_formula_button_->tooltip("Re-declaring this building block will go through all formulas and previews and rename all usages of this block accordingly; if the parameters change, this can lead to syntactical errors in other BBs which will be shown in RED");
      group->getEditor().formula_apply_formula_button_->activate();
   }
}

std::set<std::string> matchingPrefix(const std::set<std::string>& strs, const std::string& prefix)
{
   std::set<std::string> res{};

   for (const auto& str : strs) {
      if (StaticHelper::stringStartsWith(str, prefix)) {
         res.insert(str);
      }
   }

   return res;
}

std::string findCommonPrefix(const std::set<std::string>& strs)
{
   if (strs.empty()) return {};

   int min_size{ (std::numeric_limits<int>::max)() };
   for (const auto& str : strs) min_size = (std::min)(min_size, (int)str.size());
   std::string res{};
   std::string first{}; // A substring of the first string in strs which all others are compared to.

   for (int i = 0; i < min_size; i++) {
      first += StaticHelper::makeString((*strs.begin())[i]);

      for (const auto& str : strs) {
         if (str.substr(0, i + 1) != first) {
            return res;
         }
      }

      res = first;
   }

   return res;
}

// Callbacks:
void SyntaxHighlightMultilineInput::replaceWord(Fl_Widget* widget, void* data)
{
   SyntaxHighlightMultilineInput* input = static_cast<SyntaxHighlightMultilineInput*>(widget);
   if (input->being_replaced_) return;

   input->being_replaced_ = true;
   input->suggestions_box_->hide();

   if (Fl::event_state(FL_CTRL) && std::string(Fl::event_text()) == " ") {
      const char* text = input->value();
      const std::string text_str(text);
      int cursorPos = input->position();

      // Find the start and end positions of the word at the cursor position
      int wordStart = cursorPos - 1;
      while (wordStart > 0 && StaticHelper::isAlphaNumericOrOneOfThese(
         StaticHelper::makeString(text[wordStart - 1]), "_.:\"")) {
         wordStart--;
      }
      int wordEnd = cursorPos;
      while (StaticHelper::isAlphaNumericOrOneOfThese(
         StaticHelper::makeString(text[wordEnd]), "_.:\"")) {
         wordEnd++;
      }

      std::string prefix{ text_str.substr(wordStart, cursorPos - wordStart - 1) }; // Don't use wordEnd here since we only look at the prefix.
      std::set<std::string> matching{ matchingPrefix(*keywords_, prefix) };
      std::string match{ findCommonPrefix(matching) }; // Singular result is special case of common prefix with full length.
      std::string newText = text;

      if (matching.empty() || prefix.empty()) {
         input->being_replaced_ = false;
         return;
      }

      newText.replace(wordStart, wordEnd - wordStart, match);
      input->replace(0, input->size(), newText.c_str());
      int newCursorPos = wordStart + match.size();
      input->position(newCursorPos);

      if (matching.size() > 1) {
         input->suggestions_box_->clear();
         for (const auto& suggestion : matching) {
            input->suggestions_box_->add(suggestion.c_str());
         }
         input->suggestions_box_->resize(input->x() + input->w(), input->y(), 350, 300);
         Fl_Window* parent{ input->suggestions_box_->window() };
         parent->remove(input->suggestions_box_);
         parent->add(input->suggestions_box_);
         input->suggestions_box_->show();
      }
   }

   input->being_replaced_ = false;
}

void vfm::FormulaEditor::callbackNameChanged(Fl_Widget* widget, void* data)
{
   refreshApplyButton(static_cast<DragGroup*>(data));
}

std::string createJsonEntryForBBSafe(const DragGroup* group, const MCScene* mc_scene, const std::string& fmla_str)
{
   const std::regex pattern_name(REGEX_PATTERN_BB_NAMES);
   const std::regex pattern_pars(REGEX_PATTERN_BB_PARAMETERS);
   const std::regex pattern_def_with_pars(REGEX_PATTERN_BB_DECLARATION_WITH_PARAMETERS);

   std::string before{}, inner{ group->getName() }, after{};
   std::string full_func_def{ inner };

   if (!std::regex_match(inner, pattern_def_with_pars)) {
      mc_scene->addError("Error in function declaration '" + full_func_def + "'. It does not comply with regex " + REGEX_PATTERN_BB_DECLARATION_WITH_PARAMETERS + ". Aborting.");
      return "#ERROR";
   }

   int begin{ StaticHelper::indexOfFirstInnermostBeginBracket(inner, "(", ")") };
   int end{ StaticHelper::findMatchingEndTagLevelwise(inner, begin, "(", ")") };
   begin++;
   StaticHelper::distributeIntoBeforeInnerAfter(before, inner, after, begin, end);

   before.pop_back();

   StaticHelper::trim(before); // Contains function name: LaneAvailableLeft
   StaticHelper::trim(inner);  // Contains parameters: distance_front
   StaticHelper::trim(after);  // Not of interest here, but should be empty.

   if (!std::regex_match(before, pattern_name)) {
      mc_scene->addError("Error in function name '" + before + "' of declaration '" + full_func_def + "'. It does not comply with regex " + REGEX_PATTERN_BB_NAMES + ". Aborting.");
      return "#ERROR";
   }

   if (!std::regex_match(inner, pattern_pars)) {
      mc_scene->addError("Error in function parameters part '" + inner + "' of declaration '" + full_func_def + "'. It does not comply with regex " + REGEX_PATTERN_BB_PARAMETERS + ". Aborting.");
      return "#ERROR";
   }

   if (after != ")") {
      mc_scene->addError("Error in function declaration '" + full_func_def + "'. There should be nothing behind the last (and only) closing bracket, but I found '" + after + "'. Aborting.");
      return "#ERROR";
   }

   return std::string("@f(") + before + ") { " + inner + " } { " + fmla_str + " }";
}

std::string DragGroup::removeFunctionStuffFromString(const std::string& str)
{
   size_t pos = str.find('('); // Strip off any "function" syntax.
   return pos == std::string::npos ? str : str.substr(0, pos);
}

void SingleExpController::sliderCallback(Fl_Widget* widget, void* data) {
   Fl_Value_Slider* slider = (Fl_Value_Slider*)widget;
   SingleExpController* sec = static_cast<SingleExpController*>(data);
   sec->runtime_local_options_.setOptionValue(SecOptionLocalItemEnum::selected_cex_num, std::to_string((int)slider->value()));
   sec->runtime_local_options_.saveOptions();

   const std::filesystem::path path_generated_base{ sec->generated_path_ }; // TODO: Remove this sort of double code by creating a 'getMyPath()' function.
   const std::filesystem::path path_generated_base_parent{ path_generated_base.parent_path() };
   const std::filesystem::path sec_path{ path_generated_base_parent / sec->getMyId() };
   const std::filesystem::path source_path(sec_path / std::to_string((int)sec->slider_->value()));

   if (!StaticHelper::existsFileSafe(source_path)) {
      std::thread t{ [sec, sec_path]() { // Don't use references since they go out of scope.
         sec->mc_scene_->deletePreview(sec->generated_path_, false);
         sec->mc_scene_->generatePreview(sec_path.string(), sec->slider_->value());
         sec->tryToSelectController();
      } };

      t.detach();
   }

   sec->tryToSelectController();
}

void DragGroup::buttonApplyCallback(Fl_Widget* widget, void* data)
{
   auto group{ static_cast<DragGroup*>(data) };
   auto mc_scene{ group->getMCScene() };

   std::string group_name{ group->getName() };
   std::string group_json_name{ group->getJsonName() };
   std::string original_name{ mc_scene->getBBNameAndFormulaByJsonName(group_json_name).first };
   const std::string fmla_str{ group->getFormula() };
   std::string json_entry{};

   if (group_name == original_name) { // Apply
      refreshApplyButton(group, false);


      if (group_json_name == "SPEC") {
         // NO test for syntactical correctness. 
         // ==> We want to be able to write: "@{this is [i]}@.for[[i], 0, 4]"

         // Example: "SPEC": "#{ !(LCTowardsEgoAhead()) }#",
         json_entry = "#{ " + fmla_str + " }#";
      }
      else {
         // Test for syntactical correctness. 
         // ==> We want to be able to write: "@{this is [i]}@.for[[i], 0, 4]"
         mc_scene->getParser()->resetErrors(ErrorLevelEnum::error);
         TermPtr fmla{ MathStruct::parseMathStruct(fmla_str, mc_scene->getParser(), mc_scene->getData(), DotTreatment::as_operator)->toTermIfApplicable() };

         if (mc_scene->getParser()->hasErrorOccurred() || StaticHelper::stringContains(fmla->serializePlainOldVFMStyle(MathStruct::SerializationSpecial::enforce_square_array_brackets), PARSING_ERROR_STRING)) {
            mc_scene->addError("Error while parsing '" + fmla_str + "'. Aborting 'Apply'.");
            refreshApplyButton(group, true);
            return;
         }

         // Example: LaneAvailableLeft(distance_front)
         //    "#21" : "@f(LaneAvailableLeft) { distance_front } { env.ego.gaps___609___.lane_availability > distance_front }",
         json_entry = createJsonEntryForBBSafe(group, mc_scene, fmla_str);

         if (json_entry == "#ERROR") return;
      }

      group->father_mc_scene_->setValueForJSONKeyFromString(
         group->bb_json_name,
         JSON_TEMPLATE_DENOTER,
         true,
         json_entry);
   }
   else if (group_name == "LTLSPEC" || group_name == "INVARSPEC") { // CHANGE!
      mc_scene->showAllBBGroups(false);
      mc_scene->addNote("Changing from '" + original_name + "' to '" + group_name + "'.");
      mc_scene->setValueForJSONKeyFromString(
         "SPEC",
         JSON_TEMPLATE_DENOTER,
         true,
         "#{ " + group->getFormula() + " }#"); // No explicit "[LTL,INVAR]SPEC" denoter in json.
      mc_scene->setValueForJSONKeyFromBool("LTL_MODE", JSON_TEMPLATE_DENOTER, true, group_name == "LTLSPEC");
   }
   else { // RE-DECLARE!
      mc_scene->showAllBBGroups(false);
      //mc_scene->addNote("Firstly, renaming all tokens which are part of a BB in json.");

      json_entry = createJsonEntryForBBSafe(group, mc_scene, fmla_str);

      if (json_entry == "#ERROR") return;

      group->father_mc_scene_->setValueForJSONKeyFromString(
         group->bb_json_name,
         JSON_TEMPLATE_DENOTER,
         true,
         json_entry);

      //nlohmann::json j = mc_scene->getJSON();

      //for (const auto& [key_config, value_config] : j.items()) {
      //   if (key_config == JSON_TEMPLATE_DENOTER) {
      //      for (auto& [key, value] : value_config.items()) {
      //         std::string fmla_str{};

      //         if (value.type() == nlohmann::detail::value_t::string) {
      //            fmla_str = nlohmann::to_string(value);
      //         }

      //         if (StaticHelper::stringContains(fmla_str, JSON_NUXMV_FORMULA_BEGIN)) {
      //            std::string before{};
      //            std::string after{};
      //            int beg{ StaticHelper::indexOfFirstInnermostBeginBracket(fmla_str, JSON_NUXMV_FORMULA_BEGIN, JSON_NUXMV_FORMULA_END) };
      //            int end{ StaticHelper::findMatchingEndTagLevelwise(fmla_str, beg, JSON_NUXMV_FORMULA_BEGIN, JSON_NUXMV_FORMULA_END) };

      //            StaticHelper::distributeIntoBeforeInnerAfter(before, fmla_str, after, beg + JSON_NUXMV_FORMULA_BEGIN.size(), end);

      //            auto fmla = _id(MathStruct::parseMathStruct(fmla_str, mc_scene->getParser(), mc_scene->getData())->toTermIfApplicable());
      //            bool changed{ false };

      //            fmla->applyToMeAndMyChildren([&changed, &original_name, &group_name](const MathStructPtr m) {
      //               auto m_var = m->toVariableIfApplicable();

      //               if (m_var && m_var->getVariableName() == original_name) {
      //                  m->replaceJumpOverCompounds(_var(group_name));
      //                  changed = true;
      //               }
      //            });

      //            if (changed) {
      //               mc_scene->setValueForJSONKeyFromString(
      //                  key,
      //                  key_config,
      //                  true,
      //                  StaticHelper::replaceAll(before, "\"", "") + " " + fmla->child0()->serialize() + " " + StaticHelper::replaceAll(after, "\"", ""));
      //            }
      //         }
      //      }
      //   }
      //}

      const std::filesystem::path previews_path{ mc_scene->getGeneratedParentDir() + "/previews" };
      mc_scene->addNote("Renaming the tokens which are part of SPECs in the previews.");

      //try {
      for (const auto& entry : std::filesystem::directory_iterator(previews_path)) { // Find all previews
         const std::string spec_path{ entry.path().string() + "/spec.txt" };
         const std::string spec_raw{ StaticHelper::readFile(spec_path) };
         const std::string spec_str{ StaticHelper::replaceAll(
            StaticHelper::replaceAll(spec_raw, "LTLSPEC", ""), "INVARSPEC", "") };
         bool is_ltl{ StaticHelper::stringContains(spec_raw, "LTLSPEC") };
         TermPtr fmla{ _id(MathStruct::parseMathStruct(spec_str, nullptr, mc_scene->getData(), DotTreatment::as_operator)->toTermIfApplicable()) };
         std::string group_name_bb_plain{ removeFunctionStuffFromString(group_name) };
         std::string original_name_bb_plain{ removeFunctionStuffFromString(original_name) };
         bool changed{ false };

         fmla->applyToMeAndMyChildren([&changed, &original_name_bb_plain, &group_name_bb_plain](const MathStructPtr m) {
            auto m_var = m->toVariableIfApplicable();

            if (m_var && m_var->getVariableName() == original_name_bb_plain) {
               m->replaceJumpOverCompounds(_var(group_name_bb_plain));
               changed = true;
            }
            });

         if (changed) {
            const std::string spec_new{ std::string(is_ltl ? "LTLSPEC" : "INVARSPEC") + " " + fmla->child0()->serializePlainOldVFMStyle(MathStruct::SerializationSpecial::enforce_square_array_brackets) };
            StaticHelper::writeTextToFile(spec_new, spec_path);
            mc_scene->addDebug("Changed '" + spec_raw + "' to '" + spec_new + "' for '" + spec_path + "'.");
         }
      }
      //}
      //catch (const std::exception& e) {
      //   mc_scene->addError("Could not finish renaming building blocks in preview specs. Reason: " + std::string(e.what()));
      //}
   }

   group->father_mc_scene_->saveJsonText();
   refreshApplyButton(group);
}

void DragGroup::buttonSimplifyCallback(Fl_Widget* widget, void* data)
{
   auto group{ static_cast<DragGroup*>(data) };
   auto mc_scene{ group->getMCScene() };

   std::string fmla_str{ group->getFormula() };
   auto fmla{ MathStruct::parseMathStruct(fmla_str, mc_scene->getParser(), mc_scene->getData(), DotTreatment::as_operator)->toTermIfApplicable() };
   std::string fmla_simpl_str{ fmla->serializePlainOldVFMStyle(MathStruct::SerializationSpecial::enforce_square_array_brackets) };

   fmla = mc::simplification::simplifyFast(fmla, mc_scene->getParser());
   fmla_simpl_str = fmla->serializePlainOldVFMStyle(MathStruct::SerializationSpecial::enforce_square_array_brackets);

   group->getEditor().formula_editor_->replace(0, group->getEditor().formula_editor_->size(), fmla_simpl_str.c_str());
}

void vfm::DragGroup::buttonFlattenCallback(Fl_Widget* widget, void* data)
{
   auto group{ static_cast<DragGroup*>(data) };
   auto mc_scene{ group->getMCScene() };

   std::string fmla_str{ group->getFormula() };
   auto fmla{ MathStruct::parseMathStruct(fmla_str, mc_scene->getParser(), mc_scene->getData(), DotTreatment::as_operator)->toTermIfApplicable() };

   fmla = MathStruct::flattenFormula(fmla);

   std::string fmla_flat_str{ fmla->serializePlainOldVFMStyle(MathStruct::SerializationSpecial::enforce_square_array_brackets) };
   group->getEditor().formula_editor_->replace(0, group->getEditor().formula_editor_->size(), fmla_flat_str.c_str());
}

// Callbacks:

void SingleExpController::checkboxJobSelectedCallback(Fl_Widget* widget, void* data)
{
   auto sec{ static_cast<SingleExpController*>(data) };
   Fl_Check_Button* checkbox = (Fl_Check_Button*)widget;

   if (Fl::event_button() == FL_RIGHT_MOUSE) {
      for (const auto& sec2 : sec->mc_scene_->se_controllers_) {
         sec2.runtime_local_options_.setOptionValue(SecOptionLocalItemEnum::selected_job, std::to_string(checkbox->value()));
         sec2.runtime_local_options_.saveOptions();
      }
   }
   else {
      sec->runtime_local_options_.setOptionValue(SecOptionLocalItemEnum::selected_job, std::to_string(checkbox->value()));
      sec->runtime_local_options_.saveOptions();
   }
}

void vfm::SingleExpController::cancelButtonCallback(Fl_Widget* widget, void* data)
{
   auto sec{ static_cast<SingleExpController*>(data) };

   if (Fl::event_button() == FL_RIGHT_MOUSE) {
      std::string job_id{ sec->my_id_ };
      sec->mc_scene_->addNote("Sending kill signal to all jobs matching '" + job_id + "'.");
      auto pids = Process().getPIDs(job_id);

      for (const auto& pid : pids) {
         sec->mc_scene_->addNote("Sending kill signal for PID '" + std::to_string(pid) + "'.");
         Process().killByPID(pid);
      }
   }
   else {
      sec->mc_scene_->addNote("Use righ-click to cancel MC job.");
   }
}

void SingleExpController::tryToSelectController() const
{
   if (hasPreview() && hasEnvmodel()) {
      const std::filesystem::path path_generated_base{ generated_path_ };
      const std::filesystem::path path_generated_base_parent{ path_generated_base.parent_path() };
      const std::filesystem::path source_path(path_generated_base_parent / getMyId() / std::to_string((int)slider_->value()));

      std::filesystem::path source_path_preview{ source_path / "preview" };
      std::filesystem::path source_path_prose{ source_path / PROSE_DESC_NAME };
      std::filesystem::path target_path_prose{ path_generated_base / PROSE_DESC_NAME };

      if (StaticHelper::existsFileSafe(source_path_preview)) {
         try {
            std::filesystem::copy(source_path_preview, path_generated_base / "preview", std::filesystem::copy_options::recursive | std::filesystem::copy_options::overwrite_existing);
         }
         catch (std::exception& e) {
            mc_scene_->addWarning("Copying folder '" + source_path_preview.string() + "' to '" + (path_generated_base / "preview").string() + "' not possible due to error: " + std::string(e.what()));
         }
      }

      if (StaticHelper::existsFileSafe(source_path_prose)) {
         try {
            std::filesystem::copy(source_path_prose, target_path_prose, std::filesystem::copy_options::recursive | std::filesystem::copy_options::overwrite_existing);
         }
         catch (std::exception& e) {
            mc_scene_->addWarning("Copying '" + source_path_prose.string() + "' to '" + target_path_prose.string() + "' not possible due to error: " + std::string(e.what()));
         }
      }
      else if (StaticHelper::existsFileSafe(path_generated_base / PROSE_DESC_NAME)) {
         try {
            StaticHelper::removeFileSafe(path_generated_base / PROSE_DESC_NAME);
         }
         catch (std::exception& e) {
            mc_scene_->addWarning("Removing folder '" + (path_generated_base / PROSE_DESC_NAME).string() + " not possible due to error : " + std::string(e.what()));
         }
      }

      for (const auto& other : mc_scene_->se_controllers_) {
         other.selected_ = false;
      }

      selected_ = true;
      sec_group_->color(fl_rgb_color(255, 255, 102)); // To make color change instantly.
   }
   else {
      selected_ = false;
      sec_group_->color(FL_WHITE); // Short flash.
   }
}

void ProgressDetector::loadProgressImagesFromDisc(const std::string& path_template, const int w, const int h)
{
   if (!check_image_) {
      check_image_ = new Fl_PNG_Image((path_template + "/check.png").c_str());
      progress_image_ = new Fl_PNG_Image((path_template + "/progress.png").c_str());
      cross_image_ = new Fl_PNG_Image((path_template + "/cross.png").c_str());
      empty_image_ = new Fl_PNG_Image((path_template + "/empty.png").c_str());

      check_image_->scale(w, h);
      progress_image_->scale(w, h);
      cross_image_->scale(w, h);
      empty_image_->scale(w, h);
   }
}

void vfm::ProgressDetector::placeProgressImage(
   const std::string& path_template, 
   const std::set<std::string>& packages, 
   const std::set<SingleExpController>& se_controllers,
   const std::filesystem::path& path_generated_base_parent)
{
   loadProgressImagesFromDisc(path_template, 12, 12);

   for (const auto& package : packages) { // Show progress images.
      for (auto& sec : se_controllers) {
         if (sec.getMyId() == package) {
            int preview_num{ 0 };

            const double slider_min{ (double) 0 };
            const double slider_max{ (std::max)(0.0, (double) StaticHelper::extractMCTracesFromNusmvFile(
               (path_generated_base_parent / package / "debug_trace_array.txt").string(), StaticHelper::TraceExtractionMode::quick_only_detect_if_empty).size() - 1) };

            sec.slider_->range(slider_min, slider_max);
            sec.slider_->value((std::min)(slider_max, (std::max)(slider_min, sec.slider_->value())));

            if (sec.hasSlider()) {
               sec.slider_->show();
               preview_num = (int)sec.slider_->value();
               sec.slider_->type(FL_HORIZONTAL);
               sec.slider_->color(fl_rgb_color(254, 255, 224));
               sec.slider_->precision(0);
            }
            else {
               sec.slider_->hide();
            }

            if (StaticHelper::existsFileSafe(path_generated_base_parent / package / "EnvModel.smv")) {
               sec.image_box_envmodel_->image(check_image_);
               sec.has_envmodel_ = true;
            }
            else {
               int file_count{ 0 };

               try {
                  for (const auto& entry : std::filesystem::directory_iterator(path_generated_base_parent / package)) {
                     file_count++;
                  }
               }
               catch (const std::exception& e) {}

               if (file_count <= 1) { // Only runtime.morty in folder.
                  sec.image_box_envmodel_->image(cross_image_);
               }
               else {
                  sec.image_box_envmodel_->image(progress_image_);
               }

               sec.has_envmodel_ = false;
            }

            sec.image_box_envmodel_->size(12, 12);
            sec.cancel_button_->hide();

            if (StaticHelper::existsFileSafe(path_generated_base_parent / package / std::to_string(preview_num) / "preview" / "preview.gif")
               && sec.getProgressStage() != "preview") {
               sec.image_box_mcrun_->image(check_image_);
               sec.has_preview_ = true;
            }
            else {
               if (StaticHelper::existsFileSafe(path_generated_base_parent / package / "planner.cpp_combined.k2.smv")) {
                  const std::filesystem::path trace_path{ path_generated_base_parent / package / "debug_trace_array.txt" };

                  if (StaticHelper::existsFileSafe(trace_path)) {
                     std::vector<MCTrace> traces{ StaticHelper::extractMCTracesFromNusmvFile(trace_path.string(), StaticHelper::TraceExtractionMode::quick_only_detect_if_empty) };

                     if (traces.empty()) {
                        sec.image_box_mcrun_->image(empty_image_);
                     }
                  }
                  else {
                     sec.image_box_mcrun_->image(progress_image_);
                     sec.cancel_button_->show();
                  }
               }
               else {
                  sec.image_box_mcrun_->image(cross_image_);
               }

               sec.has_preview_ = false;
            }

            static const std::string FIRST_STAGE_NAME{ "full" };
            static const std::string LAST_STAGE_NAME{ "smooth-with-arrows-birdseye" };
            static const std::string LAST_STAGE_NUM{ "7" };

            sec.image_box_mcrun_->size(12, 12);
            auto path_temp_first = path_generated_base_parent / package / ("cex-" + FIRST_STAGE_NAME);
            auto path_temp_last = path_generated_base_parent / package / ("cex-" + LAST_STAGE_NAME) / ("cex-" + LAST_STAGE_NAME + ".gif");

            if (StaticHelper::existsFileSafe(path_temp_last)
               && sec.getProgressStage() != LAST_STAGE_NAME + " (" + LAST_STAGE_NUM + "/" + LAST_STAGE_NUM + ")") {
               sec.image_box_testcase_->image(check_image_);
               sec.has_testcase_ = true;
            }
            else {
               if (StaticHelper::existsFileSafe(path_temp_first)) {
                  sec.image_box_testcase_->image(progress_image_);
               }
               else {
                  sec.image_box_testcase_->image(cross_image_);
               }

               sec.has_testcase_ = false;
            }

            sec.image_box_testcase_->size(12, 12);

            break;
         }
      }
   }
}
