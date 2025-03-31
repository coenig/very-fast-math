//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2024 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file
#pragma once

#include "static_helper.h"
#include "gui/gui_options.h"

#include <FL/Fl_Multiline_Input.H>
#include <FL/Fl_Hold_Browser.H>
#include <FL/Fl_Input.H>
#include <FL/Fl_Button.H>
#include <FL/Fl_Scroll.H>
#include <FL/Fl_Value_Slider.H>
#include <FL/Fl_Box.H>
#include <FL/Fl_Check_Button.H>
#include <FL/Fl_Progress.H>
#include <FL/fl_draw.H>
#include <FL/Fl_PNG_Image.H>

namespace vfm {

class MCScene;

static const std::string APPLY_BUTTON_TOOLTIP{ "Apply the current formula to this building block" };

class SyntaxHighlightMultilineInput : public Fl_Multiline_Input {
public:
   SyntaxHighlightMultilineInput(int x, int y, int w, int h, const char* label = nullptr)
      : Fl_Multiline_Input(x, y, w, h, label) {
      callback(replaceWord);
      when(FL_WHEN_CHANGED);

      suggestions_box_ = new Fl_Hold_Browser(0, 0, 200, 300);
      hide();
   }

   void hide() override
   {
      suggestions_box_->hide();
      Fl_Multiline_Input::hide();
   }

   void draw() override {
      if (!visible()) return;
      if (!keywords_ || keywords_->empty()) return;

      fl_color(FL_BLUE);

      std::string val = StaticHelper::replaceAll(std::string(value()), "\n", "");
      std::string wrapped_val = getWrappedText(val, w());

      const char* text = wrapped_val.c_str();

      //size(w(), 20 * (position_to_line(wrapped_val.size(), wrapped_val.c_str()) + 1));
      Fl_Multiline_Input::draw(); // Call the base class draw() method first

      // Loop through the keywords and highlight them
      for (std::string keyword_str : *keywords_) {
         if (!keyword_str.empty()) {
            const char* keyword = keyword_str.c_str();
            const char* pos = text;
            while ((pos = strstr(pos, keyword)) != nullptr) {
               int start = pos - text;
               int end = start + strlen(keyword);

               if ((start == 0 || isspace(text[start - 1]) || isSpecialChar(text[start - 1]))
                  && (end == strlen(text) || isspace(text[end]) || isSpecialChar(text[end]))) {
                  //draw_highlight(start, end, text);
               }

               pos += strlen(keyword);
            }
         }
      }
   }

   static inline void setKeywords(std::set<std::string>* keywords)
   {
      keywords_ = keywords;
   }

   static void replaceWord(Fl_Widget* widget, void* data);

private:
   bool being_replaced_{ false };

   Fl_Hold_Browser* suggestions_box_{};

   inline bool isSpecialChar(const char c)
   {
      return c == '(' || c == ')' || c == '&' || c == '|' || c == '+' || c == '-' || c == '*' || c == '/'
         || c == '%' || c == '>' || c == '<' || c == '=' || c == '!';
   }

   // Note that there is a similar function in StaticHelper. The below one
   // is specific for the fltk use case. 
   // 
   // TODO: In future, they might
   // be merged (meaning that the below one could go as a general solution
   // into StaticHelper. I am not sure if there might be implications on that,
   // therefore, I will not do that for now.)
   inline std::string getWrappedText(const std::string& str, const int widgetWidth)
   {
      const char* text = str.c_str();
      int textLength = strlen(text);

      std::string wrappedText;
      int currentWidth = 0;
      int lastWhitespacePos = -1;

      for (int i = 0; i < textLength; i++) {
         int charWidth = fl_width(text[i]);
         currentWidth += charWidth;

         if (isspace(text[i])) {
            lastWhitespacePos = i;
         }

         if (currentWidth > widgetWidth) {
            if (lastWhitespacePos != -1) {
               wrappedText += std::string(text, lastWhitespacePos + 1) + "\n";
               text += lastWhitespacePos + 1;
               textLength -= lastWhitespacePos + 1;
               currentWidth = 0;
               lastWhitespacePos = -1;
               i = -1; // Reset the loop counter
            }
            else {
               wrappedText += std::string(text, i) + "\n";
               text += i;
               textLength -= i;
               currentWidth = 0;
               i = -1; // Reset the loop counter
            }
         }
      }

      wrappedText += text;

      return wrappedText;
   }

   void draw_highlight(int start, int end, const char* text) {
      int x = this->x() + Fl::box_dx(box());
      int y = this->y() + Fl::box_dy(box());
      int w = Fl::box_dw(box());
      int h = Fl::box_dh(box());

      int line_height = fl_height();
      int line_start = position_to_line(start, text);
      int line_end = position_to_line(end, text);

      for (int line = line_start; line <= line_end; line++) {
         int line_y = y + line * line_height;
         int line_start_pos = line_to_position(line, text);
         int line_end_pos = line_to_position(line + 1, text);
         int highlight_start = std::max(start, line_start_pos);
         int highlight_end = std::min(end, line_end_pos);
         int highlight_width = position_to_x(highlight_end, line, text) - position_to_x(highlight_start, line, text);

         fl_rounded_rect(x + position_to_x(highlight_start, line, text), line_y, highlight_width + 2, line_height, 5);
      }
   }

   int position_to_line(int pos, const char* text) {
      int line = 0;
      int current_pos = 0;

      while (current_pos < pos && text[current_pos] != '\0') {
         if (text[current_pos] == '\n') {
            line++;
         }
         current_pos++;
      }

      return line;
   }

   int line_to_position(int line, const char* text) {
      int current_line = 0;
      int current_pos = 0;

      while (current_line < line && text[current_pos] != '\0') {
         if (text[current_pos] == '\n') {
            current_line++;
         }
         current_pos++;
      }

      return current_pos;
   }

   int position_to_x(int pos, int line_no, const char* text) {
      const auto lines = StaticHelper::split(std::string(text), "\n");

      int chars{ line_no };

      for (int i = 0; i < line_no; i++) chars += lines[i].size();

      const char* text2 = lines[line_no].c_str();

      int width_full = 0;
      int width_end = 0;
      int height = 0;
      int descent = 0;
      fl_font(textfont(), textsize());
      fl_measure(text2, width_full, height, descent);
      fl_measure(text2 + pos - chars, width_end, height, descent);

      return width_full - width_end;
   }

   static std::set<std::string>* keywords_;
};

struct FormulaEditor
{
   inline void showOnScreen(const bool show)
   {
      if (show) {
         formula_name_editor_->show();
         formula_editor_->show();
         formula_apply_formula_button_->show();
         formula_simplify_button_->show();
         formula_flatten_button_->show();
         preview_scroll_->show();
         preview_description_->show();
         slider_->show();
      }
      else {
         formula_name_editor_->hide();
         formula_editor_->hide();
         formula_apply_formula_button_->hide();
         formula_simplify_button_->hide();
         formula_flatten_button_->hide();
         preview_scroll_->hide();
         preview_description_->hide();
         slider_->hide();
      }
   }

   inline void setPreviewScrollHeight(const int val)
   {
      preview_scroll_height_ = val;
   }

   inline int getPreviewScrollHeight() const
   {
      return preview_scroll_height_;
   }

   static void callbackNameChanged(Fl_Widget* widget, void* data);

   Fl_Input* formula_name_editor_{ nullptr };
   SyntaxHighlightMultilineInput* formula_editor_{ nullptr };
   Fl_Button* formula_apply_formula_button_{ nullptr };
   Fl_Button* formula_simplify_button_{ nullptr };
   Fl_Button* formula_flatten_button_{ nullptr };
   Fl_Scroll* preview_scroll_{ nullptr };
   Fl_Input* preview_description_{ nullptr };
   Fl_Value_Slider* slider_ = { nullptr };
   int preview_scroll_height_{};
};

class DragGroup : public Fl_Group {
public:
   DragGroup(int x, int y, int w, int h, const std::string& json_name, const std::string& name, const std::string& value, MCScene* mc_scene)
      : Fl_Group(x, y, w, h), bb_json_name{ json_name }, father_mc_scene_{ mc_scene }
   {
      box(FL_DOWN_BOX);
      end();

      initializeEditor(name, value);
   }

   int handle(int event) {
      switch (event) {
      case FL_PUSH:
         // Start drag operation
         if (Fl::event_button() == FL_LEFT_MOUSE) {
            offset_x = Fl::event_x() - x();
            offset_y = Fl::event_y() - y();
            return 1;
         }
         break;
      case FL_DRAG:
         // Update group position during drag
         if (Fl::event_button() == FL_LEFT_MOUSE) {
            position(Fl::event_x() - offset_x, Fl::event_y() - offset_y);
            return 1;
         }
         break;
      case FL_RELEASE:
         if (callback()) {
            do_callback(); // Call the provided callback function
            return 1;
         }
         break;
      default:
         break;
      }

      return Fl_Group::handle(event);
   }

   inline void resetOffsets()
   {
      offset_x = 0;
      offset_y = 0;
   }

   inline std::string getJsonName() const { return bb_json_name; }
   inline std::string getName() const { return editor_.formula_name_editor_->value(); }
   inline void setName(const std::string& value) { editor_.formula_name_editor_->value(value.c_str()); }
   inline std::string getFormula() const { return editor_.formula_editor_->value(); }
   inline void setValue(const std::string& value) { editor_.formula_editor_->value(value.c_str()); }
   inline FormulaEditor getEditor() const { return editor_; }
   inline MCScene* getMCScene() const { return father_mc_scene_; }

   void initializeEditor(const std::string& name, const std::string& formula);

   static std::string removeFunctionStuffFromString(const std::string& str);
   static void buttonApplyCallback(Fl_Widget* widget, void* data);
   static void buttonSimplifyCallback(Fl_Widget* widget, void* data);
   static void buttonFlattenCallback(Fl_Widget* widget, void* data);

private:
   int offset_x{ 0 };
   int offset_y{ 0 };

   std::string bb_json_name{};
   FormulaEditor editor_{};
   MCScene* father_mc_scene_{};
};

class TransparentButton : public Fl_Button {
public:
   TransparentButton(int x, int y, int width, int height, const char* label = nullptr)
      : Fl_Button(x, y, width, height, label) {
   }
   void draw() override {
      //Fl_Button::draw();
   }
};

enum class ProgressElement {
   value,
   maximum,
   stage
};

class SingleExpController {
public:
   SingleExpController(
      const std::string& template_path,
      const std::string& generated_path,
      const std::string& my_id,
      const std::string& short_id,
      MCScene* mc_scene
   );

   Fl_Group* sec_group_ = new Fl_Group(10, 10, 380, 450, "Group"); // The size of the group is set elsewhere. Leave these numbers alone!
   Fl_Box* image_box_envmodel_ = new Fl_Box(0, 45, 12, 12);
   Fl_Box* labelBox_envmodel_ = new Fl_Box(20, 70, 60, 30, "EnvModel");
   Fl_Box* image_box_mcrun_ = new Fl_Box(0, 135, 12, 12);
   Fl_Box* labelBox_mcrun_ = new Fl_Box(20, 160, 60, 30, "MC Run");
   Fl_Box* image_box_testcase_ = new Fl_Box(0, 225, 12, 12);
   Fl_Box* labelBox_testcase_ = new Fl_Box(20, 250, 60, 30, "Test case");
   TransparentButton* invisible_widget_ = new TransparentButton(10, 10, 380, 280, "");
   Fl_Button* cancel_button_ = new Fl_Button(100, 135, 40, 100);
   Fl_Check_Button* checkbox_selected_job_ = new Fl_Check_Button(0, 0, 15, 15);
   Fl_Progress* progress_bar_sec_ = new Fl_Progress(0, 1000, sec_group_->w(), 30);
   Fl_Box* progress_description_ = new Fl_Box(0, 1150, sec_group_->w(), 30);
   Fl_Value_Slider* slider_ = new Fl_Value_Slider(15, 350, sec_group_->w() - 80, sec_group_->h() / 5);

   void tryToSelectController(
      const std::filesystem::path& source_path,
      const std::filesystem::path& path_generated_base) const;

   std::string getMyId() const;
   std::string getShortId() const;
   std::string getAbbreviatedShortId() const;
   std::filesystem::path getPathOnDisc() const;
   void adjustAbbreviatedShortID() const;

   void adjustWidgetsAppearances() const;

   std::map<ProgressElement, std::string> getProgress() const;
   std::string getProgressStage() const;
   void removeProgressFile() const;

   bool operator<(const SingleExpController& other) const
   {
      return my_id_ < other.my_id_;
   }

   bool hasEnvmodel() const;
   bool hasPreview() const;
   void setHasTestcase(const bool value);
   void setHasEnvmodel(const bool value);
   void setHasPreview(const bool value);
   bool hasTestcase() const;
   bool hasSlider() const;

   std::vector<std::string> getVariableNames() const;
   std::vector<float> getVariableValues() const;

   std::string template_path_{};
   std::string generated_path_{};
   std::string my_id_{};
   std::string my_short_id_{};
   std::string my_abbr_short_id_{};
   MCScene* mc_scene_{};

   mutable OptionsLocal runtime_local_options_;

   mutable bool has_envmodel_{};
   mutable bool has_preview_{};
   mutable bool has_testcase_{};
   mutable bool selected_{};

   // Callbacks
   static void checkboxJobSelectedCallback(Fl_Widget* widget, void* data);
   static void cancelButtonCallback(Fl_Widget* widget, void* data);
   static void sliderCallback(Fl_Widget* widget, void* data);

   void registerCallbacks() const;

private:
};

class CustomMultiBrowser : public Fl_Hold_Browser {
public:
   CustomMultiBrowser(int x, int y, int w, int h, const char* label = 0)
      : Fl_Hold_Browser(x, y, w, h, label) {
   }

   void item_draw(void* item, int X, int Y, int W, int H) const override {
      const char* itemText = item_text(item);

      if (itemText[0] == '*') {
         const_cast<CustomMultiBrowser*>(this)->textcolor(FL_RED);
      }
      else {
         const_cast<CustomMultiBrowser*>(this)->textcolor(FL_BLACK);
      }

      Fl_Hold_Browser::item_draw(item, X, Y, W, H);
   }
};

class ProgressDetector {
public:
   inline ProgressDetector() {}

   void placeProgressImage(
      const std::string& path_template, 
      const std::set<std::string>& packages, 
      const std::set<SingleExpController>& se_controllers,
      const std::filesystem::path& path_generated_base_parent);

private:
   Fl_PNG_Image* check_image_ = nullptr;
   Fl_PNG_Image* progress_image_ = nullptr;
   Fl_PNG_Image* cross_image_ = nullptr;
   Fl_PNG_Image* empty_image_ = nullptr;

   void loadProgressImagesFromDisc(const std::string& path_template, const int w, const int h);
};

} // vfm