//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2024 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file

#include "gui/interpreter_terminal.h"
#include "model_checking/simplification.h"
#include "vfmacro/script.h"

using namespace vfm;

InterpreterTerminal::InterpreterTerminal(const std::shared_ptr<DataPack> data, const std::shared_ptr<FormulaParser> parser, int X, int Y, int W, int H, const char* L) 
   : EditorForInterpreterTerminal(X, Y, W, H, L), data_{ data }, parser_{ parser }
{
   buff = new Fl_Text_Buffer();
   buffer(buff);
   textfont(FL_COURIER);
   textsize(12);

   data_->addOrChangeErrorOrOutputStream(output_, true);
   data_->addOrChangeErrorOrOutputStream(output_, false);
   parser_->addOrChangeErrorOrOutputStream(output_, true);
   parser_->addOrChangeErrorOrOutputStream(output_, false);
}

void vfm::InterpreterTerminal::appendPlain(const char* s)
{
   buff->append(s);
}

void vfm::InterpreterTerminal::appendAndSetCursorToEnd(const char* s)
{
   appendPlain(s);
   insert_position(buffer()->length());
   scroll(count_lines(0, buffer()->length(), 1), 0);
}

void vfm::InterpreterTerminal::fool(
   const std::string& command_str, 
   const std::shared_ptr<DataPack> data, 
   const std::shared_ptr<FormulaParser> parser,
   InterpreterTerminal* term,
   const bool only_one_step)
{
   term->result_ = macro::Script::processScript(command_str, macro::Script::DataPreparation::none, only_one_step, data, parser, nullptr, macro::Script::SpecialOption::add_default_dynamic_methods);
}

void vfm::InterpreterTerminal::runCommand(const char* command)
{
   output_.str("");
   const std::string command_str{ command };
   std::string res{};

   if (command_str == "#cls") {
      buffer()->replace(0, buffer()->length(), "");
   } else {
      output_ << "\n";
      try {
         result_ = WAITING;
         std::thread t{ fool, command_str, data_, parser_, this, false };
         t.detach();
      }
      catch (const std::exception& e) {
         res += "#Error '" + std::string(e.what()) + "' occurred during script processing.";
      }
   }
   
   output_ << res;
   appendAndSetCursorToEnd(output_.str().c_str());
}

void vfm::InterpreterTerminal::expandMultilineScript(const std::string& script, const bool only_one_step)
{
   result_ = WAITING;
   optional_script_ = script;
   last_script_enclosing_begin_tag_ = only_one_step ? BEGIN_TAG_MULTILINE_SCRIPT_SINGLE_STEP : BEGIN_TAG_MULTILINE_SCRIPT;
   last_script_enclosing_end_tag_ = only_one_step ? END_TAG_MULTILINE_SCRIPT_SINGLE_STEP : END_TAG_MULTILINE_SCRIPT;
   readonly(true);

   std::thread t{ fool, script, data_, parser_, this, only_one_step };
   t.detach();
}

bool vfm::InterpreterTerminal::surroundSelectionWithBrackets(const std::string& before, const std::string& after)
{
   auto sel = buffer()->primary_selection();

   if (sel->selected()) {
      const int start = sel->start();
      const int end = sel->end();
      const std::string selected_text{ std::string(buffer()->text_range(start, end)) };
      buffer()->replace(start, end, (before + "\n" + selected_text + "\n" + after).c_str());
      insert_position(start + before.size() + 1);

      return true;
   }

   return false;
}

bool vfm::InterpreterTerminal::insertBrackets(const std::string& before, const std::string& after)
{
   const int pos = insert_position();
   buffer()->insert(pos, (before + "\n\n" + after).c_str());
   insert_position(pos + before.size() + 1);
   return true;
}

int vfm::InterpreterTerminal::handle(int e)
{
   int key = Fl::event_key();
   
   switch (e) {
      case FL_KEYUP: {
         if (key == FL_Enter && !Fl::event_state(FL_SHIFT)) return(1); // hide Enter from editor
         break;
      }
      case FL_KEYDOWN: {
         if (key == FL_Enter && !Fl::event_state(FL_SHIFT)) {
            const std::string all_editor_text{ std::string(buffer()->text()) };
            const int position = insert_position();

            if (StaticHelper::isWithinLevelwise(all_editor_text, position, BEGIN_TAG_MULTILINE_SCRIPT, END_TAG_MULTILINE_SCRIPT)) {
               const bool is_automatic_single_step{ StaticHelper::isWithinLevelwise(all_editor_text, position, BEGIN_TAG_MULTILINE_SCRIPT_SINGLE_STEP, END_TAG_MULTILINE_SCRIPT_SINGLE_STEP) };
               std::string script{ StaticHelper::extractPartWithinEnclosingBrackets(
                  all_editor_text, 
                  position, 
                  is_automatic_single_step ? BEGIN_TAG_MULTILINE_SCRIPT_SINGLE_STEP : BEGIN_TAG_MULTILINE_SCRIPT,
                  is_automatic_single_step ? END_TAG_MULTILINE_SCRIPT_SINGLE_STEP : END_TAG_MULTILINE_SCRIPT) };
               expandMultilineScript(script, is_automatic_single_step);
            }
            else {
               const auto lines = StaticHelper::split(all_editor_text, "\n");
               const char* text = buffer()->text();
               int line_number = 0;

               for (int i = 0; i < position; ++i) {
                  if (text[i] == '\n') {
                     line_number++;
                  }
               }
               
               if (line_number < 0 || line_number >= lines.size()) return(1); // Just a sanity check, should never happen.

               runCommand(lines[line_number].c_str());
               appendAndSetCursorToEnd("\n");
            }

            return(1); // hide 'Enter' from text widget
         }
         else if (Fl::event_state(FL_CTRL) && (key == 's' || key == 'y' || key == 'w')) {
            const std::string open{ key == 's'
               ? macro::INSCR_BEG_TAG
               : (key == 'y' ? BEGIN_TAG_MULTILINE_SCRIPT_SINGLE_STEP : BEGIN_TAG_MULTILINE_SCRIPT) };
            const std::string close{ key == 's'
               ? macro::INSCR_END_TAG + macro::METHOD_CHAIN_SEPARATOR + "mymethod"
               : (key == 'y' ? END_TAG_MULTILINE_SCRIPT_SINGLE_STEP : END_TAG_MULTILINE_SCRIPT)};

            if (surroundSelectionWithBrackets(open, close)) {
               return 1;
            }
            else if (insertBrackets(open, close)) {
               return 1;
            }
         }

         break;
      }
   }

   return(Fl_Text_Editor::handle(e));
}

std::string vfm::InterpreterTerminal::getResult() const
{
   return result_;
}

std::string vfm::InterpreterTerminal::getOptionalScript() const
{
   return optional_script_;
}

void vfm::InterpreterTerminal::setResult(const std::string& result)
{
   result_ = result;
}

void vfm::InterpreterTerminal::setOptionalScript(const std::string& result)
{
   optional_script_ = result;
}

void vfm::InterpreterTerminal::setText(const std::string& result)
{
   buffer()->replace(0, buffer()->length(), result.c_str());
}
