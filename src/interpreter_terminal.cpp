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
   : Fl_Text_Editor(X, Y, W, H, L), data_{ data }, parser_{ parser } 
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

void vfm::InterpreterTerminal::append(const char* s)
{
   buff->append(s);
   // Go to end of line
   insert_position(buffer()->length());
   scroll(count_lines(0, buffer()->length(), 1), 0);
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
         res += macro::Script::processScript(command_str, data_, parser_);
      }
      catch (const std::exception& e) {
         res += "#Error '" + std::string(e.what()) + "' occurred during script processing.";
      }
   }
   
   output_ << res;
   append(output_.str().c_str());
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
            const auto lines = StaticHelper::split(std::string(buffer()->text()), "\n");
            const int position = insert_position();
            const char* text = buffer()->text();
            int line_number = 0;

            for (int i = 0; i < position; ++i) {
               if (text[i] == '\n') {
                  line_number++;
               }
            }

            if (line_number < 0 || line_number >= lines.size()) return(1); // Just a sanity check, should never happen.

            runCommand(lines[line_number].c_str());
            append("\n");
            return(1); // hide 'Enter' from text widget
         }
         break;
      }
   }

   return(Fl_Text_Editor::handle(e));
}
