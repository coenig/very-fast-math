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
   cmd[0] = 0;

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
      res += macro::Script::processScript(command_str, data_, parser_);
   }
   
   output_ << res;
   append(output_.str().c_str());
}

int vfm::InterpreterTerminal::handle(int e)
{
   int key = Fl::event_key();
   
   switch (e) {
      case FL_KEYUP: {
         if (key == FL_Enter) return(1); // hide Enter from editor
         if (key == FL_BackSpace && cmd[0] == 0) return(0);
         break;
      }
      case FL_KEYDOWN: {
         if (key == FL_Enter) {
            runCommand(cmd);
            cmd[0] = 0;
            append("\n");
            return(1); // hide 'Enter' from text widget
         }
         if (key == FL_BackSpace) {
            if (cmd[0]) {
               cmd[strlen(cmd) - 1] = 0;
               break;
            }
            else {
               return(0);
            }
         }
         else {
            strncat(cmd, Fl::event_text(), sizeof(cmd) - 1);
            cmd[sizeof(cmd) - 1] = 0;
         }
         break;
      }
   }

   if (key == FL_Left || key == FL_Right || key == FL_Up || key == FL_Down) {
      return 1; // Ignore all arrow keys since the command is only filled towards the end.
   }

   return(Fl_Text_Editor::handle(e));
}
