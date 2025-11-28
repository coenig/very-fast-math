//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2024 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file
#pragma once

#include "parser.h"
#include "data_pack.h"
#include <string.h>
#include <stdio.h>
#include <FL/Fl.H>
#include <FL/Fl_Double_Window.H>
#include <FL/Fl_Text_Editor.H>

namespace vfm {

static const std::string WAITING{ "$!W-A-I-T-I-N-G!$" };
static const std::string EMPTY{ "$!E-M-P-T-Y!$" };

static const std::string BEGIN_TAG_MULTILINE_SCRIPT{ "@<" };
static const std::string END_TAG_MULTILINE_SCRIPT{ ">@" };
static const std::string BEGIN_TAG_MULTILINE_SCRIPT_SINGLE_STEP{ "@<<" };
static const std::string END_TAG_MULTILINE_SCRIPT_SINGLE_STEP{ ">>@" };


class EditorForInterpreterTerminal : public Fl_Text_Editor
{
public:
   inline EditorForInterpreterTerminal(int x, int y, int w, int h, const char* l = 0)
      :Fl_Text_Editor(x, y, w, h, l), readonly_{ false }, grey_{ FL_BACKGROUND_COLOR }, normal_{ FL_BACKGROUND2_COLOR }
   {}

   inline int handle(int e)
   {
      int rv = 0;
      if (readonly_)
         rv = Fl_Text_Display::handle(e);
      else
         rv = Fl_Text_Editor::handle(e);
      return rv;
   }

   inline void readonly(bool in_set)
   {
      readonly_ = in_set;
      color(readonly_ ? grey_ : normal_);
   }

   inline bool isReadonly() const
   {
      return readonly_;
   }

private:
   bool readonly_{};
   Fl_Color grey_;
   Fl_Color normal_;
};

class InterpreterTerminal : public EditorForInterpreterTerminal {

public:
   InterpreterTerminal(const std::shared_ptr<DataPack> data, const std::shared_ptr<FormulaParser> parser, int X, int Y, int W, int H, const char* L = 0);
   void appendPlain(const char* s);
   void appendAndSetCursorToEnd(const char* s);
   int handle(int e); // Handle events in the Fl_Text_Editor
   std::string getResult() const;
   std::string getOptionalScript() const;
   void setResult(const std::string& result);
   void setOptionalScript(const std::string& result);
   void setText(const std::string& test);

   static void fool(
      const std::string& command_str, 
      const std::shared_ptr<DataPack> data, 
      const std::shared_ptr<FormulaParser> parser, 
      InterpreterTerminal* term,
      const bool only_one_step);

   std::string last_script_enclosing_begin_tag_{};
   std::string last_script_enclosing_end_tag_{};

private:
   void runCommand(const char* command); // Run the specified command, append output to terminal
   void expandMultilineScript(const std::string& script, const bool only_one_step);
   std::shared_ptr<DataPack> data_{};
   std::shared_ptr<FormulaParser> parser_{};
   std::ostringstream output_{};

   std::string result_{ EMPTY };
   std::string optional_script_{ EMPTY };

   Fl_Text_Buffer *buff;
};

} // vfm
