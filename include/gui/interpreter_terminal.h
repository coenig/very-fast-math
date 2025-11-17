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

class InterpreterTerminal : public Fl_Text_Editor {

public:
   InterpreterTerminal(const std::shared_ptr<DataPack> data, const std::shared_ptr<FormulaParser> parser, int X, int Y, int W, int H, const char* L = 0);
   void append(const char* s); // Append to buffer, keep cursor at end
   int handle(int e); // Handle events in the Fl_Text_Editor
   std::string getResult() const;
   void setResult(const std::string& result);

private:
   void runCommand(const char* command); // Run the specified command, append output to terminal
   static void fool(const std::string& command_str, const std::shared_ptr<DataPack> data, const std::shared_ptr<FormulaParser> parser, InterpreterTerminal* term);

   std::shared_ptr<DataPack> data_{};
   std::shared_ptr<FormulaParser> parser_{};
   std::ostringstream output_{};

   std::string result_{ EMPTY };

   Fl_Text_Buffer *buff;
};

} // vfm
