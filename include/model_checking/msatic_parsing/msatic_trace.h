//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file
/// @author Christian Heinzemann, translated from Python by Lukas König
#pragma once

#include "model_checking/mc_types.h"
#include "static_helper.h"
#include "failable.h"

namespace vfm {

namespace mc::msatic {

struct Parser : public Failable {
   Parser() : Failable("MSATIC-Parser") {}

   std::vector<std::string> extractLinesFromRawTrace(const std::string& raw_trace)
   {
      bool witness_flag_found = false;
      bool first_state_found = false;
      std::vector<std::string> trace{};

      for (auto curLine : StaticHelper::split(raw_trace, "\n")) {
         curLine = StaticHelper::trimAndReturn(curLine);
         if (!witness_flag_found && StaticHelper::stringContains(curLine, "Witness")) {
            witness_flag_found = true;
         }
         else if (StaticHelper::stringStartsWith(curLine, "Unsafe")) {
            break;
         }
         else if (witness_flag_found && !first_state_found) {
            // we expect a line with a 0 and possibly several empty lines
            if (curLine.size() < 1 || StaticHelper::stringStartsWith(curLine, "0")) {
               continue;
            }
            else {
               first_state_found = true;
               trace.push_back(curLine);
            }
         }
         else if (witness_flag_found && first_state_found && curLine.size() >= 1) {
            trace.push_back(curLine);
         }
      }

      return trace;
   }

   int getValueForEncodedInt(std::vector<std::pair<int, int>>& bits)
   {
      int value = 0;

      for (const auto& bit : bits) {
         value += bit.second * std::pow(2, (bits.size() - 1 - bit.first));
      }

      return value;
   }

   void parseSingleStep(std::string& step, TraceStep& trace_step)
   {
      addNote("Going through step '" + step + "' (" + trace_step.first + ").\n");

      // there are three kinds of variables: bools, integers, and encoded integers/enums
      // bools appear either as literal "someBool" or negated in parenthesis "(not someBool)"
      // integers always appear in the form "(= someInt value)" for positive values or "(= someInt (value))" for negative values
      // encoded integers/enums appear as multiple, non consecutive variables encoded as booleans "someEncInt.0" or "(not someEndInt.1)"
      step = step + " ";
      size_t pos = 0;
      std::map<std::string, std::vector<std::pair<int, int>>> encoded_ints{};
      int index{};

      while (pos < step.size()) {
         std::string var_string{};

         if (step[pos] == '(') {
            // search closing ")" and parse content in-between
            auto close_idx = step.find(')', pos);
            if (close_idx < 0) {
               addError("Malformed trace");
            }
            var_string = step.substr(pos + 1, close_idx - (pos + 1));
            // if the var_string contains another '(', then we have an entry with a negative value. If so, search for another closing ')' after the current close_idx
            if (StaticHelper::stringContains(var_string, "(")) {
                close_idx = step.find(')', close_idx + 1);
                if (close_idx == -1) {
                    addError("Malformed trace");
            }
                var_string = step.substr(pos + 1, close_idx - (pos + 1));
            }
            pos = close_idx + 1;
         }
         else if (step[pos] == ' ') {
            pos = pos + 1;
            continue;
         }
         else {
            // we are at a var string, search next " " and parse content in-between
            auto close_idx = step.find(" ", pos);
            if (close_idx == -1) {
               addError("Malformed trace");
            }
            var_string = step.substr(pos, close_idx - pos);
            pos = close_idx + 1;
         }

         // parse var_string
         if (StaticHelper::stringStartsWith(var_string, "=")) {
            // this is an integer variable with explicit value
            var_string = StaticHelper::replaceAll(var_string, "(", "");
            var_string = StaticHelper::replaceAll(var_string, ")", "");
            auto parts = StaticHelper::split(var_string, " ");
            if (parts.size() == 3) {
               // positive value
               auto var_name = parts[1];
               auto var_value = parts[2];
               trace_step.second[var_name] = var_value;
            }
            else if (parts.size() == 4) {
               //negative value;
               auto var_name = parts[1];
               auto var_value = parts[2] + parts[3];
               trace_step.second[var_name] = var_value;
            }
            else {
               addError("Malformed trace, error parsing integer variable in " + var_string);
            }
         }
         else if (StaticHelper::stringStartsWith(var_string, "not")) {
            auto parts = StaticHelper::split(var_string, " ");
            if (parts.size() != 2) {
               addError("Malformed trace, error parsing negated bool in " + var_string);
            }
            auto var_ref = parts[1];

            // check if variable contains an array and remove marking if so 
            if (StaticHelper::stringStartsWith(var_ref, "|")) {
               var_ref = var_ref.substr(2);
               var_ref.pop_back();
               var_ref.pop_back();
            }

            //negated bool or bit -> check if char second to last is a '.' and last is a digit
            if (var_ref[var_ref.size() - 2] == '.' && std::isdigit(var_ref[var_ref.size() - 1])) {
               // bit of an encoded integer -> store for later processing
               auto var_name = var_ref.substr(0, var_ref.size() - 2);
               index = std::stoi(StaticHelper::makeString(var_ref[var_ref.size() - 1]));
               if (!encoded_ints.count(var_name)) {
                   encoded_ints[var_name] = {};
               }
               encoded_ints[var_name].push_back({ index, 0 });
            }
            else {
               // bool
               trace_step.second[var_ref] = "FALSE";
            }
         }
         else {
            // check if variable contains an array and remove marking if so 
            if (StaticHelper::stringStartsWith(var_string, "|")) {
               var_string = var_string.substr(2, var_string.size() - 2 - 2);
            }

            // bool or bit with value 1 -> check if char second to last is a '.' and last is a digit
            if (var_string[var_string.size() - 2] == '.' && std::isdigit(var_string[var_string.size() - 1])) {
               // bit of an encoded integer -> store for later processing
               auto var_name = var_string.substr(0, var_string.size() - 2);
               index = std::stoi(StaticHelper::makeString(var_string[var_string.size() - 1]));

               if (!encoded_ints.count(var_name)) {
                   encoded_ints[var_name] = {};
               }
               encoded_ints[var_name].push_back({ index, 1 });
            }
            else {
               // bool
               trace_step.second[var_string] = "TRUE";
            }
         }
      }

      // finally, obtain values for all encoded ints
      for (const auto& var_name : encoded_ints) {
         auto value = getValueForEncodedInt(encoded_ints[var_name.first]);
         trace_step.second[var_name.first] = std::to_string(value);
      }
   }

   MCTrace parseStepsFromTrace(const std::vector<std::string>& str_trace)
   {
      MCTrace parsed_steps{};
      int num = 1;

      for (const auto& step : str_trace) {
         std::string step_repl = StaticHelper::replaceAll(step, "__AT0", "");
         TraceStep trace_step{ "1." + std::to_string(num), {} };
         num++;
         parseSingleStep(step_repl, trace_step);
         parsed_steps.push_back(trace_step);
      }

      return parsed_steps;
   }
};

} // mc::msatic
} // vfm