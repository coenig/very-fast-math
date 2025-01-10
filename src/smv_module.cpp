//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file

#include "model_checking/smv_parsing/smv_module.h"
#include "static_helper.h"

using namespace vfm::mc::smv;


vfm::mc::smv::Module::Module() : Failable("SMV Module Element") {}

std::string vfm::mc::smv::Module::serialize() const
{
   std::string s = "MODULE " + module_name_ + "\n\n";

   for (const auto& pair : elements_singleton_) {
      std::visit([&s](auto&& module_element) {
         s += module_element.serialize();
      }, pair.second);
   }
   
   return s;
}

void vfm::mc::smv::Module::clear()
{
   elements_singleton_.clear();
   module_name_ = "";
}

bool vfm::mc::smv::Module::parseProgram(const std::string& program_raw)
{
   std::string program = StaticHelper::removeSingleLineComments(program_raw, "--");

   for (const auto& el_names_pair : elements_singleton_) {
      if (!el_names_pair.first.empty()) {
         program = StaticHelper::replaceAll(program, el_names_pair.first + " ", el_names_pair.first + "\n");
      }
   }

   ModuleElement* el{ &elements_singleton_.at(NAME_NULL_ELEMENT) };

   for (const auto& line : StaticHelper::split(program, "\n")) {
      if (StaticHelper::stringStartsWith(StaticHelper::removeWhiteSpace(line), "MODULE")) {
         module_name_ = StaticHelper::removeWhiteSpace(StaticHelper::replaceAll(line, "MODULE", ""));
      }
      else if (!StaticHelper::removeWhiteSpace(line).empty()) {
         if (StaticHelper::toUpperCase(line) == line && StaticHelper::isAlphaNumeric(StaticHelper::removeWhiteSpace(line))) {
            el = &elements_singleton_.at(StaticHelper::trimAndReturn(line));
         }
         else {
            std::visit([&line](auto&& module_element) {
               module_element.appendCodeLine(StaticHelper::trimAndReturn(line));
            }, *el);
         }
      }
   }

   return true;
}

std::vector<std::string> vfm::mc::smv::Module::splitAlongLines(const std::string& raw_code) const
{
   auto res_raw = vfm::StaticHelper::removeBlankLines(vfm::StaticHelper::removeSingleLineComments(raw_code, "--"));
   auto res = vfm::StaticHelper::split(
      res_raw,
      ";",
      [](const std::string& split_str, const int pos) -> bool {
         return !StaticHelper::isWithinLevelwise(split_str, pos, "case", "esac");
      });

   for (auto&& el : res) {
      StaticHelper::trim(el);
   }

   res.erase(std::remove_if(res.begin(), res.end(), [](const std::string& s) { return StaticHelper::isEmptyExceptWhiteSpaces(s); }), res.end());

   return res;
}

std::map<std::string, std::shared_ptr<SMVDatatype>> vfm::mc::smv::Module::getVariablesWithTypes() const
{
   const static std::string TEMP_DOUBLE_COLON_REPLACEMENT = "$$$-DC-$$$";
   std::map<std::string, std::shared_ptr<SMVDatatype>> res;

   for (const auto& line_raw : splitAlongLines(std::get<VarSection>(elements_singleton_.at(NAME_VAR)).getRawCode())) {
      auto line = StaticHelper::split(StaticHelper::trimAndReturn(StaticHelper::replaceAll(line_raw, "::", TEMP_DOUBLE_COLON_REPLACEMENT)), ":");

      if (line.size() != 2) {
         addError("Line '" + line_raw + "' malformed, cannot derive variable with type.");
      }

      std::string name = StaticHelper::replaceAll(StaticHelper::trimAndReturn(line[0]), TEMP_DOUBLE_COLON_REPLACEMENT, "::");
      std::string type_str = StaticHelper::replaceAll(StaticHelper::trimAndReturn(line[1]), TEMP_DOUBLE_COLON_REPLACEMENT, "::");
      std::shared_ptr<SMVDatatype> type{};

      if (type_str == "integer") {
         type = std::make_shared<SMVDatatypeInt>();
      }
      else if (type_str == "boolean") {
         type = std::make_shared<SMVDatatypeBool>();
      }
      else if (StaticHelper::stringContains(type_str, "..")) {
         type = std::make_shared<SMVDatatypeBoundedInt>();
      }
      else if (StaticHelper::stringContains(type_str, "{")) {
         type = std::make_shared<SMVDatatypeEnum>();
      }

      addFailableChild(type);
      type->parseProgram(type_str);
      res.insert({ name, type });
   }

   return res;
}

std::map<std::string, std::string> vfm::mc::smv::Module::getAllVariableInits() const
{
   std::map<std::string, std::string> res;

   for (const auto& line_raw : splitAlongLines(std::get<AssignSection>(elements_singleton_.at(NAME_ASSIGN)).getRawCode())) {
      auto line = StaticHelper::removeWhiteSpace(line_raw);

      if (StaticHelper::stringStartsWith(line, "init(")) {
         auto pair = StaticHelper::split(line, ":=");

         assert(pair.size() == 2);

         auto name = StaticHelper::replaceAll(StaticHelper::replaceAll(pair[0], "init(", ""), ")", "");
         auto init = StaticHelper::replaceAll(pair[1], ";", "");

         res.insert({ name, init });
      }
   }

   return res;
}

std::map<std::string, std::string> vfm::mc::smv::Module::getVariableDefines() const
{
   std::map<std::string, std::string> res;
   auto raw_code = splitAlongLines(std::get<DefineSection>(elements_singleton_.at(NAME_DEFINE)).getRawCode());

   for (const auto& line_raw : raw_code) {
      auto line_split = StaticHelper::split(line_raw, ":=");

      if (line_split.size() != 2) {
         addError("Line '" + line_raw + "' malformed, cannot derive define.");
      }

      std::string name = StaticHelper::trimAndReturn(line_split[0]);
      std::string definition = StaticHelper::trimAndReturn(line_split[1]);

      res.insert({ name, definition });
   }

   return res;
}

std::set<std::string> vfm::mc::smv::Module::getAllVariablesAndDefinesForOutsideUsage() const
{
   std::set<std::string> vec{};

   auto vars = getVariablesWithTypes();
   auto defs = getVariableDefines();

   for (const auto& var : vars) {
      vec.insert(var.first);
   }

   for (const auto& def : defs) {
      vec.insert(def.first);
   }

   return vec;
}
