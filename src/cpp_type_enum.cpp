//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file

#include "cpp_parsing/cpp_type_enum.h"

using namespace vfm;

vfm::CppTypeEnum::CppTypeEnum(const std::string& name, const std::shared_ptr<std::map<float, std::string>> possible_values) 
   : CppTypeAtomic(name, DataPack::AssociationType::Char), possible_values_(possible_values ? *possible_values : std::map<float, std::string>{ {INVALID_NUM, "**INVALID_ENUM**"} })
{
}

std::shared_ptr<std::map<float, std::string>> createFullEnumMapping(const std::vector<std::string>& possible_values)
{
   auto res = std::make_shared< std::map<float, std::string>>();

   for (int i = 0; i < possible_values.size(); i++) {
      res->insert({ i, possible_values[i] });
   }

   return res;
}

vfm::CppTypeEnum::CppTypeEnum(const std::string& name, const std::vector<std::string>& possible_values) : CppTypeEnum(name, createFullEnumMapping(possible_values))
{
}

std::map<float, std::string> vfm::CppTypeEnum::getPossibleValues() const
{
   return possible_values_;
}

std::string vfm::CppTypeEnum::toString(const std::string& prefix, const std::string& prefix_extender, const std::string& name) const
{
   std::string s{};

   s = CppTypeAtomic::toString(prefix, prefix_extender, "") + " { ";

   for (const auto& el : possible_values_) {
      s += el.second + (el.first != INVALID_NUM ? " (" + std::to_string(el.first) + ")" : "") + ", ";
   }

   if (!possible_values_.empty()) {
      s = s.substr(0, s.size() - 2);
   }

   s += " }" + (name.empty() ? "" : " " + name);

   return s + (isConst() ? " <const>" : "<non-const>");
}

std::shared_ptr<CppTypeStruct> vfm::CppTypeEnum::copy(const bool copy_constness) const
{
   auto new_map{ std::make_shared<std::map<float, std::string>>() };

   for (const auto el : possible_values_) {
      new_map->insert(el);
   }

   return std::make_shared<CppTypeEnum>(getName(), new_map);
}

std::shared_ptr<CppTypeEnum> vfm::CppTypeEnum::toEnumIfApplicable()
{
   return std::dynamic_pointer_cast<CppTypeEnum>(this->shared_from_this());
}
