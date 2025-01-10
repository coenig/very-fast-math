//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file
#pragma once
#include "cpp_parsing/cpp_type_struct.h"
#include "equation.h"
#include "data_pack.h"
#include <vector>
#include <string>
#include <functional>


namespace vfm {

using AssociativityFunction = std::function<std::string(const std::string&, const std::string&, const std::string&, const std::string&, const bool)>;
using AssociativityPretext = std::function<std::vector<std::string>(
   const std::string& type_name,
   const std::string& generic_struct_name,
   const std::string& base_type, 
   const std::string& private_member_name, 
   const bool first)>;
using AssociativitySuite = std::pair<AssociativityPretext, AssociativityFunction>;

const auto DEFAULT_ASSOCIATIVITY_PRETEXT = [](const std::string&, const std::string&, const std::string&, const std::string&, const bool) { 
   return std::vector<std::string>{ "", "", "" };
};

const auto DEFAULT_ASSOCIATIVITY_CREATOR = [](const std::string& str, const std::string& type, const std::string& generic_struct_name, const std::string& original_type, const bool is_const) {
   std::string inner_part = "&" + str;

   if (is_const) {
      inner_part = "const_cast<" + original_type + "*>(" + inner_part + ")";
   }

   return std::string("reinterpret_cast<") + type + "*>(" + inner_part + ")";
};

const auto DEFAULT_VFC_ASSOCIATIVITY_CREATOR = [](const std::string& str, const std::string& type, const std::string& generic_struct_name, const std::string& original_type, const bool is_const) {
   std::string s = "&(" + str + ".*get(" + generic_struct_name + "()))";

   if (is_const) {
      s = "const_cast<" + type + "*>(" + s + ")";
   }

   return s;
};

const auto DEFAULT_VFC_ASSOCIATIVITY_PRETEXT = [](
   const std::string& type_name,           // vfc::CSI::si_kilometre_per_hour_f32_t
   const std::string& generic_struct_name, // GenericMember*i*
   const std::string& base_type, 
   const std::string& private_member_name,
   const bool first) {
   std::string part1 = "struct " + generic_struct_name + "\n\
{\n\
   typedef " + base_type + " " + type_name + "::*type;\n\
   friend type get(" + generic_struct_name + ");\n\
};\n\n";

   std::string part2 = first ? "template <typename T, typename T::type M>\n\
struct Rob\n\
{\n\
   friend typename T::type get(T)\n\
   {\n\
      return M;\n\
   }\n\
};\n\n" : "";
   
   std::string part3 = "template struct Rob<" + generic_struct_name + ", &" + type_name + "::" + private_member_name + ">; \n";

   return std::vector<std::string>{ part1, part2, part3 };
};

const AssociativitySuite DEFAULT_ASSOCIATIVITY_SUITE = { DEFAULT_ASSOCIATIVITY_PRETEXT, DEFAULT_ASSOCIATIVITY_CREATOR };
const AssociativitySuite DEFAULT_VFC_ASSOCIATIVITY_SUITE = { DEFAULT_VFC_ASSOCIATIVITY_PRETEXT, DEFAULT_VFC_ASSOCIATIVITY_CREATOR };

const std::string IN_VAR_NAME = "_IN";
const std::string OUT_VAR_NAME = "_EX";

enum class BaseCppType {

};

class CppTypeAtomic : public CppTypeStruct {
public:
   CppTypeAtomic(
      const std::string& type_name,
      const DataPack::AssociationType base_on_type,
      const AssociativitySuite& association_creator = DEFAULT_ASSOCIATIVITY_SUITE,
      const std::shared_ptr<Equation> in_mapping = nullptr,
      const std::shared_ptr<Equation> out_mapping = nullptr,
      const float = 1,
      const float = -1);

   CppTypeAtomic(
      const std::string& type_name,
      const DataPack::AssociationType base_on_type,
      const float range_low,
      const float range_high);

   virtual std::shared_ptr<CppTypeStruct> copy(const bool copy_constness) const override;
   std::vector<std::pair<std::vector<std::string>, std::string>> getCppAssociations(const std::string& base_name, const bool is_const, const int id) const override;
   std::shared_ptr<CppTypeAtomic> toIntegerIfApplicable() override;
   std::shared_ptr<CppTypeAtomic> toAtomicIfApplicable() override;
   std::shared_ptr<CppTypeAtomic> toBoolIfApplicable() override;
   std::shared_ptr<Equation> getInMapping() const;
   std::shared_ptr<Equation> getOutMapping() const;
   DataPack::AssociationType getBaseType() const;
   std::pair<float, float> getRange() const;
   void setRange(const float low, const float high);
   void extendRange(const float low, const float high);
   void extendRange(const std::pair<float, float>& range);

private:
   DataPack::AssociationType base_on_type_{};
   AssociativitySuite association_creator_{};
   std::shared_ptr<Equation> in_mapping_{};
   std::shared_ptr<Equation> out_mapping_{};
   std::pair<float, float> optional_range_{ 1, -1 };
};

} // vfm