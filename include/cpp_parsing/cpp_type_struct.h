//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file

#pragma once
#include "term.h"
#include "failable.h"
#include <memory>
#include <map>
#include <vector>
#include <string>


namespace vfm {
const std::string GLOBAL_VARIABLES_PREFIX = "global.";

class CppTypeAtomic;
class CppTypeEnum;
class CppTypeStruct;

using PairOfInitValueAndRangeOrAkaDescription = std::pair<std::shared_ptr<std::string>, std::pair<std::shared_ptr<std::string>, std::shared_ptr<std::string>>>;
using TypeWithInitAndRangeOrAka = std::pair<std::shared_ptr<CppTypeStruct>, PairOfInitValueAndRangeOrAkaDescription>;
using MembersType = std::vector<std::pair<std::string, TypeWithInitAndRangeOrAka>>; // { Name, { Type, { InitValue, { RangeDescription, AkaDescription } } } }

// <LTL Specifications, CTL Specifications>
typedef std::pair<std::vector<TermPtr>, std::vector<TermPtr>> MCSpecification;

using CppType = std::shared_ptr<CppTypeStruct>;
using CppTypeWithQualifiers = std::pair<CppType, std::vector<std::string>>;
using CppFullTypeWithName = std::pair<std::string, CppTypeWithQualifiers>;

const std::string IMMUTABLE_ENUM_TYPE_NAME = "ImmutableEnumType";
const std::string IMMUTABLE_BOOL_CONST_TYPE_NAME = "ImmutableBoolConstType";

class CppTypeStruct : public std::enable_shared_from_this<CppTypeStruct> {
public:
   static TypeWithInitAndRangeOrAka copy(const TypeWithInitAndRangeOrAka& type, const bool copy_constness);

   CppTypeStruct(const std::string& name, const std::shared_ptr<MembersType> members = nullptr);
   std::string getName() const;

   TypeWithInitAndRangeOrAka deriveSubTypeFor(
      const std::string& full_qualified_name,
      const bool is_local_var_name,
      const bool start_at_first = false, 
      const ErrorLevelEnum log_level_on_error = ErrorLevelEnum::error);
   
   std::vector<std::string> getAllAtomicMemberNamesRecursively() const;
   std::vector<std::string> getAllAtomicMemberNamesRecursively(const std::string& current_name) const;
   std::vector<std::string> getMemberNamesPlain() const;
   void setConst(const bool is_const);
   bool isConst() const;
   MembersType getMembers() const;

   virtual std::string toString() const;
   virtual std::string toStringNuXmv() const;
   virtual std::string toString(const std::string& prefix, const std::string& prefix_extender = "  ", const std::string& name = "") const;
   virtual std::shared_ptr<CppTypeStruct> copy(const bool copy_constness) const;
   virtual std::vector<std::pair<std::vector<std::string>, std::string>> getCppAssociations(const std::string& base_name, const bool is_const, const int id) const;
   virtual std::shared_ptr<CppTypeAtomic> toIntegerIfApplicable();
   virtual std::shared_ptr<CppTypeAtomic> toAtomicIfApplicable();
   virtual std::shared_ptr<CppTypeEnum> toEnumIfApplicable();
   virtual std::shared_ptr<CppTypeAtomic> toBoolIfApplicable();

private:
   std::string type_name_{};
   MembersType members_{};
   bool const_{ false };
};

} // vfm