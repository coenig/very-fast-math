//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file

#include "cpp_parsing/cpp_type_struct.h"
#include "cpp_parsing/cpp_type_enum.h"
#include "static_helper.h"

using namespace vfm;

TypeWithInitAndRangeOrAka vfm::CppTypeStruct::copy(const TypeWithInitAndRangeOrAka& type, const bool copy_constness)
{
   auto init_value_copy = type.second.first ? std::make_shared<std::string>(*type.second.first) : nullptr;
   auto range_description_copy = type.second.second.first ? std::make_shared<std::string>(*type.second.second.first) : nullptr;
   auto aka_description_copy = type.second.second.second ? std::make_shared<std::string>(*type.second.second.second) : nullptr;
   return { type.first->copy(copy_constness), { init_value_copy, { range_description_copy, aka_description_copy } } };
}

vfm::CppTypeStruct::CppTypeStruct(const std::string& name, const std::shared_ptr<MembersType> members)
   : type_name_(name), members_(members ? *members : MembersType{})
{
}

std::string vfm::CppTypeStruct::getName() const
{
   return type_name_;
}

TypeWithInitAndRangeOrAka vfm::CppTypeStruct::deriveSubTypeFor(
   const std::string& full_qualified_name_raw,
   const bool is_local_var_name, // Has been renamed from C++ name "var" to "some_function.var".
   const bool start_at_first, 
   const ErrorLevelEnum log_level_on_error)
{
   if (StaticHelper::stringStartsWith(full_qualified_name_raw, GLOBAL_VARIABLES_PREFIX)) {
      return TypeWithInitAndRangeOrAka{ shared_from_this(), { nullptr, { nullptr, nullptr } } };
   }

   std::string full_qualified_name = full_qualified_name_raw;
   
   if (is_local_var_name) {
      size_t pos0 = full_qualified_name.find('.');
      full_qualified_name.at(pos0) = '$';

      int pos1 = full_qualified_name.find('.', pos0 + 1);
      int pos2 = full_qualified_name.find('.', pos1 + 1);

      if (pos1 > 0) {
         pos2 = pos2 < 0 ? full_qualified_name.size() : pos2;
         bool is_int = true;
         for (size_t i = pos1 + 1; i < pos2; i++) {
            if (full_qualified_name[i] < '0' || full_qualified_name[i] > '9') {
               is_int = false;
               break;
            }
         }

         if (is_int) {
            full_qualified_name.at(pos1) = '$';
         }
      }
   }

   auto vec = StaticHelper::split(full_qualified_name, ".");

   vec.at(0) = StaticHelper::replaceAll(vec.at(0), "$", ".");
   full_qualified_name = StaticHelper::replaceAll(full_qualified_name, "$", ".");

   auto child = shared_from_this();
   std::shared_ptr<std::string> init_value;
   std::shared_ptr<std::string> range_description;
   std::shared_ptr<std::string> aka_description;
   bool first = !start_at_first;

   for (const auto& subpart : vec) {
      if (first) {
         first = false;
      } else {
         bool found = false;

         for (const auto& member : child->members_) {
            if (subpart == member.first) {
               child = member.second.first;
               init_value = member.second.second.first ? std::make_shared<std::string>(*member.second.second.first) : nullptr;
               range_description = member.second.second.second.first ? std::make_shared<std::string>(*member.second.second.second.first) : nullptr;
               aka_description = member.second.second.second.second ? std::make_shared<std::string>(*member.second.second.second.second) : nullptr;
               found = true;
               break;
            }
         }

         if (!found) {
            auto members = child->getMemberNamesPlain();
            Failable::getSingleton(FAILABLE_NAME_CPP_DATATYPES)->addError(
               "Could not derive type for '" + full_qualified_name + "'. "
               + "I got stuck at '" + subpart + "' which is not a member of '" + child->getName() + "'. ", log_level_on_error, "\n");

            if (!members.empty()) {
               auto matches = StaticHelper::findClosest(members, subpart);
               Failable::getSingleton(FAILABLE_NAME_CPP_DATATYPES)->addError("Did you mean '" + members.at(std::get<0>(matches)) + "'?", log_level_on_error, "\n");
            }

            return { nullptr, {} };
         }
      }
   }

   return TypeWithInitAndRangeOrAka{ child, { init_value, { range_description, aka_description } } };
}

std::vector<std::string> vfm::CppTypeStruct::getAllAtomicMemberNamesRecursively() const
{
   return getAllAtomicMemberNamesRecursively("");
}

std::vector<std::string> vfm::CppTypeStruct::getAllAtomicMemberNamesRecursively(const std::string& current_name) const
{
   std::vector<std::string> vec;
   auto child = shared_from_this();

   for (const auto& member : child->members_) {
      if (member.second.first->toAtomicIfApplicable()) {
         vec.push_back(current_name + member.first);
      }
      else {
         auto vec2 = member.second.first->getAllAtomicMemberNamesRecursively(current_name + member.first + ".");
         vec.insert(vec.end(), vec2.begin(), vec2.end());
      }
   }

   return vec;
}

std::vector<std::string> vfm::CppTypeStruct::getMemberNamesPlain() const
{
   std::vector<std::string> vec;

   for (const auto& member : members_) {
      vec.push_back(member.first);
   }

   return vec;
}

std::string vfm::CppTypeStruct::toString() const
{
   return toString("");
}

std::string vfm::CppTypeStruct::toStringNuXmv() const
{
   auto nct = const_cast<CppTypeStruct*>(this);

   if (nct->toEnumIfApplicable()) {
      std::string res{ "{ " };

      for (const auto& pair : nct->toEnumIfApplicable()->getPossibleValues()) {
         res += getName() + "____" + pair.second + ",";
      }

      if (!res.empty()) {
         res.pop_back();
      }

      return StaticHelper::replaceAll(res, ",", ", ") + " }";
   }
   else if (nct->toBoolIfApplicable()) {
      return "boolean";
   }
   else if (nct->toAtomicIfApplicable()) {
      return "integer"; // TODO: For now all non-bool, non-enum is int.
   }
   else {
      return "#non-atomic-struct-cannot-be-exported-to-nuxmv#";
   }
}

std::string vfm::CppTypeStruct::toString(const std::string& prefix, const std::string& prefix_extender, const std::string& name) const
{
   std::string s{ prefix + type_name_ + " " + name + (isConst() ? " <const>" : "<non-const>") };

   for (const auto& el : members_) {
      s += "\n" + el.second.first->toString(prefix + prefix_extender, prefix_extender, el.first) + (el.second.second.first ? " = " + *el.second.second.first : "");
   }

   return s;
}

std::shared_ptr<CppTypeStruct> vfm::CppTypeStruct::copy(const bool copy_constness) const
{
   auto new_map = std::make_shared<MembersType>();

   for (const auto& el : members_) {
      new_map->push_back({ el.first, { el.second.first->copy(copy_constness), el.second.second } });
   }

   auto cp = std::make_shared<CppTypeStruct>(type_name_, new_map);

   if (copy_constness && isConst()) {
      cp->setConst(true);
   }

   return cp;
}

std::vector<std::pair<std::vector<std::string>, std::string>> vfm::CppTypeStruct::getCppAssociations(const std::string& base_name, const bool is_const, const int id) const
{
   std::vector<std::pair<std::vector<std::string>, std::string>> vec;

   for (const auto& m : members_) {
      const auto child_assoc = m.second.first->getCppAssociations(base_name + "." + m.first, is_const, id);

      for (const auto& el : child_assoc) {
         vec.push_back(el);
      }
   }

   return vec;
}

std::shared_ptr<CppTypeAtomic> vfm::CppTypeStruct::toIntegerIfApplicable()
{
   return nullptr;
}

std::shared_ptr<CppTypeAtomic> vfm::CppTypeStruct::toAtomicIfApplicable()
{
   return nullptr;
}

std::shared_ptr<CppTypeEnum> vfm::CppTypeStruct::toEnumIfApplicable()
{
   return nullptr;
}

std::shared_ptr<CppTypeAtomic> vfm::CppTypeStruct::toBoolIfApplicable()
{
   return nullptr;
}

void vfm::CppTypeStruct::setConst(const bool is_const)
{
   const_ = is_const;
}

bool vfm::CppTypeStruct::isConst() const
{
   return const_;
}

MembersType vfm::CppTypeStruct::getMembers() const
{
   return members_;
}
