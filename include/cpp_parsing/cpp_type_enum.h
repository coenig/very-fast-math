//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file
#pragma once
#include "cpp_parsing/cpp_type_atomic.h"
#include <map>
#include <string>
#include <memory>
#include <limits>


namespace vfm {

constexpr int INVALID_NUM = std::numeric_limits<int>::max();

class CppTypeEnum : public CppTypeAtomic {
public:
   CppTypeEnum(const std::string& name, const std::shared_ptr<std::map<float, std::string>> possible_values);
   CppTypeEnum(const std::string& name, const std::vector<std::string>& possible_values);
   std::map<float, std::string> getPossibleValues() const;
   std::string toString(const std::string& prefix = "", const std::string& prefix_extender = "  ", const std::string& name = "") const override;
   std::shared_ptr<CppTypeStruct> copy(const bool copy_constness) const override;
   std::shared_ptr<CppTypeEnum> toEnumIfApplicable() override;

private:
   std::map<float, std::string> possible_values_;
};

} // vfm