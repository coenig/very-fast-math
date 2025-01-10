//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file

#include "model_checking/smv_parsing/smv_module_elements.h"

using namespace vfm::mc::smv;

vfm::mc::smv::ModuleElementBase::ModuleElementBase(const std::string& name) : Failable(name)
{
}

std::string vfm::mc::smv::ModuleElementBase::getModuleElementName() const
{
   return getFailableName();
}

std::string vfm::mc::smv::ModuleElementBase::getRawCode() const
{
   return raw_code_;
}

void vfm::mc::smv::ModuleElementBase::setCode(const std::string& to)
{
   raw_code_ = to;
}

void vfm::mc::smv::ModuleElementBase::appendCodeLine(const std::string& line)
{
   raw_code_ += "   " + line + "\n";
}

std::string vfm::mc::smv::ModuleElementBase::serialize() const
{
   if (getRawCode().empty()) {
      return "";
   }

   return getModuleElementName() + "\n" + getRawCode() + "\n";
}
