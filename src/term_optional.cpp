//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file

#include "term_optional.h"
#include "parser.h"
#include "string"
#include <sstream>
#include <exception>
#include <limits>

using namespace vfm;

const OperatorStructure TermOptional::my_struct(OutputFormatEnum::prefix, AssociativityTypeEnum::left, OPNUM_OPTIONAL, PRIO_OPTIONAL, SYMB_OPTIONAL, false, false);

OperatorStructure TermOptional::getOpStruct() const
{
    return my_struct;
}

float TermOptional::eval(const std::shared_ptr<DataPack>& varVals, const std::shared_ptr<FormulaParser> parser, const bool suppress_sideeffects)
{
#if defined(ASMJIT_ENABLED)
   if (assembly_created_ == AssemblyState::yes) return (*fast_eval_func_)();
#endif

   return -std::numeric_limits<float>::infinity();
}

std::shared_ptr<Term> TermOptional::copy(const std::shared_ptr<std::map<std::shared_ptr<Term>, std::shared_ptr<Term>>> already_copied) {
   std::shared_ptr<Term> ptr(new TermOptional(getOperands()[0]->copy(already_copied), getOperands()[1]->copy(already_copied)));
   ptr->setChildrensFathers(false, false);
   return ptr;
}

#if defined(ASMJIT_ENABLED)
std::shared_ptr<x86::Xmm> TermOptional::createSubAssembly(asmjit::x86::Compiler& cc, const std::shared_ptr<DataPack>& d, const std::shared_ptr<FormulaParser>& p)
{
   return nullptr;
}
#endif

bool vfm::TermOptional::isTermOptional() const
{
   return true;
}

std::shared_ptr<TermOptional> vfm::TermOptional::toTermOptionalIfApplicable()
{
   return std::dynamic_pointer_cast<TermOptional>(this->shared_from_this());
}

TermOptional::TermOptional(const std::shared_ptr<Term>& meta, const std::shared_ptr<Term>& replacement_value) : Term(std::vector<std::shared_ptr<Term>>{meta, replacement_value}) {}