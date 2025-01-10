//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file

#include "term_fctin.h"

using namespace vfm;

const OperatorStructure TermFctIn::my_struct(OutputFormatEnum::prefix, AssociativityTypeEnum::left, OPNUM_FCTIN, PRIO_FCTIN, SYMB_FCTIN, true, true);

TermFctIn::TermFctIn() : Term({}) {}

OperatorStructure TermFctIn::getOpStruct() const
{
   return my_struct;
}

float TermFctIn::eval(const std::shared_ptr<DataPack>& varVals, const std::shared_ptr<FormulaParser> parser, const bool suppress_sideeffects)
{
#if defined(ASMJIT_ENABLED)
   if (assembly_created_ == AssemblyState::yes) return (*fast_eval_func_)();
#endif

   std::string val;
   std::getline(std::cin, val);

   if (val.empty()) {
      val = "0";
   }

   return MathStruct::parseMathStruct(val, parser, varVals)->eval(varVals, parser, suppress_sideeffects);
}

#if defined(ASMJIT_ENABLED)
std::shared_ptr<x86::Xmm> TermFctIn::createSubAssembly(asmjit::x86::Compiler& cc, const std::shared_ptr<DataPack>& d, const std::shared_ptr<FormulaParser>& p)
{
   return nullptr;
}
#endif

std::shared_ptr<Term> TermFctIn::copy(const std::shared_ptr<std::map<std::shared_ptr<Term>, std::shared_ptr<Term>>> already_copied) {
   return std::make_shared<TermFctIn>();
}
