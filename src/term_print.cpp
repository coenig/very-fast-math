//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file

#include "term_print.h"
#include "parser.h"
#include "meta_rule.h"

using namespace vfm;

const OperatorStructure TermPrint::my_struct(OutputFormatEnum::prefix, AssociativityTypeEnum::left, OPNUM_PRINT, PRIO_PRINT, SYMB_PRINT, false, false);

OperatorStructure TermPrint::getOpStruct() const
{
   return my_struct;
}

float TermPrint::eval(const std::shared_ptr<DataPack>& varVals, const std::shared_ptr<FormulaParser> parser, const bool suppress_sideeffects)
{
#if defined(ASMJIT_ENABLED)
   if (assembly_created_ == AssemblyState::yes) return (*fast_eval_func_)();
#endif

   float beg = getOperands()[0]->eval(varVals, parser, suppress_sideeffects);
   int len = getOperands()[1]->eval(varVals, parser, suppress_sideeffects);

   if (len >= 0) {
      for (int i = beg; i < beg + len; i++) {
         auto val = varVals->getVfmMemory();
         std::cout << (char) val.at(i);
      }
   }
   else {
      std::cout << beg;
   }

   return beg;
}

#if defined(ASMJIT_ENABLED)
std::shared_ptr<x86::Xmm> TermPrint::createSubAssembly(asmjit::x86::Compiler& cc, const std::shared_ptr<DataPack>& d, const std::shared_ptr<FormulaParser>& p)
{
   return nullptr;
}
#endif

TermPrint::TermPrint(const std::vector<std::shared_ptr<Term>>& opds) : Term(opds)
{
}

vfm::TermPrint::TermPrint(const std::shared_ptr<Term> opd1, const std::shared_ptr<Term> opd2) : TermPrint(std::vector<std::shared_ptr<Term>>{opd1, opd2})
{
}

std::shared_ptr<Term> TermPrint::copy(const std::shared_ptr<std::map<std::shared_ptr<Term>, std::shared_ptr<Term>>> already_copied) {
   return copySimple<TermPrint>(already_copied);
}

std::set<MetaRule> vfm::TermPrint::getSimplificationRules(const std::shared_ptr<FormulaParser> parser) const
{
   std::set<MetaRule> vec = MathStruct::getSimplificationRules(parser);

   vec.insert(MetaRule(_print(_vara(), _varb()), _val0(), SIMPLIFICATION_STAGE_MAIN, { {_vara(), ConditionType::has_no_sideeffects }, {_varb(), ConditionType::is_constant_0 } })); // Just return 0 on empty string.

   return vec;
}

bool vfm::TermPrint::isTermPrint() const
{
   return true;
}
