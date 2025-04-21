//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file

#include "term_ln.h"
#include "meta_rule.h"

using namespace vfm;

const OperatorStructure TermSin::my_struct(OutputFormatEnum::prefix, AssociativityTypeEnum::left, OPNUM_SIN, PRIO_SIN, SYMB_SIN, false, false);

OperatorStructure TermSin::getOpStruct() const
{
   return my_struct;
}

std::shared_ptr<Term> TermSin::getInvPattern(int argNum)
{
   return nullptr;
}

float TermSin::eval(const std::shared_ptr<DataPack>& varVals, const std::shared_ptr<FormulaParser> parser, const bool suppress_sideeffects)
{
#if defined(ASMJIT_ENABLED)
   if (assembly_created_ == AssemblyState::yes) return (*fast_eval_func_)();
#endif

   return std::log(getOperands()[0]->eval(varVals, parser, suppress_sideeffects));
}

std::set<MetaRule> TermSin::getSimplificationRules(const std::shared_ptr<FormulaParser> parser) const
{
   std::set<MetaRule> vec = MathStruct::getSimplificationRules(parser);
   return vec;
}

#if defined(ASMJIT_ENABLED)
std::shared_ptr<x86::Xmm> TermSin::createSubAssembly(asmjit::x86::Compiler& cc, const std::shared_ptr<DataPack>& d, const std::shared_ptr<FormulaParser>& p)
{
   return nullptr;
}
#endif

TermSin::TermLn(const std::vector<std::shared_ptr<Term>>& opds) : TermArith(opds)
{
}

TermSin::TermLn(const std::shared_ptr<Term> opd1) : TermLn(std::vector<std::shared_ptr<Term>>{opd1})
{
}

std::shared_ptr<Term> TermSin::copy(const std::shared_ptr<std::map<std::shared_ptr<Term>, std::shared_ptr<Term>>> already_copied)
{
   return copySimple<TermLn>(already_copied);
}

void vfm::TermSin::serialize(
   std::stringstream& os, 
   const std::shared_ptr<MathStruct> highlight, 
   const SerializationStyle style,
   const SerializationSpecial special,
   const int indent,
   const int indent_step, 
   const int line_len,
   const std::shared_ptr<DataPack> data,
   std::set<std::shared_ptr<const MathStruct>>& visited) const
{
   if (style == SerializationStyle::nusmv) {
      Failable::getSingleton()->addError("ln operator '" + getOptor() + "' cannot be translated to nusmv.");
      os << "#Serialization-to-nusmv-not-possible-for-LN-operator";
   } 
   else {
      MathStruct::serialize(os, highlight, style, special, indent, indent_step, line_len, data, visited);
   }

   visited.insert(shared_from_this());
}
