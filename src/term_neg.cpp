//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file

#include "term_neg.h"
#include "meta_rule.h"
#include "parser.h"

using namespace vfm;

const OperatorStructure TermNeg::my_struct(OutputFormatEnum::prefix, AssociativityTypeEnum::left, OPNUM_NEG, PRIO_NEG, SYMB_NEG, true, true);

TermNeg::TermNeg(const std::vector<std::shared_ptr<Term>>& t) : TermArith(t) {}
TermNeg::TermNeg(const std::shared_ptr<Term>& t) : TermNeg(std::vector<std::shared_ptr<Term>>{t}) {}

std::shared_ptr<Term> TermNeg::getInvPattern(int argNum)
{
   return MathStruct::parseMathStruct("--(p_(0))", false)->toTermIfApplicable();
}

OperatorStructure TermNeg::getOpStruct() const
{
   return my_struct;
}

float TermNeg::eval(const std::shared_ptr<DataPack>& varVals, const std::shared_ptr<FormulaParser> parser, const bool suppress_sideeffects)
{
#if defined(ASMJIT_ENABLED)
   if (assembly_created_ == AssemblyState::yes) return (*fast_eval_func_)();
#endif

   float subresult = getOperands()[0]->eval(varVals, parser, suppress_sideeffects);
   if (subresult == 0) return subresult;
   return -subresult;
}

#if defined(ASMJIT_ENABLED)
std::shared_ptr<x86::Xmm> TermNeg::createSubAssembly(asmjit::x86::Compiler & cc, const std::shared_ptr<DataPack>& d, const std::shared_ptr<FormulaParser>& p)
{
   lambda_so lambda = [](asmjit::x86::Compiler& cc, x86::Xmm& x) {
      x86::Xmm y = cc.newXmm();
      cc.mov(asmjit::x86::eax, 0x80000000);
      cc.movd(y, asmjit::x86::eax);
      cc.xorps(x, y);
   };

   return subAssemblySingleOp(cc, d, lambda);
}
#endif

std::shared_ptr<Term> TermNeg::copy(const std::shared_ptr<std::map<std::shared_ptr<Term>, std::shared_ptr<Term>>> already_copied) {
   return copySimple<TermNeg>(already_copied);
}

std::set<MetaRule> vfm::TermNeg::getSimplificationRules(const std::shared_ptr<FormulaParser> parser) const
{
   std::set<MetaRule> vec = MathStruct::getSimplificationRules(parser);

   vec.insert(MetaRule(_neg(_neg(_vara())), _vara(), SIMPLIFICATION_STAGE_MAIN));                                                                            // "--(--(a))", "a"
   vec.insert(MetaRule(_neg(_mult(_vara(), _varb())), _mult(_neg(_vara()), _varb()), SIMPLIFICATION_STAGE_MAIN, { {_vara(), ConditionType::is_constant} })); // "--(a * b)", "--(a) * b"

   return vec;
}

std::string TermNeg::getNuSMVOperator() const
{
   return "-";
}

void TermNeg::serialize(
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
      if (special == SerializationSpecial::enforce_postfix) {
         os << getOperandsConst()[0]->serialize(highlight, style, special, indent, indent_step, line_len, data) << getNuSMVOperator() << POSTFIX_DELIMITER;
      } else {
         os << getNuSMVOperator() << getOperandsConst()[0]->serialize(highlight, style, special, indent, indent_step, line_len, data);
      }
   } else {
      MathStruct::serialize(os, highlight, style, special, indent, indent_step, line_len, data, visited);
   }

   visited.insert(shared_from_this());
}

std::string vfm::TermNeg::getK2Operator() const
{
   return "neg";
}