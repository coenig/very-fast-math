//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file

#include "term_pow.h"
#include "term_meta_compound.h"
#include "term_minus.h"
#include "term_neg.h"
#include "term_val.h"
#include "meta_rule.h"
#include <functional>
#include <vector>

using namespace vfm;

const OperatorStructure TermPow::my_struct(OutputFormatEnum::infix, AssociativityTypeEnum::left, OPNUM_POW, PRIO_POW, SYMB_POW, false, false);

OperatorStructure TermPow::getOpStruct() const
{
   return my_struct;
}

std::shared_ptr<Term> TermPow::getInvPattern(int argNum)
{
   return nullptr;
}

float TermPow::eval(const std::shared_ptr<DataPack>& varVals, const std::shared_ptr<FormulaParser> parser, const bool suppress_sideeffects)
{
#if defined(ASMJIT_ENABLED)
   if (assembly_created_ == AssemblyState::yes) return (*fast_eval_func_)();
#endif

   return std::pow(getOperands()[0]->eval(varVals, parser, suppress_sideeffects), getOperands()[1]->eval(varVals, parser, suppress_sideeffects));
}

std::set<MetaRule> TermPow::getSimplificationRules(const std::shared_ptr<FormulaParser> parser) const
{
   std::set<MetaRule> vec = MathStruct::getSimplificationRules(parser);
   return vec;
}

// SLOW
//float ipow(float x, float n)
//{
//   long int result = 1;
//   while (n > 0) {
//      result *= x;
//      n -= 1;
//   }
//
//   return(result);
//}

// SLOW
//ipow(float, float) :
//   push rbp
//   mov rbp, rsp
//   movss DWORD PTR[rbp - 20], xmm0
//   movss DWORD PTR[rbp - 24], xmm1
//   mov QWORD PTR[rbp - 8], 1
//   .L4:
//movss xmm0, DWORD PTR[rbp - 24]
//pxor xmm1, xmm1
//comiss xmm0, xmm1
//jbe.L7
//cvtsi2ss xmm0, QWORD PTR[rbp - 8]
//mulss xmm0, DWORD PTR[rbp - 20]
//cvttss2si rax, xmm0
//mov QWORD PTR[rbp - 8], rax
//movss xmm0, DWORD PTR[rbp - 24]
//movss xmm1, DWORD PTR.LC1[rip]
//subss xmm0, xmm1
//movss DWORD PTR[rbp - 24], xmm0
//jmp.L4
//.L7:
//cvtsi2ss xmm0, QWORD PTR[rbp - 8]
//pop rbp
//ret
//.LC1 :
//   .long 1065353216

// FAST
//ipow(float, float):
//  push rbp
//  mov rbp, rsp
//  movss DWORD PTR [rbp-20], xmm0
//  movss DWORD PTR [rbp-24], xmm1
//  movss xmm0, DWORD PTR [rbp-24]
//  cvttss2si rax, xmm0
//  mov DWORD PTR [rbp-4], eax
//  mov DWORD PTR [rbp-8], 1
//.L4:
//  cmp DWORD PTR [rbp-4], 0
//  je .L2
//  mov eax, DWORD PTR [rbp-4]
//  and eax, 1
//  test eax, eax
//  je .L3
//  cvtsi2ss xmm0, DWORD PTR [rbp-8]
//  mulss xmm0, DWORD PTR [rbp-20]
//  cvttss2si eax, xmm0
//  mov DWORD PTR [rbp-8], eax
//.L3:
//  shr DWORD PTR [rbp-4]
//  movss xmm0, DWORD PTR [rbp-20]
//  mulss xmm0, xmm0
//  movss DWORD PTR [rbp-20], xmm0
//  jmp .L4
//.L2:
//  cvtsi2ss xmm0, DWORD PTR [rbp-8]
//  pop rbp
//  ret

// FAST
//float ipow(float x, float nn)
//{
//   unsigned int n = nn;
//   int result = 1;
//
//   while (n)
//   {
//      if (n & 1) result *= x;
//      n >>= 1;
//      x *= x;
//   }
//
//   return result;
//}

#if defined(ASMJIT_ENABLED)
std::shared_ptr<x86::Xmm> TermPow::createSubAssembly(asmjit::x86::Compiler& cc, const std::shared_ptr<DataPack>& d, const std::shared_ptr<FormulaParser>& p)
{
   return nullptr;
}
#endif

TermPow::TermPow(const std::vector<std::shared_ptr<Term>>& opds) : TermArith(opds)
{
}

TermPow::TermPow(const std::shared_ptr<Term> opd1, const std::shared_ptr<Term> opd2) : TermPow(std::vector<std::shared_ptr<Term>>{opd1, opd2})
{
}

std::shared_ptr<Term> TermPow::copy(const std::shared_ptr<std::map<std::shared_ptr<Term>, std::shared_ptr<Term>>> already_copied) 
{
   return copySimple<TermPow>(already_copied);
}

void vfm::TermPow::serialize(
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
      auto exp = getOperandsConst()[1]->constEval(nullptr, true);

      if (!getOperandsConst()[1]->isOverallConstant()) {
         Failable::getSingleton()->addError("Power operator '" + getOptor() + "' has non-constant exponent '" + getOperandsConst()[1]->serialize() + "'. Cannot translate to nusmv.");
         MathStruct::serialize(os, highlight, style, special, indent, indent_step, line_len, data, visited);
         return;
      }
      else if (exp != (int) exp) {
         Failable::getSingleton()->addWarning("Power operator '" + getOptor() + "' has non-integer exponent '" + std::to_string(exp) + "' which makes translation to nusmv inaccurate.");
      }
      else if (exp < 0) {
         Failable::getSingleton()->addWarning("Power operator '" + getOptor() + "' has negative exponent '" + std::to_string(exp) + "' which makes translation to nusmv inaccurate.");
      }

      auto base = getOperandsConst()[0]->serialize(highlight, style, special, indent, indent_step, line_len, data);

      os << OPENING_BRACKET;
      os << OPENING_BRACKET << base << CLOSING_BRACKET;
      for (int i = 1; i < (int) exp; i++) {
         os << " " << SYMB_MULT << " " << OPENING_BRACKET << base << CLOSING_BRACKET;
      }
      os << CLOSING_BRACKET;
   } 
   else {
      MathStruct::serialize(os, highlight, style, special, indent, indent_step, line_len, data, visited);
   }

   visited.insert(shared_from_this());
}
