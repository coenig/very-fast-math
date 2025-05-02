//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file

#include "terms_trigonometry.h"

using namespace vfm;

// SIN
const OperatorStructure TermSin::my_struct(OutputFormatEnum::prefix, AssociativityTypeEnum::left, OPNUM_SIN, PRIO_SIN, SYMB_SIN, false, false);

TermSin::TermSin(const std::vector<std::shared_ptr<Term>>& opds) : TermArith(opds) {}
TermSin::TermSin(const std::shared_ptr<Term> opd1) : TermSin(std::vector<std::shared_ptr<Term>>{opd1}) {}
OperatorStructure TermSin::getOpStruct() const { return my_struct; }
std::shared_ptr<Term> TermSin::getInvPattern(int argNum) { return nullptr; }
std::shared_ptr<Term> TermSin::copy(const std::shared_ptr<std::map<std::shared_ptr<Term>, std::shared_ptr<Term>>> already_copied) { return copySimple<TermSin>(already_copied); }

float TermSin::eval(const std::shared_ptr<DataPack>& varVals, const std::shared_ptr<FormulaParser> parser, const bool suppress_sideeffects)
{
#if defined(ASMJIT_ENABLED)
   if (assembly_created_ == AssemblyState::yes) return (*fast_eval_func_)();
#endif

   return std::sin(getOperands()[0]->eval(varVals, parser, suppress_sideeffects));
}

#if defined(ASMJIT_ENABLED)
std::shared_ptr<x86::Xmm> TermSin::createSubAssembly(asmjit::x86::Compiler& cc, const std::shared_ptr<DataPack>& d, const std::shared_ptr<FormulaParser>& p)
{
   return nullptr;
}
#endif

// COS
const OperatorStructure TermCos::my_struct(OutputFormatEnum::prefix, AssociativityTypeEnum::left, OPNUM_COS, PRIO_COS, SYMB_COS, false, false);

TermCos::TermCos(const std::vector<std::shared_ptr<Term>>& opds) : TermArith(opds) {}
TermCos::TermCos(const std::shared_ptr<Term> opd1) : TermCos(std::vector<std::shared_ptr<Term>>{opd1}) {}
OperatorStructure TermCos::getOpStruct() const { return my_struct; }
std::shared_ptr<Term> TermCos::getInvPattern(int argNum) { return nullptr; }
std::shared_ptr<Term> TermCos::copy(const std::shared_ptr<std::map<std::shared_ptr<Term>, std::shared_ptr<Term>>> already_copied) { return copySimple<TermCos>(already_copied); }

float TermCos::eval(const std::shared_ptr<DataPack>& varVals, const std::shared_ptr<FormulaParser> parser, const bool suppress_sideeffects)
{
#if defined(ASMJIT_ENABLED)
   if (assembly_created_ == AssemblyState::yes) return (*fast_eval_func_)();
#endif

   return std::cos(getOperands()[0]->eval(varVals, parser, suppress_sideeffects));
}

#if defined(ASMJIT_ENABLED)
std::shared_ptr<x86::Xmm> TermCos::createSubAssembly(asmjit::x86::Compiler& cc, const std::shared_ptr<DataPack>& d, const std::shared_ptr<FormulaParser>& p)
{
   return nullptr;
}
#endif

// TAN
const OperatorStructure TermTan::my_struct(OutputFormatEnum::prefix, AssociativityTypeEnum::left, OPNUM_TAN, PRIO_TAN, SYMB_TAN, false, false);

TermTan::TermTan(const std::vector<std::shared_ptr<Term>>& opds) : TermArith(opds) {}
TermTan::TermTan(const std::shared_ptr<Term> opd1) : TermTan(std::vector<std::shared_ptr<Term>>{opd1}) {}
OperatorStructure TermTan::getOpStruct() const { return my_struct; }
std::shared_ptr<Term> TermTan::getInvPattern(int argNum) { return nullptr; }
std::shared_ptr<Term> TermTan::copy(const std::shared_ptr<std::map<std::shared_ptr<Term>, std::shared_ptr<Term>>> already_copied) { return copySimple<TermTan>(already_copied); }

float TermTan::eval(const std::shared_ptr<DataPack>& varVals, const std::shared_ptr<FormulaParser> parser, const bool suppress_sideeffects)
{
#if defined(ASMJIT_ENABLED)
   if (assembly_created_ == AssemblyState::yes) return (*fast_eval_func_)();
#endif

   return std::tan(getOperands()[0]->eval(varVals, parser, suppress_sideeffects));
}

#if defined(ASMJIT_ENABLED)
std::shared_ptr<x86::Xmm> TermTan::createSubAssembly(asmjit::x86::Compiler& cc, const std::shared_ptr<DataPack>& d, const std::shared_ptr<FormulaParser>& p)
{
   return nullptr;
}
#endif

// ASIN
const OperatorStructure TermASin::my_struct(OutputFormatEnum::prefix, AssociativityTypeEnum::left, OPNUM_ARCSIN, PRIO_ARCSIN, SYMB_ARCSIN, false, false);

TermASin::TermASin(const std::vector<std::shared_ptr<Term>>& opds) : TermArith(opds) {}
TermASin::TermASin(const std::shared_ptr<Term> opd1) : TermASin(std::vector<std::shared_ptr<Term>>{opd1}) {}
OperatorStructure TermASin::getOpStruct() const { return my_struct; }
std::shared_ptr<Term> TermASin::getInvPattern(int argNum) { return nullptr; }
std::shared_ptr<Term> TermASin::copy(const std::shared_ptr<std::map<std::shared_ptr<Term>, std::shared_ptr<Term>>> already_copied) { return copySimple<TermASin>(already_copied); }

float TermASin::eval(const std::shared_ptr<DataPack>& varVals, const std::shared_ptr<FormulaParser> parser, const bool suppress_sideeffects)
{
#if defined(ASMJIT_ENABLED)
   if (assembly_created_ == AssemblyState::yes) return (*fast_eval_func_)();
#endif

   return std::asin(getOperands()[0]->eval(varVals, parser, suppress_sideeffects));
}

#if defined(ASMJIT_ENABLED)
std::shared_ptr<x86::Xmm> TermASin::createSubAssembly(asmjit::x86::Compiler& cc, const std::shared_ptr<DataPack>& d, const std::shared_ptr<FormulaParser>& p)
{
   return nullptr;
}
#endif

// ACOS
const OperatorStructure TermACos::my_struct(OutputFormatEnum::prefix, AssociativityTypeEnum::left, OPNUM_ARCCOS, PRIO_ARCCOS, SYMB_ARCCOS, false, false);

TermACos::TermACos(const std::vector<std::shared_ptr<Term>>& opds) : TermArith(opds) {}
TermACos::TermACos(const std::shared_ptr<Term> opd1) : TermACos(std::vector<std::shared_ptr<Term>>{opd1}) {}
OperatorStructure TermACos::getOpStruct() const { return my_struct; }
std::shared_ptr<Term> TermACos::getInvPattern(int argNum) { return nullptr; }
std::shared_ptr<Term> TermACos::copy(const std::shared_ptr<std::map<std::shared_ptr<Term>, std::shared_ptr<Term>>> already_copied) { return copySimple<TermACos>(already_copied); }

float TermACos::eval(const std::shared_ptr<DataPack>& varVals, const std::shared_ptr<FormulaParser> parser, const bool suppress_sideeffects)
{
#if defined(ASMJIT_ENABLED)
   if (assembly_created_ == AssemblyState::yes) return (*fast_eval_func_)();
#endif

   return std::acos(getOperands()[0]->eval(varVals, parser, suppress_sideeffects));
}

#if defined(ASMJIT_ENABLED)
std::shared_ptr<x86::Xmm> TermACos::createSubAssembly(asmjit::x86::Compiler& cc, const std::shared_ptr<DataPack>& d, const std::shared_ptr<FormulaParser>& p)
{
   return nullptr;
}
#endif

// ATAN
const OperatorStructure TermATan::my_struct(OutputFormatEnum::prefix, AssociativityTypeEnum::left, OPNUM_ARCTAN, PRIO_ARCTAN, SYMB_ARCTAN, false, false);

TermATan::TermATan(const std::vector<std::shared_ptr<Term>>& opds) : TermArith(opds) {}
TermATan::TermATan(const std::shared_ptr<Term> opd1) : TermATan(std::vector<std::shared_ptr<Term>>{opd1}) {}
OperatorStructure TermATan::getOpStruct() const { return my_struct; }
std::shared_ptr<Term> TermATan::getInvPattern(int argNum) { return nullptr; }
std::shared_ptr<Term> TermATan::copy(const std::shared_ptr<std::map<std::shared_ptr<Term>, std::shared_ptr<Term>>> already_copied) { return copySimple<TermATan>(already_copied); }

float TermATan::eval(const std::shared_ptr<DataPack>& varVals, const std::shared_ptr<FormulaParser> parser, const bool suppress_sideeffects)
{
#if defined(ASMJIT_ENABLED)
   if (assembly_created_ == AssemblyState::yes) return (*fast_eval_func_)();
#endif

   return std::atan(getOperands()[0]->eval(varVals, parser, suppress_sideeffects));
}

#if defined(ASMJIT_ENABLED)
std::shared_ptr<x86::Xmm> TermATan::createSubAssembly(asmjit::x86::Compiler& cc, const std::shared_ptr<DataPack>& d, const std::shared_ptr<FormulaParser>& p)
{
   return nullptr;
}
#endif
