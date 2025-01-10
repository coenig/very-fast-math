//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file

#include "term_func_lambda.h"
#include "parser.h"

using namespace vfm;

const OperatorStructure TermFuncLambda::my_struct(OutputFormatEnum::prefix, AssociativityTypeEnum::left, OPNUM_LAMBDA, PRIO_LAMBDA, SYMB_LAMBDA, false, false);

TermFuncLambda::TermFuncLambda(const std::vector<std::shared_ptr<Term>>& t) : Term(t) {}
TermFuncLambda::TermFuncLambda(const std::shared_ptr<Term> t) : TermFuncLambda(std::vector<std::shared_ptr<Term>>{t}) {}

OperatorStructure TermFuncLambda::getOpStruct() const
{
   return my_struct;
}

float TermFuncLambda::eval(const std::shared_ptr<DataPack>& varVals, const std::shared_ptr<FormulaParser> parser, const bool suppress_sideeffects)
{
#if defined(ASMJIT_ENABLED)
   if (assembly_created_ == AssemblyState::yes) return (*fast_eval_func_)();
#endif

   return parser->addLambdaTerm(getOperands()[0]);
}

#if defined(ASMJIT_ENABLED)
std::shared_ptr<x86::Xmm> TermFuncLambda::createSubAssembly(asmjit::x86::Compiler & cc, const std::shared_ptr<DataPack>& d, const std::shared_ptr<FormulaParser>& p)
{
   return nullptr;
}
#endif

std::shared_ptr<Term> TermFuncLambda::copy(const std::shared_ptr<std::map<std::shared_ptr<Term>, std::shared_ptr<Term>>> already_copied) {
   std::shared_ptr<Term> ptr(new TermFuncLambda(this->getOperands()[0]->copy(already_copied)));
   ptr->setChildrensFathers(false, false);
   return ptr;
}

bool TermFuncLambda::isTermLambda() const
{
   return true;
}