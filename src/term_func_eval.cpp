//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file

#include "term_func_eval.h"
#include "parser.h"

using namespace vfm;

const OperatorStructure TermFuncEval::my_struct(OutputFormatEnum::prefix, AssociativityTypeEnum::left, OPNUM_FUNC_EVAL, PRIO_FUNC_EVAL, SYMB_FUNC_EVAL, false, false);

TermFuncEval::TermFuncEval(const std::vector<std::shared_ptr<Term>>& t) : Term(t) {}

OperatorStructure TermFuncEval::getOpStruct() const
{
   return my_struct;
}

float TermFuncEval::eval(const std::shared_ptr<DataPack>& varVals, const std::shared_ptr<FormulaParser> parser, const bool suppress_sideeffects)
{
#if defined(ASMJIT_ENABLED)
   if (assembly_created_ == AssemblyState::yes) return (*fast_eval_func_)();
#endif

   int func_address = getOperands()[0]->eval(varVals, parser, suppress_sideeffects);

   if (!parser->isFunctionDeclared(func_address)) {
      return -1;
   }

   std::vector<std::shared_ptr<Term>>::const_iterator first = getOperands().begin() + 1;
   std::vector<std::shared_ptr<Term>>::const_iterator last = getOperands().end();
   std::vector<std::shared_ptr<Term>> arguments{ first, last };

   auto function = parser->getFunctionName(func_address);

   if (function.second != arguments.size()) {
      parser->addError("Function at address " + std::to_string(func_address) + " (named '" + function.first + "') expects " + std::to_string(function.second) + " parameter(s), but received " + std::to_string(arguments.size()) + ". '" + getOptor() + "' operation failed.");
      parser->addError("These are the declared functions:\n");
      for (const auto& el : parser->getAllDeclaredFunctions()) {
         parser->addErrorPlain(std::to_string(el.first) + ": " + el.second.first + " (" + std::to_string(el.second.second) + ")");
      }
      return -12345;
   }

   auto act_func = parser->termFactory(function.first, arguments);

   return act_func->eval(varVals, parser, suppress_sideeffects);
}

#if defined(ASMJIT_ENABLED)
std::shared_ptr<x86::Xmm> TermFuncEval::createSubAssembly(asmjit::x86::Compiler & cc, const std::shared_ptr<DataPack>& d, const std::shared_ptr<FormulaParser>& p)
{
   return nullptr;
}
#endif

std::shared_ptr<Term> TermFuncEval::copy(const std::shared_ptr<std::map<std::shared_ptr<Term>, std::shared_ptr<Term>>> already_copied) {
   return copySimple<TermFuncEval>(already_copied);
}
