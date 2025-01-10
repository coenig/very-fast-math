//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file

#include "term_func_ref.h"
#include "term_var.h"
#include "parser.h"
#include "static_helper.h"

using namespace vfm;

const OperatorStructure TermFuncRef::my_struct(OutputFormatEnum::prefix, AssociativityTypeEnum::left, OPNUM_FUNC_REF, PRIO_FUNC_REF, SYMB_FUNC_REF, false, false);

TermFuncRef::TermFuncRef(const std::vector<std::shared_ptr<Term>>& t) : Term(t) {}
TermFuncRef::TermFuncRef(const std::shared_ptr<Term> t) : TermFuncRef(std::vector<std::shared_ptr<Term>>{t}) {}
TermFuncRef::TermFuncRef(const std::shared_ptr<Term> t1, const std::shared_ptr<Term> t2) : TermFuncRef(std::vector<std::shared_ptr<Term>>{ t1, t2 }) {}

OperatorStructure TermFuncRef::getOpStruct() const
{
   return my_struct;
}

float TermFuncRef::eval(const std::shared_ptr<DataPack>& varVals, const std::shared_ptr<FormulaParser> parser, const bool suppress_sideeffects)
{
#if defined(ASMJIT_ENABLED)
   if (assembly_created_ == AssemblyState::yes) return (*fast_eval_func_)();
#endif
   std::string func_name = getOperands()[0]->getOptor();

   int auto_num = getOperands().size() > 2
      ? getOperands().size() - 2 // Function definition with explicit parameter definition: @f(f, x, y, x + y)
      : -1;

   if (auto_num < 0 && getOperands().size() == 2) { // Function definition with implicit parameter definition in body: @f(f, p_(0) + p_(1))
      auto_num = StaticHelper::deriveParNum(getOperands()[1]);
   }

   int num_params_explicit = auto_num;
   int actual_num_params = -1;

   if (StaticHelper::stringContains(func_name, PROGRAM_OP_STRUCT_DELIMITER)) { // func_name = "2:.:-" means 2-operand minus operator.
      bool dummy;
      OperatorStructure op_struct(func_name, dummy);
      num_params_explicit = op_struct.arg_num;
      func_name = op_struct.op_name;
   }

   auto pparser = parser;

   if (!pparser) {
      pparser = SingletonFormulaParser::getInstance();
   }

   std::set<int> all_possible_par_nums = pparser->getNumParams(func_name);

   if (all_possible_par_nums.size() == 1) { // There is exactly one function with the given name (no overloading).
      actual_num_params = *all_possible_par_nums.begin();
   }
   else {
      actual_num_params = num_params_explicit;
   }

   if (pparser->isFunctionDefined(func_name, actual_num_params)) {
      assert(actual_num_params >= 0);
      assert(num_params_explicit < 0 || actual_num_params == num_params_explicit);

      return pparser->getFunctionAddressOf(func_name, actual_num_params);
   }

   return -1;
}

#if defined(ASMJIT_ENABLED)
std::shared_ptr<x86::Xmm> TermFuncRef::createSubAssembly(asmjit::x86::Compiler& cc, const std::shared_ptr<DataPack>& d, const std::shared_ptr<FormulaParser>& p)
{
   return nullptr;
}
#endif

std::shared_ptr<Term> TermFuncRef::copy(const std::shared_ptr<std::map<std::shared_ptr<Term>, std::shared_ptr<Term>>> already_copied) {
   std::shared_ptr<Term> ptr(new TermFuncRef(getOperands()[0]->copy(already_copied)));
   ptr->setChildrensFathers(false, false);
   return ptr;
}
