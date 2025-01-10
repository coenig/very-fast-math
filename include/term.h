//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file
#pragma once

#include "data_pack.h"
#include "math_struct.h"
#include <vector>
#include <string>
#include <map>
#include <functional>
#include <cmath>
#include <iostream>
#include <memory>


namespace vfm {

class DataPack;
class MathStruct;

using TermPtr = std::shared_ptr<Term>;

#if defined(ASMJIT_ENABLED)
using namespace asmjit;

typedef std::function<void(asmjit::x86::Compiler& cc, x86::Xmm& xx, x86::Xmm& yy)> lambda_vno; // var_num_ops
typedef std::function<void(asmjit::x86::Compiler& cc, x86::Xmm& xx)> lambda_so;                      // single_op
#endif

class Term : public MathStruct
{
public:
   Term(const std::vector<std::shared_ptr<Term>>& opds);

   template<class Func>
   float evalSimple(
        const std::shared_ptr<DataPack>& varVals, 
        const Func& f, 
        float initVal, 
        int startInd,
        const std::shared_ptr<FormulaParser> parser = nullptr,
        const bool suppress_side_effects = false,
        const bool invert_if_any_operand_is_nan = false,
        const bool invert_only_if_another_operand_is_finite_and_true = false,
        const bool invert_only_if_another_operand_is_finite_and_false = false);

   template<class T>
   std::shared_ptr<T> copySimple(const std::shared_ptr<std::map<std::shared_ptr<Term>, std::shared_ptr<Term>>> already_copied);

   float constEval(const std::shared_ptr<FormulaParser> parser = nullptr, const bool suppress_sideeffects = false) override;

   std::shared_ptr<Term> toTermIfApplicable() override;
   virtual std::shared_ptr<Term> copy(const std::shared_ptr<std::map<std::shared_ptr<Term>, std::shared_ptr<Term>>> already_copied = nullptr) = 0;
   virtual bool resolveTo(const std::function<bool(std::shared_ptr<MathStruct>)>& match_cond, bool from_left) override;
   virtual std::shared_ptr<Term> getInvPattern(int argNum);

#if defined(ASMJIT_ENABLED)
   std::shared_ptr<x86::Xmm> subAssemblyVarnumOps(asmjit::x86::Compiler& cc, const std::shared_ptr<DataPack>& d, lambda_vno& fct);
   std::shared_ptr<x86::Xmm> subAssemblySingleOp(asmjit::x86::Compiler& cc, const std::shared_ptr<DataPack>& d, lambda_so& fct, const int index_of_single_op = 0);
   static void setXmmVar(asmjit::x86::Compiler& cc, x86::Xmm& x, float);
   static void setXmmVar(asmjit::x86::Compiler& cc, x86::Xmm& x, x86::Xmm& y);
   static void setXmmVarToAddressLocation(asmjit::x86::Compiler& cc, x86::Xmm& x, const bool* address);
   static void setXmmVarToAddressLocation(asmjit::x86::Compiler& cc, x86::Xmm& x, const char* address);
   static void setXmmVarToAddressLocation(asmjit::x86::Compiler& cc, x86::Xmm& x, const char* address, x86::Xmm& y, const int& factor = 1);
   static void setXmmVarToAddressLocation(asmjit::x86::Compiler& cc, x86::Xmm& x, const float* address, const int& index = 0);
   static void setXmmVarToAddressLocation(asmjit::x86::Compiler& cc, x86::Xmm& x, const float* address, x86::Xmm& y);
   static void setXmmVarToValueAtAddressFromArray(asmjit::x86::Compiler& cc, x86::Xmm& x, float** ref_array, x86::Xmm& index);
#endif
};

template<class Func>
float Term::evalSimple(
     const std::shared_ptr<DataPack>& varVals,
     const Func& f, 
     float initVal, 
     int startInd,
     const std::shared_ptr<FormulaParser> parser,
     const bool suppress_side_effects,
     const bool invert_if_any_operand_is_nan,
     const bool invert_only_if_another_operand_is_finite_and_true,
     const bool invert_only_if_another_operand_is_finite_and_false) {
   float d = initVal;
   bool finite_true_found{ std::isfinite(initVal) && initVal };
   bool finite_false_found{ std::isfinite(initVal) && !initVal };
   bool nan_found{ std::isnan(initVal) };

   for (size_t i = startInd; i < getOperands().size(); ++i) {
      float sub_result{ getOperands()[i]->eval(varVals, parser, suppress_side_effects) };
      finite_true_found = finite_true_found || (std::isfinite(sub_result) && sub_result);
      finite_false_found = finite_false_found || (std::isfinite(sub_result) && !sub_result);
      nan_found = nan_found || std::isnan(sub_result);
      d = f(d, sub_result);
   }

   if ((nan_found && invert_if_any_operand_is_nan)
        && (!invert_only_if_another_operand_is_finite_and_true || finite_true_found)
        && (!invert_only_if_another_operand_is_finite_and_false || finite_false_found)) {
      return !d; // Do this to emulate behavior of x86 assembly for functions <, <=, ==, !=, and, or etc. in very special cases.
   }

   return d;
}

template<class T>
std::shared_ptr<T> Term::copySimple(const std::shared_ptr<std::map<std::shared_ptr<Term>, std::shared_ptr<Term>>> already_copied) {
   std::vector<std::shared_ptr<Term>> opc{};
   opc.reserve(getOperands().size());

   for (size_t i = 0; i < getOperands().size(); ++i) {
      opc.push_back(getOperands()[i]->copy(already_copied));
   }

   std::shared_ptr<T> tc{ new T(opc) };
   tc->setChildrensFathers(false, false);

   return tc;
}

} // vfm