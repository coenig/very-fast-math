//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file
#pragma once
#include "term.h"
#include "meta_rule.h"
#include <vector>


namespace vfm {

/// <summary>
/// The operator part of a TermCompound. For example, the "isprime(x)" part in
/// "compound(isprime(x), ...).
/// </summary>
class TermCompoundOperator final : public Term
{
public:

   OperatorStructure getOpStruct() const override;
   std::shared_ptr<Term> copy(const std::shared_ptr<std::map<std::shared_ptr<Term>, std::shared_ptr<Term>>> already_copied = nullptr) override;

   // To obey class restrictions, not really used.
   float eval(const std::shared_ptr<DataPack>& varVals, const std::shared_ptr<FormulaParser> parser = nullptr, const bool suppress_sideeffects = false) override;
   float constEval(const std::shared_ptr<FormulaParser> parser = nullptr, const bool suppress_sideeffects = false) override;
   bool resolveTo(const std::function<bool(std::shared_ptr<MathStruct>)>& match_cond, bool from_left) override;
   bool isCompoundOperator() const override;
   std::shared_ptr<TermCompoundOperator> toCompoundOperatorIfApplicable() override;

#if defined(ASMJIT_ENABLED)
   std::shared_ptr<x86::Xmm> createSubAssembly(asmjit::x86::Compiler& cc, const std::shared_ptr<DataPack>& d, const std::shared_ptr<FormulaParser>& p = nullptr) override;
#endif
   // EO To obey class restrictions, not really used.

   std::string getOptorOnCompoundLevel() const override;
   std::string getCPPOperator() const override;

   TermCompoundOperator(
      const std::vector<std::shared_ptr<Term>>& opds,
      const OperatorStructure& operator_structure);
   
private:
   OperatorStructure my_struct_{ OutputFormatEnum::postfix, AssociativityTypeEnum::left, -1, -1, "#INVALID_OP_NAME", false, false, AdditionalOptionEnum::none };
};

} // vfm