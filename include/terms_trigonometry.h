//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file
#pragma once
#include "term_arith.h"


namespace vfm {

class TermSin :
   public TermArith
{
protected:
#if defined(ASMJIT_ENABLED)
   std::shared_ptr<x86::Xmm> createSubAssembly(asmjit::x86::Compiler& cc, const std::shared_ptr<DataPack>& d, const std::shared_ptr<FormulaParser>& p = nullptr) override;
#endif

public:
   const static OperatorStructure my_struct;

   OperatorStructure getOpStruct() const override;
   std::shared_ptr<Term> getInvPattern(int argNum) override;
   float eval(const std::shared_ptr<DataPack>& varVals, const std::shared_ptr<FormulaParser> parser = nullptr, const bool suppress_sideeffects = false) override;
   std::set<MetaRule> getSimplificationRules(const std::shared_ptr<FormulaParser> parser) const override;
   virtual std::shared_ptr<Term> copy(const std::shared_ptr<std::map<std::shared_ptr<Term>, std::shared_ptr<Term>>> already_copied = nullptr) override;

   void serialize(
      std::stringstream& os, 
      const std::shared_ptr<MathStruct> highlight,
      const SerializationStyle style,
      const SerializationSpecial special,
      const int indent,
      const int indent_step,
      const int line_len,
      const std::shared_ptr<DataPack> data,
      std::set<std::shared_ptr<const MathStruct>>& visited) const override;

   TermSin(const std::vector<std::shared_ptr<Term>>& opds);
   TermSin(const std::shared_ptr<Term> opd1);
};

} // vfm