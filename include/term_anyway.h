//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file
#pragma once
#include "term.h"


namespace vfm {

class TermAnyway :
   public Term
{
public:
   const static OperatorStructure my_struct;
   OperatorStructure getOpStruct() const override;

   float eval(const std::shared_ptr<DataPack>& varVals, const std::shared_ptr<FormulaParser> parser = nullptr, const bool suppress_sideeffects = false) override;
   std::shared_ptr<Term> copy(const std::shared_ptr<std::map<std::shared_ptr<Term>, std::shared_ptr<Term>>> already_copied = nullptr) override;
#if defined(ASMJIT_ENABLED)
   std::shared_ptr<x86::Xmm> createSubAssembly(asmjit::x86::Compiler & cc, const std::shared_ptr<DataPack>& d, const std::shared_ptr<FormulaParser>& p = nullptr) override;
#endif
   std::shared_ptr<TermAnyway> toTermAnywayIfApplicable() override;
   std::set<MetaRule> getSimplificationRules(const std::shared_ptr<FormulaParser> parser) const override;

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

   TermAnyway(const std::vector<std::shared_ptr<Term>> terms);

private:
   static std::set<MetaRule> simplifications_;
};

} // vfm
