//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file
#pragma once
#include "term.h"


namespace vfm {

   class TermPrint :
      public Term
   {
   public:
      const static OperatorStructure my_struct;

      OperatorStructure getOpStruct() const override;
      float eval(const std::shared_ptr<DataPack>& varVals, const std::shared_ptr<FormulaParser> parser = nullptr, const bool suppress_sideeffects = false) override;
#if defined(ASMJIT_ENABLED)
      std::shared_ptr<x86::Xmm> createSubAssembly(asmjit::x86::Compiler & cc, const std::shared_ptr<DataPack>& d, const std::shared_ptr<FormulaParser>& p = nullptr) override;
#endif
      std::shared_ptr<Term> copy(const std::shared_ptr<std::map<std::shared_ptr<Term>, std::shared_ptr<Term>>> already_copied = nullptr) override;
      std::set<MetaRule> getSimplificationRules(const std::shared_ptr<FormulaParser> parser) const override;
      bool isTermPrint() const override;

      TermPrint(const std::vector<std::shared_ptr<Term>>& opds);
      TermPrint(const std::shared_ptr<Term> opd1, const std::shared_ptr<Term> opd2);
   };
} // vfm