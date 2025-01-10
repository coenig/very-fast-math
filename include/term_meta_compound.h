//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file
#pragma once
#include "term.h"
#include "term_val.h"


namespace vfm {

class TermMetaCompound :
   public Term
{
public:
   const static OperatorStructure my_struct;
   OperatorStructure getOpStruct() const override;

   /// Meta's const status depends on the const status of the calling compound structure,
   /// therefore, has to be tested on that level. Here we return false.
   /// (As placeholder for inverting equations it has no sensible const-ness status.)
   bool isOverallConstant(const std::shared_ptr<std::vector<int>> non_const_metas = nullptr,
                          const std::shared_ptr<std::set<std::shared_ptr<MathStruct>>> visited = nullptr) override;

   float eval(const std::shared_ptr<DataPack>& varVals, const std::shared_ptr<FormulaParser> parser = nullptr, const bool suppress_sideeffects = false) override;
   std::shared_ptr<Term> copy(const std::shared_ptr<std::map<std::shared_ptr<Term>, std::shared_ptr<Term>>> already_copied = nullptr) override;
#if defined(ASMJIT_ENABLED)
   std::shared_ptr<x86::Xmm> createSubAssembly(asmjit::x86::Compiler& cc, const std::shared_ptr<DataPack>& d, const std::shared_ptr<FormulaParser>& p = nullptr) override;
#endif
   bool isMetaCompound() const override;
   std::shared_ptr<TermMetaCompound> toMetaCompoundIfApplicable() override;
   void prohibitUpdateStack();
   void allowUpdateStack();
   
   /// A meta always needs to assume it has side effects. 
   /// For example, "compound(a = b; c, p_(0) * 0 + p_(1))" cannot optimize the "a = b" away.
   /// We also cannot figure it out automatically (by looking if the 0-th argument has side effects), 
   /// because the compound structure might be referenced from several places.
   bool hasSideeffectsThis() const override;

   std::string getK2Operator() const override;

   /// NOTE: The Term t has to be constant for assembly to work.
   /// This means, "5+7*2" is ok, but not "a" or "H(4)".
   TermMetaCompound(const std::shared_ptr<Term>& t);
   TermMetaCompound();

private:
   bool okToUpdateStack{ true };
};

} // vfm
