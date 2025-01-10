//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file
#pragma once
#include "term.h"


namespace vfm {

/// Note that this term structure does not yet compile to assembly at runtime.
class TermSetArr :
   public Term
{
public:
   static const OperatorStructure my_struct;

   bool isTermSetArr() const override;
   std::shared_ptr<TermSetArr> toSetArrIfApplicable() override;
   OperatorStructure getOpStruct() const override;
   float eval(const std::shared_ptr<DataPack>& varVals, const std::shared_ptr<FormulaParser> parser = nullptr, const bool suppress_sideeffects = false) override;
#if defined(ASMJIT_ENABLED)
   std::shared_ptr<x86::Xmm> createSubAssembly(asmjit::x86::Compiler & cc, const std::shared_ptr<DataPack>& d, const std::shared_ptr<FormulaParser>& p = nullptr) override;
#endif
   std::shared_ptr<Term> copy(const std::shared_ptr<std::map<std::shared_ptr<Term>, std::shared_ptr<Term>>> already_copied = nullptr) override;
   bool hasSideeffectsThis() const override;

   /// Set might be seen as "overall constant" in the classic pure-function view if the value it
   /// sets to is overall constant. However, this view might be unintuitive and does not
   /// gain any benefits as set is not a pure function. Therefore this method returns false.
   virtual bool isOverallConstant(const std::shared_ptr<std::vector<int>> non_const_metas = nullptr,
                                  const std::shared_ptr<std::set<std::shared_ptr<MathStruct>>> visited = nullptr) override;

   std::string getArrName(const std::shared_ptr<DataPack> d, const std::shared_ptr<FormulaParser> parser = nullptr) const;
   void changeArrToSet(const std::string& var_name, const std::shared_ptr<DataPack> d);
      
   TermSetArr(const std::vector<std::shared_ptr<Term>>& vt);
   TermSetArr(const std::shared_ptr<Term> arr, const std::shared_ptr<Term> idx, const std::shared_ptr<Term> val);
   TermSetArr(const std::string& var_name, const std::shared_ptr<Term> idx, const std::shared_ptr<Term> val, const std::shared_ptr<DataPack> d);

private:
   static std::shared_ptr<Term> getArrAddress(const std::string& var_name, const std::shared_ptr<DataPack> d);
};

} // vfm