//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file
#pragma once
#include "term_arith.h"


namespace vfm {

class TermSin : public TermArith
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
   virtual std::shared_ptr<Term> copy(const std::shared_ptr<std::map<std::shared_ptr<Term>, std::shared_ptr<Term>>> already_copied = nullptr) override;

   TermSin(const std::vector<std::shared_ptr<Term>>& opds);
   TermSin(const std::shared_ptr<Term> opd1);
};

class TermCos : public TermArith
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
   virtual std::shared_ptr<Term> copy(const std::shared_ptr<std::map<std::shared_ptr<Term>, std::shared_ptr<Term>>> already_copied = nullptr) override;

   TermCos(const std::vector<std::shared_ptr<Term>>& opds);
   TermCos(const std::shared_ptr<Term> opd1);
};

class TermTan : public TermArith
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
   virtual std::shared_ptr<Term> copy(const std::shared_ptr<std::map<std::shared_ptr<Term>, std::shared_ptr<Term>>> already_copied = nullptr) override;

   TermTan(const std::vector<std::shared_ptr<Term>>& opds);
   TermTan(const std::shared_ptr<Term> opd1);
};

class TermASin : public TermArith
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
   virtual std::shared_ptr<Term> copy(const std::shared_ptr<std::map<std::shared_ptr<Term>, std::shared_ptr<Term>>> already_copied = nullptr) override;

   TermASin(const std::vector<std::shared_ptr<Term>>& opds);
   TermASin(const std::shared_ptr<Term> opd1);
};

class TermACos : public TermArith
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
   virtual std::shared_ptr<Term> copy(const std::shared_ptr<std::map<std::shared_ptr<Term>, std::shared_ptr<Term>>> already_copied = nullptr) override;

   TermACos(const std::vector<std::shared_ptr<Term>>& opds);
   TermACos(const std::shared_ptr<Term> opd1);
};

class TermATan : public TermArith
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
   virtual std::shared_ptr<Term> copy(const std::shared_ptr<std::map<std::shared_ptr<Term>, std::shared_ptr<Term>>> already_copied = nullptr) override;

   TermATan(const std::vector<std::shared_ptr<Term>>& opds);
   TermATan(const std::shared_ptr<Term> opd1);
};

} // vfm