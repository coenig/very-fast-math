//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file

#include "term.h"
#include "term_val.h"
#include "data_pack.h"
#include "parser.h"
#include <string>
#include <sstream>
#include <stdint.h>
#include <cassert>

using namespace vfm;

// Find out if 32 or 64 bit build, from https://stackoverflow.com/a/1505664/7302562.
template<int> bool is64BitBuildHelper();
template<> bool is64BitBuildHelper<4>() { return false; }
template<> bool is64BitBuildHelper<8>() { return true; }
inline bool is64BitBuild() { return is64BitBuildHelper<sizeof(size_t)>(); }
// EO Find out if 32 or 64 bit build.

#if defined(ASMJIT_ENABLED)
using namespace asmjit;
using namespace x86;

std::shared_ptr<x86::Xmm> Term::subAssemblyVarnumOps(x86::Compiler& cc, const std::shared_ptr<DataPack>& d, lambda_vno& fct) {
   auto x{ getOperands()[0]->createSubAssembly(cc, d) };

   if (!x) {
      return nullptr;
   }

   for (int i = 1; i < getOperands().size(); i++) {
      auto y{ getOperands()[i]->createSubAssembly(cc, d) };
      
      if (!y) {
         return nullptr;
      }
      
      fct(cc, *x, *y);
   }

   return x;
}

std::shared_ptr<x86::Xmm> Term::subAssemblySingleOp(x86::Compiler& cc, const std::shared_ptr<DataPack>& d, lambda_so& fct, const int index_of_single_op)
{
   auto x{ getOperands()[index_of_single_op]->createSubAssembly(cc, d) };

   if (!x) {
      return nullptr;
   }

   fct(cc, *x);

   return x;
}

void Term::setXmmVar(x86::Compiler& cc, x86::Xmm& v, float f)
{
   // Version from: https://github.com/lukedodd/JitCalc/blob/master/main.cpp#L204
   //X86Gp gpreg(cc.newGpd());
   //uint64_t* i = reinterpret_cast<uint64_t*>(&f);
   //cc.mov(gpreg, i[0]);
   //cc.movq(v, gpreg);
   //cc.unuse(gpreg);

   // Inspired by http://www.lukedodd.com/jitcalc-an-example-of-code-generation-with-c-and-asmjit/
   uint32_t *i = reinterpret_cast<uint32_t*>(&f);
   cc.mov(eax, i[0]);
   cc.movd(v, eax);
   //cc.shufps(v, v, 0);
}

void Term::setXmmVar(x86::Compiler& cc, x86::Xmm& set, x86::Xmm& to)
{
   cc.movd(eax, to);
   cc.movd(set, eax);

   // TODO: Maybe needed...? Delete if not.
   //auto regster = is64BitBuild() ? rax : eax;
   //cc.movd(regster, to);
   //cc.movd(set, regster);
}

/// \brief Read char from address "b" into Xmm register "v".
void Term::setXmmVarToAddressLocation(x86::Compiler& cc, x86::Xmm& v, const bool* b) {
   setXmmVarToAddressLocation(cc, v, reinterpret_cast<const char*>(b)); // Assuming sizeof(char) == sizeof(bool)
}

/// \brief Read char from address "c" into Xmm register "v".
void Term::setXmmVarToAddressLocation(x86::Compiler& cc, x86::Xmm& v, const char* c) {
   x86::Xmm index = cc.newXmm();
   cc.pxor(index, index); // Index is 0.
   setXmmVarToAddressLocation(cc, v, c, index, sizeof(bool));
}

/// \brief Read char from array at address "c" at index "index" into Xmm register "x" ("factor" has no influence yet).
void Term::setXmmVarToAddressLocation(x86::Compiler& cc, x86::Xmm& x, const char* c, x86::Xmm& index, const int& factor /*= 1*/) {
   x86::Gp regster = cc.newIntPtr("tmpPtr");
  
   cc.cvttss2si(ebx, index);
   cc.mov(regster, reinterpret_cast<uintptr_t>(c));

   x86::Mem m_addr = ptr(regster, ebx, 0, 0, factor); // Construct memory address referencing [x_gp + y_gp].
   cc.pxor(x, x);
   cc.movsx(ebx, m_addr);
   cc.cvtsi2ss(x, ebx);
}

/// \brief Read float from array at address "f" at index "index" into Xmm register "x".
void Term::setXmmVarToAddressLocation(x86::Compiler& cc, x86::Xmm& x, const float* f, x86::Xmm& index /*= 0*/) {
   x86::Gp regster = cc.newIntPtr("tmpPtr");

   cc.mov(regster, reinterpret_cast<uintptr_t>(f));
   cc.cvttss2si(ebx, index);
   cc.add(ebx, ebx); // = 2 * index
   cc.add(ebx, ebx); // = 4 * index (safer than multiplying due to overflow issues)
   x86::Mem mem = ptr(regster, ebx);
   cc.movss(x, mem);
}

void vfm::Term::setXmmVarToValueAtAddressFromArray(x86::Compiler& cc, x86::Xmm& x, float** ref_array, x86::Xmm& index)
{
   setXmmVarToAddressLocation(cc, x, *ref_array, index);
}

void Term::setXmmVarToAddressLocation(x86::Compiler& cc, x86::Xmm& v, const float* f, const int& index)
{
   x86::Gp regster = cc.newIntPtr("tmpPtr");
   cc.mov(regster, x86::Mem(reinterpret_cast<std::uintptr_t>(f), index));
   cc.movd(v, regster);

   // TODO: There's a suggestion to make this more efficient:
   // https://stackoverflow.com/questions/70062766/set-xmm-register-via-address-location-for-x86-64?noredirect=1#comment123852446_70062766
   // I have tried this before, but didn't work immediately. Anyway, it's probably a better solution, somehow.
   // Another more efficient version is suggested here: https://stackoverflow.com/a/70128634/7302562
}
#endif

bool Term::resolveTo(const std::function<bool(std::shared_ptr<MathStruct>)>& match_cond, bool from_left)
{
   // There's nothing to resolve in a Term.
   std::stringstream ss;
   ss << "Term '" << this << " (" << typeid(this).name() << ")" << "' cannot be resolved to subterm.";
   throw std::runtime_error(ss.str());
}

std::shared_ptr<Term> Term::getInvPattern(int argNum)
{
   std::stringstream ss;
   ss << "Term with operator '" << this->getOptor() << " (" << typeid(this).name() << ")" << "' has no inverse.";
   throw std::runtime_error(ss.str());
}

std::shared_ptr<Term> Term::toTermIfApplicable()
{
   return std::dynamic_pointer_cast<Term>(shared_from_this());
}

float Term::constEval(const std::shared_ptr<FormulaParser> parser, const bool suppress_sideeffects)
{
#if defined(ASMJIT_ENABLED)
   if (assembly_created_ == AssemblyState::yes) return (*fast_eval_func_)();
#endif

   std::shared_ptr<FormulaParser> p = parser ? parser : SingletonFormulaParser::getInstance();
   std::shared_ptr<DataPack> d = DataPack::getSingleton(); // TODO: This is a compromise which is NOT thread-safe...
   d->reset(); // ...std::make_shared<DataPack> would be a solution, but is very expensive. The way to go, in future, is to send nullptr to eval, and figure it out there.

   return eval(d, p, suppress_sideeffects);
}

Term::Term(const std::vector<std::shared_ptr<Term>>& opds) : MathStruct(opds) {}
