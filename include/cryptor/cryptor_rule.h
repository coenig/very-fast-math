//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file
#pragma once
#include <memory>
#include <vector>
#include <functional>
#include <string>

namespace vfm {

class MathStruct;
class Equation;
class Term;
class DataPack;

class CryptorRule
{
public:
   //friend std::ostream& operator<<(std::ostream &os, CryptorRule const &m);

   std::shared_ptr<CryptorRule> copy();
   void resolveEquationsTo(const std::function<bool(std::shared_ptr<MathStruct>)>& match_cond);
   bool apply(const std::shared_ptr<DataPack> data);
   void addEquation(const std::shared_ptr<Equation>& eq);
   std::shared_ptr<Term> getCondition() const;
   std::vector<std::shared_ptr<Equation>> getEquations() const;
   void removeEquationsWith(const std::string left_side_symb);
   void simplify();

   CryptorRule(const std::shared_ptr<Term>& cond);

private:
   std::shared_ptr<Term> condition_;
   std::vector<std::shared_ptr<Equation>> equations_;
};

} // vfm