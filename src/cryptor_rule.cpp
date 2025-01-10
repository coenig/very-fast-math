//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file

#include "cryptor/cryptor_rule.h"
#include "fsm.h"
#include "math_struct.h"
#include "term.h"
#include "equation.h"
#include <exception>
#include <sstream>

using namespace vfm;

void CryptorRule::simplify()
{
   condition_ = MathStruct::simplify(condition_)->toTermIfApplicable();

   for (const auto& eq : equations_) {
      eq->setOpnds({ MathStruct::simplify(eq->getOperands()[0])->toTermIfApplicable(), MathStruct::simplify(eq->getOperands()[1])->toTermIfApplicable(), MathStruct::simplify(eq->getOperands()[2])->toTermIfApplicable() });
   }
}

std::shared_ptr<CryptorRule> CryptorRule::copy()
{
	std::shared_ptr<Term> term_cp = condition_->copy()->toTermIfApplicable();
	term_cp->setChildrensFathers();
   std::shared_ptr<CryptorRule> cp = std::make_shared<CryptorRule>(term_cp);

	for (auto eq : equations_) {
		std::shared_ptr<Equation> eq_cp = eq->copy()->toEquationIfApplicable();
		eq->setChildrensFathers();
		cp->equations_.push_back(eq_cp);
	}

	return cp;
}

void CryptorRule::resolveEquationsTo(const std::function<bool(std::shared_ptr<MathStruct>)>& match_cond)
{
	for (size_t i = 0; i < equations_.size(); ++i) {
		auto e = equations_[i];
		auto e_copy = e->copy()->toEquationIfApplicable();

		if (e->resolveTo(match_cond, false)) {
			e->swap();
		}
		else {
			equations_[i] = e_copy;
		}
	}
}

bool CryptorRule::apply(const std::shared_ptr<DataPack> data)
{
	if (this->condition_->eval(data)) {
		for (auto eq : this->equations_) {
			std::shared_ptr<MathStruct> ms0 = eq->getOperands()[0];
			std::shared_ptr<MathStruct> ms1 = eq->getOperands()[1];
			std::string opt0 = ms0->getOptor();
			if (ms0->isTermVar()) { // Is constant variable?
				data->addOrSetSingleVal(opt0, ms1->eval(data));
			}
			else { // Should be array then.
				data->addArrayAndOrSetArrayVal(opt0, static_cast<int>(ms0->getOperands()[0]->eval(data)), ms1->eval(data));
			}
		}

		return true;
	}

	return false;
}

void CryptorRule::addEquation(const std::shared_ptr<Equation>& eq)
{
   equations_.push_back(eq);
}

std::shared_ptr<Term> CryptorRule::getCondition() const
{
	return condition_;
}

std::vector<std::shared_ptr<Equation>> CryptorRule::getEquations() const
{
	return equations_;
}

void CryptorRule::removeEquationsWith(const std::string left_side_symb)
{
	for (size_t i = 0; i < equations_.size(); i++) {
		auto eq = equations_[i];

		if (eq->getOperands()[0]->getOptor() == left_side_symb) {
			equations_.erase(equations_.begin() + i); // Does this work (is it good practice) in C++?
			i--;
		}
	}
}

CryptorRule::CryptorRule(const std::shared_ptr<Term>& cond) : condition_(cond), equations_{}
{
	if (!cond->isTerm()) {
		std::stringstream ss;
		ss << "Condition '" << cond->serialize() << "' is not term, but '" << typeid(cond).name() << "'.";
		throw std::runtime_error(ss.str());
	}
}

//std::ostream& operator<<(std::ostream &os, CryptorRule const &m) {
//	os << "[" << *m.get_cond() << "] ";
//	for (auto eq : m.get_eqs()) {
//		os << *eq << "; ";
//	}
//	return os;
//}
