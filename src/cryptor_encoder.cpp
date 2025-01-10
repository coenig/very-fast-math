//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file

#include "cryptor/cryptor_encoder.h"
#include "cryptor/cryptor_starter.h"
#include "static_helper.h"
#include "math_struct.h"
#include "equation.h"
#include "term.h"
#include "cryptor/cryptor_rule.h"
#include "parser.h"
#include "dat_src_arr_as_readonly_string.h"
#include "dat_src_arr_as_string.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <sys/stat.h>

using namespace vfm;

void CryptorEncoder::createAssembly()
{
#if defined(ASMJIT_ENABLED)
   //addNote("Creating assembly for compilable subtrees...");

   auto data = data_;

   const auto func = [data](const std::shared_ptr<MathStruct> m) {
      if (m->isTermArray() && !data->isArrayDeclared(m->getOptor())) {
         data->addArrayAndOrSetArrayVal(m->getOptor(), -1, 0);
      }
   };

   if (termination_condition_) {
      termination_condition_ = termination_condition_->copy();
      termination_condition_->applyToMeAndMyChildren(func);
      termination_condition_->createAssembly(data_);
   }

   std::vector<std::shared_ptr<CryptorRule>> new_vec;

   for (const auto& r2 : rules_) {
      auto r = r2->copy();
      new_vec.push_back(r);

      const auto cond = r->getCondition();
      if (cond) {
         cond->applyToMeAndMyChildren(func);
         cond->createAssembly(data_);
      }

      for (const auto& eq : r->getEquations()) {
         if (eq) {
            eq->applyToMeAndMyChildren(func);
            eq->createAssembly(data_);
         }
      }
   }

   rules_ = new_vec;
#else
	std::cerr << "Assembly not activated." << std::endl;
	std::exit(1);
#endif
}

void CryptorEncoder::simplify()
{
   termination_condition_ = MathStruct::simplify(termination_condition_);

   for (const auto& r : rules_) {
      r->simplify();
   }
}

void CryptorEncoder::applyRules(const std::shared_ptr<DataSrcArray>& in_array, const std::shared_ptr<DataSrcArray>& out_array)
{
	data_->reset();
	data_->setArrayViaRawSource(in_symb_, in_array);
	data_->setArrayViaRawSource(out_symb_, out_array);

   if (pars_->jit) {
      createAssembly();
   }

   int cnt = 0;

   bool dots_printed = false;

   while (termination_condition_->eval(data_)) {
		for (auto r : rules_) {
			r->apply(data_);
		}

      if (++cnt % 10000 == 0) {
         std::cout << "'";
         dots_printed = true;
      }
	}

   if (dots_printed) {
      std::cout << std::endl;
   }
}

void CryptorEncoder::addRule(
	const std::shared_ptr<Term>& cond,
	const std::vector<std::shared_ptr<Equation>>& eqq)
{
	std::shared_ptr<CryptorRule> curr_rule = std::make_shared<CryptorRule>(cond);
   auto data = data_;

	for (auto eq : eqq) {
      curr_rule->addEquation(eq);
	}

	rules_.push_back(curr_rule);
}

void CryptorEncoder::setTerminationCondition(const std::string& cond)
{
	termination_condition_ = MathStruct::parseMathStruct(cond, true)->toTermIfApplicable();
}

std::pair<std::string, std::string> CryptorEncoder::split_cond_and_eqs(const std::string & rule) {
	size_t cl_pos = rule.find(CLOSING_COND_BRACKET);
	const std::string cond(rule.substr(rule.find(OPENING_COND_BRACKET) + 1, cl_pos - 1));
	std::string eqs(rule.substr(cl_pos + 1));
	return { cond, eqs };
}

std::string CryptorEncoder::process_easy(const std::string& word)
{
	std::shared_ptr<DataSrcArrayAsReadonlyString> in = std::make_shared<DataSrcArrayAsReadonlyString>(word);
	std::shared_ptr<DataSrcArrayAsString> out = std::make_shared<DataSrcArrayAsString>(word.size() + 100);
	applyRules(in, out);
	return out->toPlainString();
}

void CryptorEncoder::addRule(const std::string& cond, std::string& eqs)
{
	addRule(MathStruct::parseMathStruct(cond, true)->toTermIfApplicable(), MathStruct::parseEquations(eqs));
}

void CryptorEncoder::addRule(const std::string& rule) {
	std::pair<std::string, std::string> p = split_cond_and_eqs(rule);
	addRule(p.first, p.second);
}

void CryptorEncoder::readAlgorithm(const std::string& alg)
{
	std::vector<std::shared_ptr<MathStruct>> vec;
   std::string algo = alg;
	std::vector<std::string> rules = MathStruct::parseLines(algo,
		{ StaticHelper::makeString(OPENING_COND_BRACKET), StaticHelper::makeString(TERM_COND_DENOTER) },
		-1);

	for (auto rule : rules) {
		readAlgRule(rule);
	}
}

void CryptorEncoder::readAlgRule(std::string& line) {
	StaticHelper::trim(line);

	if (line.size() > 0) {
		if (line[0] == TERM_COND_DENOTER) {
			setTerminationCondition(line.substr(1));
		}
		else {
			addRule(line);
		}
	}
}

void CryptorEncoder::autoCreateFromOther(const std::shared_ptr<CryptorEncoder> complementary_encoder)
{
	in_symb_ = complementary_encoder->out_symb_;
	out_symb_ = complementary_encoder->in_symb_;

   termination_condition_ = complementary_encoder->termination_condition_->copy();

	// Swapping C and D ist not really useful if term_cond is "good".
	//to_term_cond->apply_to_me_and_my_children(
	//	[](std::shared_ptr<MathStruct> m) -> void {
	//	for (size_t i = 0; i < m->get_terms().size(); i++) {
	//		auto opnd = m->get_terms()[i];
	//		if (opnd->get_optor() == symb_arr_dec) 
	//			m->get_terms()[i] = std::make_shared<term_array>(opnd->get_terms()[0], symb_arr_enc);
	//		else if (opnd->get_optor() == symb_arr_enc) 
	//			m->get_terms()[i] = std::make_shared<term_array>(opnd->get_terms()[0], symb_arr_dec);
	//		else if (opnd->get_optor() == array_prefix + symb_arr_dec)
	//			m->get_terms()[i] = std::make_shared<term_var>(array_prefix + symb_arr_enc);
	//		else if (opnd->get_optor() == array_prefix + symb_arr_enc) 
	//			m->get_terms()[i] = std::make_shared<term_var>(array_prefix + symb_arr_dec);
	//	}
	//});

	std::string symb = out_symb_;
	rules_.clear();
	std::function<bool(std::shared_ptr<MathStruct>)> match_cond = [symb](std::shared_ptr<MathStruct> m) -> bool {
		return m->getOptor() == symb;
	};

	for (const auto& r : complementary_encoder->rules_) {
		std::shared_ptr<CryptorRule> r_comp = r->copy();
		r_comp->resolveEquationsTo(match_cond);
		r_comp->removeEquationsWith(in_symb_);
      addRule(r_comp->getCondition(), r_comp->getEquations());
	}

	auto_generated_ = true;
}

bool CryptorEncoder::loadFromFile(const std::string& in_path)
{
	struct stat buffer;
	if (stat(in_path.c_str(), &buffer) != 0) return false;

	std::ifstream input(in_path);
	std::stringstream sstr;
	while (input >> sstr.rdbuf());
	readAlgorithm(sstr.str());

	return true;
}

bool CryptorEncoder::storeToFile(const std::string& in_path)
{
	std::string input = toString();
	std::ofstream out(in_path);
	out << input;
	out.close();

	return true;
}

bool CryptorEncoder::isAutoGenerated() const
{
	return auto_generated_;
}

std::string CryptorEncoder::toString() const
{
	std::stringstream s;
	s << TERM_COND_DENOTER << *termination_condition_ << std::endl;
	
   for (const auto& r : this->rules_) {
      std::stringstream s2;
      s2 << "[" << *r->getCondition() << "] ";
	   for (auto eq : r->getEquations()) {
		   s2 << *eq << " " << PROGRAM_COMMAND_SEPARATOR << " ";
	   }

		s << s2.str() << std::endl;
	}

	return s.str();
}

CryptorEncoder::CryptorEncoder(const std::string& in_symb, const std::string& out_symb, const std::shared_ptr<DataPack> data, Parameters& pars)
   : Failable("CryptorEncoder"), termination_condition_(_val(0)), in_symb_(in_symb), out_symb_(out_symb), data_(data), pars_(&pars)
{
}
