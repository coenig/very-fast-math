//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file
#pragma once
#include "term_val.h"
#include "parsable.h"
#include "term_logic_and.h"
#include "term_logic_or.h"
#include "term_logic_sm.h"
#include "term_logic_smeq.h"
#include "term_logic_eq.h"
#include "term_logic_neq.h"
#include "term_logic_greq.h"
#include "term_logic_gr.h"
#include "term_plus.h"
#include "term_pow.h"
#include "term_ln.h"
#include "term_minus.h"
#include "term_mult.h"
#include "term_div.h"
#include "term_neg.h"
#include "term_print.h"
#include "term_mod.h"
#include "term_max.h"
#include "term_meta_compound.h"
#include "term_meta_simplification.h"
#include "term_optional.h"
#include "term_anyway.h"
#include "term_min.h"
#include "term_val.h"
#include "term_var.h"
#include "term_abs.h"
#include "term_rand.h"
#include "term_trunc.h"
#include "term_sqrt.h"
#include "term_rsqrt.h"
#include "term_set_var.h"
#include "term_set_arr.h"
#include "term_get_arr.h"
#include "term_while_limited.h"
#include "term_array.h"
#include "term_array_length.h"
#include "term_func_ref.h"
#include "term_func_eval.h"
#include "term_func_lambda.h"
#include "term_literal.h"
#include "term_fctin.h"
#include "term_malloc.h"
#include "term_delete.h"
#include "term_id.h"
#include "terms_trigonometry.h"
#include "equation.h"
#include "earley/earley_grammar.h"
#include <vector>
#include <stack>
#include <sstream>
#include <string>
#include <iterator>
#include <map>


namespace vfm {

class Term;
class OperatorStructure;
class TermCompound;

const std::string GRAMMAR_SIMPLE_START_SYMBOL = earley::START_SYMBOL_DEFAULT;
const std::string GRAMMAR_START_SYMBOL_PREFIX = GRAMMAR_SIMPLE_START_SYMBOL;

std::shared_ptr<Term> _val0();

enum class TLType {
   LTL,
   CTL,
   both
};

enum class StackObjectType {
   Function,
   Operator,
   Other
};

enum class OperatorDeclarationState {
   undeclared,
   unique,
   overloaded
};

class FormulaParser : public Parsable, public std::enable_shared_from_this<FormulaParser>
{
public:
   FormulaParser();

   void init();
   void createSimpleAnyways(); // Adds ~, ~~, ~~~, ... to allow parsing meta rules like "(x ~ y) ~~ z ==> x ~ (y ~~ z)".
   void addDefaultDynamicTerms(); // This includes createSimpleAnyways(). 
   void pushToOutput(const std::string& raw_el, std::stack<int>& nested_arg_nums_stack);
   int prio(const std::string& optor, const int num_params) const;
   bool isLeftAssoc(const std::string& optor, const int num_params) const;
   bool isArray(const std::string& optor);
   bool isOperatorPossibly(const std::string& optor) const;
   bool isFunctionOrOperator(const std::string& optor) const; // Use -1 to return true on any found overload.
   bool isOperatorForSure(const std::string& optor) const; /// Returns true only if there is an operator with that name and NO other overloaded function with that name.
   OperatorDeclarationState getOperatorDeclarationState(const std::string& optor) const;
   void treatNestedFunctionArgs(const std::string& token, const int i, const std::shared_ptr<std::vector<std::string>>& tokens, std::stack<int>& nested_function_args_stack, const bool valid);
   std::shared_ptr<Term> parseShuntingYard(const std::shared_ptr<std::vector<std::string>>& tokens);

   std::string toStringCompoundFunctions(const bool flatten = false) const;

   /// Returns nullptr if optor is unknown.
   /// For functions without parameters.
   std::shared_ptr<Term> termFactory(const std::string& optor);

   /// Returns nullptr if optor is unknown.
   std::shared_ptr<Term> termFactory(
      const std::string& optor, 
      const std::shared_ptr<Term>& term);

   /// Returns nullptr if optor is unknown.
   std::shared_ptr<Term> termFactory(
      const std::string& optor, 
      const std::vector<std::shared_ptr<Term>>& terms,
      const bool suppress_create_new_dynamic_term_via_term_func_ref = false // Has an effect only if optor if SYMB_FUNC_REF.
   );

   void applyFormulaToParser(const std::string& formula_str, const std::shared_ptr<DataPack> data = nullptr);

   /// <summary>
   ///  Uses dummy operands to retrieve the Term.
   /// </summary>
   /// <param name="optor">The name of the operator.</param>
   /// <returns>A dummy term with correct operator, but dummy operands.</returns>
   std::shared_ptr<Term> termFactoryDummy(const std::string& optor, int num_pars);
   
   std::shared_ptr<Term> termFactoryDummy(const std::string& optor);

   void forwardDeclareDynamicTerm(const OperatorStructure& op_struct);
   void forwardDeclareDynamicTerm(
      const OperatorStructure& op_struct, 
      const std::string& op_name,
      const bool is_native = false,
      const int arg_num = -1,
      const int precedence = -1,
      const OutputFormatEnum op_pos_type = OutputFormatEnum::prefix,
      const AssociativityTypeEnum associativity = AssociativityTypeEnum::left,
      const bool assoc = false,
      const bool commut = false,
      const AdditionalOptionEnum option = AdditionalOptionEnum::none);

   void addDynamicTerm(const OperatorStructure& op_struct, const std::string& meta_struct, const std::shared_ptr<DataPack> data = nullptr, const bool is_native = false);
   void addDynamicTerm(const OperatorStructure& op_struct, const std::shared_ptr<Term> meta_struct, const bool is_native = false);
   void addDynamicTerm(
      const std::shared_ptr<Term> meta_struct,
      const std::string& op_name,
      const bool is_native = false,
      const int arg_num = -1,
      const int precedence = -1,
      const OutputFormatEnum op_pos_type = OutputFormatEnum::prefix,
      const AssociativityTypeEnum associativity = AssociativityTypeEnum::left,
      const bool assoc = false,
      const bool commut = false,
      const AdditionalOptionEnum option = AdditionalOptionEnum::none);
      
   int addLambdaTerm(const std::shared_ptr<Term> meta_struct);

   /// The number of arguments of the given operator. Returns 0 if operator
   /// is unknown (assuming TermVar variable name). 
   ///
   /// TODO: Couldn't this be replaced by the less expensive argNum(.)?
   std::set<int> getNumParams(const std::string& optor);

   OperatorStructure getOperatorStructure(const std::string& op_name, const int num_params);
   std::shared_ptr<Term> getDynamicTermMeta(const std::string& op_name, const int num_params) const;
   std::map<std::string, std::map<int, std::shared_ptr<Term>>> getDynamicTermMetas();
   
   std::string serializeDynamicTerms(const bool include_native_terms = false);
   static std::string preprocessCPPProgram(const std::string& program);
   void postprocessFormulaFromCPP(const std::shared_ptr<MathStruct> formula, const std::map<std::string, std::set<int>>& pointerized);
   std::string preprocessProgram(const std::string& program) const override;
   bool parseProgram(const std::string& dynamic_term_commands) override;
   std::map<std::string, std::map<int, OperatorStructure>>& getAllOps();

   void checkForUndeclaredVariables(const std::shared_ptr<DataPack> d);

   inline std::string getCurrentAutoExtractedFunctionName() const
   {
      return current_auto_extracted_function_name_;
   }

   inline void setCurrentAutoExtractedFunctionName(const std::string& name) {
      current_auto_extracted_function_name_ = name;
   }

   void simplifyFunctions(const bool mc_mode = false, bool include_those_with_metas = false);
   bool isFunctionDeclared(const std::string& func_name, const int num_params) const;
   bool isFunctionDeclared(const int func_id) const;
   bool isFunctionDefined(const std::string& func_name, const int num_params) const;
   int getFunctionAddressOf(const std::string& func_name, const int num_params) const;
   std::pair<std::string, int> getFunctionName(const int func_address) const;

   void addDotOperator();
   void addnuSMVOperators(const vfm::TLType mode);

   std::shared_ptr<Term> parseFromSExpression(const std::string& s_expression);
   std::shared_ptr<Term> parseFromPostfix(const std::string& postfix);

   bool isTermNative(const std::string& func_name, const int num_params) const;

   std::shared_ptr<earley::Grammar> createGrammar(const bool include_full_brace_transformations = false, const std::set<std::string>& use_only = {}) const;

   /// \brief Uses values from "other" to initialize the corresponding values in this.
   void initializeValuesBy(const FormulaParser& other);
   void initializeValuesBy(const std::shared_ptr<FormulaParser> other);

   void resetRawDynamicTerm(const std::string& op_name, const TermPtr new_term, const int num_params);
   void removeDynamicTerm(const std::string& op_name, const int num_params);
   std::map<std::string, std::map<int, std::set<MetaRule>>> getAllRelevantSimplificationRulesOrMore();

   // TODO: This function should be obsolete since we have a generic
   // overloading mechanism now. Commented-out for now, feel free to delete later.
   //void requestFlexibleNumberOfArgumentsFor(const std::string& op_name);

   std::map<int, std::pair<std::string, int>> getAllDeclaredFunctions() const;
   std::map<int, std::string> getParameterNamesFor(const std::string dynamic_term_name, const int num_overloaded_params) const;

private:
   std::vector<std::shared_ptr<Term>> curr_terms_;
   std::map<std::string, std::map<int, OperatorStructure>> all_ops_; // f(), f(x), f(x, y), f(x, y, z)
   std::map<std::string, std::map<int, OperatorStructure>> all_arrs_;
   std::map<std::string, std::map<int, std::shared_ptr<Term>>> dynamic_term_metas_; /// These are the compound structures of the dynamic terms.
   std::vector<std::shared_ptr<TermCompound>> forward_declared_terms_;
   std::string current_auto_extracted_function_name_ = std::string(1, ALPHA_UPP[0]);
   std::string current_lambda_function_counter_ = std::string(1, ALPHA_UPP[0]);
   std::vector<std::string> parsed_variables_;
   std::map<std::pair<std::string, int>, int> names_to_addresses{};
   std::map<int, std::pair<std::string, int>> addresses_to_names{};
   int function_count_ = 0;
   bool is_in_function_ref_mode_ = false;
   //std::set<std::string> operators_with_flexible_number_of_arguments_; // TODO: Delete including all commented-out usages!

   std::set<std::pair<std::string, int>> native_funcs_{};

   // Mapping from { function_name, num_params } to map from parameter num to its name.
   std::map<std::pair<std::string, int>, std::map<int, std::string>> naming_of_parameters_{};

   std::string preprocessProgramSquareStyleArrayAccess(const std::string& program) const;
   std::string preprocessProgramBraceStyleFunctionDenotation(const std::string& program) const;

   std::pair<std::string, StackObjectType> popAndReturn(std::vector<std::pair<std::string, StackObjectType>>& stack);

   inline std::string getCurrentLambdaFunctionName() const
   {
      return current_lambda_function_counter_;
   }

   inline void setCurrentLambdaFunctionName(const std::string& name) {
      current_lambda_function_counter_ = name;
   }

   void registerAddress(const std::string& func_name, const int num_params);
   void addDynamicTermViaFuncRef(const std::vector<std::shared_ptr<Term>>& terms);

   static std::string getStartSymbol(const int i);
};

extern "C" float evaluateProgramAndFormula(
   const char* program,
   const char* stl_formula,
   const float* arr,
   const int cols,
   const int rows,
   const char* array_names,
   const char* path_to_base_dir);

class SingletonFormulaParser
{
public:
   static std::shared_ptr<FormulaParser> getInstance(); /// A full instance of the singleton parser including default dynamic terms.
   static std::shared_ptr<FormulaParser> getLightInstance(); /// A lightweight instance of the singleton parser WITHOUT default dynamic terms.

private:
   static std::shared_ptr<FormulaParser> instance_;
   static std::shared_ptr<FormulaParser> light_instance_;
};

class SingletonFormulaGrammar
{
public:
   static earley::Grammar& getInstance(const std::shared_ptr<FormulaParser> ref_parser);
   static earley::Grammar& getInstance();
   static earley::Grammar& getLightInstance();

private:
   static std::map<std::shared_ptr<FormulaParser>, std::shared_ptr<earley::Grammar>> instances_;
};

// *** Convenience functions for in-code Term construction. Note that MathStruct::parseMathStruct may be even more convenient in many cases. ***
// The following functions just "plainly" create the term, but don't set childrens' fathers. So, the chain may be broken at the insertion of the subterms given as arguments.
// Use below functions to also set childrens' fathers.
inline std::shared_ptr<Term> _val_plain(const float term1) { return std::make_shared<TermVal>(term1); }
inline std::shared_ptr<Term> _var_plain(const std::string& term1) { return std::make_shared<TermVar>(term1); }
inline std::shared_ptr<Term> _arr_plain(const std::string& term1, const std::shared_ptr<Term> term2) { return std::make_shared<TermArray>(term2, term1); }
inline std::shared_ptr<Term> _arrlen_plain(const std::shared_ptr<Term> term1) { return std::make_shared<TermArrayLength>(term1); }
inline std::shared_ptr<Term> _arrlen_plain(const std::string& term1) { return std::make_shared<TermArrayLength>(_var_plain(SYMB_REF + term1)); }

inline std::shared_ptr<Term> _not_plain(const std::shared_ptr<Term> term) { return SingletonFormulaParser::getInstance()->termFactory(SYMB_NOT, term); }
inline std::shared_ptr<Term> _and_plain(const std::shared_ptr<Term> term1, const std::shared_ptr<Term> term2) { return std::make_shared<TermLogicAnd>(term1, term2); }
inline std::shared_ptr<Term> _or_plain(const std::shared_ptr<Term> term1, const std::shared_ptr<Term> term2) { return std::make_shared<TermLogicOr>(term1, term2); }
inline std::shared_ptr<Term> _sm_plain(const std::shared_ptr<Term> term1, const std::shared_ptr<Term> term2) { return std::make_shared<TermLogicSm>(term1, term2); }
inline std::shared_ptr<Term> _gr_plain(const std::shared_ptr<Term> term1, const std::shared_ptr<Term> term2) { return std::make_shared<TermLogicGr>(term1, term2); }
inline std::shared_ptr<Term> _smeq_plain(const std::shared_ptr<Term> term1, const std::shared_ptr<Term> term2) { return std::make_shared<TermLogicSmEq>(term1, term2); }
inline std::shared_ptr<Term> _greq_plain(const std::shared_ptr<Term> term1, const std::shared_ptr<Term> term2) { return std::make_shared<TermLogicGrEq>(term1, term2); }
inline std::shared_ptr<Term> _eq_plain(const std::shared_ptr<Term> term1, const std::shared_ptr<Term> term2) { return std::make_shared<TermLogicEq>(term1, term2); }
inline std::shared_ptr<Term> _neq_plain(const std::shared_ptr<Term> term1, const std::shared_ptr<Term> term2) { return std::make_shared<TermLogicNeq>(term1, term2); }
inline std::shared_ptr<Term> _approx_plain(const std::shared_ptr<Term> term1, const std::shared_ptr<Term> term2) { return SingletonFormulaParser::getInstance()->termFactory(SYMB_APPROX, { term1, term2 }); }
inline std::shared_ptr<Term> _napprox_plain(const std::shared_ptr<Term> term1, const std::shared_ptr<Term> term2) { return SingletonFormulaParser::getInstance()->termFactory(SYMB_NAPPROX, { term1, term2 }); }

inline std::shared_ptr<Term> _plus_plain(const std::shared_ptr<Term> term1, const std::shared_ptr<Term> term2) { return std::make_shared<TermPlus>(term1, term2); }
inline std::shared_ptr<Term> _minus_plain(const std::shared_ptr<Term> term1, const std::shared_ptr<Term> term2) { return std::make_shared<TermMinus>(term1, term2); }
inline std::shared_ptr<Term> _mult_plain(const std::shared_ptr<Term> term1, const std::shared_ptr<Term> term2) { return std::make_shared<TermMult>(term1, term2); }
inline std::shared_ptr<Term> _div_plain(const std::shared_ptr<Term> term1, const std::shared_ptr<Term> term2) { return std::make_shared<TermDiv>(term1, term2); }
inline std::shared_ptr<Term> _pow_plain(const std::shared_ptr<Term> term1, const std::shared_ptr<Term> term2) { return std::make_shared<TermPow>(term1, term2); }
inline std::shared_ptr<Term> _ln_plain(const std::shared_ptr<Term> term1) { return std::make_shared<TermLn>(term1); }
inline std::shared_ptr<Term> _sin_plain(const std::shared_ptr<Term> term1) { return std::make_shared<TermSin>(term1); }
inline std::shared_ptr<Term> _cos_plain(const std::shared_ptr<Term> term1) { return std::make_shared<TermCos>(term1); }
inline std::shared_ptr<Term> _tan_plain(const std::shared_ptr<Term> term1) { return std::make_shared<TermTan>(term1); }
inline std::shared_ptr<Term> _asin_plain(const std::shared_ptr<Term> term1) { return std::make_shared<TermASin>(term1); }
inline std::shared_ptr<Term> _acos_plain(const std::shared_ptr<Term> term1) { return std::make_shared<TermACos>(term1); }
inline std::shared_ptr<Term> _atan_plain(const std::shared_ptr<Term> term1) { return std::make_shared<TermATan>(term1); }

inline std::shared_ptr<Term> _neg_plain(const std::shared_ptr<Term> term) { return std::make_shared<TermNeg>(term); }
inline std::shared_ptr<Term> _neg_one_minus_plain(const std::shared_ptr<Term> term1) { return SingletonFormulaParser::getInstance()->termFactory(SYMB_NEG_ALT, { term1 }); }
inline std::shared_ptr<Term> _mod_plain(const std::shared_ptr<Term> term1, const std::shared_ptr<Term> term2) { return std::make_shared<TermMod>(term1, term2); }
inline std::shared_ptr<Term> _max_plain(const std::shared_ptr<Term> term1, const std::shared_ptr<Term> term2) { return std::make_shared<TermMax>(term1, term2); }
inline std::shared_ptr<Term> _min_plain(const std::shared_ptr<Term> term1, const std::shared_ptr<Term> term2) { return std::make_shared<TermMin>(term1, term2); }

inline std::shared_ptr<Term> _abs_plain(const std::shared_ptr<Term> term1) { return std::make_shared<TermAbs>(term1); }
inline std::shared_ptr<Term> _trunc_plain(const std::shared_ptr<Term> term1) { return std::make_shared<TermTrunc>(term1); }
inline std::shared_ptr<Term> _sqrt_plain(const std::shared_ptr<Term> term1) { return std::make_shared<TermSQRT>(term1); }
inline std::shared_ptr<Term> _rsqrt_plain(const std::shared_ptr<Term> term1) { return std::make_shared<TermRSQRT>(term1); }

inline std::shared_ptr<Term> _rand_plain(const std::shared_ptr<Term> term1) { return std::make_shared<TermRand>(term1); }

inline std::shared_ptr<Term> _set_plain(const std::string& term1, const std::string& term2) { return SingletonFormulaParser::getInstance()->termFactory(SYMB_SET_VAR_A, { _var_plain(SYMB_REF + term1), _var_plain(term2) }); }
inline std::shared_ptr<Term> _set_plain(const std::string& term1, const std::shared_ptr<Term> term2) { return SingletonFormulaParser::getInstance()->termFactory(SYMB_SET_VAR_A, { _var_plain(SYMB_REF + term1), term2 }); }
inline std::shared_ptr<Term> _set_alt_plain(const std::shared_ptr<Term> term1, const std::shared_ptr<Term> term2) { return SingletonFormulaParser::getInstance()->termFactory(SYMB_SET_VAR_A, { term1, term2 }); }
inline std::shared_ptr<Term> _set_plain(const std::shared_ptr<Term> term1, const std::shared_ptr<Term> term2) { return std::make_shared<TermSetVar>(term1, term2); }
inline std::shared_ptr<Term> _Set_plain(const std::string& term1, const std::shared_ptr<Term> term2, const std::shared_ptr<Term> term3, const std::shared_ptr<DataPack> d) { return std::make_shared<TermSetArr>(term1, term2, term3, d); }
inline std::shared_ptr<Term> _Set_plain(const std::shared_ptr<Term> term1, const std::shared_ptr<Term> term2, const std::shared_ptr<Term> term3) { return std::make_shared<TermSetArr>(term1, term2, term3); }
inline std::shared_ptr<Term> _get_plain(const std::string& term1) { return SingletonFormulaParser::getInstance()->termFactory(SYMB_GET_VAR, _var_plain(SYMB_REF + term1)); }
inline std::shared_ptr<Term> _get_plain(const std::shared_ptr<Term> term1) { return SingletonFormulaParser::getInstance()->termFactory(SYMB_GET_VAR, term1); }
inline std::shared_ptr<Term> _Get_plain(const std::string& term1, const std::shared_ptr<Term> term2, const std::shared_ptr<DataPack> d) { return std::make_shared<TermGetArr>(term1, term2, d); }
inline std::shared_ptr<Term> _Get_plain(const std::shared_ptr<Term> term1, const std::shared_ptr<Term> term2) { return std::make_shared<TermGetArr>(term1, term2); }

inline std::shared_ptr<Term> _seq_plain(const std::shared_ptr<Term> term1, const std::shared_ptr<Term> term2) { return SingletonFormulaParser::getInstance()->termFactory(SYMB_SEQUENCE, { term1, term2 }); }
inline std::shared_ptr<Term> _if_plain(const std::shared_ptr<Term> term1, const std::shared_ptr<Term> term2) { return SingletonFormulaParser::getInstance()->termFactory(SYMB_IF, { term1, term2 }); }
inline std::shared_ptr<Term> _ifelse_plain(const std::shared_ptr<Term> term1, const std::shared_ptr<Term> term2, const std::shared_ptr<Term> term3) { return SingletonFormulaParser::getInstance()->termFactory(SYMB_IF_ELSE, { term1, term2, term3 }); }
inline std::shared_ptr<Term> _while_plain(const std::shared_ptr<Term> term1, const std::shared_ptr<Term> term2) { return SingletonFormulaParser::getInstance()->termFactory(SYMB_WHILE, { term1, term2 }); }
inline std::shared_ptr<Term> _whilelim_plain(const std::shared_ptr<Term> term1, const std::shared_ptr<Term> term2, const std::shared_ptr<Term> term3) { return std::make_shared<TermWhileLimited>(term1, term2, term3); }

inline std::shared_ptr<Term> _id_plain(const std::shared_ptr<Term> term1) { return std::make_shared<TermID>(term1); }
inline std::shared_ptr<Term> _boolify_plain(const std::shared_ptr<Term> term1) { return SingletonFormulaParser::getInstance()->termFactory(SYMB_BOOLIFY, term1); }

inline std::shared_ptr<TermMetaCompound> _meta_compound_plain(const std::shared_ptr<Term> term1) { return std::make_shared<TermMetaCompound>(term1)->setChildrensFathers()->toMetaCompoundIfApplicable(); }
inline std::shared_ptr<TermMetaCompound> _meta_compound_plain(const int term1) { return std::make_shared<TermMetaCompound>(_val_plain(term1))->setChildrensFathers()->toMetaCompoundIfApplicable(); }
inline std::shared_ptr<TermMetaSimplification> _meta_simplification_plain(const std::shared_ptr<Term> term1) { return std::make_shared<TermMetaSimplification>(term1)->setChildrensFathers()->toMetaSimplificationIfApplicable(); }
inline std::shared_ptr<TermMetaSimplification> _meta_simplification_plain(const int term1) { return std::make_shared<TermMetaSimplification>(_val_plain(term1))->setChildrensFathers()->toMetaSimplificationIfApplicable(); }
inline std::shared_ptr<Term> _optional_plain(const std::string& meta, const int replacement) { return std::make_shared<TermOptional>(_var_plain(meta), _val_plain(replacement)); }
inline std::shared_ptr<Term> _optional_plain(const std::shared_ptr<TermMetaSimplification> meta, const int replacement) { return std::make_shared<TermOptional>(meta, _val_plain(replacement)); }
inline std::shared_ptr<Term> _optional_plain(const std::shared_ptr<TermMetaSimplification> meta, const std::shared_ptr<Term> replacement) { return std::make_shared<TermOptional>(meta, replacement); }
inline std::shared_ptr<Term> _anyway_plain(const std::vector<std::shared_ptr<Term>>& terms) { return std::make_shared<TermAnyway>(terms); }
inline std::shared_ptr<Term> _funcRef_plain(const std::shared_ptr<Term> term1) { return std::make_shared<TermFuncRef>(term1); }
inline std::shared_ptr<Term> _funcEval_plain(const std::vector<std::shared_ptr<Term>>& terms) { return std::make_shared<TermFuncEval>(terms); }
inline std::shared_ptr<Term> _lambda_plain(const std::shared_ptr<Term> term1) { return std::make_shared<TermFuncLambda>(term1); }

inline std::shared_ptr<Term> _malloc_plain(const std::shared_ptr<Term> term1) { return std::make_shared<TermMalloc>(term1); }
inline std::shared_ptr<Term> _malloc_plain(const float term1) { return std::make_shared<TermMalloc>(_val_plain(term1)); }
inline std::shared_ptr<Term> _delete_plain(const std::shared_ptr<Term> term1) { return std::make_shared<TermDelete>(term1); }
inline std::shared_ptr<Term> _delete_plain(const float term1) { return std::make_shared<TermDelete>(_val_plain(term1)); }

inline std::shared_ptr<Term> _print_plain(const std::shared_ptr<Term> term1, const std::shared_ptr<Term> term2) { return std::make_shared<TermPrint>(term1, term2); }

// The following functions also set childrens' fathers, guaranteeing a full chain of fathers in the tree, if the given terms' chain is not broken.
inline std::shared_ptr<Term> _val(const float term1) { return _val_plain(term1)->setChildrensFathers(false, false)->toTermIfApplicable(); }
inline std::shared_ptr<Term> _var(const std::string& term1) { return _var_plain(term1)->setChildrensFathers(false, false)->toTermIfApplicable(); }
inline std::shared_ptr<Term> _arr(const std::string& term1, const std::shared_ptr<Term> term2) { return _arr_plain(term1, term2)->setChildrensFathers(false, false)->toTermIfApplicable(); }
inline std::shared_ptr<Term> _arrlen(const std::shared_ptr<Term> term1) { return _arrlen_plain(term1)->setChildrensFathers(false, false)->toTermIfApplicable(); }
inline std::shared_ptr<Term> _arrlen(const std::string& term1) { return _arrlen_plain(_var(SYMB_REF + term1))->setChildrensFathers(false, false)->toTermIfApplicable(); }

inline std::shared_ptr<Term> _not(const std::shared_ptr<Term> term) { return _not_plain(term)->setChildrensFathers(false, false)->toTermIfApplicable(); }
inline std::shared_ptr<Term> _and(const std::shared_ptr<Term> term1, const std::shared_ptr<Term> term2) { return _and_plain(term1, term2)->setChildrensFathers(false, false)->toTermIfApplicable(); }
inline std::shared_ptr<Term> _or(const std::shared_ptr<Term> term1, const std::shared_ptr<Term> term2) { return _or_plain(term1, term2)->setChildrensFathers(false, false)->toTermIfApplicable(); }
inline std::shared_ptr<Term> _sm(const std::shared_ptr<Term> term1, const std::shared_ptr<Term> term2) { return _sm_plain(term1, term2)->setChildrensFathers(false, false)->toTermIfApplicable(); }
inline std::shared_ptr<Term> _gr(const std::shared_ptr<Term> term1, const std::shared_ptr<Term> term2) { return _gr_plain(term1, term2)->setChildrensFathers(false, false)->toTermIfApplicable(); }
inline std::shared_ptr<Term> _smeq(const std::shared_ptr<Term> term1, const std::shared_ptr<Term> term2) { return _smeq_plain(term1, term2)->setChildrensFathers(false, false)->toTermIfApplicable(); }
inline std::shared_ptr<Term> _greq(const std::shared_ptr<Term> term1, const std::shared_ptr<Term> term2) { return _greq_plain(term1, term2)->setChildrensFathers(false, false)->toTermIfApplicable(); }
inline std::shared_ptr<Term> _eq(const std::shared_ptr<Term> term1, const std::shared_ptr<Term> term2) { return _eq_plain(term1, term2)->setChildrensFathers(false, false)->toTermIfApplicable(); }
inline std::shared_ptr<Term> _neq(const std::shared_ptr<Term> term1, const std::shared_ptr<Term> term2) { return _neq_plain(term1, term2)->setChildrensFathers(false, false)->toTermIfApplicable(); }
inline std::shared_ptr<Term> _approx(const std::shared_ptr<Term> term1, const std::shared_ptr<Term> term2) { return _approx_plain(term1, term2)->setChildrensFathers(false, false)->toTermIfApplicable(); }
inline std::shared_ptr<Term> _napprox(const std::shared_ptr<Term> term1, const std::shared_ptr<Term> term2) { return _napprox_plain(term1, term2)->setChildrensFathers(false, false)->toTermIfApplicable(); }

inline std::shared_ptr<Term> _plus(const std::shared_ptr<Term> term1, const std::shared_ptr<Term> term2) { return _plus_plain(term1, term2)->setChildrensFathers(false, false)->toTermIfApplicable(); }
inline std::shared_ptr<Term> _minus(const std::shared_ptr<Term> term1, const std::shared_ptr<Term> term2) { return _minus_plain(term1, term2)->setChildrensFathers(false, false)->toTermIfApplicable(); }
inline std::shared_ptr<Term> _mult(const std::shared_ptr<Term> term1, const std::shared_ptr<Term> term2) { return _mult_plain(term1, term2)->setChildrensFathers(false, false)->toTermIfApplicable(); }
inline std::shared_ptr<Term> _div(const std::shared_ptr<Term> term1, const std::shared_ptr<Term> term2) { return _div_plain(term1, term2)->setChildrensFathers(false, false)->toTermIfApplicable(); }
inline std::shared_ptr<Term> _pow(const std::shared_ptr<Term> term1, const std::shared_ptr<Term> term2) { return _pow_plain(term1, term2)->setChildrensFathers(false, false)->toTermIfApplicable(); }
inline std::shared_ptr<Term> _ln(const std::shared_ptr<Term> term1) { return _ln_plain(term1)->setChildrensFathers(false, false)->toTermIfApplicable(); }
inline std::shared_ptr<Term> _sin(const std::shared_ptr<Term> term1) { return _sin_plain(term1)->setChildrensFathers(false, false)->toTermIfApplicable(); }
inline std::shared_ptr<Term> _cos(const std::shared_ptr<Term> term1) { return _cos_plain(term1)->setChildrensFathers(false, false)->toTermIfApplicable(); }
inline std::shared_ptr<Term> _tan(const std::shared_ptr<Term> term1) { return _tan_plain(term1)->setChildrensFathers(false, false)->toTermIfApplicable(); }
inline std::shared_ptr<Term> _asin(const std::shared_ptr<Term> term1) { return _asin_plain(term1)->setChildrensFathers(false, false)->toTermIfApplicable(); }
inline std::shared_ptr<Term> _acos(const std::shared_ptr<Term> term1) { return _acos_plain(term1)->setChildrensFathers(false, false)->toTermIfApplicable(); }
inline std::shared_ptr<Term> _atan(const std::shared_ptr<Term> term1) { return _atan_plain(term1)->setChildrensFathers(false, false)->toTermIfApplicable(); }

inline std::shared_ptr<Term> _neg(const std::shared_ptr<Term> term) { return _neg_plain(term)->setChildrensFathers(false, false)->toTermIfApplicable(); }
inline std::shared_ptr<Term> _neg_one_minus(const std::shared_ptr<Term> term) { return _neg_one_minus_plain(term)->setChildrensFathers(false, false)->toTermIfApplicable(); }
inline std::shared_ptr<Term> _mod(const std::shared_ptr<Term> term1, const std::shared_ptr<Term> term2) { return _mod_plain(term1, term2)->setChildrensFathers(false, false)->toTermIfApplicable(); }
inline std::shared_ptr<Term> _max(const std::shared_ptr<Term> term1, const std::shared_ptr<Term> term2) { return _max_plain(term1, term2)->setChildrensFathers(false, false)->toTermIfApplicable(); }
inline std::shared_ptr<Term> _min(const std::shared_ptr<Term> term1, const std::shared_ptr<Term> term2) { return _min_plain(term1, term2)->setChildrensFathers(false, false)->toTermIfApplicable(); }

inline std::shared_ptr<Term> _abs(const std::shared_ptr<Term> term1) { return _abs_plain(term1)->setChildrensFathers(false, false)->toTermIfApplicable(); }
inline std::shared_ptr<Term> _trunc(const std::shared_ptr<Term> term1) { return _trunc_plain(term1)->setChildrensFathers(false, false)->toTermIfApplicable(); }
inline std::shared_ptr<Term> _sqrt(const std::shared_ptr<Term> term1) { return _sqrt_plain(term1)->setChildrensFathers(false, false)->toTermIfApplicable(); }
inline std::shared_ptr<Term> _rsqrt(const std::shared_ptr<Term> term1) { return _rsqrt_plain(term1)->setChildrensFathers(false, false)->toTermIfApplicable(); }

inline std::shared_ptr<Term> _rand(const std::shared_ptr<Term> term1) { return _rand_plain(term1)->setChildrensFathers(false, false)->toTermIfApplicable(); }

inline std::shared_ptr<Term> _set(const std::string& term1, const std::string& term2) { return _set_plain(term1, term2)->setChildrensFathers(false, false)->toTermIfApplicable(); }
inline std::shared_ptr<Term> _set(const std::string& term1, const std::shared_ptr<Term> term2) { return _set_plain(term1, term2)->setChildrensFathers(false, false)->toTermIfApplicable(); }
inline std::shared_ptr<Term> _set_alt(const std::shared_ptr<Term> term1, const std::shared_ptr<Term> term2) { return _set_alt_plain(term1, term2)->setChildrensFathers(false, false)->toTermIfApplicable(); }
inline std::shared_ptr<Term> _set(const std::shared_ptr<Term> term1, const std::shared_ptr<Term> term2) { return _set_plain(term1, term2)->setChildrensFathers(false, false)->toTermIfApplicable(); }
inline std::shared_ptr<Term> _Set(const std::string& term1, const std::shared_ptr<Term> term2, const std::shared_ptr<Term> term3, const std::shared_ptr<DataPack> d) { return _Set_plain(term1, term2, term3, d)->setChildrensFathers(false, false)->toTermIfApplicable(); }
inline std::shared_ptr<Term> _Set(const std::shared_ptr<Term> term1, const std::shared_ptr<Term> term2, const std::shared_ptr<Term> term3) { return _Set_plain(term1, term2, term3)->setChildrensFathers(false, false)->toTermIfApplicable(); }
inline std::shared_ptr<Term> _get(const std::string& term1) { return _get_plain(term1)->setChildrensFathers(false, false)->toTermIfApplicable(); }
inline std::shared_ptr<Term> _get(const std::shared_ptr<Term> term1) { return _get_plain(term1)->setChildrensFathers(false, false)->toTermIfApplicable(); }
inline std::shared_ptr<Term> _Get(const std::string& term1, const std::shared_ptr<Term> term2, const std::shared_ptr<DataPack> d) { return _Get_plain(term1, term2, d)->setChildrensFathers(false, false)->toTermIfApplicable(); }
inline std::shared_ptr<Term> _Get(const std::shared_ptr<Term> term1, const std::shared_ptr<Term> term2) { return _Get_plain(term1, term2)->setChildrensFathers(false, false)->toTermIfApplicable(); }

inline std::shared_ptr<Term> _seq(const std::shared_ptr<Term> term1, const std::shared_ptr<Term> term2) { return _seq_plain(term1, term2)->setChildrensFathers(false, false)->toTermIfApplicable(); }
inline std::shared_ptr<Term> _if(const std::shared_ptr<Term> term1, const std::shared_ptr<Term> term2) { return _if_plain(term1, term2)->setChildrensFathers(false, false)->toTermIfApplicable(); }
inline std::shared_ptr<Term> _ifelse(const std::shared_ptr<Term> term1, const std::shared_ptr<Term> term2, const std::shared_ptr<Term> term3) { return _ifelse_plain(term1, term2, term3)->setChildrensFathers(false, false)->toTermIfApplicable(); }
inline std::shared_ptr<Term> _while(const std::shared_ptr<Term> term1, const std::shared_ptr<Term> term2) { return _while_plain(term1, term2)->setChildrensFathers(false, false)->toTermIfApplicable(); }
inline std::shared_ptr<Term> _whilelim(const std::shared_ptr<Term> term1, const std::shared_ptr<Term> term2, const std::shared_ptr<Term> term3) { return _whilelim_plain(term1, term2, term3)->setChildrensFathers(false, false)->toTermIfApplicable(); }

inline std::shared_ptr<Term> _id(const std::shared_ptr<Term> term1) { return _id_plain(term1)->setChildrensFathers(false, false)->toTermIfApplicable(); }
inline std::shared_ptr<Term> _boolify(const std::shared_ptr<Term> term1) { return _boolify_plain(term1)->setChildrensFathers(false, false)->toTermIfApplicable(); }

inline std::shared_ptr<Term> _ltl_until_plain(const std::shared_ptr<Term> term1, const std::shared_ptr<Term> term2) { return SingletonFormulaParser::getInstance()->termFactory(SYMB_LTL_UNTIL, { term1, term2 }); }
inline std::shared_ptr<Term> _ltl_released_plain(const std::shared_ptr<Term> term1, const std::shared_ptr<Term> term2) { return SingletonFormulaParser::getInstance()->termFactory(SYMB_LTL_RELEASED, { term1, term2 }); }
inline std::shared_ptr<Term> _ltl_next_plain(const std::shared_ptr<Term> term1) { return SingletonFormulaParser::getInstance()->termFactory(SYMB_LTL_NEXT, { term1 }); }
inline std::shared_ptr<Term> _ltl_globally_plain(const std::shared_ptr<Term> term1) { return SingletonFormulaParser::getInstance()->termFactory(SYMB_LTL_GLOBALLY, { term1 }); }
inline std::shared_ptr<Term> _ltl_finally_plain(const std::shared_ptr<Term> term1) { return SingletonFormulaParser::getInstance()->termFactory(SYMB_LTL_FINALLY, { term1 }); }
inline std::shared_ptr<Term> _ltl_xor_plain(const std::shared_ptr<Term> term1, const std::shared_ptr<Term> term2) { return SingletonFormulaParser::getInstance()->termFactory(SYMB_LTL_XOR, { term1, term2 }); }
inline std::shared_ptr<Term> _ltl_xnor_plain(const std::shared_ptr<Term> term1, const std::shared_ptr<Term> term2) { return SingletonFormulaParser::getInstance()->termFactory(SYMB_LTL_XNOR, { term1, term2 }); }
inline std::shared_ptr<Term> _ltl_implication_plain(const std::shared_ptr<Term> term1, const std::shared_ptr<Term> term2) { return SingletonFormulaParser::getInstance()->termFactory(SYMB_LTL_IMPLICATION, { term1, term2 }); }
inline std::shared_ptr<Term> _ltl_iff_plain(const std::shared_ptr<Term> term1, const std::shared_ptr<Term> term2) { return SingletonFormulaParser::getInstance()->termFactory(SYMB_LTL_IFF, { term1, term2 }); }
inline std::shared_ptr<Term> _ltl_until(const std::shared_ptr<Term> term1, const std::shared_ptr<Term> term2) { return _ltl_until_plain(term1, term2)->setChildrensFathers(false, false)->toTermIfApplicable(); }
inline std::shared_ptr<Term> _ltl_released(const std::shared_ptr<Term> term1, const std::shared_ptr<Term> term2) { return _ltl_released_plain(term1, term2)->setChildrensFathers(false, false)->toTermIfApplicable(); }
inline std::shared_ptr<Term> _ltl_next(const std::shared_ptr<Term> term1) { return _ltl_next_plain(term1)->setChildrensFathers(false, false)->toTermIfApplicable(); }
inline std::shared_ptr<Term> _ltl_globally(const std::shared_ptr<Term> term1) { return _ltl_globally_plain(term1)->setChildrensFathers(false, false)->toTermIfApplicable(); }
inline std::shared_ptr<Term> _ltl_finally(const std::shared_ptr<Term> term1) { return _ltl_finally_plain(term1)->setChildrensFathers(false, false)->toTermIfApplicable(); }
inline std::shared_ptr<Term> _ltl_xor(const std::shared_ptr<Term> term1, const std::shared_ptr<Term> term2) { return _ltl_xor_plain(term1, term2)->setChildrensFathers(false, false)->toTermIfApplicable(); }
inline std::shared_ptr<Term> _ltl_xnor(const std::shared_ptr<Term> term1, const std::shared_ptr<Term> term2) { return _ltl_xnor_plain(term1, term2)->setChildrensFathers(false, false)->toTermIfApplicable(); }
inline std::shared_ptr<Term> _ltl_implication(const std::shared_ptr<Term> term1, const std::shared_ptr<Term> term2) { return _ltl_implication_plain(term1, term2)->setChildrensFathers(false, false)->toTermIfApplicable(); }
inline std::shared_ptr<Term> _ltl_iff(const std::shared_ptr<Term> term1, const std::shared_ptr<Term> term2) { return _ltl_iff_plain(term1, term2)->setChildrensFathers(false, false)->toTermIfApplicable(); }

inline std::shared_ptr<TermMetaCompound> _meta_compound(const std::shared_ptr<Term> term1) { return _meta_compound_plain(term1)->setChildrensFathers()->toMetaCompoundIfApplicable(); }
inline std::shared_ptr<TermMetaCompound> _meta_compound(const int term1) { return _meta_compound_plain(term1)->setChildrensFathers()->toMetaCompoundIfApplicable(); }
inline std::shared_ptr<TermMetaSimplification> _meta_simplification(const std::shared_ptr<Term> term1) { return _meta_simplification_plain(term1)->setChildrensFathers()->toMetaSimplificationIfApplicable(); }
inline std::shared_ptr<TermMetaSimplification> _meta_simplification(const int term1) { return _meta_simplification_plain(term1)->setChildrensFathers()->toMetaSimplificationIfApplicable(); }
inline std::shared_ptr<Term> _optional(const std::string& meta, const int replacement) { return _optional_plain(meta, replacement)->setChildrensFathers(false, false)->toTermIfApplicable(); }
inline std::shared_ptr<Term> _optional(const std::shared_ptr<TermMetaSimplification> meta, const int replacement) { return _optional_plain(meta, replacement)->setChildrensFathers(false, false)->toTermIfApplicable(); }
inline std::shared_ptr<Term> _optional(const std::shared_ptr<TermMetaSimplification> meta, const std::shared_ptr<Term> replacement) { return _optional_plain(meta, replacement)->setChildrensFathers(false, false)->toTermIfApplicable(); }
inline std::shared_ptr<Term> _anyway(const std::vector<std::shared_ptr<Term>>& terms) { return _anyway_plain(terms)->setChildrensFathers(false, false)->toTermIfApplicable(); }
inline std::shared_ptr<Term> _anyway(const std::shared_ptr<Term> id, const std::shared_ptr<Term> t1, const std::shared_ptr<Term> t2) { return _anyway(std::vector<std::shared_ptr<Term>>({ id, t1, t2 }))->setChildrensFathers(false, false)->toTermIfApplicable(); }
inline std::shared_ptr<Term> _funcRef(const std::shared_ptr<Term> term1) { return _funcRef_plain(term1)->setChildrensFathers(false, false)->toTermIfApplicable(); }
inline std::shared_ptr<Term> _funcEval(const std::vector<std::shared_ptr<Term>>& terms) { return _funcEval_plain(terms)->setChildrensFathers(false, false)->toTermIfApplicable(); }
inline std::shared_ptr<Term> _lambda(const std::shared_ptr<Term> term1) { return _lambda_plain(term1)->setChildrensFathers(false, false)->toTermIfApplicable(); }

inline std::shared_ptr<Term> _malloc(const std::shared_ptr<Term> term1) { return _malloc_plain(term1)->setChildrensFathers(false, false)->toTermIfApplicable(); }
inline std::shared_ptr<Term> _malloc(const float term1) { return _malloc_plain(term1)->setChildrensFathers(false, false)->toTermIfApplicable(); }
inline std::shared_ptr<Term> _delete(const std::shared_ptr<Term> term1) { return _delete_plain(term1)->setChildrensFathers(false, false)->toTermIfApplicable(); }
inline std::shared_ptr<Term> _delete(const float term1) { return _delete_plain(term1)->setChildrensFathers(false, false)->toTermIfApplicable(); }

inline std::shared_ptr<Term> _print(const std::shared_ptr<Term> term1, const std::shared_ptr<Term> term2) { return std::make_shared<TermPrint>(term1, term2)->setChildrensFathers(false, false)->toTermIfApplicable(); }

inline std::shared_ptr<Term> _false() { return _val(0); }
inline std::shared_ptr<Term> _val0() { return _val(0); }
inline std::shared_ptr<Term> _true() { return _val(1); }
inline std::shared_ptr<Term> _val1() { return _val(1); }
inline std::shared_ptr<Term> _vara() { return _var("a"); }
inline std::shared_ptr<Term> _varb() { return _var("b"); }
inline std::shared_ptr<Term> _varc() { return _var("c"); }
inline std::shared_ptr<Term> _vard() { return _var("d"); }
inline std::shared_ptr<Term> _vare() { return _var("e"); }
inline std::shared_ptr<Term> _varf() { return _var("f"); }

inline std::shared_ptr<Term> _fctin() { return std::make_shared<TermFctIn>(); }

inline std::shared_ptr<Equation> _equation(const std::shared_ptr<Term> term1, const std::shared_ptr<Term> term2) { return std::make_shared<Equation>(term1, term2)->setChildrensFathers()->toEquationIfApplicable(); }

// *** EO Convenience functions for in-code Term construction. ***

inline static const std::map<std::string, std::string> _FUNCTION_NAMES{
   { SYMB_FOR, "_not_implemented" },
   { SYMB_FOR_C, "_not_implemented" },
   { SYMB_PRINT, "_print" },
   { SYMB_IF, "_if" },
   { SYMB_IF_ELSE, "_ifelse" },
   { SYMB_WHILE, "_while" },
   { SYMB_WHILELIM, "_whilelim" },
   { SYMB_FCTIN, "_not_implemented" },
   { SYMB_SQRT, "_sqrt" },
   { SYMB_RSQRT, "_rsqrt" },
   { SYMB_NOT, "_not" },
   { SYMB_BOOLIFY, "_boolify" },
   { SYMB_ID, "_id" },
   { SYMB_RAND, "_rand" },
   { SYMB_NEG, "_neg" },
   //{ SYMB_NEG_ALT, "_neg_one_minus" }, // Cannot do that on overloaded operators.
   { SYMB_TRUNC, "_trunc" },
   { SYMB_ABS, "_abs" },
   { SYMB_LENGTH, "_arrlen" },
   { SYMB_REF, "_not_implemented" },
   { SYMB_FUNC_REF, "_funcRef" },
   { SYMB_FUNC_EVAL, "_funcEval" },
   { SYMB_LITERAL, "_not_implemented" },
   { SYMB_LAMBDA, "_lambda" },
   { SYMB_SET_VAR, "_set" },
   { SYMB_SET_ARR, "_Set" },
   { SYMB_GET_VAR, "_get" },
   { SYMB_GET_ARR, "_Get" },
   { SYMB_MALLOC, "_malloc" },
   { SYMB_DELETE, "_delete" },
   { SYMB_META_CMP, "_meta_compound" },
   { SYMB_META_SIMP, "_meta_simplification" },
   { SYMB_OPTIONAL, "_optional" },
   { SYMB_ANYWAY, "_anyway" },
   { SYMB_SEQUENCE, "_seq" },
   { SYMB_EQUATION, "_equation" },
   { SYMB_SET_VAR_A, "_set_alt" },
   { SYMB_OR, "_or" },
   { SYMB_AND, "_and" },
   { SYMB_EQ, "_eq" },
   { SYMB_APPROX, "_approx" },
   { SYMB_NAPPROX, "_napprox" },
   { SYMB_NEQ, "_neq" },
   { SYMB_SMEQ, "_smeq" },
   { SYMB_SM, "_sm" },
   { SYMB_GREQ, "_greq" },
   { SYMB_GR, "_gr" },
   { SYMB_PLUS, "_plus" },
   { SYMB_MINUS, "_minus" },
   { SYMB_MULT, "_mult" },
   { SYMB_DIV, "_div" },
   { SYMB_POW, "_pow" },
   { SYMB_LN, "_ln" },
   { SYMB_SIN, "_sin" },
   { SYMB_COS, "_cos" },
   { SYMB_TAN, "_tan" },
   { SYMB_ARCSIN, "_asin" },
   { SYMB_ARCCOS, "_acos" },
   { SYMB_ARCTAN, "_atan" },
   { SYMB_MOD, "_mod" },
   { SYMB_MIN, "_min" },
   { SYMB_MAX, "_max" },
   { SYMB_LTL_UNTIL,    "_ltl_until" },
   { SYMB_LTL_RELEASED, "_ltl_released" },
   { SYMB_LTL_NEXT,     "_ltl_next" },
   { SYMB_LTL_GLOBALLY, "_ltl_globally" },
   { SYMB_LTL_FINALLY,  "_ltl_finally" },
   { SYMB_LTL_XOR,  "_ltl_xor" },
   { SYMB_LTL_XNOR,  "_ltl_xnor" },
   { SYMB_LTL_IMPLICATION,  "_ltl_implication" },
   { SYMB_LTL_IFF,  "_ltl_iff" },
};

inline std::string getTrailForMetaRuleCodeGeneration(const std::vector<int>& trail_orig) // Copy from static_helper
{
   std::string trail{};

   for (const auto& el : trail_orig) {
      trail += "_" + std::to_string(el);
   }

   return trail;
}

inline std::string _getCodeFromFormula(
   const TermPtr formula, 
   const bool use_plain_functions, 
   const std::map<int, std::vector<std::pair<std::string, std::vector<int>>>>& anyway_mapping,
   const std::string& parser_ptr_name,
   std::set<std::string>& already_used,
   const bool allow_set_childrens_fathers)
{
   if (formula->isTermCompound()) {
      return _getCodeFromFormula(formula->getOperands()[0], use_plain_functions, anyway_mapping, parser_ptr_name, already_used, allow_set_childrens_fathers);
   }

   std::string s{};
   std::string plain_denoter{ use_plain_functions ? "_plain" : "" };

   if (_FUNCTION_NAMES.count(formula->getOptor())) {
      std::string comma{ "" };
      int begin{ 0 };

      if (formula->toTermAnywayIfApplicable()) {
         auto anyway_info{ anyway_mapping.at((int)formula->getOperands()[0]->constEval()).front() };
         std::string anyway_name{ anyway_info.first + getTrailForMetaRuleCodeGeneration(anyway_info.second) };
         begin = 1;
         s += parser_ptr_name + "->termFactory(" + anyway_name + "->getOptorJumpIntoCompound(), { ";
      } else {
         s += _FUNCTION_NAMES.at(formula->getOptor()) + plain_denoter + "(";
      }

      for (int i = begin; i < formula->getOperands().size(); i++) {
         s += comma + _getCodeFromFormula(formula->getOperands()[i], use_plain_functions, anyway_mapping, parser_ptr_name, already_used, allow_set_childrens_fathers);
         comma = ", ";
      }

      if (formula->toTermAnywayIfApplicable()) {
         s += " })" + std::string(allow_set_childrens_fathers ? "->setChildrensFathers(false, false)" : "") + "->toTermIfApplicable()";
      }
      else {
         s += ")";
      }
   }
   else if (formula->isTermVar()) {
      std::string optor{ formula->getOptor() };
      s += optor;

      if (already_used.count(optor)) {
         s += "->copy()";
      }

      already_used.insert(optor);
   }
   else if (formula->isTermVal()) {
      s += "_val" + plain_denoter + "(" + std::to_string(formula->constEval()) + ")";
   }

   return s;
}

inline std::string _getCodeFromFormula(
   const TermPtr formula, 
   const bool use_plain_functions, 
   const std::map<int, std::vector<std::pair<std::string, std::vector<int>>>>& anyway_mapping, 
   const std::string& parser_ptr_name,
   const bool allow_set_childrens_fathers) {
   std::set<std::string> already_used_dummy;
   return _getCodeFromFormula(formula, use_plain_functions, anyway_mapping, parser_ptr_name, already_used_dummy, allow_set_childrens_fathers);
}

} // vfm
