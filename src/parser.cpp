//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file

//#define FORMULA_PARSER_DEBUG

#include "parser.h"
#include "math_struct.h"
#include "term_compound.h"
#include "static_helper.h"
#include "operator_structure.h"
#include "term.h"
#include "fsm.h"
#include "model_checking/simplification.h"
#include "simplification/simplification.h"
#include <exception>
#include <algorithm>
#include <stack>
#include <deque>

#ifndef _WIN32
#include <dlfcn.h>
#endif

using namespace vfm;

FormulaParser::FormulaParser() : Parsable("FormulaParser")
{
    init();
}

void FormulaParser::addDefaultDynamicTerms()
{
   //requestFlexibleNumberOfArgumentsFor(SYMB_IF);

   OperatorStructure questionmark_str(OutputFormatEnum::infix, AssociativityTypeEnum::left, OPNUM_COND_QUES, PRIO_COND_QUES, SYMB_COND_QUES, false, false, AdditionalOptionEnum::always_print_brackets);
   OperatorStructure colon_str(OutputFormatEnum::infix, AssociativityTypeEnum::left, OPNUM_COND_COL, PRIO_COND_COL, SYMB_COND_COL, false, false, AdditionalOptionEnum::always_print_brackets);
   OperatorStructure approx_str(OutputFormatEnum::infix, AssociativityTypeEnum::left, 2, PRIO_APPROX, SYMB_APPROX, false, true, AdditionalOptionEnum::always_print_brackets);
   OperatorStructure napprox_str(OutputFormatEnum::infix, AssociativityTypeEnum::left, 2, PRIO_NAPPROX, SYMB_NAPPROX, false, true, AdditionalOptionEnum::always_print_brackets);
   OperatorStructure sequence_str(OutputFormatEnum::infix, AssociativityTypeEnum::left, 2, PRIO_SEQUENCE, SYMB_SEQUENCE, true, false);
   OperatorStructure negation_str(OutputFormatEnum::prefix, AssociativityTypeEnum::left, OPNUM_NEG_ALT, PRIO_NEG_ALT, SYMB_NEG_ALT, true, false);
   OperatorStructure set_str(OutputFormatEnum::infix, AssociativityTypeEnum::left, 2, PRIO_SET_VAR_A, SYMB_SET_VAR_A, false, false);
   OperatorStructure getk_str(OutputFormatEnum::prefix, AssociativityTypeEnum::left, 3, -1, "Getk", false, false);
   OperatorStructure setk_str(OutputFormatEnum::prefix, AssociativityTypeEnum::left, 4, -1, "Setk", false, false); // Not strictly necessary, since no recursive call.
   OperatorStructure arrayk_str(OutputFormatEnum::prefix, AssociativityTypeEnum::left, 2, -1, "arrayk", false, false);

   addDynamicTerm(set_str, _set(_meta_compound(0), _meta_compound(1)), true);              // @x = y := set(@x, y)

   addDynamicTerm(_eq(_meta_compound(0), _val(0)), SYMB_NOT, true);                        // !()    := p_(0) == 0

   addDynamicTerm(_neq(_meta_compound(0), _val(0)), SYMB_BOOLIFY, true);                   // bool()   := p_(0) != 0
   
   addDynamicTerm(_meta_compound(0), "return", true);                                      // return() := p_(0)

   addDynamicTerm(                                                                // if()     := whilelim(p_(0), p_(1), 1)
      _whilelim(_meta_compound(0), _meta_compound(1), _val(1)),
      SYMB_IF, 
      true, 
      -1, 
      -1, 
      OutputFormatEnum::braces_from_second_argument);

   addDynamicTerm(negation_str, _neg(_meta_compound(0)), true); // -x := --x

   addDynamicTerm(sequence_str, _plus(_mult(_meta_compound(0), _val(0)), _meta_compound(1)), true); // ;()     := p_(0) * 0 + p_(1)

   addDynamicTerm(MathStruct::parseMathStruct(                                    // ifelse() := ...
         "whilelim(p_(0), (set(@_res, p_(1)); set(@_dont, 1)), 1); whilelim(!(p_(0)), set(@_res, p_(2)), !(_dont)); _res",
         *this, false)->toTermIfApplicable(),
      SYMB_IF_ELSE, 
      true, -1, -1, OutputFormatEnum::braces_from_second_argument);

   addDynamicTerm(approx_str, _smeq(_abs(_minus(_meta_compound(0), _meta_compound(1))), _val(5)), true); // approx() := abs(p_(0)-p_(1)) <= 5

   addDynamicTerm(napprox_str, _not(_approx(_meta_compound(0), _meta_compound(1))), true);               // napprox() := !(p_(0) === p_(1))

   addDynamicTerm(_Get(_meta_compound(0), _val(0)), SYMB_GET_VAR, true);                        // get() := Get(p_(0), 0)

   addDynamicTerm(                                                                     // while()
      _whilelim(_meta_compound(0), _meta_compound(1), _val(MAX_ITERATIONS_FOR_WHILE_LOOP)),
      SYMB_WHILE, 
      true, 
      -1, 
      -1, 
      OutputFormatEnum::braces_from_second_argument);

   addDynamicTerm(MathStruct::parseMathStruct(
      "@_new_arr = malloc(p_(1)); set(@_i, --(1)); while((set(@_i, _i + 1) < p_(1)), Set(_new_arr, _i, Get(p_(0), _i))); _new_arr", 
      *this, false)->toTermIfApplicable(), "copy", true);

   addDynamicTerm(_seq(_print(_meta_compound(0), _meta_compound(1)), _delete(_meta_compound(1))), "print", true); // print() := print_plain(p_(0), p_(1)) ; free(p_(1))

   addDynamicTerm(_seq(_print(_meta_compound(0), _neg(_val(1))), _meta_compound(0)), "printf", true);    // printf() := print_plain(p_(0), -1) ; p_(0)

   addDynamicTerm(MathStruct::parseMathStruct(                                   // println() := ...
         "set(@ncInt__, 10); print(p_(0), p_(1)); print_plain(@ncInt__, 1)",
         *this, false)->toTermIfApplicable(), "println", true);

   addDynamicTerm(MathStruct::parseMathStruct(                                   // printfln() := ...
         "println(p_(0), --(1)); p_(0)",
         *this, false)->toTermIfApplicable(), "printfln", true);

   addDynamicTerm(MathStruct::parseMathStruct(                                   // printErr() := ...
         "print(\"VFM ERROR: \"); print(p_(0), p_(1)); println(\".\")",
         *this, false)->toTermIfApplicable(), "printErr", true);

   forwardDeclareDynamicTerm(arrayk_str); // Instantiate array in k dimensions. Fwd. decl. needed due to recursive call in function.
   addDynamicTerm(MathStruct::parseMathStruct("\
      ifelse (p_(1) < 1) {\
         printErr(\"Cannot instantiate array of dimension smaller than 1\") ; --(1)\
      }\
      else {\
         @_arr = malloc(get(p_(0)));\
         \
         if (p_(1) > 1) {\
            set(@_i, --(1));\
            while ((@_i = _i + 1) < get(p_(0))) {\
               _arr + _i = arrayk((p_(0) + 1), (p_(1) - 1))\
            }\
         };\
         _arr\
      }\
      ",
      *this, false)->toTermIfApplicable(), arrayk_str.op_name, true);

   forwardDeclareDynamicTerm(getk_str); // Needed due to recursive call in function.
   addDynamicTerm(MathStruct::parseMathStruct( // Get value of array in k dimensions. Params: array-ref, dims-ref, dims-size.
      "\
       ifelse (p_(2) == 1) {\
          Get(p_(0), get(p_(1)))\
       }\
       else {\
          Get(Getk(p_(0), p_(1), (p_(2) - 1)), get(p_(1) + p_(2) - 1))\
       }\
       ",
      *this, false)->toTermIfApplicable(), getk_str.op_name, true); // Along the lines of: Getk(arr, dim, dim_num) := Get(Get(Get(arr, 2), 3), 4)

   addDynamicTerm(MathStruct::parseMathStruct( // Set value of array in k dimensions. Params: array-ref, dims-ref, dims-size, value-to-set
      "\
       ifelse (p_(2) == 1) {\
          Set(p_(0), get(p_(1)), p_(3))\
       }\
       else {\
          Set(Getk(p_(0), p_(1), (p_(2) - 1)), get(p_(1) + p_(2) - 1), p_(3))\
       }\
       ",
      *this, false)->toTermIfApplicable(), setk_str.op_name, true); // Along the lines of: Setk(arr, dim, dim_num, value) := Set(Get(Get(arr, 2), 3), 4, 7.7)

   addDynamicTerm(MathStruct::parseMathStruct(                                   // instantiate array in 1 dimension (just malloc).
      "malloc(p_(0))",
      *this, false)->toTermIfApplicable(), "array", true);
   
   addDynamicTerm(MathStruct::parseMathStruct(                                   // instantiate array in 2 dimensions...
      "set(@_arr, malloc(p_(0)));\
       set(@_i, (_arr - 1));\
       while((set(@_i, _i + 1) < (_arr + p_(0))), (set(_i, malloc(p_(1)))));\
       _arr",
      *this, false)->toTermIfApplicable(), "array2", true);
   
   addDynamicTerm(MathStruct::parseMathStruct(                                   // instantiate array in 3 dimensions...
      "set(@_arr, malloc(p_(0)));\
       set(@_i, (_arr - 1));\
       while((set(@_i, _i + 1) < (_arr + p_(0))), (set(_i, malloc(p_(1)))); \
             set(@_col, get(_i));\
             set(@_j, (_col - 1));\
             while((set(@_j, _j + 1) < (_col + p_(1))), (set(_j, malloc(p_(2))))));\
       _arr",
      *this, false)->toTermIfApplicable(), "array3", true);

   addDynamicTerm(MathStruct::parseMathStruct(                                   // get value from array in 2 dimensions...
      "Get(Get(p_(0), p_(1)), p_(2))",
      *this, false)->toTermIfApplicable(), "Get2", true);
   
   addDynamicTerm(MathStruct::parseMathStruct(                                   // get value from array in 3 dimensions...
      "Get(Get2(p_(0), p_(1), p_(2)), p_(3))",
      *this, false)->toTermIfApplicable(), "Get3", true);
   
   addDynamicTerm(MathStruct::parseMathStruct(                                   // set value of array in 2 dimensions...
      "Set(Get(p_(0), p_(1)), p_(2), p_(3))",
      *this, false)->toTermIfApplicable(), "Set2", true);
   
   addDynamicTerm(MathStruct::parseMathStruct(                                   // set value of array in 3 dimensions...
      "Set(Get2(p_(0), p_(1), p_(2)), p_(3), p_(4))",
      *this, false)->toTermIfApplicable(), "Set3", true);
   
   addDynamicTerm(MathStruct::parseMathStruct(                                   // print array in 2 dimensions...
      "set(@wsp__, 32) ;\
         set(@_i, --(1)) ;\
         while((set(@_i, _i + 1) < p_(1)),\
               (set(@_j, --(1)) ;\
               while((set(@_j, _j + 1) < p_(2)),\
                     printf(Get2(p_(0), _i, _j)) ; \
                     print_plain(@wsp__, 1)) ; \
               println(\"\")))",
      *this, false)->toTermIfApplicable(), "print2", true);

   addDynamicTerm(MathStruct::parseMathStruct(                                   // set all values of array in 3 dimensions...
      "@_i = --(1) ;\
       while((@_i = _i + 1) < p_(2)) {\
          @_j = --(1) ;\
          while((@_j = _j + 1) < p_(3)) {\
             @_k = --(1) ;\
             while((@_k = _k + 1) < p_(4)) {\
                Set3(p_(0), _i, _j, _k, func(p_(1), _i, _j, _k))\
             }\
          }\
       }",
      *this, false)->toTermIfApplicable(), "fillArray3", true);

   addDynamicTerm(MathStruct::parseMathStruct(                                   // forRange (@i, 0, 10) { *BODY* }
      "p_(0) = p_(1) ;\
       ifelse(p_(1) < p_(2)) {\
          while (get(p_(0)) <= p_(2)) {\
             func(p_(3)) ;\
             p_(0) = get(p_(0)) + 1\
          }\
       } else {\
          while (get(p_(0)) >= p_(2)) {\
             func(p_(3)) ;\
             p_(0) = get(p_(0)) - 1\
          }\
       }",
      *this, false)->toTermIfApplicable(), SYMB_FOR, true);

   addDynamicTerm(MathStruct::parseMathStruct( // C-style for function: for (@i = 0, i < 10, @i = i + 1) { *BODY* }
      "func(p_(0)) ;\
       while (func(p_(1))) {\
          func(p_(3)) ;\
          func(p_(2))\
       }\
      ",
      *this, false)->toTermIfApplicable(), SYMB_FOR_C, true);

   addDynamicTerm( // TODO: p_(0) in first lambda not working, misinterpretated as lambda's argument; and local vars cannot be captured.
      MathStruct::parseMathStruct(
         R"(
         @address_begin = p_(0);
         for (@cnt_var = address_begin, get(cnt_var) != 0, @cnt_var = cnt_var + 1) {
            0
         };
         print(p_(0), cnt_var - p_(0))
         )")->toTermIfApplicable(),
      "Print",
      true); // Print() := print_plain, but stop on '\0'.

   createSimpleAnyways();

   addDynamicTerm(questionmark_str, _div(_plus(_meta_compound(0), _meta_compound(1)), _val(0)), true);
   addDynamicTerm(colon_str, _neg(_div(_plus(_meta_compound(0), _meta_compound(1)), _val(0))), true);

   // TODO: Here come optional or test functions that might get stripped away later (no helper functions).
   addDynamicTerm(MathStruct::parseMathStruct(
         "set(@_i, --(1)) ; p_(2) ; while ((set(@_i, _i + 1) < p_(4)), Set(p_(2), _i, func(p_(3), Get(p_(0), _i), Get(p_(1), _i))))",
         *this, false)->toTermIfApplicable(), 
      "combineArrays", true);

   OperatorStructure fib_str(OutputFormatEnum::prefix, AssociativityTypeEnum::left, 1, -1, "fib", false, false);
   forwardDeclareDynamicTerm(fib_str);
   addDynamicTerm(MathStruct::parseMathStruct("ifelse(((p_(0) < 2)), p_(0), (fib(p_(0) - 1) + fib(p_(0) - 2)))", *this, false)->toTermIfApplicable(), fib_str.op_name, true);

   OperatorStructure q_str(OutputFormatEnum::prefix, AssociativityTypeEnum::left, 1, -1, "q_func", false, false);
   forwardDeclareDynamicTerm(q_str);
   addDynamicTerm(MathStruct::parseMathStruct("ifelse(((p_(0) <= 2)), 1, (q_func(p_(0) - q_func(p_(0) - 1)) + q_func(p_(0) - q_func(p_(0) - 2))))", *this, false)->toTermIfApplicable(), q_str.op_name, true);
   
   addDynamicTerm(MathStruct::parseMathStruct("p_(0) * p_(0)", *this, false)->toTermIfApplicable(), "square", true);
   addDynamicTerm(MathStruct::parseMathStruct(R"(
      p_(0) == 2 
      || p_(0) == 3 
      || (
         @_c = (@_p = 1); while(square(@_c = _c + 1) <= p_(0) && _p) {
            @_p = _p && p_(0) % _c;
         } 
      )
   )", *this, false)->toTermIfApplicable(), "isprime", true); // Basically: p == 2 || p == 3 || p % 2 && p % 3 && p % 4 && ... && p % sqrt(p)

   addnuSMVOperators(TLType::LTL);
   addDotOperator();
}

void FormulaParser::createSimpleAnyways()
{
   std::string symb_any{};
   for (int i = 0; i < CREATE_SIMPLE_ANYWAYS_UP_TO; i++) {
      symb_any += SYMB_ANYWAY_S;
      addDynamicTerm(_val0(), symb_any, true, -2, -1, OutputFormatEnum::prefix);
      //requestFlexibleNumberOfArgumentsFor(symb_any);
   }
}

// --- Register static operators. (Don't forget to update arithmetic, logic or misc. terms in MathStruct.h.) ---
void FormulaParser::init() {
   all_ops_ = {
      { SYMB_PLUS, { { TermPlus::my_struct.arg_num, TermPlus::my_struct } } },
      { SYMB_POW, { { TermPow::my_struct.arg_num, TermPow::my_struct } } },
      { SYMB_LN, { { TermLn::my_struct.arg_num, TermLn::my_struct } } },
      { SYMB_SIN, { { TermSin::my_struct.arg_num, TermSin::my_struct } } },
      { SYMB_COS, { { TermCos::my_struct.arg_num, TermCos::my_struct } } },
      { SYMB_TAN, { { TermTan::my_struct.arg_num, TermTan::my_struct } } },
      { SYMB_ARCSIN, { { TermASin::my_struct.arg_num, TermASin::my_struct } } },
      { SYMB_ARCCOS, { { TermACos::my_struct.arg_num, TermACos::my_struct } } },
      { SYMB_ARCTAN, { { TermATan::my_struct.arg_num, TermATan::my_struct } } },
      { SYMB_MINUS, { { TermMinus::my_struct.arg_num, TermMinus::my_struct } } },
      { SYMB_MULT, { { TermMult::my_struct.arg_num, TermMult::my_struct } } },
      { SYMB_DIV, { { TermDiv::my_struct.arg_num, TermDiv::my_struct } } },
      { SYMB_NEG, { { TermNeg::my_struct.arg_num, TermNeg::my_struct } } },
      { SYMB_MOD, { { TermMod::my_struct.arg_num, TermMod::my_struct } } },
      { SYMB_MAX, { { TermMax::my_struct.arg_num, TermMax::my_struct } } },
      { SYMB_MIN, { { TermMin::my_struct.arg_num, TermMin::my_struct } } },
      { SYMB_AND, { { TermLogicAnd::my_struct.arg_num, TermLogicAnd::my_struct } } },
      { SYMB_OR, { { TermLogicOr::my_struct.arg_num, TermLogicOr::my_struct } } },
      { SYMB_SM, { { TermLogicSm::my_struct.arg_num, TermLogicSm::my_struct } } },
      { SYMB_SMEQ, { { TermLogicSmEq::my_struct.arg_num, TermLogicSmEq::my_struct } } },
      { SYMB_GREQ, { { TermLogicGrEq::my_struct.arg_num, TermLogicGrEq::my_struct } } },
      { SYMB_GR, { { TermLogicGr::my_struct.arg_num, TermLogicGr::my_struct } } },
      { SYMB_EQ, { { TermLogicEq::my_struct.arg_num, TermLogicEq::my_struct } } },
      { SYMB_NEQ, { { TermLogicNeq::my_struct.arg_num, TermLogicNeq::my_struct } } },
      { SYMB_LENGTH, { { TermArrayLength::my_struct.arg_num, TermArrayLength::my_struct } } },
      { SYMB_RAND, { { TermRand::my_struct.arg_num, TermRand::my_struct } } },
      { SYMB_ABS, { { TermAbs::my_struct.arg_num, TermAbs::my_struct } } },
      { SYMB_TRUNC, { { TermTrunc::my_struct.arg_num, TermTrunc::my_struct } } },
      { SYMB_COMPOUND, { { TermCompound::term_compound_my_struct_.arg_num, TermCompound::term_compound_my_struct_ } } },
      { SYMB_META_CMP, { { TermMetaCompound::my_struct.arg_num, TermMetaCompound::my_struct } } },
      { SYMB_META_SIMP, { { TermMetaSimplification::my_struct.arg_num, TermMetaSimplification::my_struct } } },
      { SYMB_OPTIONAL, { { TermOptional::my_struct.arg_num, TermOptional::my_struct } } },
      { SYMB_ANYWAY, { { TermAnyway::my_struct.arg_num, TermAnyway::my_struct } } },
      { SYMB_WHILELIM, { { TermWhileLimited::my_struct.arg_num, TermWhileLimited::my_struct } } },
      { SYMB_SQRT, { { TermSQRT::my_struct.arg_num, TermSQRT::my_struct } } },
      { SYMB_RSQRT, { { TermRSQRT::my_struct.arg_num, TermRSQRT::my_struct } } },
      { SYMB_SET_VAR, { { TermSetVar::my_struct.arg_num, TermSetVar::my_struct } } },
      { SYMB_SET_ARR, { { TermSetArr::my_struct.arg_num, TermSetArr::my_struct } } },
      { SYMB_GET_ARR, { { TermGetArr::my_struct.arg_num, TermGetArr::my_struct } } },
      { SYMB_FUNC_REF, { { TermFuncRef::my_struct.arg_num, TermFuncRef::my_struct } } },
      { SYMB_FUNC_EVAL, { { TermFuncEval::my_struct.arg_num, TermFuncEval::my_struct } } },
      { SYMB_LAMBDA, { { TermFuncLambda::my_struct.arg_num, TermFuncLambda::my_struct } } },
      { SYMB_LITERAL, { { TermLiteral::my_struct.arg_num, TermLiteral::my_struct } } },
      { SYMB_MALLOC, { { TermMalloc::my_struct.arg_num, TermMalloc::my_struct } } },
      { SYMB_DELETE, { { TermDelete::my_struct.arg_num, TermDelete::my_struct } } },
      { SYMB_PRINT, { { TermPrint::my_struct.arg_num, TermPrint::my_struct } } },
      { SYMB_ID, { { TermID::my_struct.arg_num, TermID::my_struct } } },
      { SYMB_FCTIN, { { TermFctIn::my_struct.arg_num, TermFctIn::my_struct } } },
   };

   for (const auto& func_names : all_ops_) {
      for (const auto& func_name : func_names.second) {
         registerAddress(func_names.first, func_name.first);
      }
   }

   all_arrs_ = {
      { SYMB_ARR_ENC, { { 1, TermArray::getOpStruct(SYMB_ARR_ENC) } } },
      { SYMB_ARR_DEC, { { 1, TermArray::getOpStruct(SYMB_ARR_DEC) } } },
   };

   all_ops_.insert(all_arrs_.begin(), all_arrs_.end());

   curr_terms_.clear();
   dynamic_term_metas_.clear();
   forward_declared_terms_.clear();
   //operators_with_flexible_number_of_arguments_.clear();

   //requestFlexibleNumberOfArgumentsFor(SYMB_FUNC_EVAL);
   //requestFlexibleNumberOfArgumentsFor(SYMB_FUNC_REF);
   //requestFlexibleNumberOfArgumentsFor(SYMB_ANYWAY);
}

std::shared_ptr<Term> FormulaParser::termFactory(
   const std::string& optor, 
   const std::vector<std::shared_ptr<Term>>& terms,
   const bool suppress_create_new_dynamic_term_via_term_func_ref) {

   if (optor == SYMB_COMPOUND && terms.size() == OPNUM_COMPOUND) {
      auto actual_term{ terms[0]->thisPtrGoIntoCompound() };
      auto compound_structure{ terms[1] };

      return TermCompound::compoundFactory(actual_term->getOperands(), compound_structure, actual_term->getOpStruct());
   }

   if (optor == SYMB_PLUS && terms.size() == OPNUM_PLUS) return std::make_shared<TermPlus>(terms);
   if (optor == SYMB_POW && terms.size() == OPNUM_POW) return std::make_shared<TermPow>(terms);
   if (optor == SYMB_LN && terms.size() == OPNUM_LN) return std::make_shared<TermLn>(terms);
   if (optor == SYMB_SIN && terms.size() == OPNUM_SIN) return std::make_shared<TermSin>(terms);
   if (optor == SYMB_COS && terms.size() == OPNUM_COS) return std::make_shared<TermCos>(terms);
   if (optor == SYMB_TAN && terms.size() == OPNUM_TAN) return std::make_shared<TermTan>(terms);
   if (optor == SYMB_ARCSIN && terms.size() == OPNUM_ARCSIN) return std::make_shared<TermASin>(terms);
   if (optor == SYMB_ARCCOS && terms.size() == OPNUM_ARCCOS) return std::make_shared<TermACos>(terms);
   if (optor == SYMB_ARCTAN && terms.size() == OPNUM_ARCTAN) return std::make_shared<TermATan>(terms);
   if (optor == SYMB_MINUS && terms.size() == OPNUM_MINUS) return std::make_shared<TermMinus>(terms);
   if (optor == SYMB_MULT && terms.size() == OPNUM_MULT) return std::make_shared<TermMult>(terms);
   if (optor == SYMB_DIV && terms.size() == OPNUM_DIV) return std::make_shared<TermDiv>(terms);
   if (optor == SYMB_NEG && terms.size() == OPNUM_NEG) return std::make_shared<TermNeg>(terms[0]);
   if (optor == SYMB_MOD && terms.size() == OPNUM_MOD) return std::make_shared<TermMod>(terms);
   if (optor == SYMB_MAX && terms.size() == OPNUM_MAX) return std::make_shared<TermMax>(terms);
   if (optor == SYMB_MIN && terms.size() == OPNUM_MIN) return std::make_shared<TermMin>(terms);
   if (optor == SYMB_AND && terms.size() == OPNUM_AND) return std::make_shared<TermLogicAnd>(terms);
   if (optor == SYMB_OR && terms.size() == OPNUM_OR) return std::make_shared<TermLogicOr>(terms);
   if (optor == SYMB_SM && terms.size() == OPNUM_SM) return std::make_shared<TermLogicSm>(terms);
   if (optor == SYMB_SMEQ && terms.size() == OPNUM_SMEQ) return std::make_shared<TermLogicSmEq>(terms);
   if (optor == SYMB_EQ && terms.size() == OPNUM_EQ) return std::make_shared<TermLogicEq>(terms);
   if (optor == SYMB_NEQ && terms.size() == OPNUM_NEQ) return std::make_shared<TermLogicNeq>(terms);
   if (optor == SYMB_GREQ && terms.size() == OPNUM_GREQ) return std::make_shared<TermLogicGrEq>(terms);
   if (optor == SYMB_GR && terms.size() == OPNUM_GR) return std::make_shared<TermLogicGr>(terms);
   if (optor == SYMB_LENGTH && terms.size() == OPNUM_LENGTH) return std::make_shared<TermArrayLength>(terms[0]);
   if (optor == SYMB_RAND && terms.size() == OPNUM_RAND) return std::make_shared<TermRand>(terms[0]);
   if (optor == SYMB_ABS && terms.size() == OPNUM_ABS) return std::make_shared<TermAbs>(terms[0]);
   if (optor == SYMB_TRUNC && terms.size() == OPNUM_TRUNC) return std::make_shared<TermTrunc>(terms[0]);

   if (optor == SYMB_META_CMP && terms.size() == OPNUM_META_CMP) return std::make_shared<TermMetaCompound>(terms[0]);
   if (optor == SYMB_META_SIMP && terms.size() == OPNUM_META_SIMP) return std::make_shared<TermMetaSimplification>(terms[0]);
   if (optor == SYMB_OPTIONAL && terms.size() == OPNUM_OPTIONAL) return std::make_shared<TermOptional>(terms[0], terms[1]);
   if (optor == SYMB_ANYWAY) return std::make_shared<TermAnyway>(terms); // Regardless of terms.size(), construct TermAnyway with any number or operands.
   if (optor == SYMB_WHILELIM && terms.size() == OPNUM_WHILELIM) return std::make_shared<TermWhileLimited>(terms);
   if (optor == SYMB_SQRT && terms.size() == OPNUM_SQRT) return std::make_shared<TermSQRT>(terms[0]);
   if (optor == SYMB_RSQRT && terms.size() == OPNUM_RSQRT) return std::make_shared<TermRSQRT>(terms[0]);
   if (optor == SYMB_SET_VAR && terms.size() == OPNUM_SET_VAR) return std::make_shared<TermSetVar>(terms[0], terms[1]);
   if (optor == SYMB_SET_ARR && terms.size() == OPNUM_SET_ARR) return std::make_shared<TermSetArr>(terms[0], terms[1], terms[2]);
   if (optor == SYMB_GET_ARR && terms.size() == OPNUM_GET_ARR) return std::make_shared<TermGetArr>(terms[0], terms[1]);

   if (optor == SYMB_FUNC_REF) {
      if (!suppress_create_new_dynamic_term_via_term_func_ref) {
         if (StaticHelper::stringStartsWith(optor, StaticHelper::makeString(NATIVE_ARRAY_PREFIX_FIRST))) {
            addError("Cannot create function '" + optor + "' because it starts with the array denoter '" + StaticHelper::makeString(NATIVE_ARRAY_PREFIX_FIRST) + "'.");
         }
         else {
            terms[terms.size() - 1]->setChildrensFathers(); // This is usually done at the end of parsing in math_struct.cpp, but here we need it for the subsequent function call.
            addDynamicTermViaFuncRef(terms);
         }
      }

      return std::make_shared<TermFuncRef>(terms);
   }

   if (optor == SYMB_FUNC_EVAL) return std::make_shared<TermFuncEval>(terms);
   if (optor == SYMB_LAMBDA && terms.size() == OPNUM_LAMBDA) return std::make_shared<TermFuncLambda>(terms);
   if (optor == SYMB_LITERAL && terms.size() == OPNUM_LITERAL) return std::make_shared<TermLiteral>(terms);
   if (optor == SYMB_MALLOC && terms.size() == OPNUM_MALLOC) return std::make_shared<TermMalloc>(terms);
   if (optor == SYMB_DELETE && terms.size() == OPNUM_DELETE) return std::make_shared<TermDelete>(terms);
   if (optor == SYMB_PRINT && terms.size() == OPNUM_PRINT) return std::make_shared<TermPrint>(terms);
   if (optor == SYMB_ID && terms.size() == OPNUM_ID) return std::make_shared<TermID>(terms);
   if (optor == SYMB_FCTIN && terms.size() == OPNUM_FCTIN) return std::make_shared<TermFctIn>();

   /* 
    * Leading dot "." used to denote static arrays. Keep the array factory below the
    * static functions to allow other static functions to start with a dot, as well.
    * (It is not possible like this for dynamic functions, though,
    * and, at least for now, not worth the effort to change.)
    * => Don't move this below the dynamic functions, and best practice should be to
    * just start all functions with a non-dot letter.
    *
    * Deprecation note: static arrays are deprecated and will be replaced completely by arrays
    * in dynamic memory. Then dots will be allowed at the beginning of variables and functions.
    */
   if (isArray(optor)) {
      if (terms.empty()) {
         addError("Array index missing for array '" + optor + "'.");
         return _var(PARSING_ERROR_STRING);
      }
      else {
         return std::make_shared<TermArray>(terms[0], optor);
      }
   }

   auto meta_key = dynamic_term_metas_.find(optor);
   auto op_key = all_ops_.find(optor);

   // TODO: Here and below some optiomizations are possible, see comments.
   if (op_key != all_ops_.end()
      && !all_ops_.at(optor).count(PARNUM_FLEXIBLE_MODE)
      && !all_ops_.at(optor).count(terms.size())) { // TODO: Use already found element, here and below.
      std::string possible = " ";

      for (const auto& el : all_ops_.at(optor)) {
         possible += std::to_string(el.first) + " ";
      }

      std::string operands_err{};
      for (const auto& op : terms) {
         operands_err += op->serialize() + "\n";
      }
      addError("Term Factory received an invalid number of arguments to create function '" + optor + "'. Any of {" + possible + "} required, but " + std::to_string(terms.size()) + " given.");
      addErrorPlain("These are the operands in question:\n" + operands_err);
      return _var(PARSING_ERROR_STRING + "OPERATOR_" + OperatorStructure(optor, terms.size()).serialize() + "_MISSING#");
   }

   if (meta_key != dynamic_term_metas_.end() && meta_key->second.count(terms.size()) && meta_key->second.at(terms.size())) { // Operator fully defined. // TODO: Avoid double effort for count and at. Do we need the last "at" part?
      std::shared_ptr<Term> meta_term = dynamic_term_metas_.at(optor).at(terms.size()); // TODO: Avoid this third lookup as well.
      OperatorStructure opstruct{ all_ops_.at(optor).at(terms.size()) };
      return TermCompound::compoundFactory(terms, meta_term, opstruct);
   } else if (meta_key != dynamic_term_metas_.end() && meta_key->second.count(PARNUM_FLEXIBLE_MODE) && meta_key->second.at(PARNUM_FLEXIBLE_MODE)) { // Operator fully defined. // TODO: Do we need the last "at" part?
      std::shared_ptr<Term> meta_term = dynamic_term_metas_.at(optor).at(PARNUM_FLEXIBLE_MODE); // TODO: Avoid this third lookup as well.
      OperatorStructure opstruct{ all_ops_.at(optor).at(PARNUM_FLEXIBLE_MODE) };
      return TermCompound::compoundFactory(terms, meta_term, opstruct);
   } else if (op_key != all_ops_.end()) { // Operator forward declared.
      OperatorStructure opstruct{ 
         all_ops_.at(optor).count(PARNUM_FLEXIBLE_MODE) // Always return flexible if available. (TODO: Prohibit other entrances if flexible available.)
         ? OperatorStructure(all_ops_.at(optor).at(PARNUM_FLEXIBLE_MODE))
         : OperatorStructure(all_ops_.at(optor).at(terms.size()))};
      std::shared_ptr<Term> dummy_meta = std::make_shared<TermVar>(FORWARD_DECLARATION_PREFIX + "<" + optor + ", " + std::to_string(terms.size()) + ">");
      auto unfinished_term = TermCompound::compoundFactory(terms, dummy_meta, opstruct);
      forward_declared_terms_.push_back(unfinished_term);
      return unfinished_term;
   }

   return nullptr;
}
// --- EO Register static operators. ---

void vfm::FormulaParser::addDynamicTermViaFuncRef(const std::vector<std::shared_ptr<Term>>& terms)
{
   static const std::string INVALID_FUNCTION_NAME{ "#INVALID" };
   std::string func_name{ terms[0]->getOptor() };
   bool dummy{};
   OperatorStructure op_struct{ func_name, dummy };

   if (terms[0]->isTermVar()) { // New function definition (or forward declaration).
      if (terms.size() == 1) {
         forwardDeclareDynamicTerm(op_struct);
      } else {
         auto meta_term{ _id(terms[terms.size() - 1]) };

         for (int arg_num = 0; arg_num < terms.size() - 2; arg_num++) {
            if (terms[arg_num + 1]->isTermVar()) {
               std::string arg_name{ terms[arg_num + 1]->toVariableIfApplicable()->getVariableName() };

               auto key = std::pair<std::string, int>{ func_name, terms.size() - 2 };

               if (!naming_of_parameters_.count(key)) {
                  naming_of_parameters_.insert({ key, {} });
               }

               auto& naming_map = naming_of_parameters_.at(key);
               naming_map.insert({ arg_num, arg_name });

               meta_term->applyToMeAndMyChildrenIterative([&arg_name, arg_num, this](const MathStructPtr m) {
                  if (m->isTermVar() && m->toVariableIfApplicable()->getVariableName() == arg_name) {
                     m->getFather()->replaceOperand(m->toTermIfApplicable(), _meta_compound(arg_num));
                  }
               }, TraverseCompoundsType::avoid_compound_structures);
            }
            else {
               addError("Expected variable as name of parameter #" + std::to_string(arg_num) + " in definition of function '" + func_name + "', but found '" + terms[arg_num + 1]->serialize() + "'.");
            }
         }

         if (op_struct.arg_num == PARNUM_AUTO_MODE && terms.size() > 2) {
            op_struct.arg_num = terms.size() - 2;
         }

         addDynamicTerm(op_struct, meta_term->getTermsJumpIntoCompounds()[0]);
      }
   }
   else {
      addError("Expected variable as first argument of '" + SYMB_FUNC_REF + "', but found '" + terms[0]->serialize() + "'.");
   }
}

std::shared_ptr<Term> vfm::FormulaParser::termFactory(const std::string& optor)
{
   return termFactory(optor, std::vector<std::shared_ptr<Term>>{});
}

std::shared_ptr<Term> vfm::FormulaParser::termFactory(const std::string& optor, const std::shared_ptr<Term>& term)
{
   return termFactory(optor, std::vector<std::shared_ptr<Term>>{ term });
}

std::shared_ptr<Term> FormulaParser::termFactoryDummy(const std::string& optor, int num_pars)
{
   static const std::vector<std::vector<TermPtr>> dummy_terms{
      {},
      { _val0() },
      { _val0(), _val0() },
      { _val0(), _val0(), _val0() },
      { _val0(), _val0(), _val0(), _val0() },
      { _val0(), _val0(), _val0(), _val0(), _val0() },
      { _val0(), _val0(), _val0(), _val0(), _val0(), _val0() },
      { _val0(), _val0(), _val0(), _val0(), _val0(), _val0(), _val0() },
      { _val0(), _val0(), _val0(), _val0(), _val0(), _val0(), _val0(), _val0() },
      { _val0(), _val0(), _val0(), _val0(), _val0(), _val0(), _val0(), _val0(), _val0() },
      { _val0(), _val0(), _val0(), _val0(), _val0(), _val0(), _val0(), _val0(), _val0(), _val0() },
      { _val0(), _val0(), _val0(), _val0(), _val0(), _val0(), _val0(), _val0(), _val0(), _val0(), _val0() },
      { _val0(), _val0(), _val0(), _val0(), _val0(), _val0(), _val0(), _val0(), _val0(), _val0(), _val0(), _val0() },
      { _val0(), _val0(), _val0(), _val0(), _val0(), _val0(), _val0(), _val0(), _val0(), _val0(), _val0(), _val0(), _val0() },
      { _val0(), _val0(), _val0(), _val0(), _val0(), _val0(), _val0(), _val0(), _val0(), _val0(), _val0(), _val0(), _val0(), _val0() },
      { _val0(), _val0(), _val0(), _val0(), _val0(), _val0(), _val0(), _val0(), _val0(), _val0(), _val0(), _val0(), _val0(), _val0(), _val0() },
   };

   if (num_pars == PARNUM_FLEXIBLE_MODE) {
      return termFactory(optor, nullptr); // nullptr as code for flexible number of arguments.
   }

   TermPtr term_result = nullptr;

   if (num_pars >= dummy_terms.size()) {
      std::vector<TermPtr> dummy_terms_flex;

      for (int i = 0; i < num_pars; i++) {
         dummy_terms_flex.push_back(_val0());
      }

      term_result = termFactory(optor, dummy_terms_flex);
   }
   else {
      term_result = termFactory(optor, dummy_terms[num_pars]);
   }

   return term_result ? term_result->setChildrensFathers()->toTermIfApplicable() : nullptr;
}

std::shared_ptr<Term> vfm::FormulaParser::termFactoryDummy(const std::string& optor)
{
   return termFactoryDummy(optor, *getNumParams(optor).begin());
}

std::set<int> FormulaParser::getNumParams(const std::string& optor)
{
   auto ops = all_ops_.find(optor);

   if (ops == all_ops_.end()) {
      if (isArray(optor)) {
         return { 1 };
      }

      return {}; // No instance of this operator name declared.
   }

   std::set<int> par_nums;

   for (const auto& par_num : ops->second) { // Has to have at least one declared instance.
      if (par_num.first >= 0) {
         par_nums.insert(par_num.first);
      }
   }

   return par_nums;
}

OperatorStructure FormulaParser::getOperatorStructure(const std::string& op_name, const int num_params)
{
   return all_ops_.at(op_name).at(num_params);
}

bool FormulaParser::isLeftAssoc(const std::string& optor, const int num_params) const
{
   if (all_ops_.at(optor).count(PARNUM_FLEXIBLE_MODE)) { // TODO: Optimizing this function might make parsing faster.
      return all_ops_.at(optor).at(PARNUM_FLEXIBLE_MODE).associativity == AssociativityTypeEnum::left;
   }

   return all_ops_.at(optor).at(num_params).associativity == AssociativityTypeEnum::left;
}

int FormulaParser::prio(const std::string& optor, const int num_params) const
{
   if (all_ops_.count(optor)) {
      auto at_optor{ all_ops_.at(optor) };

      if (at_optor.count(PARNUM_FLEXIBLE_MODE)) { // TODO: Optimizing this function might make parsing faster.
         return at_optor.at(PARNUM_FLEXIBLE_MODE).precedence;
      }
      else if (at_optor.count(num_params)) {
         return at_optor.at(num_params).precedence;
      }
   }

   addError("Operator '" + optor + "' with " + std::to_string(num_params) + " parameters not found.");
   return std::numeric_limits<int>::max();
}

bool vfm::FormulaParser::isTermNative(const std::string& func_name, const int num_params) const
{
   return native_funcs_.count({ func_name, num_params });
}

// Cf. https://stackoverflow.com/questions/26471876/how-to-tell-the-precedence-of-operators-in-a-context-free-grammar
// Cf. http://www.cs.ecu.edu/karl/5220/spr16/Notes/CFG/precedence.html
std::shared_ptr<earley::Grammar> vfm::FormulaParser::createGrammar(const bool include_full_brace_transformations, const std::set<std::string>& use_only) const
{
   const auto op_bracket = StaticHelper::makeString(OPENING_BRACKET);
   const auto cl_bracket = StaticHelper::makeString(CLOSING_BRACKET);
   const auto op_brace = StaticHelper::makeString(OPENING_BRACKET_BRACE_STYLE);
   const auto cl_brace = StaticHelper::makeString(CLOSING_BRACKET_BRACE_STYLE);
   const auto delimiter = StaticHelper::makeString(ARGUMENT_DELIMITER);
   std::set<std::string> terminals;
   auto g = std::make_shared<earley::Grammar>(getStartSymbol(0));
   std::vector<std::deque<std::string>> rhs_s;
   std::vector<std::string> lhs_s;
   int symbol_num = 0;

   auto cmp = [](OperatorStructure a, OperatorStructure b) -> bool { return a.precedence < b.precedence || a.precedence == b.precedence && a.op_name < b.op_name; };
   std::set<OperatorStructure, decltype(cmp)> all_ops_by_precedence(cmp);
   
   for (const auto& ops : all_ops_) {
      for (const auto& op : ops.second) {
         if (!StaticHelper::stringStartsWith(op.second.op_name, GRAMMAR_SIMPLE_START_SYMBOL)) { // Particularly excludes the TermAnyway simple operators.
            if (use_only.empty() || use_only.count(ops.first)) {
               all_ops_by_precedence.insert(op.second);
            }
         }
      }
   }

   if (include_full_brace_transformations) {
      g->addTerminal("else");
   }

   g->addTerminals({
      op_bracket, 
      cl_bracket, 
      op_brace,
      cl_brace,
      delimiter,
   });

   //g.addProduction(getStartSymbol(symbol_num), { "x" });

   std::vector<std::deque<std::string>> rhs_container_tmp;

   for (const auto& op : all_ops_by_precedence) {
      g->addTerminal(op.op_name);
      std::deque<std::string> rhs;

      if (op.arg_num == 2 && op.op_pos_type == OutputFormatEnum::infix) {
         if (op.associativity == AssociativityTypeEnum::left) {
            rhs = { getStartSymbol(symbol_num), op.op_name, getStartSymbol(symbol_num + 1) }; // S + T
         }
         else {
            rhs = { getStartSymbol(symbol_num + 1), op.op_name, getStartSymbol(symbol_num) }; // T + S
         }
         
         lhs_s.push_back(getStartSymbol(symbol_num));
         rhs_s.push_back(rhs);
         lhs_s.push_back(getStartSymbol(symbol_num));
         rhs_s.push_back({ getStartSymbol(symbol_num + 1) });
         symbol_num++;

         //rhs = { op_bracket, getStartSymbol(0), op.op_name, getStartSymbol(0), cl_bracket }; // (S + S)
         //lhs_s.push_back(getStartSymbol(0));
         //rhs_s.push_back(rhs);

         //rhs = { getStartSymbol(0), op.op_name, getStartSymbol(0) }; // (S + S)
         //lhs_s.push_back(getStartSymbol(0));
         //rhs_s.push_back(rhs);
      }

      int start = op.arg_num;
      if (include_full_brace_transformations) {
         start = 0;
      }

      for (int i = start; i <= op.arg_num; i++) {
         rhs.clear();
         rhs.push_back(op.op_name);
         rhs.push_back(op_bracket);

         for (int j = 0; j < i; j++) {
            if (j > 0) {
               rhs.push_back(delimiter);
            }

            rhs.push_back(getStartSymbol(0));
         }

         rhs.push_back(cl_bracket);

         for (int k = i; k < op.arg_num; k++) {
            rhs.push_back(op_brace);
            rhs.push_back(getStartSymbol(0));
            rhs.push_back(cl_brace);
         }

         rhs_container_tmp.push_back(rhs); // Insert later for last symb_num, see below.
      }
   }

   for (const auto& rhs : rhs_container_tmp) {
      lhs_s.push_back(getStartSymbol(symbol_num));
      rhs_s.push_back(rhs);
   }

   for (int i = 0; i < rhs_s.size(); i++) {
      auto r = rhs_s[i];
      auto l = lhs_s[i];
      g->addProduction(l, r);
   }

   g->addProduction(getStartSymbol(symbol_num), { "x" });
   g->addProduction(getStartSymbol(symbol_num), { "(", getStartSymbol(0), ")" });

   //g.addProduction(GRAMMAR_START_SYMBOL_PREFIX, { GRAMMAR_START_SYMBOL_PREFIX, SYMB_SEQUENCE });

   return g;
}

void vfm::FormulaParser::initializeValuesBy(const FormulaParser& other)
{
   curr_terms_.clear();
   dynamic_term_metas_.clear();
   forward_declared_terms_.clear();

   for (const auto& el : other.curr_terms_) {
      curr_terms_.push_back(el->copy());
   }

   for (const auto& el : other.dynamic_term_metas_) {
      for (const auto& el2 : el.second) {
         dynamic_term_metas_.insert({ el.first, {} });
         dynamic_term_metas_.at(el.first).insert({ el2.first, el2.second->copy() });
      }
   }

   for (const auto& el : other.forward_declared_terms_) {
      forward_declared_terms_.push_back(el->copy()->toTermCompoundIfApplicable());
   }

   all_ops_ = other.all_ops_;
   all_arrs_ = other.all_arrs_;
   current_auto_extracted_function_name_ = other.current_auto_extracted_function_name_;
   current_lambda_function_counter_ = other.current_lambda_function_counter_;
   parsed_variables_ = other.parsed_variables_;
   names_to_addresses = other.names_to_addresses;
   addresses_to_names = other.addresses_to_names;
   function_count_ = other.function_count_;
   is_in_function_ref_mode_ = other.is_in_function_ref_mode_;
   //operators_with_flexible_number_of_arguments_ = other.operators_with_flexible_number_of_arguments_;
   native_funcs_ = other.native_funcs_;

   //for (const auto& op : getAllOps()) {
   //   addFailableChild(op.second, "");
   //}
}

void vfm::FormulaParser::initializeValuesBy(const std::shared_ptr<FormulaParser> other)
{
   initializeValuesBy(*other);
}

void vfm::FormulaParser::resetRawDynamicTerm(const std::string& op_name, const TermPtr new_term, const int num_params)
{
   dynamic_term_metas_.at(op_name) = {};
   dynamic_term_metas_.at(op_name).insert({ num_params, new_term });
}

void vfm::FormulaParser::removeDynamicTerm(const std::string& op_name, const int num_params)
{
   dynamic_term_metas_.at(op_name).erase(num_params);
}

std::map<std::string, std::map<int, std::set<MetaRule>>> vfm::FormulaParser::getAllRelevantSimplificationRulesOrMore()
{
   for (const auto& symbs : all_ops_) {
      if (symbs.first != SYMB_FUNC_REF) { // Exclude func ref since it requires a special first operand.
         for (const auto& symb : symbs.second) {
            if (symb.first >= 0 || symb.first == PARNUM_FLEXIBLE_MODE) {
               termFactoryDummy(symbs.first, symb.first)->getStaticConvertedSimplificationRules(shared_from_this());
            }
         }
      }
   }

   return MathStruct::getAllRulesSoFar();
}

bool FormulaParser::isArray(const std::string& optor)
{
   if (all_arrs_.count(optor)) {
      return true;
   }

   if (StaticHelper::stringStartsWith(optor, SYMB_SET_ARR) || StaticHelper::stringStartsWith(optor, SYMB_GET_ARR)) {
      return false;
   }

   bool isit = StaticHelper::isVariableNameDenotingArray(optor);

   if (isit) { // Note that term_array_float has the same op_struct as TermArray.
      all_ops_.insert({ optor, { { 1, TermArray::getOpStruct(optor) } } });
      all_arrs_.insert({ optor, { { 1, TermArray::getOpStruct(optor) } } });
   }

   return isit;
}

bool vfm::FormulaParser::isFunctionOrOperator(const std::string& optor) const
{
   const bool regular_function_or_optor{ /*arg_num == -1
      ?*/ (bool) all_ops_.count(optor)
      /*: (bool) all_ops_.count(optor) && all_ops_.at(optor).count(arg_num)*/
   };
   
   return regular_function_or_optor || const_cast<FormulaParser*>(this)->isArray(optor);
}

bool vfm::FormulaParser::isOperatorPossibly(const std::string& optor) const
{
   auto operators_it = all_ops_.find(optor);
   if (operators_it == all_ops_.end()) {
      return false;
   }

   auto operator_it = operators_it->second.find(2); // Operator has 2 operands.
   if (operator_it == operators_it->second.end()) {
      return false;
   }

   // Return true even if there is another overload.
   return operator_it->first == 2 && operator_it->second.op_pos_type == OutputFormatEnum::infix;
}

bool vfm::FormulaParser::isOperatorForSure(const std::string& optor) const
{
   auto operators_it = all_ops_.find(optor);
   if (operators_it == all_ops_.end()) {
      return false;
   }

   if (operators_it->second.size() != 1) { // This is the difference to isOperatorPossibly. // TODO: Get rid of double code.
      return false;
   }

   auto operator_it = operators_it->second.find(2);
   if (operator_it == operators_it->second.end()) {
      return false;
   }

   // Return true only if there is just the one overload.
   return operator_it->first == 2 && operator_it->second.op_pos_type == OutputFormatEnum::infix;
}

OperatorDeclarationState vfm::FormulaParser::getOperatorDeclarationState(const std::string& optor) const
{
   auto num_paramss = const_cast<FormulaParser*>(this)->getNumParams(optor).size();

   if (num_paramss > 1) {
      return OperatorDeclarationState::overloaded;
   }
   else if (num_paramss == 0) {
      return OperatorDeclarationState::undeclared;
   }
   else {
      return OperatorDeclarationState::unique;
   }
}

void FormulaParser::pushToOutput(const std::string& raw_el, std::stack<int>& nested_arg_nums_stack)
{
   std::string el = raw_el;

   if (!is_in_function_ref_mode_ && isFunctionOrOperator(el)) {
      std::vector<std::shared_ptr<Term>> terms;
      int arg_num;

      if (true /*std::find(operators_with_flexible_number_of_arguments_.begin(), operators_with_flexible_number_of_arguments_.end(), el) != operators_with_flexible_number_of_arguments_.end()*/) {
         arg_num = nested_arg_nums_stack.top();
         nested_arg_nums_stack.pop();

         if (SYMB_IF == el && arg_num == 3) {
            el = SYMB_IF_ELSE;
         }

         if (arg_num < 0) { // Has to be operator after all, not a function.
            arg_num = 2;
         }
      }
      else { // No flexible number of operands, so particularly not an overloaded operator.
         auto arg_nums = getNumParams(el);

         if (arg_nums.size() > 1) {
            addError("Unexpectedly found overloaded operator '" + el + "' without flexible number of operands.");
         }
         else if (arg_nums.empty()) {
            addError("Operator '" + el + "' is undefined.");
            return;
         }

         arg_num = *arg_nums.begin();
      }

      for (auto i = curr_terms_.size() - arg_num; i < curr_terms_.size(); ++i) {
         terms.push_back(curr_terms_[i]);
      }

      for (auto i = 0; i < arg_num; ++i) {
         if (curr_terms_.empty()) {
            addError("Tape ran out of items.");
            curr_terms_.push_back(_val(0));
            return;
         }
         else {
            curr_terms_.pop_back();
         }
      }

      std::shared_ptr<Term> ptr = termFactory(el, terms);
      curr_terms_.push_back(ptr);
   }
   else {
      if (!is_in_function_ref_mode_ && StaticHelper::isConstVal(el)) {
         std::istringstream os(el);
         float d;
         os >> d;
         curr_terms_.push_back(std::make_shared<TermVal>(d));
      }
      else if (is_in_function_ref_mode_ || StaticHelper::isConstVar(el)) { // We need to recognize '%' in '@f(%, ...)' as variable.
         curr_terms_.push_back(std::make_shared<TermVar>(el));
         parsed_variables_.push_back(el);
         is_in_function_ref_mode_ = false;
      }
      else {
         addError("Token '" + el + "'" + " is not a known operator nor wellformed to be a variable name or a numeric value nor at the right place to be a paranthesis or a delimiter.");
         curr_terms_.push_back(std::make_shared<TermVar>(PARSING_ERROR_STRING));
      }
   }
}

void FormulaParser::forwardDeclareDynamicTerm(const OperatorStructure& op_struct)
{
   addDynamicTerm(op_struct, (std::shared_ptr<Term>) nullptr);
}

void vfm::FormulaParser::forwardDeclareDynamicTerm(const OperatorStructure& op_struct, const std::string& op_name, const bool is_native, const int arg_num, const int precedence, const OutputFormatEnum op_pos_type, const AssociativityTypeEnum associativity, const bool assoc, const bool commut, const AdditionalOptionEnum option)
{
   OperatorStructure opstr(op_pos_type, associativity, arg_num, precedence, op_name, assoc, commut, option);
   forwardDeclareDynamicTerm(opstr);
}

void vfm::FormulaParser::addDynamicTerm(const OperatorStructure& op_struct, const std::string& meta_struct, const std::shared_ptr<DataPack> data, const bool is_native)
{
   addDynamicTerm(op_struct, MathStruct::parseMathStruct(meta_struct, *this, data)->toTermIfApplicable(), is_native);
}

void FormulaParser::addDynamicTerm(
   const std::shared_ptr<Term> meta_struct,
   const std::string& op_name,
   const bool is_native,
   const int arg_num,
   const int precedence,
   const OutputFormatEnum op_pos_type,
   const AssociativityTypeEnum associativity,
   const bool assoc,
   const bool commut,
   const AdditionalOptionEnum option)
{
   OperatorStructure opstr(op_pos_type, associativity, arg_num, precedence, op_name, assoc, commut, option);
   addDynamicTerm(opstr, meta_struct, is_native);
}

void FormulaParser::addDynamicTerm(
   const OperatorStructure& op_struct,
   const std::shared_ptr<Term> meta_struct,
   const bool is_native)
{
   if (is_native) {
      native_funcs_.insert({ op_struct.op_name, op_struct.arg_num });
   }

   if (op_struct.option == AdditionalOptionEnum::allow_arbitrary_number_of_arguments) {
      //requestFlexibleNumberOfArgumentsFor(op_struct.op_name);
   }

   all_ops_.insert({ op_struct.op_name, {} }); // Don't update if existing.

   int auto_num = op_struct.arg_num == PARNUM_AUTO_MODE && meta_struct ? StaticHelper::deriveParNum(meta_struct) : op_struct.arg_num;
   bool already_existed_op_struct = auto_num >= 0 && all_ops_.at(op_struct.op_name).count(auto_num);
   auto meta_existing_pair = dynamic_term_metas_.find(op_struct.op_name);
   bool error_occurred = false;
   OperatorStructure op_struct_copy(op_struct);
   op_struct_copy.arg_num = auto_num;
   std::string full_func_name = op_struct_copy.serialize();

   if (already_existed_op_struct) {
      if (!termFactoryDummy(op_struct.op_name, auto_num)->isTermCompound()) {
         if (meta_struct) {
            addError("Cannot redefine function '" + full_func_name + "'. It's a built-in function.");
         }

         return;
      }

      OperatorStructure existing_op_struct = all_ops_.at(op_struct.op_name).at(auto_num);

      if (existing_op_struct.op_pos_type != op_struct.op_pos_type // Error handling: Operator already declared with a different structure.
         || existing_op_struct.associativity != op_struct.associativity
         || (existing_op_struct.arg_num >= 0 && auto_num >= 0 && existing_op_struct.arg_num != auto_num)
         || (existing_op_struct.op_pos_type == OutputFormatEnum::infix && existing_op_struct.precedence != op_struct.precedence)
         || existing_op_struct.assoc != op_struct.assoc
         || existing_op_struct.commut != op_struct.commut
         ) {
         addWarning("The new function '" + full_func_name + "' has a different operator structure than an existing one.");
         addWarningPlain("Existing: '" + existing_op_struct.serialize() + "'.");
         addWarningPlain("Vs. new:  '" + op_struct.serialize() + "'.");
         error_occurred = true;
      }
   }

   if (meta_existing_pair != dynamic_term_metas_.end() 
      && meta_existing_pair->second.find(auto_num) != meta_existing_pair->second.end()
      && meta_existing_pair->second.find(auto_num)->second
      && already_existed_op_struct) { // Error handling: Operator already defined.

      if (meta_struct) {
         addWarning("Function '" + full_func_name + "' has already been defined.");
         auto meta_existing = meta_existing_pair->second.at(auto_num);

         if (!meta_existing->isStructurallyEqual(meta_struct)) {
            addWarning("The new function '" + full_func_name + "' has a different definition than the existing one.");
            addWarningPlain("Existing: '" + meta_existing->serialize() + "'.");
            addWarningPlain("Vs. new:  '" + meta_struct->serialize() + "'.");
            error_occurred = true;
         }

         if (error_occurred) {
            addNote("Re-defining the function '" + full_func_name + "'.");
         }
         else {
            addNote("The existing function '" + full_func_name + "' equals the new one, therefore, no action is taken.");
            return;
         }
      }
   }

   all_ops_.at(op_struct.op_name).insert({ auto_num, op_struct });
   all_ops_.at(op_struct.op_name).at(auto_num) = op_struct; // Update if existing.
   auto result = all_ops_.at(op_struct.op_name);

   registerAddress(op_struct.op_name, auto_num); // Don't update if existing.
   dynamic_term_metas_.insert({ op_struct.op_name, {} }); // Don't update if existing.
   dynamic_term_metas_[op_struct.op_name].insert({ {auto_num, meta_struct} }); // Don't update if existing.

   if (meta_struct) { // If fully defined...
      std::string name = op_struct.op_name;

      //if (op_struct.arg_num == AUTO_MODE) {
      dynamic_term_metas_.at(name).erase(PARNUM_AUTO_MODE); // Can occur from forward declaration.
      all_ops_.at(name).erase(PARNUM_AUTO_MODE); // Can occur from forward declaration.
      dynamic_term_metas_.at(name).insert({ auto_num, meta_struct });
      dynamic_term_metas_.at(name).at(auto_num) = meta_struct;
      all_ops_.at(name).insert({ auto_num, op_struct });
      all_ops_.at(name).at(auto_num) = op_struct;
      all_ops_.at(name).at(auto_num).setArgNumIfInAutoMode(auto_num);
      registerAddress(op_struct.op_name, auto_num);
      //}

      for (const auto& unfinished_term : forward_declared_terms_) { // Look if top-level unfinished terms can be updated.
         if (unfinished_term->getOptorJumpIntoCompound() == op_struct.op_name) {
            unfinished_term->setCompoundStructure(meta_struct);
         }
      }

      for (const auto& metas : dynamic_term_metas_) { // Update unfinished terms in nested compounds.
         for (const auto& meta : metas.second) { // Update unfinished terms in nested compounds.
            if (meta.second) {
               std::shared_ptr<std::vector<std::shared_ptr<TermCompound>>> storage = std::make_shared<std::vector<std::shared_ptr<TermCompound>>>();
               meta.second->applyToMeAndMyChildren([meta, op_struct, meta_struct, storage](std::shared_ptr<MathStruct> m) {
                  if (m->isTermCompound()) {
                     const auto m_compound = m->toTermCompoundIfApplicable();
                     const auto comp_structure = m_compound->getCompoundStructure();
                     if (comp_structure->isTermVar()) {
                        std::string optor_name = comp_structure->toVariableIfApplicable()->getOptor();
                        if (StaticHelper::stringStartsWith(optor_name, FORWARD_DECLARATION_PREFIX)) {
                           // Found abandoned FWD term.
                           std::string fwd_name = StaticHelper::split(StaticHelper::split(optor_name, "<")[1], ",")[0];
                           if (fwd_name == op_struct.op_name) {
                              storage->push_back(m_compound);
                           }
                        }
                     }
                  }
               }, TraverseCompoundsType::go_into_compound_structures);

               for (const auto& m_compound : *storage) {
                  m_compound->setCompoundStructure(meta_struct->copy());
               }
            }
         }
      }
   }

   //if (getNumParams(op_struct.op_name).size() > 1) { // If it's an overloaded operator/function.
      //requestFlexibleNumberOfArgumentsFor(op_struct.op_name);
   //}
}

int vfm::FormulaParser::addLambdaTerm(const std::shared_ptr<Term> meta_struct)
{
   const OutputFormatEnum out_type = OutputFormatEnum::prefix;      // Lambda is always prefix.
   const AssociativityTypeEnum assoc = AssociativityTypeEnum::left; // Since it's prefix, don't care about associativity, ...
   const int prio = -1;                  // ...prio, ...
   const bool associative = false;       // ...if it's associative, ...
   const bool commutative = false;       // ...or if it's commutative. I.e., these values could be anything.
   const std::string lambda_name = "_lambda_" + getCurrentLambdaFunctionName();
   const int par_num = StaticHelper::deriveParNum(meta_struct);

   OperatorStructure op_struct(OutputFormatEnum::prefix, AssociativityTypeEnum::left, par_num, -1, lambda_name, false, false);

   addDynamicTerm(op_struct, meta_struct);

   setCurrentLambdaFunctionName(StaticHelper::incrementAlphabeticNum(getCurrentLambdaFunctionName()));

   return getFunctionAddressOf(lambda_name, op_struct.arg_num);
}

TermPtr FormulaParser::getDynamicTermMeta(const std::string& op_name, const int num_params) const
{
   auto it_all = dynamic_term_metas_.find(op_name);

   if (it_all == dynamic_term_metas_.end()) {
      return nullptr;
   }

   auto it = num_params == -1 
      ? it_all->second.begin()
      : it_all->second.find(num_params);

   return it == it_all->second.end() ? nullptr : it->second;
}

std::map<std::string, std::map<int, std::shared_ptr<Term>>> FormulaParser::getDynamicTermMetas()
{
    return dynamic_term_metas_;
}

void vfm::FormulaParser::treatNestedFunctionArgs(const std::string& token, const int i, const std::shared_ptr<std::vector<std::string>>& tokens, std::stack<int>& nested_function_args_stack, const bool valid)
{
   //if (std::find(operators_with_flexible_number_of_arguments_.begin(), operators_with_flexible_number_of_arguments_.end(), token) != operators_with_flexible_number_of_arguments_.end()) {
      int arg_num;
      if (valid) {
         arg_num = 0;

         if (i + 2 < tokens->size() && !StaticHelper::isClosingBracket((*tokens)[i + 2])) {
            arg_num++;
            int level = 0;

            for (int count = i + 2; count < tokens->size() && level >= 0; count++) {
               std::string nested_token = (*tokens)[count];

               if (StaticHelper::isArgumentDelimiter(nested_token) && level == 0) {
                  arg_num++;
               }
               else if (StaticHelper::isOpeningBracket(nested_token)) {
                  level++;
               }
               else if (StaticHelper::isClosingBracket(nested_token)) {
                  level--;
               }
            }
         }
      }
      else {
         arg_num = -1;
      }

      nested_function_args_stack.push(arg_num);
   //}
}

std::pair<std::string, StackObjectType> FormulaParser::popAndReturn(std::vector<std::pair<std::string, StackObjectType>>& stack) {
   if (stack.empty()) {
      addError(PARSING_ERROR_STRING + ": Stack ran empty.");
      return { PARSING_ERROR_STRING, StackObjectType::Other };
   }

   auto op = stack.back();
   stack.pop_back();
   return op;
}

/*
From: https://en.wikipedia.org/wiki/Shunting-yard_algorithm#The_algorithm_in_detail
while there are tokens to be read:
    read a token.
    if the token is a number, then:  push it to the output queue.
    if the token is a function then: push it onto the operator stack
   
    if the token is an operator, then:
        while ((there is a function at the top of the operator stack)
               or (there is an operator at the top of the operator stack with greater precedence)
               or (the operator at the top of the operator stack has cmp_equal precedence and is left associative))
              and (the operator at the top of the operator stack is not a left bracket):
            pop operators from the operator stack onto the output queue.
        push it onto the operator stack.
    if the token is a left bracket, then: push it onto the operator stack.
    if the token is a right bracket, then:
        while the operator at the top of the operator stack is not a left bracket:
            pop the operator from the operator stack onto the output queue.
        pop the left bracket from the stack.

while there are still operator tokens on the stack:
   pop the operator from the operator stack onto the output queue. 
*/
// ^^ OLD ^^ vv NEW vv (TODO: Remove these comments once function considered stable.)
/*
    #SOLANGE Tokens verfügbar sind:
        #WENN Token IST-Zahl: Token ZU Ausgabe.
        #WENN Token IST-Funktion: Token ZU Stack.
        #WENN Token IST-Argumenttrennzeichen:
        #    SOLANGE Stack-Spitze IST nicht öffnende-Klammer:
        #        Stack-Spitze ZU Ausgabe.
        #WENN Token IST-Operator
        #    SOLANGE Stack IST-NICHT-LEER UND
        #            Stack-Spitze IST Operator UND
        #            Token IST-linksassoziativ UND
        #            Präzedenz von Token IST-KLEINER-GLEICH Präzedenz von Stack-Spitze
        #        Stack-Spitze ZU Ausgabe.
        #    Token ZU Stack.
        #WENN Token IST öffnende-Klammer: Token ZU Stack.   
        #WENN Token IST schließende-Klammer:
        #    SOLANGE Stack-Spitze IST nicht öffnende-Klammer:
        #        Stack-Spitze ZU Ausgabe.
        #    Stack-Spitze (öffnende-Klammer) entfernen
        #    WENN Stack-Spitze IST-Funktion: Stack-Spitze ZU Ausgabe.
    #SOLANGE Stack IST-NICHT-LEER: Stack-Spitze ZU Ausgabe.
*/
std::shared_ptr<Term> FormulaParser::parseShuntingYard(const std::shared_ptr<std::vector<std::string>>& tokens)
{
#if defined(FORMULA_PARSER_DEBUG)
   addDebug("", true, "");
   addDebugPlain("Parsing formula: '", "");
   for (const auto& token : *tokens) {
      addDebugPlain(token + " ", "");
   }
   addDebugPlain("'.", "\n");
#endif

   curr_terms_.clear();
   std::vector<std::pair<std::string, StackObjectType>> op_stack{};
   std::stack<int> nested_function_args_stack{};

   for (int i = 0; i < tokens->size(); i++) {
      auto token = (*tokens)[i]; // TODO: Find out if we can use reference here. Currently not possible due to auto-insertion of brackets for "-x" type of formula.

      if (token.empty()) continue; // For func_ref_mode we possibly insert empty tokens, see below.

      if (StaticHelper::isArgumentDelimiter(token)) { // Delimiter.
         std::string el = op_stack.back().first;
          while (!StaticHelper::isOpeningBracket(el)) {            
            pushToOutput(el, nested_function_args_stack);
            if (op_stack.empty()) {
               addError("Stack ran out of items.");
               return std::make_shared<TermVar>(PARSING_ERROR_STRING); // TODO: For now just break it off. In future we'd like to have a more meaningful tree even on errors.
            }
            else {
               op_stack.pop_back();

               if (op_stack.empty()) {
                  addError("Stack ran out of items.");
                  return std::make_shared<TermVar>(PARSING_ERROR_STRING); // TODO: For now just break it off. In future we'd like to have a more meaningful tree even on errors.
               } else {
                  el = op_stack.back().first;
               }
            }
         }
      }
      else if (StaticHelper::isOpeningBracket(token)) { // Left bracket.
         op_stack.push_back({ token, StackObjectType::Other });
      }
      else if (StaticHelper::isClosingBracket(token)) { // Right bracket.
         if (op_stack.empty()) {
            addError("Found closing bracket without matching opening bracket.");
            //pushToOutput(PARSING_ERROR_STRING, nested_function_args_stack);
            return std::make_shared<TermVar>(PARSING_ERROR_STRING); // TODO: For now just break it off. In future we'd like to have a more meaningful tree even on errors.
         }

         while (!StaticHelper::isOpeningBracket(op_stack.back().first)) {
            std::string el = popAndReturn(op_stack).first;
            pushToOutput(el, nested_function_args_stack);

            if (op_stack.empty()) {
               addError("Found closing bracket without matching opening bracket.");
               //pushToOutput(PARSING_ERROR_STRING, nested_function_args_stack);
               return std::make_shared<TermVar>(PARSING_ERROR_STRING); // TODO: For now just break it off. In future we'd like to have a more meaningful tree even on errors.
            }
         }

         if (op_stack.empty()) {
            addError("Unexpectedly run out of stack items.");
            return std::make_shared<TermVar>(PARSING_ERROR_STRING); // TODO: For now just break it off. In future we'd like to have a more meaningful tree even on errors.
         }
         else {
            op_stack.pop_back();
         }

         if (!op_stack.empty() && op_stack.back().second == StackObjectType::Function) {
            std::string el = popAndReturn(op_stack).first;

            //if (all_ops_.at(el).count())

            pushToOutput(el, nested_function_args_stack);
         }
      }
      else if (!is_in_function_ref_mode_ && isFunctionOrOperator(token) && i > 0 
           && (StaticHelper::isClosingBracket(tokens->at(i - 1))
            || StaticHelper::isConstVal(tokens->at(i - 1))
            || StaticHelper::isConstVar(tokens->at(i - 1)))) { // Operator.
         while (!op_stack.empty()
            && op_stack.back().second == StackObjectType::Operator
            && isLeftAssoc(op_stack.back().first, 2)
            && prio(token, 2) <= prio(op_stack.back().first, 2)) {

            std::string el = popAndReturn(op_stack).first;
            pushToOutput(el, nested_function_args_stack);
         }

         op_stack.push_back({ token, StackObjectType::Operator });
         treatNestedFunctionArgs(token, i, tokens, nested_function_args_stack, false);
      }
      else if (!is_in_function_ref_mode_ && isFunctionOrOperator(token)) { // Function, since we ruled out operator before.
         if (i + 1 < tokens->size()) { // At least 1 element after i.
            if (!StaticHelper::isOpeningBracket(tokens->at(i + 1))) { // We have a "-x" situation, i.e., the brackets are missing.
               tokens->insert(tokens->begin() + i + 1, StaticHelper::makeString(OPENING_BRACKET));
               int j = i + 3;

               if (j < tokens->size() && StaticHelper::isOpeningBracket(tokens->at(j))) { // We have a "-sqrt(...)" situation.
                  int bcnt = 1;

                  while (bcnt != 0) {
                     j++;

                     if (StaticHelper::isOpeningBracket(tokens->at(j))) { // Assuming at this point no literal strings etc. left in the formula.
                        bcnt++;
                     }
                     else if (StaticHelper::isClosingBracket(tokens->at(j))) {
                        bcnt--;
                     }
                  }

               }
               else {
                  while (j - 1 < tokens->size() && isFunctionOrOperator(tokens->at(j - 1))) {
                     j++;
                  }
               }

               if (j <= tokens->size()) { // TODO: Is this the right condition (or possibly off-by-one error)? Seems to be ok... But think once more about it...
                  tokens->insert(tokens->begin() + j, StaticHelper::makeString(CLOSING_BRACKET));
               }
               else {
                  addError("Syntax error in bracket-less function.");
               }
            }
         }

         if (SYMB_FUNC_REF == token) {
            int j = i + 3; // @f(*some_complex_token*, ...) ==> We're at @f, but need to start at second part of or directly after *some_complex_token*.

            while (j < tokens->size() && !StaticHelper::isArgumentDelimiter(tokens->at(j)) && !StaticHelper::isClosingBracket(tokens->at(j))) {
               // Glue together whatever might come since we want to capture numbers, letters, special characters etc. as one token.
               tokens->at(i + 2) += tokens->at(j);
               tokens->at(j) = "";
               j++;
            }

            is_in_function_ref_mode_ = true;
         }

         op_stack.push_back({ token, StackObjectType::Function });
         treatNestedFunctionArgs(token, i, tokens, nested_function_args_stack, true);
      }
      else { // Constant.
         if (i < tokens->size() - 1 && StaticHelper::isOpeningBracket((*tokens)[i + 1])) {
               addError("Undeclared function name '" + token + "'.");
               //pushToOutput(PARSING_ERROR_STRING, nested_function_args_stack);
               return std::make_shared<TermVar>(PARSING_ERROR_STRING); // TODO: For now just break it off. In future we'd like to have a more meaningful tree even on errors.
         }

         pushToOutput(token, nested_function_args_stack);
      }
   }
   
   while (!op_stack.empty()) {
      std::string el = popAndReturn(op_stack).first;
      pushToOutput(el, nested_function_args_stack);
   }

   if (curr_terms_.size() != 1) {
      std::string error_message;
      error_message += "" + std::to_string(curr_terms_.size()) + " elements remained on output tape (only 1 allowed). Tokens: | ";
      
      for (const auto& token : *tokens) {
         error_message += token + " | ";
      }

      error_message += "\nRemaining tokens on output tape: | ";

      for (const auto& token : curr_terms_) {
         error_message += (token ? token->serialize() : "nullptr") + " | ";
      }

      addError(error_message);
      
      return std::make_shared<TermVar>(PARSING_ERROR_STRING); // TODO: For now just break it off. In future we'd like to have a more meaningful tree even on errors.
   }

   return curr_terms_[0];
}

std::string vfm::FormulaParser::toStringCompoundFunctions(const bool flatten) const
{
   static const std::set<std::string> vec{ "arrayk", "Getk", "fib", "q_func" };
   std::string s;

   if (flatten) {
      addWarning("Flattening all functions can crash via stack overflow since the 'flattenFormula' method currently doesn't check for recursive functions. The following known recursive functions are hardcoded to be cut off:");
      for (const auto& el : vec) {
         addWarningPlain(el + " ");
      }
   }

   for (const auto& term_pairs : dynamic_term_metas_) {
      for (const auto& term_pair : term_pairs.second) {
         auto op_name = term_pairs.first;

         addDebug("Processing function '" + op_name + "<" + std::to_string(term_pair.first) + ">'...");

         auto term = flatten ? MathStruct::flattenFormula(term_pair.second->copy(), vec) : term_pair.second;
         auto op_struct = all_ops_.at(op_name).at(term_pair.first);

         s += op_struct.serialize() + "\n=\n";
         s += StaticHelper::wrap(term->serialize(), 150, {}, { ";" }) + "\n-----\n\n";
      }
   }

   return s;
}

std::string vfm::FormulaParser::serializeDynamicTerms(const bool include_native_terms)
{
   std::string s;

   for (const auto& metas : dynamic_term_metas_) {
      for (const auto& meta : metas.second) {
         if (include_native_terms || !isTermNative(metas.first, meta.first)) {
            s.append(all_ops_.at(metas.first).at(meta.first).serialize());
            s += PROGRAM_COMMAND_SEPARATOR;
            s.append("\n");
         }
      }
   }

   s.append("\n");

   for (const auto& metas : dynamic_term_metas_) {
      for (const auto& meta : metas.second) {
         if (include_native_terms || !isTermNative(metas.first, meta.first)) {
            s.append(all_ops_.at(metas.first).at(meta.first).op_name);
            s += PROGRAM_OPENING_OP_STRUCT_BRACKET_LEGACY;
            s += PROGRAM_CLOSING_OP_STRUCT_BRACKET_LEGACY;
            s.append(" ");
            s.append(PROGRAM_DEFINITION_SEPARATOR);
            s.append(" ");
            s.append(meta.second->serialize(nullptr, MathStruct::SerializationStyle::cpp, MathStruct::SerializationSpecial::none, 0, 0));
            s += PROGRAM_COMMAND_SEPARATOR;
            s.append("\n");
         }
      }
   }

   s.append("\n");

   return s;
}

std::string vfm::FormulaParser::preprocessCPPProgram(const std::string& program)
{
   return StaticHelper::replaceAll(StaticHelper::replaceAll(program, "};", "}"), "}", "};");
}

void vfm::FormulaParser::postprocessFormulaFromCPP(const std::shared_ptr<MathStruct> formula, const std::map<std::string, std::set<int>>& pointerized) {
   formula->applyToMeAndMyChildren([&pointerized, this](const std::shared_ptr<MathStruct> m) {
      auto auto_inject_pointers{ pointerized.find(m->getOptor()) };

      if (auto_inject_pointers != pointerized.end()) {
         for (const auto& arg_num : auto_inject_pointers->second) {
            if (arg_num < m->getOperands().size() && m->getOperands()[arg_num]->isTermVar()) {
               auto m_var{ m->getOperands()[arg_num]->toVariableIfApplicable() };
               m_var->setVariableName(SYMB_REF + m_var->getVariableName());
            }
            else {
               std::string reason{ arg_num >= m->getOperands().size()
                  ? "Operator '" + m->getOptor() + "' takes only " + std::to_string(m->getOperands().size()) + " parameter(s)."
                  : "Operand '" + m->getOperands()[arg_num]->serialize() + "' is not a variable." };

               addError("Cannot auto-inject pointer into argument " 
                        + std::to_string(arg_num) + " of operation '" + m->serialize() + "'. " + reason
               );
            }
         }
      }
   });
}

const auto open_brace = StaticHelper::makeString(OPENING_BRACKET_BRACE_STYLE);
const auto close_brace = StaticHelper::makeString(CLOSING_BRACKET_BRACE_STYLE);
const auto open_square = StaticHelper::makeString(OPENING_BRACKET_SQUARE_STYLE_FOR_ARRAYS);
const auto close_square = StaticHelper::makeString(CLOSING_BRACKET_SQUARE_STYLE_FOR_ARRAYS);
const auto open_round = StaticHelper::makeString(OPENING_BRACKET);
const auto close_round = StaticHelper::makeString(CLOSING_BRACKET);
const auto comma = StaticHelper::makeString(ARGUMENT_DELIMITER);

std::string vfm::FormulaParser::preprocessProgram(const std::string& program) const
{
   return preprocessProgramBraceStyleFunctionDenotation(preprocessProgramSquareStyleArrayAccess(program));
}

std::string vfm::FormulaParser::preprocessProgramBraceStyleFunctionDenotation(const std::string& program) const
{
   std::string prog_result = program;
   int ind_op_brace = StaticHelper::indexOfFirstInnermostBeginBracket(prog_result, open_brace, close_brace, { "\"" }, { "\"" });

   while (ind_op_brace >= 0) {
      int ind_cl_brace = StaticHelper::findMatchingEndTagLevelwise(prog_result, open_brace, close_brace, ind_op_brace, { "\"" }, { "\"" });

      if (ind_cl_brace >= 0) {
         int closing = prog_result.rfind(CLOSING_BRACKET, ind_op_brace);
         int opening = StaticHelper::findMatchingBegTagLevelwise(prog_result, open_round, close_round, closing, { "\"" }, { "\"" });

         if (closing < 0 || opening < 0) {
             return program; // Malformed program wrt. braces.
         }

         if (prog_result[closing] == CLOSING_BRACKET && prog_result[opening] == OPENING_BRACKET) {
            std::string innerPart = prog_result.substr(ind_op_brace + 1, ind_cl_brace - ind_op_brace - 1);
            std::string before = prog_result.substr(0, opening + 1);
            std::string before_inner = prog_result.substr(opening + 1, closing - opening - 1);
            std::string after = prog_result.substr(ind_cl_brace + 1);
            std::string first_inner_part = before_inner;
            std::string second_inner_part = "";
            int cc = 0;

            if (!StaticHelper::isEmptyExceptWhiteSpaces(innerPart)) {
               second_inner_part = innerPart;
               cc++;
            }

            if (!StaticHelper::isEmptyExceptWhiteSpaces(first_inner_part)) {
               first_inner_part = before_inner;
               cc++;
            }

            prog_result = before 
               + first_inner_part 
               + (cc == 2 ? StaticHelper::makeString(PROGRAM_ARGUMENT_DELIMITER) : "")
               + second_inner_part
               + CLOSING_BRACKET + after;

            ind_op_brace = StaticHelper::indexOfFirstInnermostBeginBracket(prog_result, open_brace, close_brace, { "\"" }, { "\"" });
         }
      }
      else {
         break;
      }
   }

   return prog_result;
}

std::string vfm::FormulaParser::preprocessProgramSquareStyleArrayAccess(const std::string& program) const
{
   int ind_op_square = StaticHelper::indexOfFirstInnermostBeginBracket(program, open_square, close_square, { "\"" }, { "\"" });

   if (ind_op_square >= 0) {
      std::string prog_result = program;
      int ind_cl_square = StaticHelper::findMatchingEndTagLevelwise(program, open_square, close_square, ind_op_square, { "\"" }, { "\"" });

      if (ind_cl_square >= 0) {
         int ind_pos_of_array_name_begin = ind_op_square - 1;
         std::string between_brackets = program.substr(ind_op_square + 1, ind_cl_square - ind_op_square - 1);
         std::string arr_name = StaticHelper::tokenize(program, *const_cast<FormulaParser*>(this), ind_pos_of_array_name_begin, 1, true)->at(0);
         std::string before{};

         if (arr_name == close_round) { // It's something like "(ego dot gaps)[0]".
            int ind_pos_of_array_name_begin_old = ind_pos_of_array_name_begin;
            ind_pos_of_array_name_begin = StaticHelper::findMatchingBegTagLevelwise(program, open_round, close_round, ind_pos_of_array_name_begin + 2);
            arr_name = program.substr(ind_pos_of_array_name_begin, ind_pos_of_array_name_begin_old - ind_pos_of_array_name_begin + 3);
            before = program.substr(0, ind_pos_of_array_name_begin);
         }
         else {
            before = program.substr(0, ind_pos_of_array_name_begin + 1);
         }
         std::string after = program.substr(ind_cl_square + 1);
         bool isGetter = !StaticHelper::stringStartsWith(arr_name, SYMB_REF);
         std::vector<std::string> arguments;
         std::string arr_name_plain = isGetter ? arr_name : arr_name.substr(SYMB_REF.size());
         
         int current_pos = ind_op_square + 1;

         while (program[current_pos - 1] != CLOSING_BRACKET_SQUARE_STYLE_FOR_ARRAYS) {
            int ind_of_comma = StaticHelper::indexOfOnTopLevel(program, { comma, close_square }, current_pos, open_square, close_square, { "\"" }, { "\"" });
            std::string argument = program.substr(current_pos, ind_of_comma - current_pos);
            StaticHelper::trim(argument);

            if (!argument.empty()) {
               argument = open_round + argument + close_round;
               arguments.push_back(argument);
            }

            current_pos = ind_of_comma + 1;
         }

         std::string replacement = arr_name; // Special cases for 0-d array, i.e. single cell: arr[] ==> arr; @arr[] ==> @arr.

         if (!arguments.empty()) {
            replacement = arr_name_plain + SYMB_PLUS + arguments[0];

            // Setter: @arr[x, y, z] ==> get(get(arr + x) + y) + z
            for (int i = 1; i < arguments.size(); i++) {
               replacement = SYMB_GET_VAR + open_round + replacement + close_round + SYMB_PLUS + arguments[i];
            }

            if (isGetter) { // Getter = Setter + "get": arr[x, y, z] ==> get(@arr[x, y, z])
               replacement = SYMB_GET_VAR + open_round + replacement + close_round;
            }
         }

         prog_result = before + replacement + after;

         return preprocessProgramSquareStyleArrayAccess(prog_result);
      }
   }

   return program;
}

bool vfm::FormulaParser::parseProgram(const std::string& dynamic_term_commands)
{
   bool error_in_current_call = false;
   const auto commands = StaticHelper::split(StaticHelper::removeSingleLineComments(dynamic_term_commands), PROGRAM_COMMAND_SEPARATOR);

   for (const auto& command : commands) {
      if (!StaticHelper::isEmptyExceptWhiteSpaces(command)) {
         bool error_in_current_command = false;
         const auto pair = StaticHelper::split(command, PROGRAM_DEFINITION_SEPARATOR);

         if (pair.size() < 1 || pair.size() > 2) {
            error_in_current_command = true;
         }

         bool op_struct_error = false;
         OperatorStructure op_struct(pair[0], op_struct_error);

         if (pair.size() == 1) {
            forwardDeclareDynamicTerm(op_struct);
            error_in_current_command = op_struct_error;
         }
         else if (pair.size() == 2) {
            auto meta_term = MathStruct::parseMathStruct(pair[1], *this, true);

            if (meta_term) {
               addDynamicTerm(op_struct, meta_term->toTermIfApplicable());
            }

            error_in_current_command = op_struct_error || !meta_term || meta_term->serialize() == PARSING_ERROR_STRING;
         }

         if (error_in_current_command) {
            std::string copy = command;
            StaticHelper::trim(copy);

            const std::string error_command_msg = "Could not parse command '" + copy + "'.";
            addError(error_command_msg);
            error_in_current_call = error_in_current_call || error_in_current_command; 
         }
      }
   }

   return !error_in_current_call;
}

std::map<std::string, std::map<int, OperatorStructure>>& vfm::FormulaParser::getAllOps()
{
   return all_ops_;
}

void vfm::FormulaParser::checkForUndeclaredVariables(const std::shared_ptr<DataPack> d)
{
   for (const auto& v : parsed_variables_) {
      // if (!d->isVarDeclared(v) && !StaticHelper::isPrivateVar(v)) {
      //    addWarning("The variable '" + v + "' has not been declared."); // Nobody cares...
      // }
   }
}

void vfm::FormulaParser::simplifyFunctions(const bool mc_mode, bool include_those_with_metas)
{
   int i{ 1 };
   int max{ 0 };

   for (const auto& meta : dynamic_term_metas_) {
      max += meta.second.size();
   }

   for (auto& ts : dynamic_term_metas_) {
      for (auto& t : ts.second) {
         auto term_str{ std::to_string(i) + "/" + std::to_string(max) + " '" + ts.first };

         if (include_those_with_metas || !t.second->containsMetaCompound()) { // TODO: MetaCompound correct here?
            auto before{ std::to_string(t.second->getNodeCount(TraverseCompoundsType::go_into_compound_structures)) };
            t.second->setChildrensFathers(true, false);
            auto copy{ t.second->copy() };
            t.second = mc_mode ? mc::simplification::simplifyFast(t.second, shared_from_this()) : vfm::simplification::simplifyFast(t.second, shared_from_this());
            auto after{ std::to_string(t.second->getNodeCount(TraverseCompoundsType::go_into_compound_structures)) };

            if (!copy->isStructurallyEqual(t.second)) {
               addDebug("Simplified dynamic term meta " + term_str + "'. Compression " + before + " ==> " + after + " nodes.");
               addDebug("BEFORE:\n" + copy->serialize());
               addDebug("AFTER:\n" + t.second->serialize());
            }
            else {
               addDebug("Simplified dynamic term meta " + term_str + "'. NO CHANGE.");
            }
         }
         else {
            addDebug("Skipping dynamic term meta " + term_str + "' due to meta in definition.");
         }

         i++;
      }
   }
}

bool vfm::FormulaParser::isFunctionDeclared(const std::string& func_name, const int num_params) const
{
   return names_to_addresses.count({ func_name, num_params });
}

bool vfm::FormulaParser::isFunctionDeclared(const int func_id) const
{
   return addresses_to_names.count(func_id);
}

bool vfm::FormulaParser::isFunctionDefined(const std::string& func_name, const int num_params_raw) const
{
   int num_params = num_params_raw;

   if (num_params < 0) {
      auto possible_num_params = const_cast<FormulaParser*>(this)->getNumParams(func_name); // TODO: Make getNumParams const (easy!!).

      if (possible_num_params.size() == 1) {
         num_params = *possible_num_params.begin();
      }
      else {
         return false;
      }
   }

   TermPtr term = nullptr;
   auto fmla_all_it = dynamic_term_metas_.find(func_name);

   if (fmla_all_it != dynamic_term_metas_.end()) {
      auto fmla_it = fmla_all_it->second.find(num_params);

      if (fmla_it != fmla_all_it->second.end()) {
         term = fmla_it->second;
      }
   }
   
   if (!term) { // Use term factory. (Cannot use just this call since term factory possibly returns an unfinished forward-declared term.)
      term = const_cast<FormulaParser*>(this)->termFactoryDummy(func_name, num_params);
   }

   return term != nullptr;
}

int FormulaParser::getFunctionAddressOf(const std::string& func_name, const int num_params) const
{
   return names_to_addresses.at({ func_name, num_params });
}

std::map<int, std::pair<std::string, int>> vfm::FormulaParser::getAllDeclaredFunctions() const
{
   return addresses_to_names;
}

std::map<int, std::string> vfm::FormulaParser::getParameterNamesFor(const std::string dynamic_term_name, const int num_overloaded_params) const
{
   if (!naming_of_parameters_.count({ dynamic_term_name, num_overloaded_params })) {
      std::map<int, std::string> dummy_res{};

      for (int i = 0; i < num_overloaded_params; i++) {
         dummy_res.insert({ i, "p_(" + std::to_string(i) + ")" });
      }

      return dummy_res;
   }

   return naming_of_parameters_.at({ dynamic_term_name, num_overloaded_params });
}

void FormulaParser::addDotOperator()
{
   OperatorStructure dot_str(OutputFormatEnum::infix, AssociativityTypeEnum::left, 2, PRIO_DOT, SYMB_DOT, true, false);
   OperatorStructure dot_alt_str(OutputFormatEnum::infix, AssociativityTypeEnum::left, 2, PRIO_DOT, SYMB_DOT_ALT, true, false);

   addDynamicTerm(
      dot_str,
      _plus(_meta_compound(0), _meta_compound(1)),
      true); // dot() := p_(0) + p_(1);

   addDynamicTerm(
      dot_alt_str,
      _plus(_meta_compound(0), _meta_compound(1)),
      true); // .() := p_(0) + p_(1);
}

void FormulaParser::addnuSMVOperators(const TLType mode) 
{
   for (const auto& pair : TermCompound::getNuSMVOperators()) {
      if (pair.second.second == TLType::both || pair.second.second == mode || mode == TLType::both) {
         TermPtr operation{};

         if (pair.second.first.op_name == SYMB_LTL_IFF) {
            operation = _eq(_boolify(_meta_compound(0)), _boolify(_meta_compound(1)));
         }
         else if (pair.second.first.op_name == SYMB_LTL_IMPLICATION) {
            operation = _or(_not(_meta_compound(0)), _meta_compound(1));
         }
         else if (pair.second.first.op_name == SYMB_LTL_XOR) {
            operation = _or(_and(_not(_meta_compound(0)), _meta_compound(1)), _and(_not(_meta_compound(1)), _meta_compound(0)));
         }
         else if (pair.second.first.op_name == SYMB_LTL_XNOR) {
            operation = _not(_or(_and(_not(_meta_compound(0)), _meta_compound(1)), _and(_not(_meta_compound(1)), _meta_compound(0))));
         }
         else {
            operation = _meta_compound(0); // Dummy operation in all other cases.
         }

         addDynamicTerm(pair.second.first, operation);
      }
   }
}

std::pair<std::string, int> vfm::FormulaParser::getFunctionName(const int func_address) const
{
   return addresses_to_names.at(func_address);
}

// TODO: This function should be obsolete since we have a generic
// overloading mechanism now. Commented-out for now, feel free to delete later.
//void vfm::FormulaParser::requestFlexibleNumberOfArgumentsFor(const std::string& op_name)
//{
//   operators_with_flexible_number_of_arguments_.insert(op_name);
//}

void FormulaParser::registerAddress(const std::string& func_name, const int num_params)
{
   if (num_params >= 0 && !names_to_addresses.count({ func_name, num_params })) {
      names_to_addresses.insert({ { func_name, num_params }, function_count_ });
      addresses_to_names.insert({ function_count_, { func_name, num_params } });
      ++function_count_;
   }
}

std::string vfm::FormulaParser::getStartSymbol(const int i)
{
   return i ? GRAMMAR_START_SYMBOL_PREFIX + std::to_string(i) : GRAMMAR_SIMPLE_START_SYMBOL;
}

std::shared_ptr<Term> vfm::FormulaParser::parseFromSExpression(const std::string & s_expression)
{
   // TODO
   return nullptr;
}

std::shared_ptr<Term> vfm::FormulaParser::parseFromPostfix(const std::string& postfix)
{
   size_t pos = 0;
   size_t pos_old = 0;
   std::stack<std::shared_ptr<Term>> stack;

   while(pos != std::string::npos) {
      pos = postfix.find(POSTFIX_DELIMITER, pos + 1);
      std::string op = postfix.substr(pos_old, pos - pos_old);

      if (isFunctionOrOperator(op)) {
         std::vector<std::shared_ptr<Term>> vec;

         auto argNums = getNumParams(op);

         if (argNums.size() > 1) {
            addError("Parsing from postfix not possible since '" + op + "' is an overloaded operator (" + std::to_string(argNums.size()) + " overloads).");
         }
         else if (argNums.empty()) {
            addError("Parsing from postfix not possible since '" + op + "' is an undefined operator. Returning 0.");
            return _val(0);
         }

         for (int i = 0; i < *argNums.begin(); i++) {
            vec.insert(vec.begin(), stack.top());
            stack.pop();
         }

         stack.push(termFactory(op, vec));
      } else
      {
         float f;
         std::istringstream iss(op);
         iss >> std::noskipws >> f;
         stack.push(iss.eof() && !iss.fail() ? _val(f) : _var(op));
      }
      
      pos_old = pos + POSTFIX_DELIMITER.size();
   }

   return stack.top();
}

// Connection to STL library.

/// All STL operators not available in native vfm need dummy functions to allow parsing.
void addSTLMathOperators(fsm::FSMs& m) {
   const auto ONE_PAR_DUMMY    = "p_(0)"; 
   const auto TWO_PARS_DUMMY   = "p_(0); p_(1)"; 
   const auto THREE_PARS_DUMMY = "p_(0); p_(1); p_(2)"; 
   const auto FOUR_PARS_DUMMY  = "p_(0); p_(1); p_(2); p_(3)"; 
   const auto FIVE_PARS_DUMMY  = "p_(0); p_(1); p_(2); p_(3); p_(4)"; 

   m.defineNewMathOperator("alwaysSinceBeginning", ONE_PAR_DUMMY);      // exp0
   m.defineNewMathOperator("atLeastOnceSinceBeginning", ONE_PAR_DUMMY);
   m.defineNewMathOperator("doubleToRob", ONE_PAR_DUMMY);
   m.defineNewMathOperator("zeroCrossingDetect", ONE_PAR_DUMMY);
   m.defineNewMathOperator("stepDetect", ONE_PAR_DUMMY);

   m.defineNewMathOperator("==>", TWO_PARS_DUMMY);         // exp0, exp1 (, int)
   m.defineNewMathOperator("freeze", TWO_PARS_DUMMY);
   m.defineNewMathOperator("synch", TWO_PARS_DUMMY);
   m.defineNewMathOperator("discretize", TWO_PARS_DUMMY);
   m.defineNewMathOperator("squeeze", TWO_PARS_DUMMY);
   m.defineNewMathOperator("ident_scale", TWO_PARS_DUMMY);

   m.defineNewMathOperator("&_", THREE_PARS_DUMMY);           // exp0, exp1, exp2 (, int)
   m.defineNewMathOperator("|_", THREE_PARS_DUMMY);
   m.defineNewMathOperator("synch_", THREE_PARS_DUMMY);
   m.defineNewMathOperator("discretize_", THREE_PARS_DUMMY);
   m.defineNewMathOperator("sqeeze_", THREE_PARS_DUMMY);
   m.defineNewMathOperator("ident_scale_", THREE_PARS_DUMMY);
   m.defineNewMathOperator("between", THREE_PARS_DUMMY);

   m.defineNewMathOperator("always", THREE_PARS_DUMMY);      // startTime, endTime, exp0
   m.defineNewMathOperator("atLeastOnce", THREE_PARS_DUMMY);

   m.defineNewMathOperator("until", FOUR_PARS_DUMMY);                // startTime, endTime, exp0, exp1
   m.defineNewMathOperator("timeDistance", FOUR_PARS_DUMMY);
   m.defineNewMathOperator("timeDistanceGlobally", FOUR_PARS_DUMMY);

   m.defineNewMathOperator("timeDistanceVariable", FIVE_PARS_DUMMY);         // startTime, endTime, exp0, exp1, exp2
   m.defineNewMathOperator("timeDistanceGloballyVariable", FIVE_PARS_DUMMY);

   m.defineNewMathOperator("approximatelyConstant", FOUR_PARS_DUMMY); // startTime, endTime, epsilon, exp0
}

const char ARRAY_NAMES_DELIMITER = ','; // Can be ", " -- white spaces are stripped.

extern "C" float evaluateProgramAndFormula(
   const char* program,
   const char* stl_formula,
   const float* arr,
   const int cols,
   const int rows,
   const char* array_names,
   const char* path_to_base_dir)
{
   float res = -std::numeric_limits<float>::infinity();
   fsm::FSMs m; // Use the convenience functions of the FSM class even though no actual FSM is used.
   auto d = m.getData();

   m.setOutputLevels(ErrorLevelEnum::note, ErrorLevelEnum::error);
   addSTLMathOperators(m);

   // Organize external data.
   std::string array_names_str(array_names);
   const auto split_array_names = StaticHelper::split(array_names, ARRAY_NAMES_DELIMITER);
   auto split_array_names_preprocessed = split_array_names;
   std::string curr_name;

   int j = 0;
   for (size_t i = 0; i < rows * cols; ++i) {
      if (i % cols == 0) {
         curr_name = StaticHelper::removeWhiteSpace(split_array_names[j]);
         curr_name = StaticHelper::replaceAll(curr_name, ".", "__"); // Replace namespace dot from evald with underscores.
         split_array_names_preprocessed[j] = curr_name;
         j++;
      }

      m.getData()->addArrayAndOrSetArrayVal(curr_name, i % cols, arr[i], ArrayMode::floats);
   }

   // Preprocess and parse formula, call "main" function as entry point for preprocessing.
   std::string stl_formula_preprocessed = stl_formula;
   std::string program_preprocessed = program;

   for (size_t i = 0; i < split_array_names.size(); i++) {
      std::string name_old = split_array_names[i];
      std::string name_new = split_array_names_preprocessed[i];

      if (split_array_names[i] != split_array_names_preprocessed[i]) { // Adapt to new array names without dots.
         stl_formula_preprocessed = StaticHelper::replaceAll(stl_formula_preprocessed, name_old, name_new);
         program_preprocessed = StaticHelper::replaceAll(program_preprocessed, name_old, name_new);
      }
   }

   m.parseProgram(program_preprocessed);
   m.evaluateFormula("main()");

   // Parse STL formula and re-print in postfix representation for transmission to the STL lib.
   auto stl_formula_vfm = MathStruct::parseMathStruct(stl_formula_preprocessed, true, false, m.getParser());
   std::string stl_formula_postfix = stl_formula_vfm->serializePostfix();

   // Extract preprocessed arrays from data pack for the STL lib.
   std::vector<std::string> arr_names_new;

   stl_formula_vfm->applyToMeAndMyChildren([&arr_names_new](const std::shared_ptr<MathStruct> m){
      if (m->isTermArray()) {
         arr_names_new.push_back(m->toArrayIfApplicable()->getOpStruct().op_name);
      }
   });

   int stl_cols = 0;
   int stl_rows = arr_names_new.size();
   std::string stl_array_names_str;
   
   for (const auto& a : arr_names_new) {
      stl_array_names_str += a + ARRAY_NAMES_DELIMITER;
      stl_cols = std::max(stl_cols, d->getArraySize(a)); // If arrays have different sizes, use max and pad with zeros later.
   }
   
   std::vector<float> stl_vec;
   stl_vec.reserve(stl_rows * stl_cols);

   for (const auto& a : arr_names_new) {
      for (int j = 0; j < stl_cols; j++) {
         stl_vec.push_back(d->getValFromArray(a, j)); // It might be faster to insert the whole array at once, but this way, zero padding is implicitly given.
      }
   }

   m.addNote("This is the vfm DATA used:\n" + m.getData()->toString());
   m.addNote("STL formula: " + stl_formula_vfm->serialize());
   m.addNote("STL formula postfix: " + stl_formula_postfix + "\n\n");

#ifndef _WIN32
#ifdef ACTIVATE_CONNECTION_TO_STLLIB
   // Run STL evaluation via dynamic call to the pre-compiled library.
   std::string str(path_to_base_dir); // Gains something like: /home/okl2abt/workspace/evald
   std::string path = str + "/lib/sbt-toolbox/CoreLibrary/linux/STLLib_Release/libstl.so";

   void* handle = dlopen(path.c_str(), RTLD_LAZY);    
   auto external_call = (float (*)(const char*, const int, const float*, const int, const int, const char*, const int)) dlsym(handle, "retrieveExternalData");
   res = external_call(stl_formula_postfix.c_str(), stl_formula_postfix.size(), stl_vec.data(), stl_cols, stl_rows, stl_array_names_str.c_str(), stl_array_names_str.size());
   dlclose(handle);
#endif
#endif

   return res;
}

std::shared_ptr<FormulaParser> SingletonFormulaParser::instance_;
std::shared_ptr<FormulaParser> SingletonFormulaParser::light_instance_;
std::map<std::shared_ptr<FormulaParser>, std::shared_ptr<earley::Grammar>> SingletonFormulaGrammar::instances_;

std::shared_ptr<FormulaParser> vfm::SingletonFormulaParser::getInstance()
{
   if (!instance_) {
      instance_ = std::make_shared<FormulaParser>();
      instance_->addDefaultDynamicTerms();
   }

   return instance_;
}

std::shared_ptr<FormulaParser> vfm::SingletonFormulaParser::getLightInstance()
{
   if (!light_instance_) {
      light_instance_ = std::make_shared<FormulaParser>();
   }

   return light_instance_;
}

earley::Grammar& vfm::SingletonFormulaGrammar::getInstance(const std::shared_ptr<FormulaParser> ref_parser)
{
   auto exist = instances_.find(ref_parser);
   std::shared_ptr<earley::Grammar> res;

   if (exist == instances_.end()) {
      res = ref_parser->createGrammar();
      instances_.insert({ ref_parser, res });
   }
   else {
      res = exist->second;
   }

   return *res;
}

earley::Grammar& vfm::SingletonFormulaGrammar::getInstance()
{
   return SingletonFormulaGrammar::getInstance(SingletonFormulaParser::getInstance());
}

earley::Grammar& vfm::SingletonFormulaGrammar::getLightInstance()
{
   return SingletonFormulaGrammar::getInstance(SingletonFormulaParser::getLightInstance());
}

void vfm::FormulaParser::applyFormulaToParser(const std::string& formula_str, const std::shared_ptr<DataPack> data)
{
   MathStruct::parseMathStruct(formula_str, shared_from_this(), data);
}
