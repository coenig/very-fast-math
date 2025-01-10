//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file
#pragma once
#include "operator_structure.h"
#include <set>
#include <vector>
#include <regex>
#include <functional>
#include <set>
#include <map>

#if defined(ASMJIT_ENABLED)
//#pragma GCC diagnostic push
//#pragma GCC diagnostic ignored "-Wpedantic"
//#pragma GCC diagnostic ignored "-pedantic"
//#pragma GCC diagnostic ignored "-Wunused-parameter"
//#pragma GCC diagnostic ignored "-Wignored-qualifiers"
//#pragma GCC diagnostic ignored "-Wunused-but-set-parameter"
//#pragma GCC diagnostic ignored "-Wextra"
//#pragma GCC diagnostic ignored "-Weffc++"

#include "asmjit.h"
using namespace asmjit;
//#pragma GCC diagnostic pop
#endif

namespace vfm { class MathStruct; }
std::ostream& operator<<(std::ostream &os, const vfm::MathStruct &m);

namespace vfm {

const std::string DOT_COMMAND = 
#ifdef _WIN32
      "\"C:/Program Files/Graphviz/bin/dot.exe\""; // TODO: Should be parametrizable from outside.
#else
      "dot";
#endif // _WIN32

class Term;
class Equation;
class DataPack;
class TermMetaCompound;
class TermMetaSimplification;
class TermOptional;
class TermAnyway;
class FormulaParser;
class TermSetVar;
class TermSetArr;
class TermGetArr;
class TermVar;
class TermVal;
class TermArray;
class TermCompound;
class TermCompoundOperator;
class MetaRule;

namespace earley {
   class Grammar;
}

using MathStructPtr = std::shared_ptr<MathStruct>;

enum class FSMJitLevel {
   no_jit,
   jit_cond,
   jit_full
};

enum class ConditionType {
   is_constant_and_evaluates_to_true,                                       // Definitely results in non-zero value (true) ==> isOverallConstantsAndEvaluatesToTrue.
   is_constant_and_evaluates_to_false,                                      // Definitely results in 0 (false) ==> isOverallConstantsAndEvaluatesToFalse.
   is_constant_and_evaluates_to_negative,                                   // Definitely results in below-zero value (excluding 0).
   is_constant_and_evaluates_to_positive,                                   // Definitely results in above-zero value (excluding 0).
   is_constant,                                                             // Is definitely constant with arbitrary value.
   consists_purely_of_numbers,                                              // All leaves are TermVal (and full term is, therefore, constant, except for arrays).
   is_not_constant,                                                         // Contains variable terms, i.e., may be non-constant.
   is_constant_0,                                                           // Definitely results in 0 (false) ==> direct comparison.
   is_constant_1,                                                           // Definitely results in 1 (true) ==> direct comparison.
   is_non_boolean_constant,                                                 // Definitely is constant and definitely does not result in 0 or 1.
   is_not_non_boolean_constant,                                             // Definitely is not constant or definitely does result in 0 or 1.
   is_constant_integer,                                                     // Iff it's a constant exprestion that yields a float value which is equal to an integer.
   is_constant_non_integer,                                                 // Iff it's a constant exprestion that yields a float value which is NOT equal to an integer.
   has_no_sideeffects,                                                      // Definitely has no sideeffects.
   has_sideeffects,                                                         // May have sideeffects, i.e., subterm contains setter.
   is_leaf,                                                                 // Term has no operands, i.e., variables, constants, zero-parameter functions.
   is_not_leaf,                                                             // Term has operands.
   is_variable,                                                             // It's a variable (and also a leaf).
   is_not_variable,                                                         // It's not a variable (could be a leaf, though).
   has_boolean_result,                                                      // Evaluation of this term will result in 0 or 1.
   has_non_boolean_result,                                                  // Evaluation of this term may result in values other than 0 or 1.
   of_first_two_operands_second_is_alphabetically_above,                    // Iff the term has min. two operands, and their operators are reverse-alphabetically ordered.
   first_two_operands_are_negations_of_each_other,                          // Iff the term has min. two operands (x, y), and it's true that x = !(y).
   not_is_root,                                                             // Accepts non-root terms only.
   second_operand_contained_in_first_operand,                               // Iff the term has min. two operands, and the second is contained in the first as subtree.
   second_operand_not_contained_in_first_operand,                           // Iff the term has min. two operands, and the second is NOT contained in the first as subtree.
   first_two_variables_are_equal_except_reference,                          // Iff the term has min. two operands, both are variables, and it's (a, a), (@a, a), (a, @a) or (@a, @a)
   first_two_variables_are_not_equal_except_reference,                      // The oposite of the above one.
   second_appears_as_read_variable_in_first,                                // If the second operand (without @) appears within the first one and is read there.
   second_appears_not_as_read_variable_in_first,                            // If the second operand (without @) DOES NOT appear within the first one and is read there.
   is_any_var_of_second_on_any_left_side_of_assignment_in_first,            // If any of the variables of the second operand is set in any assignment of the first operand.
   is_any_var_of_second_on_any_left_side_of_assignment_in_first_AND_INSERT, // Above PLUS: a = b; c = d; ...; x = a + c ==> a = b; c = d; ...; x = b + d
   is_any_var_of_second_on_any_left_side_earlier_AND_INSERT,                // Above, only the starting point to earlier assignments is determined automatically from one single term.
   is_no_var_of_second_on_any_left_side_of_assignment_in_first,             // If none of the variables of the second operand is set in any assignment of the first operand.
   is_commutative_operation,                                                // Iff the operator at top level is a 2-operand commutative operator.
   is_not_commutative_operation,                                            // Negation of the above.
   is_associative_operation,                                                // Iff the operator at top level is a 2-operand associative operator.
   is_not_associative_operation,                                            // Negation of the above.
   is_comparator_or_equality_operation,                                     // Iff the operation is one of { <, <=, >, >=, ==, != }.
   is_not_comparator_or_equality_operation,                                 // Negation of the above.
   contains_meta,                                                           // Iff the formula contains a meta in any sub-tree.
   contains_no_meta,                                                        // Negation of the above.
   parent_is_get,                                                           // Iff we're "x" in a "get(x)" context, where x can be an arbitrary formula.
   parent_is_not_get,                                                       // Negation of the above.
   no_check                                                                 // Everything is ok.
};

const std::map<ConditionType, std::string> condition_type_map {
   { ConditionType::is_constant_and_evaluates_to_true, "is_constant_and_evaluates_to_true" },
   { ConditionType::is_constant_and_evaluates_to_false, "is_constant_and_evaluates_to_false" },
   { ConditionType::is_constant_and_evaluates_to_negative, "is_constant_and_evaluates_to_negative" },
   { ConditionType::is_constant_and_evaluates_to_positive, "is_constant_and_evaluates_to_positive" },
   { ConditionType::is_constant, "is_constant" },
   { ConditionType::consists_purely_of_numbers, "consists_purely_of_numbers" },
   { ConditionType::is_not_constant, "is_not_constant" },
   { ConditionType::is_constant_0, "is_constant_0" },
   { ConditionType::is_constant_1, "is_constant_1" },
   { ConditionType::is_non_boolean_constant, "is_non_boolean_constant" },
   { ConditionType::is_not_non_boolean_constant, "is_not_non_boolean_constant" },
   { ConditionType::is_constant_integer, "is_constant_integer" },
   { ConditionType::is_constant_non_integer, "is_constant_non_integer" },
   { ConditionType::has_no_sideeffects, "has_no_sideeffects" },
   { ConditionType::has_sideeffects, "has_sideeffects" },
   { ConditionType::is_leaf, "is_leaf" },
   { ConditionType::is_not_leaf, "is_not_leaf" },
   { ConditionType::is_variable, "is_variable" },
   { ConditionType::is_not_variable, "is_not_variable" },
   { ConditionType::has_boolean_result, "has_boolean_result" },
   { ConditionType::has_non_boolean_result, "has_non_boolean_result" },
   { ConditionType::of_first_two_operands_second_is_alphabetically_above, "of_first_two_operands_second_is_alphabetically_above" },
   { ConditionType::first_two_operands_are_negations_of_each_other, "first_two_operands_are_negations_of_each_other" },
   { ConditionType::not_is_root, "not_is_root" },
   { ConditionType::second_operand_contained_in_first_operand, "second_operand_contained_in_first_operand" },
   { ConditionType::second_operand_not_contained_in_first_operand, "second_operand_not_contained_in_first_operand" },
   { ConditionType::first_two_variables_are_equal_except_reference, "first_two_variables_are_equal_except_reference" },
   { ConditionType::first_two_variables_are_not_equal_except_reference, "first_two_variables_are_not_equal_except_reference" },
   { ConditionType::second_appears_as_read_variable_in_first, "second_appears_as_read_variable_in_first" },
   { ConditionType::second_appears_not_as_read_variable_in_first, "second_appears_not_as_read_variable_in_first" },
   { ConditionType::is_any_var_of_second_on_any_left_side_of_assignment_in_first, "is_any_var_of_second_on_any_left_side_of_assignment_in_first" },
   { ConditionType::is_any_var_of_second_on_any_left_side_of_assignment_in_first_AND_INSERT, "is_any_var_of_second_on_any_left_side_of_assignment_in_first_AND_INSERT" },
   { ConditionType::is_any_var_of_second_on_any_left_side_earlier_AND_INSERT, "is_any_var_of_second_on_any_left_side_earlier_AND_INSERT" },
   { ConditionType::is_no_var_of_second_on_any_left_side_of_assignment_in_first, "is_no_var_of_second_on_any_left_side_of_assignment_in_first" },
   { ConditionType::is_commutative_operation, "is_commutative_operation" },
   { ConditionType::is_not_commutative_operation, "is_not_commutative_operation" },
   { ConditionType::is_associative_operation, "is_associative_operation" },
   { ConditionType::is_not_associative_operation, "is_not_associative_operation" },
   { ConditionType::is_comparator_or_equality_operation, "is_comparator_or_equality_operation" },
   { ConditionType::is_not_comparator_or_equality_operation, "is_not_comparator_or_equality_operation" },
   { ConditionType::contains_meta, "contains_meta" },
   { ConditionType::contains_no_meta, "contains_no_meta" },
   { ConditionType::parent_is_get, "parent_is_get" },
   { ConditionType::parent_is_not_get, "parent_is_not_get" },
   { ConditionType::no_check, "no_check" },
};

class ConditionTypeWrapper : public fsm::EnumWrapper<ConditionType>
{
public:
   explicit ConditionTypeWrapper(const ConditionType& enum_val) : EnumWrapper("ConditionType", condition_type_map, enum_val) {}
   explicit ConditionTypeWrapper(const int enum_val_as_num) : EnumWrapper("ConditionType", condition_type_map, enum_val_as_num) {}
   explicit ConditionTypeWrapper(const std::string& enum_val_as_string) : EnumWrapper("ConditionType", condition_type_map, enum_val_as_string) {}
   explicit ConditionTypeWrapper() : EnumWrapper("ConditionType", condition_type_map, ConditionType::no_check) {}
};

enum class FormulaTraversalType {
   PreOrder,
   PostOrder
};

enum class TraverseCompoundsType {
   go_into_compound_structures,
   avoid_compound_structures
};

enum class ArithType { // Direction of neutralization or neutralizing operations.
   left,   // "1 ** x"
   right,  // "x ** 1", "x - 0"
   both,   // "x * 0" , "x + 0"
   none
};

enum class RandomNumberType {
   Float,
   Integer,
   Bool
};

enum class AkaTalProcessing {
   none,
   do_it
};

enum class VarClassification {
   input,
   output,
   state
};

enum class DotTreatment
{
   as_varname_character,
   as_operator
};

class VariablesWithKinds {
public:
   void addInputVariable(const std::string& var_name) { input_variables_.insert(var_name); }
   void addOutputVariable(const std::string& var_name) { output_variables_.insert(var_name); }

   std::set<std::string> getInputVariables() const { return input_variables_; }
   std::set<std::string> getOutputVariables() const { return output_variables_; }

   void clearOutputVariables() { output_variables_.clear(); }
   void clearInputVariables() { input_variables_.clear(); }

   void eraseFromInputVariables(const std::string& var) { input_variables_.erase(var); }
   void eraseFromOutputVariables(const std::string& var) { output_variables_.erase(var); }

   void renameVariablesForKratos();

   std::set<std::string> getVariablesOfSpecifiedKindForMainSMV(
      const VarClassification kind,
      const std::map<std::string, std::string>& all_variables_except_immutable_enum_and_bool_with_type) const
   {
      std::set<std::string> variables{};

      if (kind == VarClassification::output) {
         for (const auto& out_var : getOutputVariables()) {
            if (!getInputVariables().count(out_var)) {
               variables.insert(out_var);
            }
         }
      }
      else if (kind == VarClassification::input) {
         for (const auto& in_var : getInputVariables()) {
            if (!getOutputVariables().count(in_var)) {
               if (all_variables_except_immutable_enum_and_bool_with_type.count(in_var)) {
                  variables.insert(in_var);
               }
            }
         }
      }
      else if (kind == VarClassification::state) {
         for (const auto& int_var : all_variables_except_immutable_enum_and_bool_with_type) {
            if (getInputVariables().count(int_var.first) && getOutputVariables().count(int_var.first)) {
               variables.insert(int_var.first);
            }
         }
      }
      else {
         Failable::getSingleton()->addError("Unknow variable kind.");
      }

      return variables;
   }

private:
   std::set<std::string> input_variables_{};
   std::set<std::string> output_variables_{};

   std::set<std::string> pure_input_variables_{};
   std::set<std::string> pure_output_variables_{};
   std::set<std::string> pure_state_variables_{};
};

enum class MCTranslationTypeEnum {
   fan_out_non_atomic_states,
   emulate_program_counter,
   atomize_states_assuming_independent_variables
};

enum class FSMCreationFromFormulaType {
   full_translation_to_statelass_states,  // Explicit fan-out to full state space - infeasible for medium-to-large FSMs.
   quick_translation_to_independent_lines // Single state applicable to MCTranslationTypeEnum::atomize_states_assuming_independent_variables
};

const std::string FAILED_COLOR = "\033[1;31m";
const std::string WARNING_COLOR = "\033[1;33m";
const std::string OK_COLOR = "\033[1;32m";
const std::string HIGHLIGHT_COLOR = "\033[1;34m";
const std::string RESET_COLOR     = "\033[1;0m";
const std::string SPECIAL_COLOR = "\033[1;46m";
const std::string SPECIAL_COLOR_2 = "\033[1;44m";
const std::string BOLD_COLOR = "\033[1m";
const std::string COLOR_PINK = "\033[38;2;255;0;127m"; // \033[38;2;<r>;<g>;<b>m
const std::string COLOR_DARK_GREEN = "\033[38;2;0;153;76m"; // \033[38;2;<r>;<g>;<b>m

constexpr int INDENT_STEP_DEFAULT = 3;
const std::string HIGHLIGHT_TERM_BEGIN = HIGHLIGHT_COLOR + "  >>> ";
const std::string HIGHLIGHT_TERM_END = " <<<  " + RESET_COLOR;

const int MAX_STRING_LENGTH_TO_DISPLAY_AS_TEXT_ONLY = 20; // Threshold for Graphviz to spawn child nodes (or else write serialization into single node).

const std::string FLOAT_ARRAY_DENOTER_SUFFIX = "_f";      // Last chars of float array name (i.e., cannot be file).
constexpr char RANDOM_ACCESS_FILE_ARRAY_DENOTER_SUFFIX = '_'; // Last char of array name for random access files (i.e., cannot be float).
constexpr char FILE_ARRAY_DENOTER_SUFFIX = '_';               // Second-to-last char of array name if last is RANDOM_ACCESS_FILE_ARRAY_DENOTER_SUFFIX.
const std::string PRIVATE_VARIABLES_PREFIX = "_";         // Prefix denoting variables to be scoped to the enclosing compound structure.
constexpr char PRIVATE_VARIABLES_INTERNAL_SYMBOL = '#';
constexpr char PRIVATE_VARIABLES_IN_RECURSIVE_CALLS_INTERNAL_SYMBOL = '$';

constexpr char NATIVE_ARRAY_PREFIX_FIRST = '.';
constexpr char NATIVE_ARRAY_PREFIX_LAST = '.';
const std::string PARSING_ERROR_STRING = "#PARSING_ERROR#";

const std::string PI_STR = "3.14159265";
const std::string E_STR =  "2.71828183";
const float MAX_ITERATIONS_FOR_WHILE_LOOP = 1e12;

constexpr char OPENING_BRACKET = '(';
constexpr char CLOSING_BRACKET = ')';
constexpr char OPENING_BRACKET_BRACE_STYLE = '{';
constexpr char CLOSING_BRACKET_BRACE_STYLE = '}';
constexpr char OPENING_BRACKET_SQUARE_STYLE_FOR_ARRAYS = '[';
constexpr char CLOSING_BRACKET_SQUARE_STYLE_FOR_ARRAYS = ']';
constexpr char OPENING_QUOTE = '\"';
constexpr char CLOSING_QUOTE = OPENING_QUOTE;
constexpr char OPENING_COND_BRACKET = '[';
constexpr char CLOSING_COND_BRACKET = ']';
constexpr char OPENING_MOD_BRACKET = '{';
constexpr char CLOSING_MOD_BRACKET = '}';
constexpr char ARGUMENT_DELIMITER = ',';
const std::string POSTFIX_DELIMITER = " ";

constexpr char PROGRAM_OPENING_OP_STRUCT_BRACKET_LEGACY = '(';
constexpr char PROGRAM_CLOSING_OP_STRUCT_BRACKET_LEGACY = ')';
constexpr char PROGRAM_OP_STRUCT_DELIMITER = '\'';
constexpr char PROGRAM_OPENING_ARRAY_BRACKET = '[';
constexpr char PROGRAM_CLOSING_ARRAY_BRACKET = ']';
constexpr char PROGRAM_OPENING_ENUM_BRACKET = '{';
constexpr char PROGRAM_CLOSING_ENUM_BRACKET = '}';
constexpr char PROGRAM_COMMAND_SEPARATOR = '$';
constexpr char PROGRAM_ARGUMENT_DELIMITER = ARGUMENT_DELIMITER;
const std::string PROGRAM_DEFINITION_SEPARATOR = ":=";
const std::string FSM_PROGRAM_TRANSITION_SEPARATOR = ":::";
const std::string FSM_PROGRAM_TRANSITION_DENOTER = "->";
const std::string FSM_PROGRAM_COMMAND_DENOTER = "|>";
const std::string FSM_PROGRAM_COMMAND_SEPARATOR = ",";
constexpr char PROGRAM_ARGUMENT_ASSIGNMENT = '=';
const std::string PROGRAM_ML_COMMENT_DENOTER_BEGIN = "/*";
const std::string PROGRAM_ML_COMMENT_DENOTER_END = "*/";
const std::string PROGRAM_COMMENT_DENOTER = "//";
constexpr char PROGRAM_PREPROCESSOR_DENOTER = '#';
const std::string PROGRAM_OPENING_FORMULA_BRACKET = "[";
const std::string PROGRAM_CLOSING_FORMULA_BRACKET = "]";
const std::string PROGRAM_ENUM_DENOTER = "enum";

const std::string FORWARD_DECLARATION_PREFIX = "^FWD";

const std::string FAILABLE_NAME_FOR_AKA_REPLACEMENT = "AKA-REPLACEMENT";

constexpr char TERM_COND_DENOTER = '#';
const std::string RULE_EQ_SEPARATOR = ";";
const std::string INDENTATION_DEBUG = "   ";

const static std::string SYMB_LTL_NEXT{ "X" }; // TODO: Add prios etc. here, as well.
const static std::string SYMB_LTL_UNTIL{ "U" };
const static std::string SYMB_LTL_RELEASED{ "V" };
const static std::string SYMB_LTL_GLOBALLY{ "G" };
const static std::string SYMB_LTL_FINALLY{ "F" };
const static std::string SYMB_LTL_XOR{ "xor" };
const static std::string SYMB_LTL_XNOR{ "xnor" };
const static std::string SYMB_LTL_IMPLICATION{ "=>" };
const static std::string SYMB_LTL_IFF{ "<=>" };

const std::string SYMB_CONST_VAL = "const";
const std::string SYMB_ARR_DEC =   ".D";          constexpr int OPNUM_ARR = 1;        constexpr int PRIO_ARRAY     = -1;
const std::string SYMB_ARR_ENC =   ".C";
const std::string SYMB_FOR =       "forRange";    constexpr int OPNUM_FOR = 4;
const std::string SYMB_FOR_C =     "for";         constexpr int OPNUM_FOR_C = 4;
const std::string SYMB_PRINT =     "print_plain"; constexpr int OPNUM_PRINT = 2;      constexpr int PRIO_PRINT     = -1;
const std::string SYMB_IF =        "if";          constexpr int OPNUM_IF = 2;         constexpr int PRIO_IF        = -1;
const std::string SYMB_IF_ELSE =   "ifelse";      constexpr int OPNUM_IF_ELSE = 3;    constexpr int PRIO_IF_ELSE   = -1;
const std::string SYMB_WHILE =     "while";       constexpr int OPNUM_WHILE = 2;      constexpr int PRIO_WHILE     = -1;
const std::string SYMB_WHILELIM =  "whilelim";    constexpr int OPNUM_WHILELIM = 3;   constexpr int PRIO_WHILELIM  = -1;
const std::string SYMB_FCTIN =     "fctin";       constexpr int OPNUM_FCTIN = 0;      constexpr int PRIO_FCTIN     = -1;
const std::string SYMB_SQRT =      "sqrt";        constexpr int OPNUM_SQRT = 1;       constexpr int PRIO_SQRT      = -1;
const std::string SYMB_RSQRT =     "rsqrt";       constexpr int OPNUM_RSQRT = 1;      constexpr int PRIO_RSQRT     = -1;
const std::string SYMB_NOT =       "!";           constexpr int OPNUM_NOT = 1;        constexpr int PRIO_NOT       = -1;
const std::string SYMB_BOOLIFY =   "boolify";     constexpr int OPNUM_BOOLIFY = 1;    constexpr int PRIO_BOOLIFY   = -1;
const std::string SYMB_ID =        "id";          constexpr int OPNUM_ID = 1;         constexpr int PRIO_ID        = -1;
const std::string SYMB_RAND =      "rand";        constexpr int OPNUM_RAND = 1;       constexpr int PRIO_RAND      = -1;
const std::string SYMB_NEG =       "--";          constexpr int OPNUM_NEG = 1;        constexpr int PRIO_NEG       = -1;
const std::string SYMB_NEG_ALT =   "-";           constexpr int OPNUM_NEG_ALT = 1;    constexpr int PRIO_NEG_ALT   = -1;
const std::string SYMB_TRUNC =     "trunc";       constexpr int OPNUM_TRUNC = 1;      constexpr int PRIO_TRUNC     = -1;
const std::string SYMB_ABS =       "abs";         constexpr int OPNUM_ABS = 1;        constexpr int PRIO_ABS       = -1;
const std::string SYMB_LENGTH =    "length";      constexpr int OPNUM_LENGTH = 1;     constexpr int PRIO_LENGTH    = -1;
const std::string SYMB_REF =       "@";           constexpr int OPNUM_REF = -1;       constexpr int PRIO_REF       = -1;
const std::string SYMB_FUNC_REF =  "@f";          constexpr int OPNUM_FUNC_REF = 1;   constexpr int PRIO_FUNC_REF  = -1;
const std::string SYMB_FUNC_EVAL = "func";        constexpr int OPNUM_FUNC_EVAL = -1; constexpr int PRIO_FUNC_EVAL = -1;
const std::string SYMB_LITERAL =   "literal";     constexpr int OPNUM_LITERAL = 1;    constexpr int PRIO_LITERAL   = -1;
const std::string SYMB_LAMBDA =    "lambda";      constexpr int OPNUM_LAMBDA = 1;     constexpr int PRIO_LAMBDA    = -1;
const std::string SYMB_SET_VAR =   "set";         constexpr int OPNUM_SET_VAR = 2;    constexpr int PRIO_SET_VAR   = -1;
const std::string SYMB_SET_ARR =   "Set";         constexpr int OPNUM_SET_ARR = 3;    constexpr int PRIO_SET_ARR   = -1;
const std::string SYMB_GET_VAR =   "get";         constexpr int OPNUM_GET_VAR = 1;    constexpr int PRIO_GET_VAR   = -1;
const std::string SYMB_GET_ARR =   "Get";         constexpr int OPNUM_GET_ARR = 2;    constexpr int PRIO_GET_ARR   = -1;
const std::string SYMB_LN =        "ln";          constexpr int OPNUM_LN = 1;         constexpr int PRIO_LN        = -1;
const std::string SYMB_MALLOC =    "malloc";      constexpr int OPNUM_MALLOC = 1;     constexpr int PRIO_MALLOC    = -1;
const std::string SYMB_DELETE =    "free";        constexpr int OPNUM_DELETE = 1;     constexpr int PRIO_DELETE    = -1;
const std::string SYMB_COMPOUND =  "compound";    constexpr int OPNUM_COMPOUND = 2;   constexpr int PRIO_COMPOUND  = -1; // Compound structures comprise two operands, a function call and the definition of the function.
const std::string SYMB_COMP_OP =   "$COMPOP$";                                                                           // This is the name of the function call part of the compound.
const std::string SYMB_META_CMP =  "p_";          constexpr int OPNUM_META_CMP = 1;   constexpr int PRIO_META_CMP  = -1; // MetaCompound is used as a placeholder for compound functions, e.g.: square = "p_(0)*p_(0)", and for defining the inverse of a function.
const std::string SYMB_META_SIMP = "s_";          constexpr int OPNUM_META_SIMP = 1;  constexpr int PRIO_META_SIMP = -1; // MetaSimplification is used for meta rules in simplification.
const std::string SYMB_OPTIONAL =  "o_";          constexpr int OPNUM_OPTIONAL = 2;   constexpr int PRIO_OPTIONAL  = -1; // Denotes in simplification a subtree that could or could not be there, as in 'o_(a, b) + c'; matches (all the '+'s in) 'x' (==> a = b) or 'x + y' or '(x + y) + z', ...
const std::string SYMB_ANYWAY =    "a_";                                              constexpr int PRIO_ANYWAY    = -1; // Matches any operator with the given number of operands. E.g., 'a_(1, x, y)' matches 'x + y', 'x - y', 'x * y', etc.
const std::string SYMB_ANYWAY_S =  "~";           constexpr int CREATE_SIMPLE_ANYWAYS_UP_TO = 6;
const std::string SYMB_MIN =       "min";         constexpr int OPNUM_MIN = 2;        constexpr int PRIO_MIN       = -1;
const std::string SYMB_MAX =       "max";         constexpr int OPNUM_MAX = 2;        constexpr int PRIO_MAX       = -1;
const std::string SYMB_SEQUENCE =  ";";           constexpr int OPNUM_SEQUENCE = 2;   constexpr int PRIO_SEQUENCE  = 0;
const std::string SYMB_EQUATION =  "====";        constexpr int OPNUM_EQUATION = 2;   constexpr int PRIO_EQUATION  = 0; // The symbol is preliminary, but the usual "=" collides with the alternative set symbol.
const std::string SYMB_SET_VAR_A = "=";           constexpr int OPNUM_SET_VAR_A = 2;  constexpr int PRIO_SET_VAR_A = 1;
const std::string SYMB_COND_COL =  ":";           constexpr int OPNUM_COND_COL = 2;   constexpr int PRIO_COND_COL  = 4;
const std::string SYMB_COND_QUES = "?";           constexpr int OPNUM_COND_QUES = 2;  constexpr int PRIO_COND_QUES = 5;
const std::string SYMB_OR =        "||";          constexpr int OPNUM_OR = 2;         constexpr int PRIO_OR        = 9;
const std::string SYMB_AND =       "&&";          constexpr int OPNUM_AND = 2;        constexpr int PRIO_AND       = 10;
const std::string SYMB_EQ =        "==";          constexpr int OPNUM_EQ = 2;         constexpr int PRIO_EQ        = 15;
const std::string SYMB_APPROX =    "===";         constexpr int OPNUM_APPROX = 2;     constexpr int PRIO_APPROX    = 15;
const std::string SYMB_NAPPROX =   "!==";         constexpr int OPNUM_NAPPROX = 2;    constexpr int PRIO_NAPPROX   = 15;
const std::string SYMB_NEQ =       "!=";          constexpr int OPNUM_NEQ = 2;        constexpr int PRIO_NEQ       = 15;
const std::string SYMB_SMEQ =      "<=";          constexpr int OPNUM_SMEQ = 2;       constexpr int PRIO_SMEQ      = 15;
const std::string SYMB_SM =        "<";           constexpr int OPNUM_SM = 2;         constexpr int PRIO_SM        = 15;
const std::string SYMB_GREQ =      ">=";          constexpr int OPNUM_GREQ = 2;       constexpr int PRIO_GREQ      = 15;
const std::string SYMB_GR =        ">";           constexpr int OPNUM_GR = 2;         constexpr int PRIO_GR        = 15;
const std::string SYMB_PLUS =      "+";           constexpr int OPNUM_PLUS = 2;       constexpr int PRIO_PLUS      = 100;
const std::string SYMB_MINUS =     "-";           constexpr int OPNUM_MINUS = 2;      constexpr int PRIO_MINUS     = 101;
const std::string SYMB_MULT =      "*";           constexpr int OPNUM_MULT = 2;       constexpr int PRIO_MULT      = 200;
const std::string SYMB_DIV =       "/";           constexpr int OPNUM_DIV = 2;        constexpr int PRIO_DIV       = 201;
const std::string SYMB_POW =       "**";          constexpr int OPNUM_POW = 2;        constexpr int PRIO_POW       = 225;
const std::string SYMB_MOD =       "%";           constexpr int OPNUM_MOD = 2;        constexpr int PRIO_MOD       = 250;
const std::string SYMB_DOT =       "dot";         constexpr int OPNUM_DOT = 2;        constexpr int PRIO_DOT       = 1000; // Denotes dot operator in C++ parsing, like in: 'obj.member'.
const std::string SYMB_DOT_ALT =   "."; // New actual "." operator which can be used similarly to C++ towards parsing. TODO: Eventually replace SYMB_DOT.

const std::string SYMB_NONDET_FUNCTION_FOR_MODEL_CHECKING = "ndet";        // Dummy function denoting arbitrarily many ndet. choices, like ndet(0, 1), which stands for a choice between 0 and 1.
const std::string SYMB_NONDET_RANGE_FUNCTION_FOR_MODEL_CHECKING = "rndet"; // Dummy function denoting a range of nondet. choices. rndet(i, j) stands for ndet(i, ..., j).

// cf. https://nusmv.fbk.eu/NuSMV/userman/v26/nusmv.pdf
const std::vector<OperatorStructure> NUSMV_TL_OERATORS = {
   OperatorStructure(OutputFormatEnum::infix, AssociativityTypeEnum::left, 2, -1 /* prio? */, "_def_", false, false),
   OperatorStructure(OutputFormatEnum::infix, AssociativityTypeEnum::left, 2, PRIO_OR /* prio? */, SYMB_LTL_XOR, true, true),
   OperatorStructure(OutputFormatEnum::infix, AssociativityTypeEnum::left, 2, PRIO_OR /* prio? */, SYMB_LTL_XNOR, true, true),
   OperatorStructure(OutputFormatEnum::infix, AssociativityTypeEnum::left, 2, 3 /* prio? */, SYMB_LTL_IMPLICATION, false, false),
   OperatorStructure(OutputFormatEnum::infix, AssociativityTypeEnum::left, 2, 3 /* prio? */, SYMB_LTL_IFF, true, true),
};

const std::vector<OperatorStructure> NUSMV_LTL_OERATORS = {
   OperatorStructure(OutputFormatEnum::prefix, AssociativityTypeEnum::left, 1, -1, SYMB_LTL_NEXT, false, false),
   OperatorStructure(OutputFormatEnum::prefix, AssociativityTypeEnum::left, 1, -1, SYMB_LTL_GLOBALLY, false, false),
   OperatorStructure(OutputFormatEnum::prefix, AssociativityTypeEnum::left, 1, -1, SYMB_LTL_GLOBALLY + "_bound", false, false),
   OperatorStructure(OutputFormatEnum::prefix, AssociativityTypeEnum::left, 1, -1, SYMB_LTL_FINALLY, false, false),
   OperatorStructure(OutputFormatEnum::prefix, AssociativityTypeEnum::left, 1, -1, SYMB_LTL_FINALLY + "_bound", false, false),
   OperatorStructure(OutputFormatEnum::prefix, AssociativityTypeEnum::left, 1, -1, "Y", false, false),
   OperatorStructure(OutputFormatEnum::prefix, AssociativityTypeEnum::left, 1, -1, "Z", false, false),
   OperatorStructure(OutputFormatEnum::prefix, AssociativityTypeEnum::left, 1, -1, "H", false, false),
   OperatorStructure(OutputFormatEnum::prefix, AssociativityTypeEnum::left, 1, -1, "H_bound", false, false),
   OperatorStructure(OutputFormatEnum::prefix, AssociativityTypeEnum::left, 1, -1, "O", false, false),
   OperatorStructure(OutputFormatEnum::prefix, AssociativityTypeEnum::left, 1, -1, "O_bound", false, false),
   OperatorStructure(OutputFormatEnum::infix, AssociativityTypeEnum::left, 2, 0 /* prio? */, SYMB_LTL_UNTIL, false, false),
   OperatorStructure(OutputFormatEnum::infix, AssociativityTypeEnum::left, 2, 0 /* prio? */, SYMB_LTL_RELEASED, false, false),
   OperatorStructure(OutputFormatEnum::infix, AssociativityTypeEnum::left, 2, 0 /* prio? */, "S", false, false),
   OperatorStructure(OutputFormatEnum::infix, AssociativityTypeEnum::left, 2, 0 /* prio? */, "T", false, false),
};

const std::vector<OperatorStructure> NUSMV_CTL_OERATORS = {
   OperatorStructure(OutputFormatEnum::prefix, AssociativityTypeEnum::left, 1, -1, "EG", false, false),
   OperatorStructure(OutputFormatEnum::prefix, AssociativityTypeEnum::left, 1, -1, "EX", false, false),
   OperatorStructure(OutputFormatEnum::prefix, AssociativityTypeEnum::left, 1, -1, "EF", false, false),
   OperatorStructure(OutputFormatEnum::prefix, AssociativityTypeEnum::left, 1, -1, "AG", false, false),
   OperatorStructure(OutputFormatEnum::prefix, AssociativityTypeEnum::left, 1, -1, "AX", false, false),
   OperatorStructure(OutputFormatEnum::prefix, AssociativityTypeEnum::left, 1, -1, "AF", false, false),

   OperatorStructure(OutputFormatEnum::infix, AssociativityTypeEnum::left, 2, 0 /* prio? */, "AU", false, false),
   OperatorStructure(OutputFormatEnum::infix, AssociativityTypeEnum::left, 2, 0 /* prio? */, "EU", false, false),
};

// Manually add new operators (including static compound terms; the below sets' union has to include all of them; don't forget to also update FormulaParser's term factory and init method). 
const std::set<std::string> TERMS_LOGIC { 
    SYMB_OR, SYMB_AND, SYMB_EQ, SYMB_NOT, SYMB_NEQ, SYMB_SMEQ, SYMB_SM, SYMB_GREQ, SYMB_GR, SYMB_APPROX, SYMB_NAPPROX, SYMB_BOOLIFY };
const std::set<std::string> TERMS_ARITH {
   SYMB_SQRT, SYMB_RSQRT, SYMB_PLUS, SYMB_MINUS, SYMB_MULT, SYMB_DIV, SYMB_MOD, SYMB_MIN, SYMB_MAX, SYMB_NEG, SYMB_TRUNC, SYMB_ABS, SYMB_ID };
const std::set<std::string> TERMS_IF_ELSE { SYMB_IF, SYMB_IF_ELSE };
const std::set<std::string> TERMS_SPECIAL { SYMB_SEQUENCE, SYMB_IF, SYMB_IF_ELSE };
const std::set<std::string> TERMS_MEMORY { SYMB_MALLOC, SYMB_DELETE };
const std::set<std::string> TERMS_MISC { SYMB_FCTIN, SYMB_SET_VAR, SYMB_SET_ARR, SYMB_GET_ARR, SYMB_EQUATION, SYMB_META_CMP, SYMB_META_SIMP, SYMB_OPTIONAL, SYMB_ANYWAY, SYMB_WHILELIM, SYMB_WHILE, SYMB_RAND, SYMB_LENGTH, SYMB_POW, SYMB_LN, SYMB_FUNC_REF, SYMB_FUNC_EVAL, SYMB_LAMBDA, SYMB_PRINT, SYMB_LITERAL }; // SYMB_POW/SYMB_LN should eventually go to TERMS_ARITH, once assembly is ready.
const std::set<std::string> TERMS_TL { /* TODO */};
// EO Manually add new operators.

const std::map<std::string, std::vector<int>> IMPLICIT_LAMBDAS = { /// Parameter numbers which are implicit lambdas for specific operators.
   { SYMB_FOR, { 3 } },
   { SYMB_FOR_C, { 0, 1, 2, 3 } },
};

const std::string DIGITS = "0123456789";
const std::string ALPHA_LOW = "abcdefghijklmnopqrstuvwxyz";
const std::string ALPHA_UPP = "ABCDEFGHIJKLMNOPQRSTUVWXYZ";
const std::string ALPHA_ALL = ALPHA_LOW + ALPHA_UPP;
const std::string DIGITS_WITH_DECIMAL_POINT = DIGITS + ".";
const std::string VAR_NAME_LETTERS = ALPHA_ALL + "_" + SYMB_REF + "'" + DIGITS + ":.";

const std::map<char, int> CHAR_NUM_MAPPING({ {'A', 0},  {'B', 1},  {'C', 2},  {'D', 3},  {'E', 4},  {'F', 5},  {'G', 6},  {'H', 7},  {'I', 8},  {'J', 9}, 
                                             {'K', 10}, {'L', 11}, {'M', 12}, {'N', 13}, {'O', 14}, {'P', 15}, {'Q', 16}, {'R', 17}, {'S', 18}, {'T', 19}, 
                                             {'U', 20}, {'V', 21}, {'W', 22}, {'X', 23}, {'Y', 24}, {'Z', 25}, });

const std::string VAR_AND_FUNCTION_NAME_REGEX_STRING_DOTLESS_BASE = std::string("a-zA-Z_") + SYMB_REF + "'" + "0-9:";
const std::string VAR_AND_FUNCTION_NAME_REGEX_STRING_BASE = VAR_AND_FUNCTION_NAME_REGEX_STRING_DOTLESS_BASE + ".";
const std::regex VAR_AND_FUNCTION_NAME_REGEX_DOTLESS = std::regex("[" + VAR_AND_FUNCTION_NAME_REGEX_STRING_DOTLESS_BASE + "]");
const std::regex VAR_AND_FUNCTION_NAME_REGEX = std::regex("[" + VAR_AND_FUNCTION_NAME_REGEX_STRING_BASE + "]");

const std::vector<std::regex> DEFAULT_REGEX_FOR_TOKENIZER = {
   //std::regex("[0-9.]"),         // Numerical values.
   VAR_AND_FUNCTION_NAME_REGEX,    // Variables and operators with names.
   std::regex("\\s"),              // White spaces (ignored, see below).
   std::regex("[?<>=!~+\\*/#%^]"), // Arith., logical and comparison operators with special symbols.
   std::regex("[\\&\\|]"),         // AND and OR.
   std::regex("[-]"),              // Special treatment of minus since it also constitutes the neg operator.
   std::regex("[(]"),              // Brackets.
   std::regex("[)]"),
   std::regex("[{]"),
   std::regex("[}]"),
   std::regex("[,]"),
   std::regex("[;]"),
   std::regex("[\"]"),
};

const std::vector<std::regex> DEFAULT_REGEX_FOR_TOKENIZER_DOTLESS = { // Same as above, but the dot is in the operators' class.
   VAR_AND_FUNCTION_NAME_REGEX_DOTLESS,
   std::regex("\\s"),
   std::regex("[?<>=!~+\\*/#%^.]"),
   std::regex("[\\&\\|]"),
   std::regex("[-]"),
   std::regex("[(]"),
   std::regex("[)]"),
   std::regex("[{]"),
   std::regex("[}]"),
   std::regex("[,]"),
   std::regex("[;]"),
   std::regex("[\"]"),
};

constexpr int DEFAULT_IGNORE_CLASS_FOR_TOKENIZER = 1; /// Characters from this class will not be included in tokens.

class MathStruct: public std::enable_shared_from_this<MathStruct>
{
public:
   typedef float(*JITFunc)();

   int visited_stack_item_ = 0; // Just for debug, maybe should be removed if efficiency becomes critical.
   
   static std::vector<float> rand_vars;
   static int rand_var_count;
   static int first_free_private_var_num_;

   MathStruct(const std::vector<std::shared_ptr<Term>>& opds);

   void addCompoundStructureReference(TermCompound* ptr_to_owning_compound);

#if defined(ASMJIT_ENABLED)
   virtual std::shared_ptr<x86::Xmm> createSubAssembly(asmjit::x86::Compiler& cc, const std::shared_ptr<DataPack>& d, const std::shared_ptr<FormulaParser>& p = nullptr) = 0;

   /// Note: JIT Assembly makes evaluation some 1000 times faster compared
   /// to plain interpretation (based on my own fuzzy observation). 
   /// There are, however, the following limitations:
   /// - JIT compilation requires some initial time which has to be compensated
   ///   for by repeated evaluation.
   /// - Assembly creation currently does not work for:
   ///   * Non-x86 architectures.
   ///   * Terms which write to or read from HDD     (NOT planned to be implemented).
   ///   * Array terms when array size changes       (TODO: basically this case only requires
   ///     dynamic re-compilation of the assembly code which can be done fairly easily).
   ///   * The "set" operator for variables          (TODO: should be really easy).
   ///   * The "Set" operator for arrays             (TODO: should be really easy).
   ///   * The "while" operator                      (TODO: should be mildly sophisticated, but doable).
   ///   * The "pow" operator                        (TODO: should be easy).
   ///   * The "ln" operator                         (TODO: should be easy).
   ///   * The "funcRef" and "funcEval" operators.   (TODO: Would be pretty cool, but for now too much effort.)
   ///   * Metas with a non-constant operand.        (NOT planned for now, this seems to be an irrelevant edge-case.)
   /// - Recursive calls with private variables would require a specific memory handling
   ///   outside of the DataPack.                    (TODO: Would be pretty cool, but for now too much effort.)
   void createAssembly(const std::shared_ptr<DataPack>& d, const std::shared_ptr<FormulaParser>& p = nullptr, const bool reset = false);

   void resetAssembly();
#endif

   static std::shared_ptr<MathStruct> parseMathStruct(const std::string& formula, 
                                                      std::shared_ptr<FormulaParser> parser = nullptr,
                                                      const std::shared_ptr<DataPack> data_to_retrieve_constants = nullptr,
                                                      const DotTreatment dot_treatment = DotTreatment::as_varname_character);

   static std::shared_ptr<MathStruct> parseMathStruct(const std::string& formula, 
                                                      const bool preprocess_cpp_array_notation_and_brace_style_brackets,
                                                      const bool preprocess_cpp_switchs_to_ifs_else_ifs_to_plain_ifs = false,
                                                      std::shared_ptr<FormulaParser> parser = nullptr,
                                                      const std::shared_ptr<DataPack> data_to_retrieve_constants = nullptr,
                                                      const DotTreatment dot_treatment = DotTreatment::as_varname_character);

   static std::shared_ptr<MathStruct> parseMathStruct(const std::string& formula,
                                                      FormulaParser& parser,
                                                      const std::shared_ptr<DataPack> data_to_retrieve_constants,
                                                      const DotTreatment dot_treatment = DotTreatment::as_varname_character);

   static std::shared_ptr<MathStruct> parseMathStruct(
      const std::string& formula, 
      FormulaParser& parser,
      const bool preprocess_cpp_array_notation_and_brace_style_brackets,
      const bool preprocess_cpp_switchs_to_ifs_else_ifs_to_plain_ifs = false,
      const std::vector<std::regex>& classes = DEFAULT_REGEX_FOR_TOKENIZER,
      const int ignore_class = DEFAULT_IGNORE_CLASS_FOR_TOKENIZER,
      const std::set<std::string>& ignore_tokens = {},
      const std::shared_ptr<DataPack> data_to_retrieve_constants = nullptr,
      const AkaTalProcessing aka_processing = AkaTalProcessing::none,
      const DotTreatment dot_treatment = DotTreatment::as_varname_character // Using DotTreatment::as_operator will override the "classes" argument to DEFAULT_REGEX_FOR_TOKENIZER_DOTLESS.
   );

   static std::shared_ptr<MathStruct> parseMathStructFromTokens(
      const std::shared_ptr<std::vector<std::string>> formula_tokens,
      FormulaParser& parser,
      const std::shared_ptr<DataPack> data_to_retrieve_constants = nullptr);

   static std::vector<std::shared_ptr<Equation>> parseEquations(std::string& formulae);
   static std::vector<std::string> parseLines(std::string& lines, const std::vector<std::string> & delimiters, const int& offset);
   static void randomize_rand_nums();

   static std::shared_ptr<Term> simplify(const std::shared_ptr<Term> term, const bool print_trace = false, const std::shared_ptr<DataPack> d = nullptr, const bool check_for_infinite_loop = false, const std::shared_ptr<FormulaParser> parser = nullptr, const std::shared_ptr<std::map<std::string, std::map<int, std::set<MetaRule>>>> rules = nullptr);
   static std::shared_ptr<Term> simplify(const std::string& formula,       const bool print_trace = false, const std::shared_ptr<DataPack> d = nullptr, const bool check_for_infinite_loop = false, const std::shared_ptr<FormulaParser> parser = nullptr, const std::shared_ptr<std::map<std::string, std::map<int, std::set<MetaRule>>>> rules = nullptr);

   static std::shared_ptr<Term> resolveTernaryOperators(
      const std::shared_ptr<Term> formula, 
      const bool check_for_consistency = true,
      const std::shared_ptr<FormulaParser> parser = nullptr);

   static void swapSubtrees(const std::shared_ptr<Term> sub_formula1, const std::shared_ptr<Term> sub_formula2);

   /// <summary>
   /// Go up over sequences until next enclosing block of other (usually "if", "while" etc.) type and return root sequence WITHIN that block.
   /// </summary>
   /// <returns>The root sequence within its enclosing block.</returns>
   std::shared_ptr<MathStruct> getPtrToRootWithinNextEnclosingBlock();
   
   std::shared_ptr<MathStruct> getPtrToRoot(
      const bool go_over_compound_bounderies = false,
      std::shared_ptr<std::vector<std::weak_ptr<TermCompound>>> visited = nullptr);

   std::shared_ptr<MathStruct> setPrintFullCorrectBrackets(const bool val);

   enum class AssociativityEnforce {
      none,
      left,
      right
   };

   static std::shared_ptr<Term> randomProgram(
      const int line_count = 5,
      const int line_depth = 3, 
      const long seed = 0,
      const int max_const_val = 256,
      const int max_array_val = 256,
      const bool workaround_for_mod_and_div = true,
      const std::shared_ptr<FormulaParser> parser = nullptr,
      const std::set<std::string>& inner_variables = { "a_inner", "b_inner", "c_inner" },
      const std::set<std::string>& input_variables = { "a_in", "b_in", "c_in" },
      const std::set<std::string>& output_variables = { "a_out", "b_out", "c_out" },
      const std::set<std::set<std::string>>& allowed_symbols = { TERMS_LOGIC, TERMS_ARITH, TERMS_IF_ELSE, { SYMB_SET_VAR_A } }
   );

   static std::shared_ptr<Term> randomTerm(
      const int depth, 
      const std::set<std::set<std::string>>& allowed_symbols = { TERMS_LOGIC, TERMS_ARITH, { "x_inner" } },
      const int max_const_val = 256,
      const int max_array_val = 256,
      const long seed = 0,
      const bool workaround_for_mod_and_div = true,
      const std::shared_ptr<FormulaParser> parser = nullptr,
      const AssociativityEnforce enforce_associativity_for_associative_operators = AssociativityEnforce::none,
      const std::vector<std::string>& use_variables_instead_of_const_val = {},
      const std::set<std::string>& disallowed_symbols = {},
      const bool enforce_variable_on_left_side_of_assignment = true,
      const bool enforce_sound_if_and_assignment_structures = false,
      const std::set<std::string>& input_variables = {},
      const std::set<std::string>& output_variables = {}
      );

   static std::shared_ptr<Term> randomTerm(
      const std::vector<std::string>& use_variables_instead_of_const_val,
      const int depth,
      const long seed = 0,
      const std::set<std::set<std::string>>& operators = { TERMS_ARITH, TERMS_LOGIC },
      const std::shared_ptr<FormulaParser> parser = nullptr);

   static std::shared_ptr<Term> randomCBTerm(
      const int regular_term_depth,
      const int cb_term_depth,
      const std::set<std::set<std::string>>& allowed_symbols = { TERMS_LOGIC, TERMS_ARITH, { "x" } },
      const std::vector<std::string>& var_names = {},
      const int max_const_val = 256,
      const int max_array_val = 256,
      const long seed = 0,
      const bool workaround_for_mod_and_div = true,
      const std::shared_ptr<FormulaParser> parser = nullptr);

   static float evalJitOrNojit(
      const std::shared_ptr<DataPack> d, 
      const std::shared_ptr<FormulaParser> p, 
      const std::shared_ptr<MathStruct> m, 
      const bool jit,
      const int repetitions,
      long& time);

   enum class EarleyChoice {
      Parser,
      Recognizer
   };

   static bool testEarleyParsing(
      const std::set<std::set<std::string>>& operators,
      const EarleyChoice choice,
      const std::string& m = "",
      const int seed = 0,
      const int depth = 3,
      const bool reverse_check_intentionally_malformed = false,
      const std::shared_ptr<earley::Grammar> grammar = nullptr,
      const std::shared_ptr<std::vector<std::vector<float>>> additional_measures = nullptr,
      const std::shared_ptr<std::vector<std::string>> additional_test_info = nullptr,
      const bool output_ok_too = false,
      const bool output_everything_always = false);

   /// Caution: Be aware that using equal_repetitions > 1 can create wrong results
   /// if the tested formula has side effects. This is due to the DataPack not being
   /// reset to allow for a more accurate time measurment.
   static bool testJitVsNojitRandom(
      const std::set<std::set<std::string>>& operators, 
      const std::shared_ptr<DataPack> d,
      const std::shared_ptr<MathStruct> m = nullptr,
      const int seed = 0,
      const int depth = 3,
      const int max_const_val = 256,
      const int max_array_val = 256,
      const float epsilon = 0.01,
      const std::shared_ptr<std::vector<std::vector<float>>> additional_measures = nullptr,
      const bool output_ok_too = false,
      const int equal_repetitions = 1);

   static bool testFormulaEvaluation(
      const MathStructPtr formula,
      const std::function<float(const std::map<std::string, float>&)>& f,
      const std::map<std::string, std::pair<RandomNumberType, std::pair<float, float>>>& replacements = { { "x", { RandomNumberType::Float, { -1000, 1000 } } } },
      const std::shared_ptr<FormulaParser> pp = nullptr,
      const std::shared_ptr<DataPack> dd = nullptr,
      const int seed = 0,
      const float epsilon = 0.01,
      const std::shared_ptr<std::vector<std::vector<float>>> additional_measures = nullptr,
      const std::shared_ptr<std::vector<std::string>> additional_test_info = nullptr,
      const bool output_ok_too = false);

   static bool testFormulaEvaluation(
      const std::string& formula = "x",
      const std::function<float(const std::map<std::string, float>&)>& f = [](std::map<std::string, float> vals) -> float { return vals.at("x"); },
      const std::map<std::string, std::pair<RandomNumberType, std::pair<float, float>>>& replacements = { { "x", { RandomNumberType::Float, { -1000, 1000 } } } },
      const std::shared_ptr<FormulaParser> pp = nullptr,
      const std::shared_ptr<DataPack> dd = nullptr,
      const int seed = 0,
      const float epsilon = 0.01,
      const std::shared_ptr<std::vector<std::vector<float>>> additional_measures = nullptr,
      const std::shared_ptr<std::vector<std::string>> additional_test_info = nullptr,
      const bool output_ok_too = false);

   /// Caution: Be aware that using equal_repetitions > 1 can create wrong results
   /// if the tested formula has side effects. This is due to the DataPack not being
   /// reset to allow for a more accurate time measurment.
   static bool testJitVsNojit(
      const std::shared_ptr<MathStruct> m, 
      const std::shared_ptr<DataPack> d = nullptr,
      const bool output_ok_too = true,
      const int equal_repetitions = 1);

   /// Caution: Be aware that using equal_repetitions > 1 can create wrong results
   /// if the tested formula has side effects. This is due to the DataPack not being
   /// reset to allow for a more accurate time measurment.
   static bool testJitVsNojit(
      const std::string& m, 
      const std::shared_ptr<DataPack> d = nullptr, 
      const bool output_ok_too = true, 
      const int equal_repetitions = 1);

   static bool testParsing(
      const std::string& formula,
      const bool output_ok_too = false,
      const std::shared_ptr<FormulaParser> parser = nullptr);

   static bool testParsing(
      const std::set<std::set<std::string>>& operators,
      const std::shared_ptr<MathStruct>& m = nullptr,
      const int seed = 0,
      const int depth = 3,
      const int max_const_val = 256,
      const int max_array_val = 256,
      const float epsilon = 0.01,
      const std::shared_ptr<std::vector<std::vector<float>>> additional_measures = nullptr,
      const std::shared_ptr<std::vector<std::string>> additional_test_info = nullptr,
      const bool output_ok_too = false,
      const std::shared_ptr<FormulaParser> parser_raw = nullptr);

   static bool testSimplifying(
      const std::shared_ptr<MathStruct> m, 
      const std::shared_ptr<DataPack> d = nullptr,
      const bool output_ok_too = true);

   static bool testSimplifying(
      const std::string& m, 
      const std::shared_ptr<DataPack> d = nullptr, 
      const bool output_ok_too = true);

   static bool testSimplifying(
      const std::set<std::set<std::string>>& operators,
      const std::shared_ptr<DataPack>& d,
      const std::shared_ptr<MathStruct>& m = nullptr,
      const int seed = 0,
      const int depth = 3,
      const int max_const_val = 256,
      const int max_array_val = 256,
      const float epsilon = 0.01,
      const std::shared_ptr<std::vector<std::vector<float>>> additional_measures = nullptr,
      const std::shared_ptr<std::vector<std::string>> additional_test_info = nullptr,
      const bool output_ok_too = false);

   static bool testFastSimplifying(const std::string& formula, const std::shared_ptr<DataPack> data = nullptr, const bool output_ok_too = true);

   static bool testFastSimplifying(
      const std::set<std::set<std::string>>& operators,
      const std::shared_ptr<MathStruct>& m = nullptr,
      const int seed = 0,
      const int depth = 3,
      const int max_const_val = 256,
      const int max_array_val = 256,
      const int min_raw_formula_size = 1,
      const std::shared_ptr<std::vector<std::vector<float>>> additional_measures = nullptr,
      const std::shared_ptr<std::vector<std::string>> additional_test_info = nullptr,
      const bool output_ok_too = false);

   static bool testVeryFastSimplifying(
      const std::set<std::set<std::string>>& operators,
      const std::shared_ptr<MathStruct>& m = nullptr,
      const int seed = 0,
      const int depth = 3,
      const int max_const_val = 256,
      const int max_array_val = 256,
      const int min_raw_formula_size = 1,
      const std::shared_ptr<std::vector<std::vector<float>>> additional_measures = nullptr,
      const std::shared_ptr<std::vector<std::string>> additional_test_info = nullptr,
      const bool output_ok_too = false);

   static bool testExtracting(const std::string& fmla, const std::shared_ptr<FormulaParser> parser = nullptr, const std::shared_ptr<DataPack> data = nullptr, const bool print_ok_too = true);

   static bool testExtracting(
      const std::set<std::set<std::string>>& operators,
      const std::shared_ptr<DataPack>& d,
      const std::shared_ptr<MathStruct>& m = nullptr,
      const int seed = 0,
      const int depth = 3,
      const int max_const_val = 256,
      const int max_array_val = 256,
      const float epsilon = 0.01,
      const int encapsulations = 10,
      const std::shared_ptr<std::vector<std::vector<float>>> additional_measures = nullptr,
      const std::shared_ptr<std::vector<std::string>> additional_test_info = nullptr,
      const bool output_ok_too = false);

   friend std::ostream& ::operator<<(std::ostream &os, MathStruct const &m);

   virtual std::string printComplete(const std::string& buffer = "", const std::shared_ptr<std::vector<std::shared_ptr<const MathStruct>>> visited = nullptr, const bool print_root = true) const;

   static float evalRange(
      const std::vector<std::pair<std::string, std::vector<std::shared_ptr<Term>>>>& assignmentRanges,
      const std::shared_ptr<Term> evalTerm,
      const bool useJIT = false);

   static float evalRange(
      const std::shared_ptr<DataPack> data,
      const std::vector<std::pair<std::string, std::vector<std::shared_ptr<Term>>>>& assignmentRanges,
      const std::shared_ptr<Term> evalTerm,
      const bool useJIT = false);

   virtual std::shared_ptr<Term> toTermIfApplicable() = 0;
   virtual std::shared_ptr<Equation> toEquationIfApplicable();
   virtual std::shared_ptr<TermVar> toVariableIfApplicable();
   virtual std::shared_ptr<TermVal> toValueIfApplicable();
   virtual std::shared_ptr<TermArray> toArrayIfApplicable();
   virtual std::shared_ptr<TermSetVar> toSetVarIfApplicable();
   virtual std::shared_ptr<TermSetArr> toSetArrIfApplicable();
   virtual std::shared_ptr<TermGetArr> toGetArrIfApplicable();
   virtual std::shared_ptr<TermMetaCompound> toMetaCompoundIfApplicable();
   virtual std::shared_ptr<TermMetaSimplification> toMetaSimplificationIfApplicable();
   virtual std::shared_ptr<TermOptional> toTermOptionalIfApplicable();
   virtual std::shared_ptr<TermAnyway> toTermAnywayIfApplicable();
   virtual std::shared_ptr<TermCompound> toTermCompoundIfApplicable();
   virtual std::shared_ptr<TermCompoundOperator> toCompoundOperatorIfApplicable();
   std::vector<std::pair<std::shared_ptr<MathStruct>, std::shared_ptr<MathStruct>>> findDuplicateSubTerms(
      const std::shared_ptr<MathStruct> other,
      const int min_nodes_to_report = 5,
      const TraverseCompoundsType traverse_into_compound_structures = TraverseCompoundsType::avoid_compound_structures);

   virtual bool hasBooleanResult() const;

   std::shared_ptr<MathStruct> copy();

   std::string generateGraphvizWithFatherFollowing(
      const int spawn_children_threshold = MAX_STRING_LENGTH_TO_DISPLAY_AS_TEXT_ONLY,
      const bool go_into_compounds = true);

   virtual std::string generateGraphviz(
      const int spawn_children_threshold = MAX_STRING_LENGTH_TO_DISPLAY_AS_TEXT_ONLY,
      const bool go_into_compounds = true,
      const std::shared_ptr<std::map<std::shared_ptr<MathStruct>, std::string>> follow_fathers = nullptr,
      const std::string cmp_counter = "0",
      const bool root = true,
      const std::string& node_name = "r",
      const std::shared_ptr<std::map<std::shared_ptr<Term>, std::string>> visited = nullptr,
      const std::shared_ptr<std::string> only_compound_edges_to = nullptr);

   bool extractSubTermToFunction(
      const std::shared_ptr<FormulaParser> parser,
      const float var_parameter_probability = 1.0,
      const float val_parameter_probability = 0.0,
      const int seed = 0,
      const int min_depth = 0,
      const int min_dist_to_root = 0,
      const bool simplify_first = true);

   std::shared_ptr<MathStruct> getUniformlyRandomSubFormula(
      const int seed = 0, 
      const bool avoid_root = true, 
      const bool enforce_leaf = false,
      const std::function<bool(std::shared_ptr<MathStruct>)>& f = [](std::shared_ptr<MathStruct>) { return true; });

   bool applyMetaRule(const MetaRule& rule, const bool print_application_note = false);
   
   bool checkCompoundConsistency();

   /// Use getStaticConvertedSimplificationRules() unless you have something special in mind.
   virtual std::set<MetaRule> getSimplificationRules(const std::shared_ptr<FormulaParser> parser) const;

   /// This method is usually preferred over getSimplificationRules(.).
   std::set<MetaRule>& getStaticConvertedSimplificationRules(const std::shared_ptr<FormulaParser> parser);

   bool isRecursive();

   int depth_to_farthest_child();
   int depth_from_root();
   bool isEquation();
   bool isTerm();
   virtual bool isMetaCompound() const;
   virtual bool isMetaSimplification() const;
   virtual bool isTermCompound() const;
   virtual bool isCompoundOperator() const;
   virtual bool isTermIf() const;
   virtual bool isTermIfelse() const;
   virtual bool isTermSequence() const;
   virtual bool isTermNot() const;
   virtual bool isTermIdentity() const;
   virtual bool isTermBool() const;
   virtual bool isTermOptional() const;
   virtual bool isTermPrint() const;
   virtual bool isTermPlus() const;
   virtual bool isTermLambda() const;
   virtual bool isTermGetVar() const;
   virtual bool isTermLogic() const;
   virtual bool isTermVarOnLeftSideOfAssignment() const;
   virtual bool isTermVarNotOnLeftSideOfAssignment() const;

   virtual bool containsMetaCompound() const;

   /// <summary>
   /// If _this_ points to a single line within a sequence or an individual single line in an if statement.
   /// </summary>
   /// <returns></returns>
   bool isSingleLineIfTypeLimited() const;

   /// <summary>
   /// 
   /// </summary>
   /// <returns>If the _this_ formula contains at least one of the NUSMV_LTL_OERATORS.</returns>
   bool hasLTLOperators() const;

   static inline const std::set<std::string> DEFAULT_EXCEPTIONS_FOR_FLATTENING{
      SYMB_SET_VAR_A,
      SYMB_GET_VAR,
      SYMB_NOT,
      SYMB_SEQUENCE,
      SYMB_IF,
      SYMB_IF_ELSE,
      "++",
      SYMB_NONDET_FUNCTION_FOR_MODEL_CHECKING,
      SYMB_NONDET_RANGE_FUNCTION_FOR_MODEL_CHECKING,
      SYMB_LTL_IFF,
      SYMB_LTL_IMPLICATION,
      SYMB_LTL_XOR,
      SYMB_LTL_XNOR,
      SYMB_LTL_UNTIL,
      SYMB_LTL_RELEASED,
      SYMB_LTL_NEXT,
      SYMB_LTL_GLOBALLY,
      SYMB_LTL_FINALLY,
      SYMB_DOT,
      SYMB_DOT_ALT
   };

   /// \brief Removes all TermCompounds recursively from the formula and replaces
   /// them by the actual code of the underlying compound structure.
   /// Similar to (the original meaning of) the inline keyword of C.
   ///
   /// TODO: Currently there is no check for recursive calls in compounds, they will
   ///       just crash via stack overflow.
   static std::shared_ptr<Term> flattenFormula(const std::shared_ptr<Term> m, const std::set<std::string>& exception_operators 
      = DEFAULT_EXCEPTIONS_FOR_FLATTENING);

   /// Iff the term is the compound structure PART OF a TermCompound. Does NOT have
   /// to be assignable by TermCompound (use isCompound() for that).
   bool isCompoundStructure() const;

   /// Returns all references to the owning compounds.
   std::vector<std::weak_ptr<TermCompound>> getCompoundStructureReferences() const;

   /// Iff the struct is a set or get function.
   virtual bool isTermSetVarOrAssignment() const;
   virtual bool isTermGetArr() const;
   virtual bool isTermSetArr() const;

   virtual bool isTermVal() const;
   virtual bool isTermVar() const;
   virtual bool isTermArray() const;

   /// Iff the formula is composed only of non-variable terms. Note that
   /// Leaf terms only need to implement this method if they are NOT constant,
   /// returning false. (Non-leaf terms should not have to implement this method
   /// as it would be strange if they could tell weather they are constant
   /// or not - other than the default implementation does anyway. 
   /// Exceptions: TermGetArr, TermSetVar and TermSetArr (non-const), TermMeta (it's complicated).)
   virtual bool isOverallConstant(const std::shared_ptr<std::vector<int>> non_const_metas = nullptr,
                                  const std::shared_ptr<std::set<std::shared_ptr<MathStruct>>> visited = nullptr);

   bool consistsPurelyOfNumbers(); /// Checks if all leaves are TermVal.

   /// Returns true only if the MathStruct is guaranteed to be always false.
   /// Can return false for some always true formulae, too.
   bool isAlwaysFalse();

   /// Returns true only if the MathStruct is guaranteed to be always true.
   /// Can return false for some always false formulae, too.
   bool isAlwaysTrue();

   virtual bool hasSideeffectsThis() const; /// Override this if your operator can have sideeffects.
   virtual bool hasSideeffects() const; /// Override this if you need a special treatment of sideeffects in children.

   void swap();
   void swapJumpIntoCompounds();
   
   virtual void applyToMeAndMyChildren(
      const std::function<void(MathStructPtr)>& f,
      const TraverseCompoundsType go_into_compounds = TraverseCompoundsType::avoid_compound_structures,
      const std::shared_ptr<bool> trigger_abandon_children_once = nullptr, // Set these triggers to true from within a lambda expression
      const std::shared_ptr<bool> trigger_break = nullptr,                 // to direct the control flow in the respective way.
      const std::shared_ptr<std::vector<std::shared_ptr<MathStruct>>> blacklist = nullptr,
      const std::shared_ptr<std::vector<std::shared_ptr<MathStruct>>> visited = nullptr,
      const std::function<bool(MathStructPtr)>& reverse_ordering_of_children = [](const MathStructPtr) { return false;  });

   virtual void applyToMeAndMyChildrenIterative(
      const std::function<void(MathStructPtr)>& f,
      const TraverseCompoundsType go_into_compounds,
      const FormulaTraversalType traversal_type = FormulaTraversalType::PreOrder);
   
   virtual void applyToMeAndMyChildrenIterative(
      const std::function<void(MathStructPtr)>& f,
      const TraverseCompoundsType go_into_compounds,
      const bool& abort,
      const FormulaTraversalType traversal_type = FormulaTraversalType::PreOrder);

   virtual void applyToMeAndMyChildrenIterativeReversePreorder(
      const std::function<void(MathStructPtr)>& f, 
      const TraverseCompoundsType go_into_compounds,
      const bool& abort,
      const FormulaTraversalType traversal_type = FormulaTraversalType::PreOrder,  // TODO: Currently only preorder possible.
      const std::set<std::string>& apply_at_and_abandon_children_of = { SYMB_SET_VAR_A }, 
      const std::set<std::string>& break_it_off_at = { SYMB_SEQUENCE }); // The defaults go line by line through assignments only.

   /// Goes depth-first through the formula tree and returns the n-th child with the given property.
   std::shared_ptr<MathStruct> retrieveNthChildWithProperty(const std::function<bool(const std::shared_ptr<MathStruct>)>& property, const int n = 0);

   /// Creates a vector of all variations of this formula where commutative terms in children (i.e., non-root terms) are flipped in both ways.
   std::vector<std::shared_ptr<MathStruct>> createAllCommVariationsInChildren(const std::shared_ptr<FormulaParser> parser = nullptr) const;
   
   /// Creates a vector of all variations of the given formulae where commutative terms in children (i.e., non-root terms) are flipped in both ways.
   static std::vector<std::shared_ptr<MathStruct>> createAllCommVariationsInChildren(const std::vector<std::shared_ptr<MathStruct>> &formulae, const std::shared_ptr<FormulaParser> parser = nullptr);
   
   static std::set<MetaRule> createAllCommVariationsOfMetaRules(const std::set<MetaRule>& rules, const bool children_only = true, const std::shared_ptr<FormulaParser> parser = nullptr);

   /// Creates a vector of all variations of the given formulae where ALL commutative terms (i.e., including root) are flipped in both ways.
   static std::vector<std::shared_ptr<MathStruct>> createAllCommVariations(const std::vector<std::shared_ptr<Term>>& formulae, const std::shared_ptr<FormulaParser> parser = nullptr);

   virtual void applyToMeAndMyParents(const std::function<void(std::shared_ptr<MathStruct>)>& f);
   virtual float eval(const std::shared_ptr<DataPack>& varVals, const std::shared_ptr<FormulaParser> parser = nullptr, const bool suppress_side_effects = false) = 0;
   
   /// Note that not suppressing side effects somewhat contradicts the const-ness of the
   /// operation, so do this with care.
   virtual float constEval(const std::shared_ptr<FormulaParser> parser = nullptr, const bool suppress_sideeffects = false) = 0;

   virtual bool resolveTo(const std::function<bool(std::shared_ptr<MathStruct>)>& match_cond, bool from_left) = 0;

   bool checkIfAllChildrenHaveConsistentFatherQuiet(const bool recursive = true) const;
   bool checkIfAllChildrenHaveConsistentFather(const bool recursive = true) const;

   enum class FatherSetterStyle {
      avoid_compound_structures,   // This is probably what you want.
      go_into_compound_structures,
   };

   std::shared_ptr<MathStruct> setChildrensFathers(
      const bool recursive = true,
      const bool go_into_compound_structures = true,
      std::set<std::shared_ptr<MathStruct>>&& visited = std::set<std::shared_ptr<MathStruct>>());

   std::shared_ptr<MathStruct> setChildrensFathers(
      const FatherSetterStyle stype,
      const bool recursive = true,
      std::set<std::shared_ptr<MathStruct>>&& visited = std::set<std::shared_ptr<MathStruct>>());

   /// <summary>
   /// The regular function to retrieve the child terms of this term.
   /// CAUTION: At compounds this is not usually what you'll want. A TermCompound has as its first
   /// operand the actual formula to execute ("compound operator"), while its second operand describes the logic to evaluate the first ("compound structure").
   /// For example: 
   ///      compound(x = y, set(p_(0), p_(1)))
   /// is the way an assignment operation is represented as formula. When traversing the formula tree, the part of interest is usually "x = y".
   /// Use getTermsJumpIntoCompounds() to jump directly to the terms of the first operand whenever a TermCompound is encountered.
   /// See also: getFatherJumpOverCompounds.
   /// </summary>
   /// <returns>The direct children of this term.</returns>
   std::vector<std::shared_ptr<Term>>& getOperands();
   std::vector<std::shared_ptr<Term>>& getOperandsConst() const;
   std::shared_ptr<Term> child0() const;
   std::shared_ptr<Term> child1() const;
   std::shared_ptr<Term> child2() const;
   std::shared_ptr<Term> child3() const;
   std::shared_ptr<Term> child4() const;

   /// <summary>
   /// See comment in getTerms().
   /// </summary>
   /// <returns>The direct children for each non-compound term, and the compound operand for TermCompound.</returns>
   std::vector<std::shared_ptr<Term>>& getTermsJumpIntoCompounds();
   std::shared_ptr<Term> child0JumpIntoCompounds() const;
   std::shared_ptr<Term> child1JumpIntoCompounds() const;
   std::shared_ptr<Term> child2JumpIntoCompounds() const;
   std::shared_ptr<Term> child3JumpIntoCompounds() const;
   std::shared_ptr<Term> child4JumpIntoCompounds() const;

   std::vector<std::shared_ptr<Term>> getTermsJumpIntoCompoundsIncludingCompoundStructure();
   
   enum class SerializationStyle {
      nusmv,
      cpp,
      plain_old_vfm,
   };

   enum class SerializationSpecial {
      none,
      enforce_postfix,
      print_compound_structures,
      enforce_postfix_and_print_compound_structures,
      remove_symb_ref,
      enforce_square_array_brackets // Will print x[i] rather than get(x + i) even when in non-cpp mode.
   };

   std::string serialize() const;
   std::string serialize(const SerializationSpecial special) const;
   std::string serializePlainOldVFMStyle(const SerializationSpecial special = SerializationSpecial::none) const;
   std::string serializePlainOldVFMStyleWithCompounds() const;

   std::string serialize(
      const std::shared_ptr<MathStruct> highlight, 
      const SerializationStyle style = SerializationStyle::cpp,
      const SerializationSpecial special = SerializationSpecial::none,
      const int indent = 0,
      const int indent_step = INDENT_STEP_DEFAULT,
      const int line_len = 200,
      const std::shared_ptr<DataPack> data = nullptr) const;

   std::string serializeWithinSurroundingFormula() const;
   std::string serializeWithinSurroundingFormula(const int levels_to_go_up) const;
   std::string serializeWithinSurroundingFormula(const int chars_left, const int chars_right) const;
   std::string serializeWithinSurroundingFormulaPlainOldVFMStyle() const;
   std::string serializeWithinSurroundingFormulaPlainOldVFMStyleWithCompounds() const;
   std::string serializePostfix() const;

   virtual void serialize(
      std::stringstream& os, 
      const std::shared_ptr<MathStruct> highlight, 
      const SerializationStyle style,
      const SerializationSpecial special,
      const int indent,
      const int indent_step,
      const int line_len,
      const std::shared_ptr<DataPack> data,
      std::set<std::shared_ptr<const MathStruct>>& visited) const;

   virtual std::string serializeK2(const std::map<std::string, std::string>& enum_values, std::pair<std::map<std::string, std::string>, std::string>& additional_functions, int& label_counter) const;
   std::string serializeK2(const std::map<std::string, std::string>& enum_values, std::pair<std::map<std::string, std::string>, std::string>& additional_functions) const;
   std::string serializeK2() const;

   int getNodeCount();
   int getLeafCount();
   int getNodeCount(const TraverseCompoundsType include_compound_structures);
   int getLeafCount(const TraverseCompoundsType include_compound_structures);
   bool printBrackets() const;
   
   virtual std::string getK2Operator() const;
   virtual std::string getNuSMVOperator() const;
   virtual std::string serializeNuSMV() const;
   virtual std::string serializeNuSMV(const std::shared_ptr<DataPack> data, const std::shared_ptr<FormulaParser> parser) const;
   virtual std::string getCPPOperator() const;

   virtual OperatorStructure getOpStruct() const = 0;
   int prio() const;

   /// <summary>
   /// The raw operator.
   /// For compounds this means SYMB_COMPOUND on TermCompound level, and the actual operator name (e.g, ";")
   /// on the TermCompoundOperator level.
   /// Note that often this is not what you'll want. Consider using getOptorOnCompoundLevel().
   /// </summary>
   /// <returns></returns>
   std::string getOptor() const;
   
   /// <summary>
   /// - Same as getOptor() for all non-compounds.
   /// - For compounds returns getCompoundOperator()->getOptor() on compound level, and constant SYMB_COMP_OP 
   ///   at the compound operator level.
   /// </summary>
   virtual std::string getOptorOnCompoundLevel() const;
   
   std::string getOptorJumpIntoCompound() const;

   virtual void setOptor(const std::string& new_optor_name);
   void setOpnds(const std::vector<std::shared_ptr<Term>>& opds);

   bool isRootTerm() const;

   void replaceOperand(const std::shared_ptr<Term> old, const std::shared_ptr<Term> by, const bool make_old_invalid = true);
   void replaceOperand(const int op_num, const std::shared_ptr<Term> by, const bool make_old_invalid = true);
   void replace(const std::shared_ptr<Term> by, const bool make_old_invalid = true);

   void replaceOperandLookIntoCompounds(const std::shared_ptr<Term> old, const std::shared_ptr<Term> by, const bool make_old_invalid = true);
   void replaceJumpOverCompounds(const std::shared_ptr<Term> by, const bool make_old_invalid = true);

   /// <summary>
   /// Assumes the father is a sequence operation ";" which also has a father.
   /// Replaces "x; this" or "this; x" with "x" and marks the removed parts invalid.
   /// </summary>
   void removeLineFromSequence();

   bool hasSideEffectsAnyChild();

   virtual std::vector<std::string> generatePostfixVector(const bool nestDownToCompoundStructures = false);

   virtual bool isStructurallyEqual(const std::shared_ptr<MathStruct> other);

   std::shared_ptr<MathStruct> getFather() const;
   std::shared_ptr<MathStruct> getFatherRaw() const;
   std::shared_ptr<MathStruct> getFatherJumpOverCompound() const;
   void setFather(std::shared_ptr<MathStruct> new_father);

   std::shared_ptr<MathStruct> thisPtrGoIntoCompound();
   std::shared_ptr<MathStruct> thisPtrGoUpToCompound();

   bool possiblyHighlightAndGoOnSerializing(
      std::stringstream& os,
      const std::shared_ptr<MathStruct> highlight,
      const SerializationStyle style,
      const SerializationSpecial special,
      const int indent,
      const int indent_step,
      const int line_len,
      const std::shared_ptr<DataPack> data,
      std::set<std::shared_ptr<const MathStruct>>& visited) const;

   void makeInvalid(const bool recursive = false);
   void makeValid(const bool recursive = false);
   bool isValid() const;

   bool isAssemblyCreated() const;

   /// <summary>
   /// First returned set is output variables from left-hand side of "=", second is set of variables
   /// on the righthand side.
   /// </summary>
   /// <param name="formula"></param>
   /// <returns></returns>
   VariablesWithKinds findSetVariables(const std::shared_ptr<DataPack> data) const;

   static void addAddMeasure(const std::shared_ptr<std::vector<std::vector<float>>> additional_measures, const int pos, const float val);

   static std::map<std::string, std::map<int, std::set<MetaRule>>> getAllRulesSoFar();
   static void resetAllRules();

protected:
   JITFunc fast_eval_func_{};

   enum class AssemblyState {
      yes,
      no,
      unknown
   };

   AssemblyState assembly_created_ = AssemblyState::unknown;
   std::string additionalOptionsForGraphvizNodeLabel();

private:
   static std::map<std::string, std::map<int, std::set<MetaRule>>> all_rules_;
   std::string resolveOptor(const SerializationStyle style) const;

   bool applicableMetaRule(
      std::shared_ptr<vfm::Term>& from,
      std::shared_ptr<vfm::Term>& to,
      std::vector<std::pair<std::shared_ptr<Term>, ConditionType>>& conditions,
      const std::shared_ptr<vfm::Term>& from_father,
      const std::shared_ptr<vfm::Term>& to_father,
      const std::shared_ptr<std::map<std::shared_ptr<Term>, std::shared_ptr<Term>>>& condition_fathers);

   /// Replaces a term t with lambda(t) at specified  positions. This allows
   /// to simply write, e.g.:
   ///   for (@i, 5, 8) { printfln(i) }
   /// instead of:
   ///   for (@i, 5, 8) { lambda() { printfln(i) } }
   void applyImplicitLambdas(const std::map<std::string, std::vector<int>>& implicit_lambdas);

   bool checkIfAllChildrenHaveConsistentFatherRaw(const bool recursive, const bool root, const bool quiet) const;

   static std::shared_ptr<std::vector<std::string>> additional_test_info_;

   /// When a father gets redirected, the former child is marked invalid.
   /// Also, children of such terms may get invalid (but don't have to, for efficiency reasons).
   bool is_valid_{ true };

   bool full_correct_brackets_{ false };
   bool contains_private_set_var_{ false };

   std::vector<TermCompound*> compound_structure_references_{}; /// Points to the owning compound structure, if any.

   std::vector<std::shared_ptr<Term>> opnds_{};
   std::weak_ptr<MathStruct> father_{};

#if defined(ASMJIT_ENABLED)
   std::shared_ptr<asmjit::JitRuntime> jit_runtime_{ nullptr };
#endif
};

using ::operator<<;
} // vfm
