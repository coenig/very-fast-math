//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file
#pragma once
#include "enum_wrapper.h"
#include "parsable.h"
#include <utility>
#include <string>
#include <iostream>

namespace vfm {

const std::string PAR_NAME_OPERATOR_NAME = "name";
const std::string PAR_NAME_ASSOCIATIVITY_TYPE = "associativityType";
const std::string PAR_NAME_OUTPUT_FORMAT = "operatorType";
const std::string PAR_NAME_PARAMETER_NUM = "parnum";
const std::string PAR_NAME_PRECEDENCE = "precedence";
const std::string PAR_NAME_COMMUTATIVITY = "isCommutative";
const std::string PAR_NAME_ASSOCIATIVITY = "isAssociative";
const std::string PAR_NAME_ADDITIONAL_OPTION = "additionalOption";

const int PARNUM_AUTO_MODE = -1;
const int PARNUM_FLEXIBLE_MODE = -2;

const std::string PARSER_NAME_OP_STRUCT = "OperatorStructure";

enum class TrueFalseEnum { True, False };
enum class OutputFormatEnum { prefix, infix, postfix, plain, braces_from_second_argument };
enum class AssociativityTypeEnum { right, left };
enum class AdditionalOptionEnum { none, always_print_brackets, allow_arbitrary_number_of_arguments };

const std::map<TrueFalseEnum, std::string> true_false_map{
   { TrueFalseEnum::False, "false" },
   { TrueFalseEnum::True, "true" }
};

const std::map<OutputFormatEnum, std::string> output_format_map {
   { OutputFormatEnum::prefix, "prefix" },
   { OutputFormatEnum::infix, "infix" },
   { OutputFormatEnum::postfix, "postfix" },
   { OutputFormatEnum::plain, "plain" },
   { OutputFormatEnum::braces_from_second_argument, "braces_from_second_argument" }
};

const std::map<AssociativityTypeEnum, std::string> associativity_type_map {
   { AssociativityTypeEnum::right, "right" },
   { AssociativityTypeEnum::left, "left" },
};

const std::map<AdditionalOptionEnum, std::string> additional_option_map {
   { AdditionalOptionEnum::none, "none" },
   { AdditionalOptionEnum::always_print_brackets, "always_print_brackets" },
   { AdditionalOptionEnum::allow_arbitrary_number_of_arguments, "allow_arbitrary_number_of_arguments" },
};

class OutputFormat : public fsm::EnumWrapper<OutputFormatEnum>
{
public:
   explicit OutputFormat(const OutputFormatEnum& enum_val) : EnumWrapper("OutputFormat", output_format_map, enum_val) {}
   explicit OutputFormat(const int enum_val_as_num) : EnumWrapper("OutputFormat", output_format_map, enum_val_as_num) {}
   explicit OutputFormat(const std::string& enum_val_as_string) : EnumWrapper("OutputFormat", output_format_map, enum_val_as_string) {}
   explicit OutputFormat() : EnumWrapper("OutputFormat", output_format_map, OutputFormatEnum::plain) {}
};

class AssociativityType : public fsm::EnumWrapper<AssociativityTypeEnum>
{
public:
   explicit AssociativityType(const AssociativityTypeEnum& enum_val) : EnumWrapper("AssociativityType", associativity_type_map, enum_val) {}
   explicit AssociativityType(const int enum_val_as_num) : EnumWrapper("AssociativityType", associativity_type_map, enum_val_as_num) {}
   explicit AssociativityType(const std::string& enum_val_as_string) : EnumWrapper("AssociativityType", associativity_type_map, enum_val_as_string) {}
   explicit AssociativityType() : EnumWrapper("AssociativityType", associativity_type_map, AssociativityTypeEnum::left) {}
};

class AdditionalOption : public fsm::EnumWrapper<AdditionalOptionEnum>
{
public:
   explicit AdditionalOption(const AdditionalOptionEnum& enum_val) : EnumWrapper("AdditionalOption", additional_option_map, enum_val) {}
   explicit AdditionalOption(const int enum_val_as_num) : EnumWrapper("AdditionalOption", additional_option_map, enum_val_as_num) {}
   explicit AdditionalOption(const std::string& enum_val_as_string) : EnumWrapper("AdditionalOption", additional_option_map, enum_val_as_string) {}
   explicit AdditionalOption() : EnumWrapper("AdditionalOption", additional_option_map, AdditionalOptionEnum::none) {}
};

class TrueFalseWrapper : public fsm::EnumWrapper<TrueFalseEnum>
{
public:
   explicit TrueFalseWrapper(const TrueFalseEnum& enum_val) : EnumWrapper("TrueFalseWrapper", true_false_map, enum_val) {}
   explicit TrueFalseWrapper(const int enum_val_as_num) : EnumWrapper("TrueFalseWrapper", true_false_map, enum_val_as_num ? TrueFalseEnum::True : TrueFalseEnum::False) {}
   explicit TrueFalseWrapper(const std::string& enum_val_as_string) : EnumWrapper("TrueFalseWrapper", true_false_map, enum_val_as_string) {}
   explicit TrueFalseWrapper() : EnumWrapper("TrueFalseWrapper", true_false_map, TrueFalseEnum::False) {}
};

class OperatorStructure
{
public:
   OutputFormatEnum op_pos_type; // prefix = function; infix = operator. (postfix so far not available.)
   AssociativityTypeEnum associativity;
   int arg_num;
   int precedence;
   std::string op_name;
   bool assoc;
   bool commut;
   AdditionalOptionEnum option;

   std::string serialize() const;

   /// \brief Sets the arg_num to the given value iff the parameter
   /// is currently set to AUTO_MODE. Returns iff the value was changed.
   bool setArgNumIfInAutoMode(const int val);

   bool parseProgram(const std::string& opstruct_command);

   explicit OperatorStructure(const std::string& fromString, bool& error_occurred);

   explicit OperatorStructure(
      const OutputFormatEnum& op_pos_type,
      const AssociativityTypeEnum& associativity,
      const int& arg_num, 
      const int& precedence, 
      const std::string& op_name,
      const bool& assoc,
      const bool& commut,
      const AdditionalOptionEnum& opt = AdditionalOptionEnum::none);

   explicit OperatorStructure(const std::string& op_name, const int& arg_num);
};

} // vfm