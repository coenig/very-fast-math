//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file

#include "operator_structure.h"
#include "math_struct.h"
#include "static_helper.h"
#include <sstream>

using namespace vfm;

OperatorStructure::OperatorStructure(const std::string& fromString, bool& error_occurred) :
   op_pos_type(OutputFormatEnum::prefix),      // Default: Function.
   associativity(AssociativityTypeEnum::left), // Default: Left-associative.
   arg_num(PARNUM_AUTO_MODE),                  // Default: Auto.
   precedence(PARNUM_AUTO_MODE),               // Default: Low precedence, does not make sense for functions.
   op_name("*D$U#M#M$Y*"),                     // Default: Unparseable chars, at least a name has to be explicitly given.
   assoc(false),                               // Default: Non-associative, does not make sense for functions.
   commut(false),                              // Default: Non-commutative, does not make sense for functions.
   option(AdditionalOptionEnum::none)          // Default: No additional option.
{
   error_occurred = !parseProgram(fromString);
}

OperatorStructure::OperatorStructure(
   const OutputFormatEnum& op_pos_type, 
   const AssociativityTypeEnum& associativity, 
   const int& arg_num, 
   const int& precedence, 
   const std::string& op_name, 
   const bool& assoc, 
   const bool& commut, 
   const AdditionalOptionEnum& opt) :
   op_pos_type(op_pos_type), 
    associativity(associativity), 
    arg_num(arg_num), 
    precedence(precedence), 
    op_name(op_name),  
    assoc(assoc), 
    commut(commut), 
    option(opt)
{
}

vfm::OperatorStructure::OperatorStructure(const std::string& op_name, const int& arg_num)
   : OperatorStructure(OutputFormatEnum::prefix, AssociativityTypeEnum::left, arg_num, -1, op_name, false, false)
{}

bool vfm::OperatorStructure::setArgNumIfInAutoMode(const int val)
{
   if (arg_num == PARNUM_AUTO_MODE) {
      arg_num = val;
      return true;
   }

   return false;
}

bool vfm::OperatorStructure::parseProgram(const std::string& opstruct_command)
{
   std::string to_parse = StaticHelper::removeWhiteSpace(opstruct_command);
   auto prog_split = StaticHelper::split(to_parse, PROGRAM_OP_STRUCT_DELIMITER);

   for (int i = 0; i < prog_split.size(); i++) {
      auto pair = StaticHelper::split(prog_split[i], PROGRAM_ARGUMENT_ASSIGNMENT);

      if (pair.size() == 1) {
         if (StaticHelper::isParsableAsInt(pair[0])) {
            arg_num = std::stoi(pair[0]); // Assume any isolated number to be number of parameters.
         }
         else {
            op_name = pair[0]; // Assume any isolated non-number to be the operator name.
         }
      }
      else if (pair.size() == 2) {
         if (pair[0] == PAR_NAME_OUTPUT_FORMAT) {
            op_pos_type = OutputFormat(pair[1]).getEnum();

            if (op_pos_type == OutputFormatEnum::infix && arg_num == PARNUM_AUTO_MODE) {
               arg_num = 2;
            }
         }
         else if (pair[0] == PAR_NAME_ASSOCIATIVITY_TYPE) {
            associativity = AssociativityType(pair[1]).getEnum();
         }
         else if (pair[0] == PAR_NAME_PARAMETER_NUM) {
            arg_num = std::stoi(pair[1]);
         }
         else if (pair[0] == PAR_NAME_PRECEDENCE) {
            precedence = std::stoi(pair[1]);

            if (precedence == -1) {
               precedence = 1000; // TODO: Still an issue how to access precedences at runtime.
            }
         }
         else if (pair[0] == PAR_NAME_ASSOCIATIVITY) {
            assoc = TrueFalseWrapper(pair[1]) == TrueFalseEnum::True;
         }
         else if (pair[0] == PAR_NAME_COMMUTATIVITY) {
            commut = TrueFalseWrapper(pair[1]) == TrueFalseEnum::True;
         }
         else if (pair[0] == PAR_NAME_ADDITIONAL_OPTION) {
            option = AdditionalOption(pair[1]).getEnum();
         }
         else {
            op_name = prog_split[i]; // Use the whole thing as operator name. After all, it could be "==" or so.
         }
      }
      else {
         Failable::getSingleton()->addFatalError("Cannot parse operator structure '" + to_parse + "'.");
         return false;
      }
   }

   return true;
}

std::string OperatorStructure::serialize() const
{
   static const std::string assign = StaticHelper::makeString(PROGRAM_ARGUMENT_ASSIGNMENT);
   static const std::string delimiter = StaticHelper::makeString(PROGRAM_OP_STRUCT_DELIMITER);
   std::string s;

   s += op_name + delimiter;
   s += std::to_string(arg_num);
   
   if (op_pos_type != OutputFormatEnum::prefix) {
      s += delimiter + PAR_NAME_OUTPUT_FORMAT + assign + OutputFormat(op_pos_type).getEnumAsString();
      s += delimiter + PAR_NAME_ASSOCIATIVITY_TYPE + assign + AssociativityType(associativity).getEnumAsString();
      s += delimiter + PAR_NAME_PRECEDENCE + assign + std::to_string(precedence);
      s += delimiter + PAR_NAME_ASSOCIATIVITY + assign + TrueFalseWrapper(assoc).getEnumAsString();
      s += delimiter + PAR_NAME_COMMUTATIVITY + assign + TrueFalseWrapper(commut).getEnumAsString();
   }

   if (option != AdditionalOptionEnum::none) {
      s += delimiter + PAR_NAME_ADDITIONAL_OPTION + assign + AdditionalOption(option).getEnumAsString();
   }

   return s;
}
