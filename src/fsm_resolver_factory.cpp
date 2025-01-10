//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file

#include "fsm_resolver_factory.h"
#include "static_helper.h"
#include <typeinfo>

std::shared_ptr<vfm::fsm::FSMResolver> vfm::fsm::FSMResolverFactory::createNewResolverFromExisting(const std::shared_ptr<FSMResolver> other)
{
   std::string s = typeid(*other).name();
   std::string s_default = typeid(FSMResolverDefault).name();
   std::string s_max_trans_weight = typeid(FSMResolverDefaultMaxTransWeight).name();
   std::string s_remain_on_no_transition = typeid(FSMResolverRemainOnNoTransition).name();
   std::string s_remain_on_no_transition_and_obey_insertion_order = typeid(FSMResolverRemainOnNoTransitionAndObeyInsertionOrder).name();

   if (s == s_default) {
      return std::make_shared<FSMResolverDefault>();
   }
   else if (s == s_max_trans_weight) {
      auto derived = std::dynamic_pointer_cast<FSMResolverDefaultMaxTransWeight>(other);
      return std::make_shared<FSMResolverDefaultMaxTransWeight>(derived->getThreshold());
   }
   else if (s == s_remain_on_no_transition) {
      return std::make_shared<FSMResolverRemainOnNoTransition>();
   }
   else if (s == s_remain_on_no_transition_and_obey_insertion_order) {
      return std::make_shared<FSMResolverRemainOnNoTransitionAndObeyInsertionOrder>();
   }

   return nullptr;
}

vfm::fsm::FSMResolverFactory::FSMResolverFactory() : Parsable("FSMResolverFactory") {}

bool vfm::fsm::FSMResolverFactory::parseProgram(const std::string& program)
{
   std::string to_parse = StaticHelper::removeWhiteSpace(program);

   if (to_parse[0] != PROGRAM_PREPROCESSOR_DENOTER) {
      addError("Command '" + to_parse + std::string("' does not start with a preprocessor denoter: '") + PROGRAM_PREPROCESSOR_DENOTER + "'.");
      return false;
   }

   auto main_pair = StaticHelper::split(program, PROGRAM_ARGUMENT_ASSIGNMENT);

   if (main_pair.size() != 2) {
      addError("Command '" + to_parse + std::string("' is malformed as an FSMResolver definition."));
      return false;
   }

   if (main_pair[0].substr(1) != RESOLVER_DENOTER) {
      addError("'" + main_pair[0] + "' is not a well-formed resolver denoter. Should be: '" + RESOLVER_DENOTER + "'.");
      return false;
   }

   std::string resolver_name = StaticHelper::split(main_pair[1], PROGRAM_COMMAND_SEPARATOR)[0]; // Remove ";" at the end.
   std::vector<std::string> pair = StaticHelper::split(resolver_name, PROGRAM_OPENING_OP_STRUCT_BRACKET_LEGACY); // Find additional parameters.
   resolver_name = pair[0];

   std::string additional_parameters = "";
   if (pair.size() == 2) {
      additional_parameters = StaticHelper::split(pair[1], PROGRAM_CLOSING_OP_STRUCT_BRACKET_LEGACY)[0]; // Remove ")" at the end.
   }

   if (resolver_name == FSMResolverDefault::RESOLVER_NAME_DEFAULT) {
      generated_resolver_ = std::make_shared<FSMResolverDefault>();
   } else if (resolver_name == FSMResolverRemainOnNoTransition::RESOLVER_NAME_REMAIN_ON_NO_TRANSITION) {
      generated_resolver_ = std::make_shared<FSMResolverRemainOnNoTransition>();
   } else if (resolver_name == FSMResolverRemainOnNoTransitionAndObeyInsertionOrder::RESOLVER_NAME_REMAIN_ON_NO_TRANSITION_AND_OBEY_INSERTION_ORDER) {
      generated_resolver_ = std::make_shared<FSMResolverRemainOnNoTransitionAndObeyInsertionOrder>();
   } else if (resolver_name == FSMResolverDefaultMaxTransWeight::RESOLVER_NAME_MAX_TRANS_WEIGHT) {
      if (!StaticHelper::isParsableAsFloat(additional_parameters)) {
         addError(std::string(FSMResolverDefaultMaxTransWeight::RESOLVER_NAME_MAX_TRANS_WEIGHT) + " requires an additional float parameter, but '" + additional_parameters + "' is not parsable to float.");
         return false;
      }
      float threshold = std::stof(additional_parameters);
      generated_resolver_ = std::make_shared<FSMResolverDefaultMaxTransWeight>(threshold);
   } else {
       addError("Command '" + to_parse + "' could not be parsed. '" + resolver_name + "' is not a known resolver name.");
       return false;
   }

   return true;
}

bool vfm::fsm::FSMResolverFactory::isResolverCommand(const std::string& command) const
{
   if (!StaticHelper::stringContains(command, PROGRAM_ARGUMENT_ASSIGNMENT)
       || !StaticHelper::stringContains(command, PROGRAM_PREPROCESSOR_DENOTER)) {
      return false;
   }

   auto real_command = StaticHelper::split(command, PROGRAM_ARGUMENT_ASSIGNMENT)[1]; // TODO: Does not cover all cases.

   return StaticHelper::stringStartsWith(real_command, FSMResolverDefault::RESOLVER_NAME_DEFAULT)
          || StaticHelper::stringStartsWith(real_command, FSMResolverRemainOnNoTransition::RESOLVER_NAME_REMAIN_ON_NO_TRANSITION)
          || StaticHelper::stringStartsWith(real_command, FSMResolverRemainOnNoTransitionAndObeyInsertionOrder::RESOLVER_NAME_REMAIN_ON_NO_TRANSITION_AND_OBEY_INSERTION_ORDER)
          || StaticHelper::stringStartsWith(real_command, FSMResolverDefaultMaxTransWeight::RESOLVER_NAME_MAX_TRANS_WEIGHT);
}

std::shared_ptr<vfm::fsm::FSMResolver> vfm::fsm::FSMResolverFactory::getGeneratedResolver() const
{
   return generated_resolver_;
}