//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file

#include "model_checking/yaaa_runnable.h"
#include "parser.h"


int vfm::YaaaRunnable::ID = 0;

vfm::YaaaRunnable::YaaaRunnable(const std::string& name) : Failable("YaaaRunnable-" + name), id_(ID), name_(name) {
   ID++;
}

std::string vfm::YaaaRunnable::getName() const 
{
   return name_;
}

std::string vfm::YaaaRunnable::getGVName() const
{
   std::string s;

   for (const auto& c : getName()) {
      if (std::isalnum(c)) {
         s += c;
      }
   }

   return s + "_" + std::to_string(id_);
}

void vfm::YaaaRunnable::completeSendConditionsForRunnable() 
{
   for (const auto& el : getOutput()) {
      if (!getSendConditions().count(el)) {
         addDebug("Auto-completing missing send condition for output port '" + el + "'.");
         addSendCondition(_true(), el);
      }
   }
}

void vfm::YaaaRunnable::addSendCondition(const std::string& condition, const std::string& output_name)
{
   if (std::find(outputs_.begin(), outputs_.end(), output_name) == outputs_.end()) {
      addError("Output '" + output_name + "' not found.");
      for (const auto& s : outputs_) addDebug("Found output port '" + s + "'.");;
      addDebug("Found " + std::to_string(outputs_.size()) + " output ports.");
   }

   auto cond_term = MathStruct::parseMathStruct(condition, true)->toTermIfApplicable();
   addSendCondition(cond_term, output_name);
}

void vfm::YaaaRunnable::addSendCondition(const std::shared_ptr<Term> condition, const std::string& output_name)
{
   send_conditions_[output_name] = condition;
}

void vfm::YaaaRunnable::addInput(const std::string& port_name)
{
   inputs_.insert(port_name);
}

void vfm::YaaaRunnable::addOutput(const std::string& port_name)
{
   outputs_.insert(port_name);
}

void vfm::YaaaRunnable::addInner(const std::string& port_name)
{
   inner_.insert(port_name);
}

std::set<std::string> vfm::YaaaRunnable::getInput() const
{
   return inputs_;
}

std::set<std::string> vfm::YaaaRunnable::getOutput() const
{
   return outputs_;
}

std::set<std::string> vfm::YaaaRunnable::getInner() const
{
   return inner_;
}

std::map<std::string, std::shared_ptr<vfm::Term>> vfm::YaaaRunnable::getSendConditions() const
{
   return send_conditions_;
}
