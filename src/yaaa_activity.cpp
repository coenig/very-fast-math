//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file

#include "model_checking/yaaa_activity.h"
#include "model_checking/yaaa_runnable.h"
#include "parser.h"
#include "term_var.h"
#include "static_helper.h"
#include <algorithm>

int vfm::YaaaActivity::ID = 0;

vfm::YaaaActivity::YaaaActivity(const std::string& name, const bool is_gateway)
   : Failable("YaaaActivity-" + name), is_gateway_(is_gateway), id_(ID), name_(name), trigger_cycle_time_(50), trigger_offset_(0), trigger_timeout_(-1) {
   ID++;
}

void vfm::YaaaActivity::addRunnable(const YaaaRunnablePtr runnable)
{
   runnables_.push_back(runnable);
   addFailableChild(runnables_[runnables_.size() - 1], "");
}

std::string vfm::YaaaActivity::getName() const
{
   return name_;
}

std::vector<vfm::YaaaRunnablePtr> vfm::YaaaActivity::getRunnables()
{
   return runnables_;
}

void vfm::YaaaActivity::addConnection(const std::string& from, const std::string& to)
{
   cache_of_senders_and_receivers_.clear();

   connections_.push_back({ from, to });
   bool from_is_runnable = false;
   bool to_is_runnable = false;
   std::string from_runnable_name;

   for (auto& runnable : runnables_) {
      if (StaticHelper::stringStartsWith(from, runnable->getName() + ".")) {
         runnable->addOutput(from.substr(runnable->getName().size() + 1));
         from_runnable_name = from.substr(0, runnable->getName().size());
         from_is_runnable = true;
      }

      if (StaticHelper::stringStartsWith(to, runnable->getName() + ".")) {
         runnable->addInput(to.substr(runnable->getName().size() + 1));
         to_is_runnable = true;
      }
   }

   if (!from_is_runnable) {
      addInput(from);
   }

   if (!to_is_runnable) {
      addOutput(to, from_runnable_name);
   }
}

std::string vfm::YaaaActivity::getGVName() const
{
   std::string s;

   for (const auto& c : getName()) {
      if (std::isalnum(c)) {
         s += c;
      }
   }

   return s + "_" + std::to_string(id_);
}

std::vector<std::pair<std::string,std::string>> vfm::YaaaActivity::getConnections() const
{
   return connections_;
}

std::vector<std::string> vfm::YaaaActivity::getDataTrigger() const
{
   return trigger_inputs_;
}

int vfm::YaaaActivity::getTriggerCycleTime() const
{
   return trigger_cycle_time_;
}

int vfm::YaaaActivity::getTriggerOffset() const
{
   return trigger_offset_;
}

int vfm::YaaaActivity::getTriggerTimeout() const
{
   return trigger_timeout_;
}

bool vfm::YaaaActivity::isCyclicTrigger() const
{
   return getTriggerCycleTime() >= 0;
}

int vfm::YaaaActivity::getRelevantTime() const
{
   return isCyclicTrigger() ? getTriggerCycleTime() : getTriggerTimeout();
}

std::set<std::string> vfm::YaaaActivity::getInputs() const
{
   return inputs_;
}

std::set<std::string> vfm::YaaaActivity::getOutputs() const
{
   return outputs_;
}

void vfm::YaaaActivity::setTriggerCycleTime(const int milis)
{
   trigger_cycle_time_ = milis;
}

void vfm::YaaaActivity::setTriggerOffset(const int offset)
{
   trigger_offset_ = offset;
}

void vfm::YaaaActivity::setTriggerTimeout(const int timeout)
{
   trigger_timeout_ = timeout;
}

void vfm::YaaaActivity::addTrigger(const std::string& trigger)
{
   trigger_cycle_time_ = -1;
   trigger_offset_ = -1;

   trigger_inputs_.push_back(trigger);
}

bool vfm::YaaaActivity::isGateway() const
{
   return is_gateway_;
}

vfm::YaaaRunnablePtr vfm::YaaaActivity::getRunnableByName(const std::string& name)
{
   for (auto& ru : getRunnables()) {
      if (ru->getName() == name) {
         return ru;
      }
   }

   throw "No runnable named '" + name + "' exists.";
}

void vfm::YaaaActivity::addInput(const std::string & port_name)
{
   inputs_.insert(port_name);
}

void vfm::YaaaActivity::addOutput(const std::string& port_name, const std::string& sending_runnable)
{
   outputs_.insert(port_name);
   runnable_name_to_output_names_.insert({ sending_runnable, {} });
   runnable_name_to_output_names_[sending_runnable].push_back(port_name);
}

void vfm::YaaaActivity::addSendCondition(const std::string& condition, const std::string& output_name)
{
   auto cond_term = MathStruct::parseMathStruct(condition, true)->toTermIfApplicable();
   addSendCondition(cond_term, output_name);
}

void vfm::YaaaActivity::addSendCondition(const std::shared_ptr<Term> condition, const std::string& output_name)
{
   if (std::find(outputs_.begin(), outputs_.end(), output_name) == outputs_.end()) {
      addError("Output '" + output_name + "' not found in activity '" + getName() + "'.");
      for (const auto& s : outputs_) addError("Found output port '" + s + "'.");;
      addError("Found " + std::to_string(outputs_.size()) + " output ports.");
   }

   send_conditions_[output_name] = condition;
}

void vfm::YaaaActivity::addSendCondition(const std::pair<std::string, std::shared_ptr<vfm::Term>> pair)
{
   addSendCondition(pair.second, pair.first);
}

void addRunnablePrefixToAllVariablesInCondition(std::shared_ptr<vfm::Term> condition, const vfm::YaaaRunnablePtr ru)
{
   condition->applyToMeAndMyChildren([&ru](const std::shared_ptr<vfm::MathStruct> m) {
      if (m->isTermVar()) {
         auto term_var = m->toVariableIfApplicable();

         if (term_var->getVariableName().find('.') == std::string::npos) {
            term_var->setVariableName(ru->getName() + "." + term_var->getVariableName());
         }
      }
   });
}

void vfm::YaaaActivity::collectFirstOrderSendConditionsForAllOutputs()
{
   for (const auto& s : outputs_) {
      auto vec = findRunnableSendingTowards(s);

      if (!vec.empty()) {
         std::string out_name;
         auto ru = vec.back();
         auto condition = ru.first->getSendConditions().at(ru.second)->copy();
         addRunnablePrefixToAllVariablesInCondition(condition, ru.first);
         addSendCondition(_id(condition), s);
      }
   }
}

std::string vfm::YaaaActivity::getOutputPortSendingToInputPort(const std::string& fully_qualified_input_port_name)
{
   for (const auto& c : connections_) {
      if (c.second == fully_qualified_input_port_name) {
         return c.first;
      }
   }

   addError("No connection points towards input port '" + fully_qualified_input_port_name + "'");
   return "#MALFORMED#";
}

bool vfm::YaaaActivity::receivesDataDirectlyFrom(const std::string& possible_receiver_runnable, const std::string& possible_sender_runnable) const
{
   for (const auto& c : connections_) {
      if (StaticHelper::stringStartsWith(c.first, possible_sender_runnable + ".")
         && StaticHelper::stringStartsWith(c.second, possible_receiver_runnable + ".")) {
         return true;
      }
   }

   return false;
}

bool vfm::YaaaActivity::receivesDataRecursivelyFrom(
   const std::string& possible_receiver_runnable, 
   const std::string& possible_sender_runnable,
   const std::set<std::string> visited) const
{
   if (cache_of_senders_and_receivers_.count({ possible_receiver_runnable, possible_sender_runnable })) {
      return cache_of_senders_and_receivers_.at({ possible_receiver_runnable, possible_sender_runnable });
   }

   if (receivesDataDirectlyFrom(possible_receiver_runnable, possible_sender_runnable)) {
      cache_of_senders_and_receivers_.insert({ { possible_receiver_runnable, possible_sender_runnable }, true });
      return true;
   }

   if (possible_receiver_runnable == possible_sender_runnable) {
      cache_of_senders_and_receivers_.insert({ { possible_receiver_runnable, possible_sender_runnable }, false });
      return false;
   }

   auto new_set = visited;
   new_set.insert(possible_receiver_runnable);
   new_set.insert(possible_sender_runnable);

   for (const auto& ru : runnables_) {
      if (!visited.count(ru->getName())) {
         if (receivesDataRecursivelyFrom(possible_receiver_runnable, ru->getName(), new_set)
            && receivesDataRecursivelyFrom(ru->getName(), possible_sender_runnable, new_set)) {
            cache_of_senders_and_receivers_.insert({ { possible_receiver_runnable, possible_sender_runnable }, true });
            return true;
         }
      }
   }

   cache_of_senders_and_receivers_.insert({ { possible_receiver_runnable, possible_sender_runnable }, false });
   return false;
}

bool vfm::YaaaActivity::deriveOutputConditionForSinglePortOnce(const std::string& output_port_name)
{
   auto condition = send_conditions_.at(output_port_name);
   bool change_occurred = false;

   condition->applyToMeAndMyChildren([this, &change_occurred](const std::shared_ptr<MathStruct> m) {
      if (m->isTermVar()) {
         auto term_var = m->toVariableIfApplicable();
         auto input_port = term_var->getVariableName();
         auto vec = findRunnableSendingTowards(input_port);
         std::shared_ptr<vfm::Term> cond_for_port;

         if (vec.empty()) { // Empty array indicates no sender, i.e., m already SHOULD BE an activity input.
            if (StaticHelper::stringContains(input_port, '.')) {
               addError("No sender found towards non-activity input port '" + input_port + "' of activity '" + getName() + "'.");
            }
         } else {
            auto ru_pair = vec[0];
            auto ru = ru_pair.first;
            auto port_name = ru_pair.second;

            if (!ru) { // The NULL runnable indicates an activity input.
               cond_for_port = _var(port_name);
            }
            else {
               cond_for_port = ru->getSendConditions().at(port_name)->copy();
               addRunnablePrefixToAllVariablesInCondition(cond_for_port, ru);
            }

            m->getFather()->replaceOperand(term_var, cond_for_port);
            change_occurred = true;
         }
      }
   });

   return change_occurred;
}

void vfm::YaaaActivity::deriveOutputConditionsFromActivityInputs()
{
   collectFirstOrderSendConditionsForAllOutputs();
   
   for (const auto& output : outputs_) {
      while (deriveOutputConditionForSinglePortOnce(output));
      addDebug("Simplifying send condition for output port '" + output + "'.");
      send_conditions_.at(output) = MathStruct::simplify(send_conditions_.at(output));
   }
}

std::shared_ptr<vfm::Term> vfm::YaaaActivity::getSendCondition(const std::string& port) const
{
   return send_conditions_.at(port);
}

std::vector<std::vector<std::pair<std::pair<std::string, std::string>, std::shared_ptr<vfm::Term>>>> vfm::YaaaActivity::getSendConditionsGroupedAndOrdered() const
{
   std::vector<std::vector<std::pair<std::pair<std::string, std::string>, std::shared_ptr<Term>>>> vec;

   for (const auto& pair : runnable_name_to_output_names_) {
      std::vector<std::pair<std::pair<std::string, std::string>, std::shared_ptr<Term>>> block;

      for (const auto& el : pair.second) {
         if (send_conditions_.count(el)) {
            block.push_back({ { el, pair.first}, send_conditions_.at(el) });
         }
         else {
            addError("Send condition not found for '" + el + "'.");
         }
      }

      vec.push_back(block);
   }

   std::sort(vec.begin(), vec.end(), [this](const auto& lhs, const auto& rhs) -> bool
   {
      if (lhs.empty() || rhs.empty()) {
         addError("Error, cannot compare apples and oranges.");
         return lhs < rhs;
      }
      else {
         return receivesDataRecursivelyFrom(rhs[0].first.second, lhs[0].first.second);
      }
   });

   return vec;
}

std::vector<std::pair<vfm::YaaaRunnablePtr, std::string>> vfm::YaaaActivity::findRunnableSendingTowards(const std::string& recipient)
{
   std::vector<std::pair<vfm::YaaaRunnablePtr, std::string>> vec;

   for (const auto& c : connections_) {
      if (c.second == recipient) {
         int pos = c.first.find('.');
         if (pos >= 0) {
            vec.push_back({ getRunnableByName(c.first.substr(0, pos)), c.first.substr(pos + 1) });
         }
         else { // It's an activity input.
            vec.push_back({ nullptr, c.first });
         }
      }
   }

   if (vec.size() > 1) {
      addError("More than one sender found for recipient '" + recipient + "'.");
      
      for (const auto& el : vec) {
         addError("Sender found: '" + el.first->getName() + "' on port '" + el.second + "'.");
      }
   }

   return vec;
}

std::vector<std::pair<vfm::YaaaRunnablePtr, std::string>> vfm::YaaaActivity::findRunnablesReceivingFrom(const std::string& sender)
{
   std::vector<std::pair<vfm::YaaaRunnablePtr, std::string>> vec;

   for (const auto& c : connections_) {
      if (c.first == sender) {
         int pos = c.second.find('.');
         if (pos >= 0) {
            vec.push_back({ getRunnableByName(c.second.substr(0, pos)), c.second.substr(pos + 1) });
         } // Else it's an activity output.
      }
   }

   return vec;
}
