//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file
#pragma once

#include "model_checking/yaaa_runnable.h"
#include "failable.h"
#include "term.h"
#include <vector>
#include <string>
#include <cctype>
#include <set>
#include <memory>

namespace vfm {

class YaaaActivity;
using YaaaActivityPtr = std::shared_ptr<YaaaActivity>;

class YaaaActivity : public Failable {
public:
   YaaaActivity(const std::string& name, const bool is_gateway = false);

   void addRunnable(const YaaaRunnablePtr runnable);
   std::string getName() const;
   std::vector<YaaaRunnablePtr> getRunnables();
   void addConnection(const std::string& from, const std::string& to);
   std::string getGVName() const;
   std::vector<std::pair<std::string, std::string>> getConnections() const;
   std::vector<std::string> getDataTrigger() const;
   
   int getTriggerCycleTime() const;
   int getTriggerOffset() const;
   int getTriggerTimeout() const;
   bool isCyclicTrigger() const;
   int getRelevantTime() const; /// This is the cycle time for cyclic triggers and the timeout otherwise.
   std::set<std::string> getInputs() const;
   std::set<std::string> getOutputs() const;

   void setTriggerCycleTime(const int milis);
   void setTriggerOffset(const int offset);
   void setTriggerTimeout(const int timeout);
   void addTrigger(const std::string& trigger);
   bool isGateway() const;
   YaaaRunnablePtr getRunnableByName(const std::string& name);

   void addInput(const std::string& port_name);
   void addOutput(const std::string& port_name, const std::string& sending_runnable);

   void deriveOutputConditionsFromActivityInputs();
   std::shared_ptr<Term> getSendCondition(const std::string& port) const;
   std::vector<std::vector<std::pair<std::pair<std::string, std::string>, std::shared_ptr<Term>>>> getSendConditionsGroupedAndOrdered() const;

   bool receivesDataDirectlyFrom(const std::string& possible_receiver_runnable, const std::string& possible_sender_runnable) const;
   bool receivesDataRecursivelyFrom(
      const std::string& possible_receiver_runnable, 
      const std::string& possible_sender_runnable,
      const std::set<std::string> visited = {}) const;

private:
   static int ID;
   const bool is_gateway_;
   int id_;
   std::string name_;
   int trigger_cycle_time_; // In ms, negative means data-triggered.
   int trigger_offset_;
   int trigger_timeout_;
   std::set<std::string> inputs_;
   std::vector<std::string> trigger_inputs_; // Subset of inputs, used when data-triggered
   std::set<std::string> outputs_;
   std::map<std::string, std::vector<std::string>> runnable_name_to_output_names_;
   std::vector<YaaaRunnablePtr> runnables_;
   std::vector<std::pair<std::string, std::string>> connections_;
   std::map<std::string, std::shared_ptr<Term>> send_conditions_;

   mutable std::map<std::pair<std::string, std::string>, bool> cache_of_senders_and_receivers_;

   void addSendCondition(const std::string& condition, const std::string& output_name);
   void addSendCondition(const std::shared_ptr<Term> condition, const std::string& output_name);
   void addSendCondition(const std::pair<std::string, std::shared_ptr<vfm::Term>> pair);

   std::vector<std::pair<YaaaRunnablePtr, std::string>> findRunnableSendingTowards(const std::string& recipient); // Empty vec if input is activity input, max one entry.
   std::vector<std::pair<YaaaRunnablePtr, std::string>> findRunnablesReceivingFrom(const std::string& sender);    // Empty vec if output is activity output.
   void collectFirstOrderSendConditionsForAllOutputs();
   bool deriveOutputConditionForSinglePortOnce(const std::string& output_port_name);
   std::string getOutputPortSendingToInputPort(const std::string& fully_qualified_input_port_name); /// The output of the previous runnable sending to this input.
};

} // vfm