//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file
#pragma once

#include "term.h"
#include "failable.h"
#include <vector>
#include <string>
#include <cctype>
#include <map>
#include <set>

namespace vfm {

const std::string VFM_SEND_CONDITIONS_DENOTER_BEGIN = "// #vfmSendConditionsBegin";
const std::string VFM_SEND_CONDITIONS_DENOTER_END = "// #vfmSendConditionsEnd";
constexpr char VFM_SEND_CONDITIONS_ASSIGNMENT = '=';
constexpr char VFM_SEND_CONDITIONS_SEPARATOR = ';';

class YaaaRunnable;
using YaaaRunnablePtr = std::shared_ptr<YaaaRunnable>;

class YaaaRunnable : public Failable {
public:
   YaaaRunnable(const std::string& name);

   std::string getName() const;
   std::string getGVName() const;
   void addInput(const std::string& port_name);
   void addOutput(const std::string& port_name);
   void addInner(const std::string& port_name);
   std::set<std::string> getInput() const;
   std::set<std::string> getOutput() const;
   std::set<std::string> getInner() const;

   void addSendCondition(const std::string& condition, const std::string& output_name);
   void addSendCondition(const std::shared_ptr<Term> condition, const std::string& output_name);
   std::map<std::string, std::shared_ptr<Term>> getSendConditions() const;
   void completeSendConditionsForRunnable();

   inline void setCppFileFound() 
   {
      if (cpp_file_found_) {
         addError("More than one file has been found for this runnable.");
      }

      cpp_file_found_ = true;
   }

   inline bool isCppFileFound()
   {
      return cpp_file_found_;
   }

private:
   static int ID;
   int id_;
   std::string name_;
   std::set<std::string> inputs_;
   std::set<std::string> outputs_;
   std::set<std::string> inner_;
   std::map<std::string, std::shared_ptr<Term>> send_conditions_;
   bool cpp_file_found_ = false;
};

} // vfm