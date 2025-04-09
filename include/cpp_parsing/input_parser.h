//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file

#pragma once

#include "failable.h"
#include <algorithm>
#include <iostream>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>


/// The command line parser.
class InputParser : public vfm::Failable
{
public:
   InputParser(int& argc, char** argv)
      : Failable("InputParser"),
      argc_{ argc },
      argv_{ argv }
   {
      for (int i = 1; i < argc; ++i)
      {
         tokens_.push_back(std::string(argv[i]));
      }
   }

   std::set<std::string> getCmdMultiOption(const std::string& option) const
   {
      if (flags_.count(option)) {
         return std::find(tokens_.begin(), tokens_.end(), option) == tokens_.end()
            ? std::set<std::string>{ "false" }
            : std::set<std::string>{ "true" };
      }

      std::vector<std::string>::const_iterator itr{ tokens_.begin()};
      std::set<std::string> result{};

      while (itr != tokens_.end()) {
         if (*itr == option) {
            result.insert(*(++itr));
         }
         else {
            if (!flags_.count(*itr)) {
               ++itr;
            }
         }

         itr++;
      }

      if (result.empty() && defaults_.find(option) != defaults_.end())
      {
         return { defaults_.at(option) };
      }

      return result;
   }

   std::string getCmdOption(const std::string& option) const
   {
      auto options{ getCmdMultiOption(option) };

      if (options.size() > 1) {
         addError("A single-value option occurred more than once (" + option + ", " + std::to_string(options.size()) + " times).");
         return "#INVALID";
      }
      else if (options.empty()) {
         addError("A single-value option was not found (" + option + ").");
         return "#INVALID";
      }

      return *options.begin();
   }

   bool cmdOptionExists(const std::string& option) const
   {
      return defaults_.find(option) != defaults_.end()
         || std::find(this->tokens_.begin(), this->tokens_.end(), option) != this->tokens_.end();
   }

   enum class ParameterMode {
      flag,
      option,
   };

   void addFlag(
      const std::string& name,
      const std::string& description) {
      addParameter(name, description, std::nullopt, ParameterMode::flag);
   }

   void addParameter(
      const std::string& name, 
      const std::string& description, 
      const std::optional<std::string>& default_value = std::nullopt,
      const ParameterMode par_mode = ParameterMode::option)
   {
      if (default_value != std::nullopt) {
         defaults_.insert({ name, default_value.value()});
      }

      descriptions_.insert({ name, description });
      allowed_options_.insert(name);

      if (par_mode == ParameterMode::flag) {
         markAsFlag(name);
      }
   }

   void triggerErrorIfArgumentMissing(const std::string& name) const
   {
      if (!cmdOptionExists(name))
      {
         addError("Please provide a " + getNameAndDesc(name) + " argument.");
      }
   }

   void triggerWarningIfParameterNameUnknown() const
   {
      for (int i = 1; i < argc_; i = i + 2)
      {
         std::string name = argv_[i];

         if (!allowed_options_.count(name))
         {
            addWarning("Unknown parameter name '" + name + "'. I will skip it...");
         }
      }
   }

   void triggerErrorifAnyArgumentMissing() const
   {
      for (const auto pair : descriptions_) {
         if (!flags_.count(pair.first)) {
            triggerErrorIfArgumentMissing(pair.first);
         }
      }
   }

   void printArgumentFull(const std::string& name) const
   { 
      addNote("Value of parameter " + getNameAndDesc(name) + ": '" + setToString(getCmdMultiOption(name)) + "'.");
   }

   void printArgumentsForMC() const
   {
      addNote("These are the parameters I am going to use:");
      addNote("---");

      for (const auto pair : descriptions_) {
         printArgumentFull(pair.first);
      }
      addNote("---");
   }

private:
   int& argc_;
   char** argv_{};
   std::vector<std::string> tokens_{};
   std::map<std::string, std::string> defaults_{};
   std::map<std::string, std::string> descriptions_{};
   mutable std::set<std::string> allowed_options_{};
   std::set<std::string> flags_{};

   static std::string setToString(const std::set<std::string> aSet)
   {
      if (aSet.empty()) return "{" + vfm::FAILED_COLOR + " *NO VALUE PROVIDED* " + vfm::RESET_COLOR + "}";

      std::string s{};

      for (auto const& e : aSet)
      {
         s += e;
         s += ',';
      }

      s.pop_back();

      return "{ " + s + " }";
   }

   void markAsFlag(const std::string parameter)
   {
      flags_.insert(parameter);
   }

   std::string getNameAndDesc(const std::string& name) const
   {
      return std::string("'") + name + "' (" + getDescriptionIfAny(name) + ")";
   }

   std::string getDescriptionIfAny(const std::string& name) const
   {
      return descriptions_.find(name) != descriptions_.end() ? descriptions_.at(name) : "NO-DESCRIPTION";
   }
};
