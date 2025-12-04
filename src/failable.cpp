//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file

#include "failable.h"
#include "math_struct.h"
#include "static_helper.h"
#include <ctime>


vfm::Failable::Failable(const std::string& failable_name) : failable_name_(failable_name) {}

std::string vfm::Failable::getColorFor(ErrorLevelEnum level)
{
   if (level == ErrorLevelEnum::debug) {
      return HIGHLIGHT_COLOR;
   }
   else if (level == ErrorLevelEnum::note) {
      return BOLD_COLOR;
   }
   else if (level == ErrorLevelEnum::warning) {
      return WARNING_COLOR;
   }
   else if (level == ErrorLevelEnum::error) {
      return FAILED_COLOR;
   }
   else {
      return FAILED_COLOR;
   }
}

int vfm::Failable::hasErrorOccurred(const ErrorLevelEnum level, const bool include_children) const
{
   int res = 0;

#ifndef CUT_OUT_ALL_DEBUG_OUTPUT_FOR_RELEASE
   if (include_children) {
      for (const auto& child : children_) {
         res += child->hasErrorOccurred(level, include_children);
      }
   }

   if (errors_.count(level)) {
      res += errors_.at(level).size();
   }
#endif

   return res;
}

std::map<vfm::ErrorLevelEnum, std::vector<std::pair<std::string, std::string>>> vfm::Failable::getErrors(const bool include_children) const
{
   std::map<vfm::ErrorLevelEnum, std::vector<std::pair<std::string, std::string>>> res;
   
#ifndef CUT_OUT_ALL_DEBUG_OUTPUT_FOR_RELEASE
   for (const auto& el : errorLevelMapping) {
      res.insert({ el.first, {} });
   }

   for (const auto& pair : errors_) { // TODO: Could insert only for specific ErrorLevel for performance reasons.
      auto& element = res.at(pair.first);

      for (const auto& message : pair.second) {
         element.push_back({ failable_name_, message });
      }
   }

   if (include_children) {
      for (const auto& child : children_) {
         const auto& child_errors = child->getErrors(include_children);

         for (const auto& pair : child_errors) {
            auto& element = res.at(pair.first);
            element.insert(element.end(), pair.second.begin(), pair.second.end());
         }
      }
   }
#endif

   return res;
}

void vfm::Failable::printError(
   const std::string& error_message, 
   const ErrorLevelEnum level, 
   const bool include_preamble, 
   const std::string& delimiter, 
   const bool force_printing, 
   const std::string& force_failable_name) const
{
#ifndef CUT_OUT_ALL_DEBUG_OUTPUT_FOR_RELEASE
   if (level == ErrorLevelEnum::invalid) {
      return;
   }

   if (paused_messages_ && level != ErrorLevelEnum::error && level != ErrorLevelEnum::fatal_error) {
      paused_messages_->push_back({ error_message, level });
      return;
   }

   auto pair = errorLevelMapping.at(level);
   auto level_int = pair.first;
   auto level_str = pair.second;
   bool is_error = level_int >= printToStdErrFrom_;
   
   if (level_int < printToStdOutFrom_ && !is_error && !force_printing) {
      return;
   }

   auto& streamvec = is_error ? error_streams_with_timer_ : output_streams_with_timer_;

   auto finalize_color = [](const std::string& color, const std::ostream& stream) {
      return (&stream == &std::cout || &stream == &std::cerr) ? color : ""; // Raison: Print colors only to terminal. TODO: Is there a better way to determine that?
   };

   for (int i = 0; i < streamvec.size(); i++) {
      auto& stream = streamvec[i].second.get(); // Note that the timer is not currently used.

      std::string highlight_color = finalize_color(getColorFor(level), stream);

      std::string output = include_preamble ? // TODO: If there is more than one stream with/without color, the same string will be generated multiple times.
         finalize_color(SPECIAL_COLOR, stream)
         + (is_error ? "ERR" : "OUT")
            + finalize_color(RESET_COLOR, stream)
            + " "
            + StaticHelper::timeStamp()
            + " "
            + highlight_color
            + "<"
            + (force_failable_name.empty() ? failable_name_ : force_failable_name)
            + "> "
            + level_str
            + ": "
            + finalize_color(RESET_COLOR, stream)
         : "";
      output += (// &stream == &std::cout ? // TODO: Reactivate in future...?
         // StaticHelper::shortenInTheMiddle(error_message, 400, 0.9, finalize_color(SPECIAL_COLOR, stream) + "...output shortened in terminal; log has full output..." + finalize_color(RESET_COLOR, stream))
         // : 
         error_message) + delimiter;

      stream << output << std::flush;
   }
#endif
}

std::string vfm::Failable::getFailableName() const
{
   return failable_name_;
}

void vfm::Failable::addError(const std::wstring& error_message, const ErrorLevelEnum level, const std::string& delimiter, const bool include_preamble) const
{
#ifndef CUT_OUT_ALL_DEBUG_OUTPUT_FOR_RELEASE
   std::stringstream ss;
   ss << error_message.c_str();
   std::string str = ss.str();

   addError(str, level, delimiter, include_preamble);
#endif
}

void vfm::Failable::addError(const std::string& error_message, const ErrorLevelEnum level, const std::string& delimiter, const bool include_preamble) const
{
#ifndef CUT_OUT_ALL_DEBUG_OUTPUT_FOR_RELEASE
   if (level == ErrorLevelEnum::invalid) {
      return;
   }

   if (!errors_.count(level)) {
      resetErrors(level, false);
   }

   std::lock_guard<std::mutex> lock(*mutex_errors_);
   errors_.at(level).push_back(error_message);
   printError(error_message, level, include_preamble, delimiter);

   if (level == ErrorLevelEnum::fatal_error) {
      throwError(error_message, level);
   }
#endif
}

void vfm::Failable::resetErrors(const ErrorLevelEnum level, const bool include_children) const
{
#ifndef CUT_OUT_ALL_DEBUG_OUTPUT_FOR_RELEASE
   if (include_children) {
      for (const auto& child : children_) {
         child->resetErrors(level, include_children);
      }
   }

   std::lock_guard<std::mutex> lock(*mutex_errors_);
   errors_[level] = std::vector<std::string>();
#endif
}

void vfm::Failable::resetAllErrors(const bool include_children) const
{
#ifndef CUT_OUT_ALL_DEBUG_OUTPUT_FOR_RELEASE
   if (include_children) {
      for (const auto& child : children_) {
         child->resetAllErrors(include_children);
      }
   }

   resetErrors(ErrorLevelEnum::debug);
   resetErrors(ErrorLevelEnum::note);
   resetErrors(ErrorLevelEnum::warning);
   resetErrors(ErrorLevelEnum::error);
   resetErrors(ErrorLevelEnum::fatal_error);
#endif
}

void vfm::Failable::throwError(const std::string& message, const ErrorLevelEnum level) const
{
#ifndef CUT_OUT_ALL_DEBUG_OUTPUT_FOR_RELEASE
   auto pair = errorLevelMapping.at(level);
   auto level_int = pair.first;
   auto level_str = pair.second;

   if (level_int >= throw_from_) {
      printError("Shutting down due to " + level_str + " '" + message + "'.", ErrorLevelEnum::fatal_error, true, "\n", true);
      std::cerr << std::endl << "<TERMINATED>" << std::endl;
      assert(false && "A fatal error has occurred.");

#ifdef _WIN32
      std::cin.get(); // Prevent Windows from closing the terminal immediately after termination.
#endif
      std::exit(EXIT_FAILURE); // TODO: Add individual exit status to each failure.
   }
#endif
}

void vfm::Failable::printAndThrowErrorsIfAny(const bool print_errors, const bool throw_error, const ErrorLevelEnum from, const bool include_children) const
{
#ifndef CUT_OUT_ALL_DEBUG_OUTPUT_FOR_RELEASE
   const auto from_int = errorLevelMapping.at(from).first;
   const auto mapping = getErrors(include_children);

   for (auto pair : errorLevelMapping) {
      auto level = pair.first;
      auto level_int = pair.second.first;
      
      if (level_int == from_int) {
         auto level_str = pair.second.second;
         int num_errors = hasErrorOccurred(level, include_children);

         if (num_errors > 0) {
            std::string error_string = "There have been " + std::to_string(num_errors) + " " + level_str + "(s):";

            if (print_errors) {
               printError(error_string, level, true, "\n", true);

               for (const auto& error_message : mapping.at(level)) {
                  printError(error_message.second, level, true, "\n", true, error_message.first);
               }
            }

            if (throw_error) {
               for (const auto& error_message : mapping.at(level)) {
                  throwError(error_message.second, level);
               }
            }
         }
      }
   }
#endif
}

void vfm::Failable::setOutputLevels(const ErrorLevelEnum toStdOutFrom, const ErrorLevelEnum toStdErrFrom, const bool include_children) const
{
#ifndef CUT_OUT_ALL_DEBUG_OUTPUT_FOR_RELEASE
   printToStdOutFrom_ = errorLevelMapping.at(toStdOutFrom).first;
   printToStdErrFrom_ = errorLevelMapping.at(toStdErrFrom).first;

   if (include_children) {
      for (const auto& child : children_) {
         child->setOutputLevels(toStdOutFrom, toStdErrFrom, include_children);
      }
   }
#endif
}

vfm::ErrorLevelEnum enumFromInt(const int enum_as_int)
{
   for (const auto& el : vfm::errorLevelMapping) {
      if (el.second.first == enum_as_int) {
         return el.first;
      }
   }

   return vfm::ErrorLevelEnum::invalid;
}

vfm::ErrorLevelEnum vfm::Failable::getStdOutLevel() const
{
#ifndef CUT_OUT_ALL_DEBUG_OUTPUT_FOR_RELEASE
   return enumFromInt(printToStdOutFrom_);
#else
   return ErrorLevelEnum::invalid;
#endif
}

vfm::ErrorLevelEnum vfm::Failable::getStdErrLevel() const
{
#ifndef CUT_OUT_ALL_DEBUG_OUTPUT_FOR_RELEASE
   return enumFromInt(printToStdErrFrom_);
#else
   return ErrorLevelEnum::invalid;
#endif
}

vfm::ErrorLevelEnum vfm::Failable::getThrowFromLevel() const
{
#ifndef CUT_OUT_ALL_DEBUG_OUTPUT_FOR_RELEASE
   return enumFromInt(throw_from_);
#else
   return ErrorLevelEnum::invalid;
#endif
}

void vfm::Failable::setThrowLevel(const ErrorLevelEnum fromLevel, const bool include_children) const
{
#ifndef CUT_OUT_ALL_DEBUG_OUTPUT_FOR_RELEASE
   if (include_children) {
      for (const auto& child : children_) {
         child->setThrowLevel(fromLevel, include_children);
      }
   }

   throw_from_ = errorLevelMapping.at(fromLevel).first;
#endif
}

void vfm::Failable::addDebug(const std::string& message, const bool include_preamble, const std::string& delimiter) const
{
#ifndef CUT_OUT_ALL_DEBUG_OUTPUT_FOR_RELEASE
   addError(message, ErrorLevelEnum::debug, delimiter, include_preamble);
#endif
}

void vfm::Failable::addNote(const std::string& message, const bool include_preamble, const std::string& delimiter) const
{
#ifndef CUT_OUT_ALL_DEBUG_OUTPUT_FOR_RELEASE
   addError(message, ErrorLevelEnum::note, delimiter, include_preamble);
#endif
}

void vfm::Failable::addWarning(const std::string& message, const bool include_preamble, const std::string& delimiter) const
{
#ifndef CUT_OUT_ALL_DEBUG_OUTPUT_FOR_RELEASE
   addError(message, ErrorLevelEnum::warning, delimiter, include_preamble);
#endif
}

void vfm::Failable::addError(const std::string& message, const bool include_preamble, const std::string& delimiter) const
{
#ifndef CUT_OUT_ALL_DEBUG_OUTPUT_FOR_RELEASE
   addError(message, ErrorLevelEnum::error, delimiter, include_preamble);
   std::cin.get();
   //addFatalError("Shutting down.");
#endif
}

void vfm::Failable::addFatalError(const std::string& message, const bool include_preamble, const std::string& delimiter) const
{
#ifndef CUT_OUT_ALL_DEBUG_OUTPUT_FOR_RELEASE
   addError(message, ErrorLevelEnum::fatal_error, delimiter, include_preamble);
#endif
}

void vfm::Failable::addDebugPlain(const std::string& message, const std::string& delimiter) const
{
#ifndef CUT_OUT_ALL_DEBUG_OUTPUT_FOR_RELEASE
   addDebug(message, false, delimiter);
#endif
}

void vfm::Failable::addNotePlain(const std::string& message, const std::string& delimiter) const
{
#ifndef CUT_OUT_ALL_DEBUG_OUTPUT_FOR_RELEASE
   addNote(message, false, delimiter);
#endif
}

void vfm::Failable::addWarningPlain(const std::string& message, const std::string& delimiter) const
{
#ifndef CUT_OUT_ALL_DEBUG_OUTPUT_FOR_RELEASE
   addWarning(message, false, delimiter);
#endif
}

void vfm::Failable::addErrorPlain(const std::string& message, const std::string& delimiter) const
{
#ifndef CUT_OUT_ALL_DEBUG_OUTPUT_FOR_RELEASE
   addError(message, false, delimiter);
#endif
}

void vfm::Failable::addFatalErrorPlain(const std::string& message, const std::string& delimiter) const
{
#ifndef CUT_OUT_ALL_DEBUG_OUTPUT_FOR_RELEASE
   addFatalError(message, false, delimiter);
#endif
}

void vfm::Failable::pauseOutputOfMessages() const
{
#ifndef CUT_OUT_ALL_DEBUG_OUTPUT_FOR_RELEASE
   for (const auto child : children_) {
      child->pauseOutputOfMessages();
   }

   paused_messages_ = std::make_shared<std::vector<std::pair<std::string, ErrorLevelEnum>>>();
#endif
}

void vfm::Failable::resumeOutputOfMessages() const
{
#ifndef CUT_OUT_ALL_DEBUG_OUTPUT_FOR_RELEASE

   for (const auto child : children_) {
      child->resumeOutputOfMessages();
   }

   if (!paused_messages_) return;
   auto paused_messages = paused_messages_;
   paused_messages_ = nullptr;

   for (const auto& message : *paused_messages) {
      printError(message.first, message.second, true, "\n", false, "");
   }
#endif
}

void vfm::Failable::setFailableName(const std::string& failable_name, const std::string& propagate_fully_qualified_name_to_children_using_separator) const
{
#ifndef CUT_OUT_ALL_DEBUG_OUTPUT_FOR_RELEASE
   if (!propagate_fully_qualified_name_to_children_using_separator.empty()) {
      for (const auto& child : children_) {
         child->setFailableName(
            failable_name + propagate_fully_qualified_name_to_children_using_separator + child->failable_name_,
            propagate_fully_qualified_name_to_children_using_separator);
      }
   }

   failable_name_ = failable_name;
#endif
}

std::map<std::string, std::shared_ptr<vfm::Failable>> singleton_;

void vfm::Failable::addFailableChild(const std::shared_ptr<const Failable> child, const std::string& propagate_fully_qualified_name_to_children_using_separator) const
{
#ifndef CUT_OUT_ALL_DEBUG_OUTPUT_FOR_RELEASE
   if (child && children_.insert(child).second) {
      child->setOutputLevels(getStdOutLevel(), getStdErrLevel());
      child->setThrowLevel(getThrowFromLevel());
      child->clearAllStreams();
      
      for (const auto& stream : output_streams_with_timer_) {
         child->addOrChangeErrorOrOutputStream(stream.second, true);
      }
      for (const auto& stream : error_streams_with_timer_) {
         child->addOrChangeErrorOrOutputStream(stream.second, false);
      }

      if (!propagate_fully_qualified_name_to_children_using_separator.empty()) {
         child->setFailableName(
            failable_name_ + propagate_fully_qualified_name_to_children_using_separator + child->failable_name_,
            propagate_fully_qualified_name_to_children_using_separator);
      }
   }
   else {
      //addDebug("Failable child '" + child->failable_name_ + "' not added because it is already a child of this failable.");
   }
#endif
}

void vfm::Failable::replaceFailableChild(const std::shared_ptr<const Failable> old_child, const std::shared_ptr<const Failable> new_child, const std::string & propagate_fully_qualified_name_to_children_using_separator) const
{
#ifndef CUT_OUT_ALL_DEBUG_OUTPUT_FOR_RELEASE
   if (children_.count(old_child)) {
      children_.erase(old_child);
   }
   else {
      addError("Could not remove failable child '" + old_child->failable_name_ + "'. No such child existed in the first place.");
   }

   addFailableChild(new_child, propagate_fully_qualified_name_to_children_using_separator);
#endif
}

std::shared_ptr<vfm::Failable> vfm::Failable::getSingleton(const std::string& display_name) {
   if (!singleton_.count(display_name)) {
      singleton_.insert({ display_name, std::make_shared<Failable>(display_name) });
   }

   return singleton_.at(display_name);
}

void vfm::Failable::addOrChangeErrorOrOutputStream(std::ostream& stream, const bool is_output_stream) const
{
#ifndef CUT_OUT_ALL_DEBUG_OUTPUT_FOR_RELEASE
   auto& streamvec = is_output_stream ? output_streams_with_timer_ : error_streams_with_timer_;
   streamvec.push_back({ 0, stream });

   for (const auto& child : children_) {
      child->addOrChangeErrorOrOutputStream(stream, is_output_stream);
   }
#endif
}

void vfm::Failable::clearAllStreams() const
{
#ifndef CUT_OUT_ALL_DEBUG_OUTPUT_FOR_RELEASE
   output_streams_with_timer_.clear();
   error_streams_with_timer_.clear();

   for (const auto& child : children_) {
      child->clearAllStreams();
   }
#endif
}

void vfm::Failable::resetTime() const
{
#ifndef CUT_OUT_ALL_DEBUG_OUTPUT_FOR_RELEASE
   time_ = std::chrono::steady_clock::now();
#endif
}

long vfm::Failable::measureElapsedTime(const std::string& reason) const
{
#ifndef CUT_OUT_ALL_DEBUG_OUTPUT_FOR_RELEASE
   const auto new_time = std::chrono::steady_clock::now();
   long elapsed = std::chrono::duration_cast<std::chrono::microseconds>(new_time - time_).count();
   addNote("Time elapsed for '" + reason + "': " + std::to_string(elapsed) + " micros.");
   return elapsed;
#else
   return -1;
#endif
}
