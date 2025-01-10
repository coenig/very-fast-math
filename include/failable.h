//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file
#pragma once
#include <iostream>
#include <vector>
#include <set>
#include <memory>
#include <map>
#include <limits>
#include <functional>
#include <chrono>
#include <mutex>


namespace vfm {

enum class ErrorLevelEnum {
   debug = -1,
   note = 0,
   warning = 1,
   error = 2,
   fatal_error = 3, // This type is thrown even if not requested.
   invalid = 4,     // This type is returned if an invalid conversion from int occurs.
};

const std::map<ErrorLevelEnum, std::pair<int, std::string>> errorLevelMapping
{
   {ErrorLevelEnum::debug, {-1, "DEBUG"}},
   {ErrorLevelEnum::note, {0, "NOTE"}},
   {ErrorLevelEnum::warning, {1, "WARNING"}},
   {ErrorLevelEnum::error, {2, "ERROR"}},
   {ErrorLevelEnum::fatal_error, {3, "FATAL_ERROR"}},
   {ErrorLevelEnum::invalid, {4, "#INVALID#"}},
};

const std::string GLOBAL_FAILABLE_NAME = "#GLOBAL";

/// \brief Provides logger functionality.
/// TODO: Not all parts of this class are thread-safe, yet. The most critical member is errors_, though, which
/// IS thread-safe. It is written to on any logging request, and thus likely to cause a collision between two threads.
class Failable
{
public:
   Failable(const std::string& failable_name);

   int hasErrorOccurred(const ErrorLevelEnum level = ErrorLevelEnum::error, const bool include_children = true) const;
   std::map<ErrorLevelEnum, std::vector<std::pair<std::string, std::string>>> getErrors(const bool include_children = true) const;
   void addError(const std::string& error_message, const ErrorLevelEnum level, const std::string& delimiter, const bool include_preamble = true) const;
   void addError(const std::wstring& error_message, const ErrorLevelEnum level, const std::string& delimiter = "\n", const bool include_preamble = true) const;
   void resetErrors(const ErrorLevelEnum level, const bool include_children = true) const;
   void resetAllErrors(const bool include_children = true) const;
   void printAndThrowErrorsIfAny(const bool print_errors = true, const bool throw_error = true, const ErrorLevelEnum from = ErrorLevelEnum::error, const bool include_children = true) const;
   void setOutputLevels(const ErrorLevelEnum toStdOutFrom = ErrorLevelEnum::note, const ErrorLevelEnum toStdErrFrom = ErrorLevelEnum::error, const bool include_children = true) const;

   ErrorLevelEnum getStdOutLevel() const;
   ErrorLevelEnum getStdErrLevel() const;
   ErrorLevelEnum getThrowFromLevel() const;
   void addDebug(const std::string& message, const bool include_preamble = true, const std::string& delimiter = "\n") const;
   void addDebugPlain(const std::string& message, const std::string& delimiter = "\n") const;
   void addNote(const std::string& message, const bool include_preamble = true, const std::string& delimiter = "\n") const;
   void addNotePlain(const std::string& message, const std::string& delimiter = "\n") const;
   void addWarning(const std::string& message, const bool include_preamble = true, const std::string& delimiter = "\n") const;
   void addWarningPlain(const std::string& message, const std::string& delimiter = "\n") const;
   void addError(const std::string& message, const bool include_preamble = true, const std::string& delimiter = "\n") const;
   void addErrorPlain(const std::string& message, const std::string& delimiter = "\n") const;
   void addFatalError(const std::string& message, const bool include_preamble = true, const std::string& delimiter = "\n") const;
   void addFatalErrorPlain(const std::string& message, const std::string& delimiter = "\n") const;

   void pauseOutputOfMessages() const;
   void resumeOutputOfMessages() const;

   /// Sets from which level on an error is thrown at call of
   /// printAndThrowErrorsIfAny. (Regardless, FATAL_ERROR will always exit
   /// on first occurrence.
   void setThrowLevel(const ErrorLevelEnum fromLevel, const bool include_children = true) const;

   /// Set propagate_fully_qualified_name_to_children_using_separator to "" to prevent changing the
   /// name in the children.
   void setFailableName(const std::string& failable_name, const std::string& propagate_fully_qualified_name_to_children_using_separator = ".") const;

   /// Adds a reference to child as an subordinate failable. This leads to:
   /// - child's settings for printing being adapted when the parent is changed.
   /// - child's error status being reported when the parents error status is requested.
   void addFailableChild(const std::shared_ptr<const Failable> child, const std::string& propagate_fully_qualified_name_to_children_using_separator = ".") const;

   void replaceFailableChild(const std::shared_ptr<const Failable> old_child, const std::shared_ptr<const Failable> new_child, const std::string& propagate_fully_qualified_name_to_children_using_separator = ".") const;

   static std::shared_ptr<Failable> getSingleton(const std::string& display_name = GLOBAL_FAILABLE_NAME);

   void addOrChangeErrorOrOutputStream(std::ostream& stream, const bool is_output_stream) const;
   void clearAllStreams() const;

   std::string getFailableName() const;

   void resetTime() const;
   long measureElapsedTime(const std::string& reason = "GENERAL") const;

protected:
   void printError(
      const std::string& error_message, 
      const ErrorLevelEnum level, 
      const bool include_preamble, 
      const std::string& delimiter, 
      const bool force_printing = false, 
      const std::string& force_failable_name = "") const;

private:
   static std::string getColorFor(ErrorLevelEnum level);
   
   void throwError(const std::string& message, const ErrorLevelEnum level) const;
   
   mutable std::string failable_name_{};

#ifndef CUT_OUT_ALL_DEBUG_OUTPUT_FOR_RELEASE
   /// The reasoning for the "mutable" is that changing the error state shouldn't affect
   /// the const-ness of the underlying object since it is often desired to change the
   /// error state (for example erase earlier errors) without considering this a change
   /// to the content of the corresponding object.
   mutable std::map<ErrorLevelEnum, std::vector<std::string>> errors_{};
   std::shared_ptr<std::mutex> mutex_errors_{ std::make_shared<std::mutex>() };

   mutable std::vector<std::pair<long, std::reference_wrapper<std::ostream>>> output_streams_with_timer_{ { 0, std::cout } };
   mutable std::vector<std::pair<long, std::reference_wrapper<std::ostream>>> error_streams_with_timer_{ { 0, std::cerr } };

   mutable int printToStdOutFrom_{ 0 };
   mutable int printToStdErrFrom_{ 2 };
   mutable int throw_from_{ 2 };
   mutable std::set<std::shared_ptr<const Failable>> children_{};
   mutable std::chrono::steady_clock::time_point time_{};

   mutable std::shared_ptr<std::vector<std::pair<std::string, ErrorLevelEnum>>> paused_messages_{ nullptr };
#endif
};

} // vfm
