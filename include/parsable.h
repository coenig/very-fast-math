//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file
#pragma once
#include "failable.h"
#include <iostream>
#include <vector>
#include <memory>


namespace vfm {

/// \brief Father class for all classes that involve parsing a program (part).
class Parsable : public Failable
{
public:
   Parsable(const std::string& failable_name) : Failable(failable_name) {}

   /// \brief An optional preprocessing method that can be used before the actual
   /// parsing method is called.
   virtual std::string preprocessProgram(const std::string& program) const
   {
      return program;
   }

   /// \brief The main parse method. The bool return value should reflect if a parsing error occurred
   /// during the current parsing session, meaning a single call of this method. (true means OK; TODO: note
   /// that this might not have been interpreted in this was at all places in the past.)
   /// Opposed to that, the father class Failable
   /// stores any parsing error occurred in the past (after the last call of resetParseError()).
   virtual bool parseProgram(const std::string& program) = 0;

   inline virtual bool parseProgram(const std::vector<std::string>& program)
   {
      std::string prog;

      for (const auto& token : program) {
         prog += token + " ";
      }

      return parseProgram(prog);
   }
};

} // vfm
