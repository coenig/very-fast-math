//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2025 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file
#pragma once

#include "failable.h"
#include <filesystem>

namespace vfm
{
namespace mc{

class McWorkflow : public Failable
{
public:
   McWorkflow();

   std::vector<std::string> McWorkflow::runMCJobs(const std::filesystem::path& working_dir);

private:
};

} // mc
} // vfm