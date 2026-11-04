//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2026 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file
#pragma once

#include <functional>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <vector>

namespace vfm {
namespace macro {

/// Lets an application embedding the script processor (e.g., XWizard) take part in the expansion of
/// scripts: methods which vfm does not know, as well as the overridden ones, are delegated to the host.
struct ScriptHost {
   using MethodCall = std::function<std::optional<std::string>(
      const std::string& body, const std::string& method_name, const std::vector<std::string>& parameters)>;

   /// Applies the method to the body. Returns std::nullopt if the host does not know the method with
   /// this number of parameters.
   MethodCall call_method{};

   /// Native methods which the host implements instead (if it knows them with the given number of
   /// parameters; otherwise the native method is used).
   std::set<std::string> overridden_methods{};

   /// If non-empty, only these native methods are available, e.g., to keep file system and process
   /// access away from scripts processed in a server.
   std::set<std::string> allowed_native_methods{};
};

/// Installs the host for all scripts processed in this process; nullptr removes it.
void setScriptHost(const std::shared_ptr<ScriptHost> host);
std::shared_ptr<ScriptHost> getScriptHost();

} // macro
} // vfm
