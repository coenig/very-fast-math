//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2026 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file
/// Script expansion server: expands vfm scripts on request of a client application (e.g., XWizard),
/// which implements the methods vfm does not know (see ScriptHost).
///
/// Protocol on stdin/stdout (binary): a message is a header line "<TYPE> <number of fields>\n", followed
/// by the fields, each as "<length in bytes>\n<bytes>\n". Everything else the process writes to stdout
/// is redirected to stderr.
///
/// The server starts with READY [version]. Requests of the client:
///   CONFIG [overridden method]...                  -> ACK
///   EXPAND script [variable name, variable value]...  -> DONE | FAILED message
///   QUIT
/// DONE carries the expanded script, the number of errors, the errors, and then the script variables after the
/// expansion as pairs of name and value.
/// While expanding, the server asks the client for the methods it does not know (or which are overridden):
///   CALL body method [parameter]...                -> RESULT value | UNKNOWN | FAIL message
/// While answering a CALL, the client may send a nested EXPAND request. It shares the script variables of the
/// enclosing expansion; the variables it presets are restored afterwards.

#include "vfmacro/script.h"
#include "vfmacro/script_host.h"
#include "failable.h"
#include <cstdio>
#include <cstdlib>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <vector>

#ifdef _WIN32
#include <fcntl.h>
#include <io.h>
#else
#include <unistd.h>
#endif

using namespace vfm;
using namespace vfm::macro;

namespace {

const std::string SERVER_VERSION{ "vfm script server 2" };

/// Native methods without access to files, processes, the network or global state.
const std::set<std::string> SAFE_NATIVE_METHODS{
   "for", "forExcept", "if", "at", "contains", "size", "seqlength", "empty", "filter", "substr", "split", "newMethod",
   "strsize", "strempty", "strNempty", "firstLetterCapital", "toUpperCamelCase", "toUpperCase", "toLowerCase",
   "removeLastFileExtension", "getLastFileExtension", "substrComplement", "startsWithUppercase", "stringStartsWith",
   "stringEndsWith", "stringContains", "shortenToMaxSize", "shortenInTheMiddle", "zeroPaddedNumStr", "removeWhiteSpace",
   "containsWhiteSpace", "isEmptyExceptWhiteSpaces", "removeMultiLineComments", "removeSingleLineComments",
   "removeComments", "removePartsOutsideOf", "removeBlankLines", "wrap", "replaceAll", "replaceAllCounting",
   "replaceAllRegex", "isParsableAsFloat", "isParsableAsInt", "isAlpha", "isAlphaNumeric", "isAlphaNumericOrUnderscore",
   "isAlphaNumericOrUnderscoreOrColonOrComma", "isValidVfmVarName", "stringConformsTo", "getFileNameFromPath",
   "getFileNameFromPathWithoutExt", "removeFileNameFromPath", "fromPascalToCamelCase", "replaceSpecialCharsHTML_G",
   "replaceMathHTML", "makeVarNameNuSMVReady", "safeString", "fromSafeString", "levensteinDistance", "equals",
   "setScriptVar", "scriptVar", "isScriptVarDeclared", "sethard", "not", "space", "listElement", "clearList", "asArray",
   "pushBack", "storeListFromSequence", "storeMapFromSequence", "keyListFromMap", "isMap", "nil", "id", "idd",
   "serialize", "serializeK2", "serializeNuXmv", "simplify", "eval", "evalSafe", "makeCachable", "makeUnCachable",
   "add", "sub", "mult", "div", "pow", "mod", "eq", "neq", "sm", "smeq", "gr", "greq",
   "AddI", "SubI", "MultI", "DivI", "PowI", "ModI", "EqI", "NeqI", "SmI", "SmeqI", "GrI", "GreqI",
   "arclengthCubicBezierFromEndPoints", "arclengthCubicBezierFromStreetTopology", "minDistConnectionForTurningRadius",
};

struct Message {
   std::string type{};
   std::vector<std::string> fields{};
};

FILE* protocol_in{ nullptr };
FILE* protocol_out{ nullptr };
std::shared_ptr<ScriptHost> host{};
std::vector<std::vector<std::string>> host_errors{}; // One entry per running (possibly nested) EXPAND request.
std::vector<std::shared_ptr<DataPack>> data_packs{}; // The data of the running (possibly nested) EXPAND requests.

bool readLine(std::string& line)
{
   line.clear();

   for (int c = std::fgetc(protocol_in); c != EOF; c = std::fgetc(protocol_in)) {
      if (c == '\n') return true;
      line += static_cast<char>(c);
   }

   return false;
}

/// @return std::nullopt if the client has closed the connection or violated the protocol.
std::optional<Message> readMessage()
{
   std::string header{};
   if (!readLine(header)) return std::nullopt;

   const size_t space{ header.find(' ') };
   Message message{ header.substr(0, space), {} };
   const int count{ space == std::string::npos ? 0 : std::atoi(header.c_str() + space + 1) };

   for (int i = 0; i < count; i++) {
      std::string length{};
      if (!readLine(length)) return std::nullopt;

      std::string field(std::strtoul(length.c_str(), nullptr, 10), '\0');
      if (!field.empty() && std::fread(&field[0], 1, field.size(), protocol_in) != field.size()) return std::nullopt;
      if (std::fgetc(protocol_in) != '\n') return std::nullopt;

      message.fields.push_back(std::move(field));
   }

   return message;
}

void writeMessage(const std::string& type, const std::vector<std::string>& fields)
{
   std::string out{ type + " " + std::to_string(fields.size()) + "\n" };

   for (const auto& field : fields) {
      out += std::to_string(field.size()) + "\n" + field + "\n";
   }

   std::fwrite(out.data(), 1, out.size(), protocol_out);
   std::fflush(protocol_out);
}

void expand(const Message& request);

std::optional<std::string> callClient(const std::string& body, const std::string& method_name, const std::vector<std::string>& parameters)
{
   std::vector<std::string> fields{ body, method_name };
   fields.insert(fields.end(), parameters.begin(), parameters.end());
   writeMessage("CALL", fields);

   for (;;) {
      const auto answer{ readMessage() };

      if (!answer) std::exit(EXIT_SUCCESS); // The client is gone.

      if (answer->type == "RESULT" && answer->fields.size() == 1) return answer->fields[0];
      if (answer->type == "UNKNOWN") return std::nullopt;

      if (answer->type == "FAIL") {
         const std::string error{ "Method '" + method_name + "' failed: " + (answer->fields.empty() ? std::string{} : answer->fields[0]) };
         if (!host_errors.empty()) host_errors.back().push_back(error);
         return "#INVALID(" + error + ")";
      }

      if (answer->type == "EXPAND") {
         expand(*answer);
         continue;
      }

      std::exit(EXIT_FAILURE); // Protocol violation.
   }
}

void expand(const Message& request)
{
   if (request.fields.empty() || request.fields.size() % 2 == 0) {
      writeMessage("FAILED", { "EXPAND expects the script, followed by pairs of variable name and value." });
      return;
   }

   const bool nested{ !data_packs.empty() };
   const auto data{ nested ? data_packs.back() : std::make_shared<DataPack>() };
   auto& variables{ data->getScriptData().list_data_ };
   std::map<std::string, std::optional<std::vector<std::string>>> preset_before{}; // Restored after a nested request.

   for (size_t i = 1; i + 1 < request.fields.size(); i += 2) {
      const std::string& name{ request.fields[i] };

      if (nested && !preset_before.count(name)) {
         preset_before[name] = variables.count(name) ? std::optional<std::vector<std::string>>{ variables.at(name) } : std::nullopt;
      }

      variables[name] = { request.fields[i + 1] };
   }

   data_packs.push_back(data);
   host_errors.emplace_back();
   bool finished{ false };

   const auto finish{ [&]() {
      if (finished) return;
      finished = true;

      for (const auto& [name, value] : preset_before) {
         if (value) variables[name] = *value;
         else variables.erase(name);
      }

      data_packs.pop_back();
      host_errors.pop_back();
   } };

   try {
      const auto script{ std::make_shared<Script>(data, nullptr) };
      const std::string expanded{ script->applyDeclarationsAndPreprocessors(request.fields[0], false) };
      const auto errors{ script->getErrors(true) };
      std::vector<std::string> error_messages{};

      for (const auto& error : errors.at(ErrorLevelEnum::error)) {
         error_messages.push_back(error.second);
      }

      error_messages.insert(error_messages.end(), host_errors.back().begin(), host_errors.back().end());
      finish();

      std::vector<std::string> fields{ expanded, std::to_string(error_messages.size()) };
      fields.insert(fields.end(), error_messages.begin(), error_messages.end());

      for (const auto& [name, values] : variables) {
         fields.push_back(name);
         fields.push_back(values.empty() ? std::string{} : values.front());
      }

      writeMessage("DONE", fields);
   }
   catch (const std::exception& e) {
      finish();
      writeMessage("FAILED", { std::string{ "Script expansion failed: " } + e.what() });
   }
   catch (...) {
      finish();
      writeMessage("FAILED", { "Script expansion failed." });
   }
}

} // namespace

int main()
{
   // The original stdout is the protocol channel; any other output to stdout (e.g., notes) goes to stderr.
#ifdef _WIN32
   _setmode(_fileno(stdin), _O_BINARY);
   const int protocol_fd{ _dup(_fileno(stdout)) };
   _setmode(protocol_fd, _O_BINARY);
   _dup2(_fileno(stderr), _fileno(stdout));
   protocol_out = _fdopen(protocol_fd, "wb");
#else
   const int protocol_fd{ dup(fileno(stdout)) };
   dup2(fileno(stderr), fileno(stdout));
   protocol_out = fdopen(protocol_fd, "wb");
#endif
   protocol_in = stdin;

   if (!protocol_out) return EXIT_FAILURE;

   Failable::setDiagnosticOutputEnabled(false);

   host = std::make_shared<ScriptHost>();
   host->call_method = callClient;
   host->allowed_native_methods = SAFE_NATIVE_METHODS;
   setScriptHost(host);

   writeMessage("READY", { SERVER_VERSION });

   for (auto message{ readMessage() }; message; message = readMessage()) {
      if (message->type == "EXPAND") {
         expand(*message);
      }
      else if (message->type == "CONFIG") {
         host->overridden_methods = std::set<std::string>(message->fields.begin(), message->fields.end());
         writeMessage("ACK", {});
      }
      else if (message->type == "QUIT") {
         break;
      }
      else {
         writeMessage("FAILED", { "Unknown request '" + message->type + "'." });
      }
   }

   return EXIT_SUCCESS;
}
