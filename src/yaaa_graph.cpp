//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file

#include "model_checking/yaaa_graph.h"
#include "json_parsing/json.hpp"
#include "static_helper.h"
#include "parser.h"
#include <fstream>
#include <iostream>
#include <stdlib.h>

#if __cplusplus >= 201703L // https://stackoverflow.com/a/51536462/7302562 and https://stackoverflow.com/a/60052191/7302562
#include <filesystem>
#endif


vfm::YaaaGraph::YaaaGraph() : Failable("YaaaGraph")
{
}

vfm::YaaaGraph::YaaaGraph(const std::string& json_file) : YaaaGraph()
{
   loadFromFile(json_file);
}

void vfm::YaaaGraph::addActivity(const vfm::YaaaActivityPtr activity)
{
   activities_.push_back(activity);
   addFailableChild(activities_[activities_.size() - 1], "");
}

vfm::YaaaActivityPtr vfm::YaaaGraph::getActivityByName(const std::string& name_raw)
{
   std::string::size_type pos = name_raw.find('.');
   std::string name = pos == std::string::npos ? name_raw : name_raw.substr(0, pos);

   for (auto& ac : activities_) {
      if (ac->getName() == name) {
         return ac;
      }
   }

   throw "No activity named '" + name + "' exists.";
}

vfm::YaaaRunnablePtr vfm::YaaaGraph::getRunnableByName(const std::string& name)
{
   for (auto& ac : activities_) {
      for (auto& ru : ac->getRunnables()) {
         if (ru->getName() == name) {
            return ru;
         }
      }
   }

   throw "No runnable named '" + name + "' exists.";
}

void vfm::YaaaGraph::addConnection(const std::string & from, const std::string & to)
{
   connections_.push_back({ from, to });
}

std::string vfm::YaaaGraph::getGraphviz()
{
   std::string s;
   int node_id = 0;

   s += "digraph G {\n";
   s += "rankdir = \"LR\";\n";

   for (auto& el : activities_) {
      std::string trigger_str = "none";

      if (!el->isGateway()) {
         if (el->getTriggerCycleTime() >= 0) {
            trigger_str = std::to_string(el->getTriggerCycleTime()) + " ms (offset: " + std::to_string(el->getTriggerOffset()) + ")";
         }
         else {
            trigger_str = "";

            for (const auto& ts : el->getDataTrigger()) {
               trigger_str += "\\n" + ts;
            }

            trigger_str += "\\nTimeout: " + std::to_string(el->getTriggerTimeout());
         }
      }

      s += "subgraph cluster_" + el->getGVName() + " {\n";
      s += "label=\"" + el->getName() 
         + "\nTrigger: "
         + trigger_str
         + "\";\n";
      s += "color=black;\n";
      s += "penwidth=3;\n";
      s += (el->isGateway() ? "fillcolor=lightgrey;\nstyle=filled;\n" : "");

      s += el->getGVName() + "_in" + " [label=\"" + el->getName() + "_in\" fillcolor=white style=filled];\n";

      for (const auto& el2 : el->getRunnables()) {
         std::string cond_node_name = "CondNode" + std::to_string(node_id++);
         s += el2->getGVName() + " [label=\"" + el2->getName() + "\" fillcolor=lightgrey style=filled];\n";

         if (!el2->getSendConditions().empty()) {
            std::string conditions;
            for (const auto& el3 : el2->getSendConditions()) {
               conditions += el3.first + " " + VFM_SEND_CONDITIONS_ASSIGNMENT + " " + el3.second->serialize() + "\\l";
            }

            s += cond_node_name + " [label=\"" + conditions + "\" shape=rect color=blue]" + ";\n";
            s += el2->getGVName() + " -> " + cond_node_name + " [arrowhead=dot color=blue];\n";
         }
      }

      std::string out_gv_name = el->getGVName() + "_out";
      s += out_gv_name + " [label=\"" + el->getName() + "_out" + "\" fillcolor=white style=filled];\n";

      auto send_conditions_ordered = el->getSendConditionsGroupedAndOrdered();
      if (!send_conditions_ordered.empty()) {
         std::string cond_node_name_ac = "CondNodeAC" + std::to_string(node_id++);
         std::string conditions = "\\l";
         
         for (const auto& el3 : send_conditions_ordered) {
            std::string ru_name;

            for (const auto& el4 : el3) {
               conditions += el4.first.first + " " + VFM_SEND_CONDITIONS_ASSIGNMENT + " " + el4.second->serialize() + "\\l";
               ru_name = el4.first.second;
            }
            conditions += "<" + ru_name + ">\\l\\l";
         }

         s += cond_node_name_ac + " [label=\"" + conditions + "\" shape=rect penwidth=3 color=blue]" + ";\n";
         s += out_gv_name + " -> " + cond_node_name_ac + " [arrowhead=dot penwidth=3 color=blue];\n";
      }

      for (const auto& el2 : el->getConnections()) {
         std::string from = el2.first;
         std::string to = el2.second;
         int pos_from = from.find(".");
         int pos_to = to.find(".");
         std::string base_name_from = pos_from >= 0 ? from.substr(0, pos_from) : from;
         std::string base_name_to = pos_to >= 0 ? to.substr(0, pos_to) : to;
         std::string rest_from = pos_from >= 0 ? from.substr(pos_from + 1) : "";
         std::string rest_to = pos_to >= 0 ? to.substr(pos_to + 1) : "";
         std::string ffrom;
         std::string tto;

         if (base_name_from == from) {
            tto = getRunnableByName(base_name_to)->getGVName();
            ffrom = el->getGVName() + "_in";
         }
         else if (base_name_to == to) {
            ffrom = getRunnableByName(base_name_from)->getGVName();
            tto = el->getGVName() + "_out";
         }
         else {
            ffrom = getRunnableByName(base_name_from)->getGVName();
            tto = getRunnableByName(base_name_to)->getGVName();
         }

         s += ffrom + " -> " + tto + " [label=\"" + rest_from + "=>" + rest_to + "\"];\n";
      }

      s += "}\n";
   }

   for (const auto& el : connections_) {
      std::string from = el.first;
      std::string to = el.second;
      int pos_from = from.find(".");
      int pos_to = to.find(".");
      std::string base_name_from = pos_from >= 0 ? from.substr(0, pos_from) : from;
      std::string base_name_to = pos_to >= 0 ? to.substr(0, pos_to) : to;
      std::string rest_from = pos_from >= 0 ? from.substr(pos_from + 1) : "";
      std::string rest_to = pos_to >= 0 ? to.substr(pos_to + 1) : "";
      std::string ffrom = getActivityByName(base_name_from)->getGVName() + "_out";
      std::string tto = getActivityByName(base_name_to)->getGVName() + "_in";

      s += ffrom + " -> " + tto + " [label=\"" + rest_from + "=>" + rest_to + "\" penwidth=3];\n";
   }

   s += "}\n";

   return s;
}

void vfm::YaaaGraph::addSendConditionsToRunnable(YaaaRunnablePtr runnable, const std::string& file_name)
{
   std::ifstream t(file_name);

   if(t.good()) {
      std::stringstream buffer;
      buffer << t.rdbuf();
      std::string contents = buffer.str();

      contents = StaticHelper::removePartsOutsideOf(contents, VFM_SEND_CONDITIONS_DENOTER_BEGIN, VFM_SEND_CONDITIONS_DENOTER_END);
      contents = StaticHelper::removeBlankLines(contents);
      contents = StaticHelper::replaceAll(contents, "//", "");
      contents = StaticHelper::removeWhiteSpace(contents);

      if (contents.empty()) {
         addWarning("No send conditions found in '" + file_name + "' for runnable '" + runnable->getName() + "'.");
      }

      for (const auto& line : StaticHelper::split(contents, VFM_SEND_CONDITIONS_SEPARATOR)) {
         auto pair = StaticHelper::split(line, VFM_SEND_CONDITIONS_ASSIGNMENT);
         runnable->addSendCondition(pair[1], pair[0]);
      }
   }
   else {
      addError("File not found: '" + file_name + "'.");
   }
}

void vfm::YaaaGraph::loadFromFile(const std::string& json_file)
{
   activities_.clear();
   connections_.clear();

   std::ifstream i(json_file);
   if (i.good()) {
      nlohmann::json j;
      i >> j;

      for (const auto& el : j.at("activity_instances").items()) {
         YaaaActivityPtr ac = std::make_shared<YaaaActivity>(el.key());
         auto stimulus = el.value().at("activity_instance_type").at("stimulus");

         for (const auto& el2 : el.value().at("activity_instance_type").at("runnables").items()) {
            YaaaRunnablePtr runnable = std::make_shared<YaaaRunnable>(el2.key());
            ac->addRunnable(runnable);
         }

         for (const auto& el2 : el.value().at("activity_instance_type").at("connections").at("inputs").items()) {
            for (const auto& el3 : el2.value().at("to_")) {
               ac->addConnection(el2.value().at("from_"), el3);
            }
         }

         for (const auto& el2 : el.value().at("activity_instance_type").at("connections").at("outputs").items()) {
            for (const auto& el3 : el2.value().at("to_")) {
               ac->addConnection(el2.value().at("from_"), el3);
            }
         }

         for (const auto& el2 : el.value().at("activity_instance_type").at("connections").at("graph").items()) {
            for (const auto& el3 : el2.value().at("to_")) {
               ac->addConnection(el2.value().at("from_"), el3);
            }
         }

         for (const auto& el2 : el.value().at("connections").items()) {
            auto from = StaticHelper::replaceAll(el2.value().at("from_").dump(), "\"", "");
            auto to = ac->getName() + "." + StaticHelper::replaceAll(el2.value().at("to_").dump(), "\"", "");
            addConnection(from, to);
         }

         if (stimulus.at("kind").dump() == "\"cyclic\"") {
            ac->setTriggerCycleTime(std::stoi(stimulus.at("cycle_ms").dump()));
            ac->setTriggerOffset(std::stoi(stimulus.at("offset_ms").dump()));
         }
         else if (stimulus.at("kind").dump() == "\"data\"") {
            ac->setTriggerTimeout(std::stoi(stimulus.at("timeout_ms").dump()));

            for (const auto& el2 : stimulus.at("trigger")) {
               ac->addTrigger(StaticHelper::replaceAll(el2.dump(), "\"", ""));
            }
         }
         else {
            addError("Only 'data' and 'cyclic' triggers allowed. Unknown trigger " + stimulus.at("kind").dump() + ".");
         }
      
         addActivity(ac);
      }

      for (const auto& el : j.at("in_gateway_instances").items()) {
         YaaaActivityPtr ac = std::make_shared<YaaaActivity>(el.key(), true);
         addActivity(ac);
      }

      for (const auto& el : j.at("out_gateway_instances").items()) {
         YaaaActivityPtr ac = std::make_shared<YaaaActivity>(el.key(), true);
         addActivity(ac);
      }
   }
   else {
      addError("Json file '" + json_file + "' could not be read.");
   }

   setOutputLevels(getStdOutLevel(), getStdErrLevel());
}

#if __cplusplus >= 201703L
bool isFileFor(const std::filesystem::path& file, const vfm::YaaaRunnablePtr ru)
{
   if (!vfm::StaticHelper::stringEndsWith(file.string(), "_runnable.cpp")) {
      return false;
   }

   std::ifstream t(file);

   if (t.good()) {
      std::stringstream buffer;
      buffer << t.rdbuf();
      std::string contents = buffer.str();
      std::string ru_header_name = ru->getName();

      ru_header_name = vfm::StaticHelper::fromPascalToCamelCase(ru_header_name);

      std::string ru_header_full1 = "#include<" + ru_header_name + ".h>";
      std::string ru_header_full2 = "#include\"" + ru_header_name + ".h\"";
      contents = vfm::StaticHelper::removeComments(contents);
      contents = vfm::StaticHelper::removeWhiteSpace(contents);
      
      if (vfm::StaticHelper::stringContains(contents, ru_header_full1) || vfm::StaticHelper::stringContains(contents, ru_header_full2)) {
         return true;
      }
   }

   return false;
}
#endif

void vfm::YaaaGraph::addSendConditions(const std::string& root_path)
{
#if __cplusplus >= 201703L
   std::vector<std::filesystem::path> files;
   for (std::filesystem::recursive_directory_iterator i(root_path), end; i != end; ++i)
      if (!std::filesystem::is_directory(i->path()))
         files.push_back(i->path());


   for (auto& ac : activities_) {
      for (auto& ru : ac->getRunnables()) {
         for (const auto& path : files) {
            if (isFileFor(path, ru)) {
               addDebug("Assuming '" + path.string() + "' to be the cpp file for runnable '" + ru->getName() + "'.");
               addSendConditionsToRunnable(ru, path.string());
               ru->setCppFileFound();
            }
         }

         if (!ru->isCppFileFound()) {
            addError("No cpp file found for runnable '" + ru->getName() + "'.");
         }

         ru->completeSendConditionsForRunnable();
      }

      ac->deriveOutputConditionsFromActivityInputs();
   }
#else
   addError("You need C++17 for the 'addSendConditions' functionality. Recursing through a dir not supported.");
#endif
}

vfm::fsm::FSMs vfm::YaaaGraph::createStateMachineForMC(const std::string& base_file_name) const
{
   fsm::FSMs fsm;
   std::string ms_name = "Main";
   std::string time_name = "time";
   std::vector<int> times;
   std::string full_callback;
   int gcd;
   int lcm;
   std::set<std::string> inputs_to_system;

   for (const auto& ac : activities_) {
      if (!ac->isGateway()) {
         times.push_back(ac->getRelevantTime());
      }
   }

   gcd = StaticHelper::greatestCommonDivisor(times);
   lcm = StaticHelper::leastCommonMultiple(times) / gcd;

   full_callback += "if (" + time_name + " < " + std::to_string(lcm - 1) + ") { @" + time_name + " = " + time_name + " + 1 } else { @" + time_name + " = 0 };\n";

   for (const auto& connection : connections_) {
      if (const_cast<YaaaGraph*>(this)->getActivityByName(connection.first)->isGateway()) {
         inputs_to_system.insert(connection.second);
      }
      else {
         full_callback += "@" + connection.second + " = " + connection.first + ";\n";
      }
   }

   for (const auto& ac : activities_) {
      if (!ac->isGateway()) {
         for (const auto& output : ac->getOutputs()) {
            const auto cond = ac->getSendCondition(output);
            const std::string& fully_qualified_name = ac->getName() + "." + output;

            cond->applyToMeAndMyChildren([&ac, &inputs_to_system](const std::shared_ptr<MathStruct> m) {
               if (m->isTermVar()) {
                  auto v = m->toVariableIfApplicable();
                  const std::string& fully_qualified_name = ac->getName() + "." + v->getVariableName();
                  v->setVariableName(fully_qualified_name);
                  inputs_to_system.insert(fully_qualified_name);
               }
            });

            int trigger_time = ac->getRelevantTime() / gcd;
            std::string trigger_time_str = std::to_string(trigger_time);
            std::string trigger_time_minus_one_str = std::to_string(trigger_time - 1);
            std::string sub_formula;
            std::string last_step_str = "  @" + fully_qualified_name + " = " + cond->serialize() + ";\n";

            if (trigger_time > 1) {
               sub_formula += "if (" + time_name + " % " + trigger_time_str + " == " + trigger_time_minus_one_str + ") {\n";
               sub_formula += last_step_str;
               sub_formula += "} else {\n";
               sub_formula += "  @" + fully_qualified_name + " = ndet(" + fully_qualified_name + ", " + cond->serialize() + ");\n";
               sub_formula += "};\n";
            }
            else {
               sub_formula += last_step_str;
            }

            //full_callback += MathStruct::simplify(sub_formula, fsm.getParser())->serialize() + ";\n";
            full_callback += sub_formula;
         }
      }
   }

   for (const auto& input_to_system : inputs_to_system) {
      fsm.getData()->addSingleValIfUndeclared(input_to_system);
      full_callback += "@" + input_to_system + " = TRUE;\n";
   }

   //fsm.addSpecialCaseForNuSMV({ time_name, "0.." + std::to_string(lcm) + "" }); // TODO: This method is not available anymore.
   fsm.setInitialStateName(ms_name);
   fsm.addInternalActionToState(ms_name, full_callback);
   fsm.encapsulateLongCallbacksIntoFunctions(0);
   fsm.createGraficOfCurrentGraph(base_file_name + "_fsm.dot", true, "pdf");

   return fsm;
}

//void vfm::YaaaGraph::setOutputLevels(const ErrorLevelEnum toStdOutFrom, const ErrorLevelEnum toStdErrFrom)
//{
//   Failable::setOutputLevels(toStdOutFrom, toStdErrFrom);
//
//   for (auto& ac : activities_) {
//      ac.setOutputLevels(toStdOutFrom, toStdErrFrom);
//
//      for (auto& ru : ac.getRunnables()) {
//         ru.setOutputLevels(toStdOutFrom, toStdErrFrom);
//      }
//   }
//}

void vfm::YaaaGraph::createGraphvizDot(const std::string& base_filename, const std::string& format)
{
   std::string graph = getGraphviz();
   std::ofstream file;
   std::string error_msg = "digraph G {A [label\"Malformed FSM\"];}";

   file.open(base_filename);

   if (!graph.empty()) {
      file << graph;
   }
   else {
      file << error_msg;
   }

   file.close();
   addNote("Created graphviz in '" + base_filename + "'.");

   if (!format.empty()) {
      std::string location = base_filename + "." + format;
      std::string command = DOT_COMMAND + " -T" + format + " " + base_filename + " > " + location;
      if (system(command.c_str()) == 0) {
         addNote("Created '" + format + "' in '" + location + "'.");
      }
      else {
         addError("Something went wrong while creating '" + format + "' in '" + location + "'.");
      }
   }
}

char* result_ptr_to_external_;

extern "C" void freeModelCheckerReturnValue()
{
   delete[](result_ptr_to_external_);
}

extern "C" const char* yaaaModelCheckerWrapper(const char* yaaa_json_filename, const char* root_path, const bool debug)
{
   std::string json_file(yaaa_json_filename);
   std::string root(root_path);

   vfm::YaaaGraph g{};
   
   if (debug) {
      g.addNote("Entering debug mode.");
      g.setOutputLevels(vfm::ErrorLevelEnum::debug, vfm::ErrorLevelEnum::error);
   }
   else {
      g.setOutputLevels(vfm::ErrorLevelEnum::note, vfm::ErrorLevelEnum::error);
   }

   g.loadFromFile(json_file);
   g.addSendConditions(root);
   g.createGraphvizDot(json_file + ".dot");

   g.addError("TODO: The serialization towards nuxmv is not available. (If you're looking for the three classic translation methods, they're in 'cool_code_from_cpp_parsing_2nd_workflow.cpp'.)");

   std::string result_str =
      "#SerializationError";
      //g.createStateMachineForMC(json_file).serializeNuSMV(
      //vfm::fsm::MCTranslationTypeEnum::atomize_states_assuming_independent_variables, { { vfm::_true() }, { vfm::_true() } }).c_str();

   result_ptr_to_external_ = new char[result_str.size() + 1];
   strcpy(result_ptr_to_external_, result_str.c_str());

   return result_ptr_to_external_;
}
