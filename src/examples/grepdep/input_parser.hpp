//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2021 Robert Bosch GmbH.
//
// The reproduction, distribution and utilization of this file as
// well as the communication of its contents to others without express
// authorization is prohibited. Offenders will be held liable for the
// payment of damages. All rights reserved in the event of the grant
// of a patent, utility model or design.
//============================================================================================================

#ifndef MIGRATION_GREPDEP_INPUT_PARSER_HPP
#define MIGRATION_GREPDEP_INPUT_PARSER_HPP

#include <algorithm>
#include <iostream>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>

// required inputs
const std::string CMD_FILES = "--files";
// optional inputs (with default values)
const std::string CMD_CACHE_FILENAME = "--cache-filename";
const std::string CMD_CACHE_MIGRATED = "--cache-migrated";
const std::string CMD_COPY = "--copy";
const std::string CMD_CREATE_PDF = "--create-pdf";
const std::string CMD_EXCLUDE_DIRS = "--exclude-dir";
const std::string CMD_EXCLUDE_PATTERNS = "--exclude-patterns";
const std::string CMD_EXCLUDE_REG = "--exclude-reg";
const std::string CMD_FOLDER_INTERMEDIATE = "--folder-intermediate";
const std::string CMD_FOLDERS_MIGRATED = "--folders-migrated";
const std::string CMD_INCLUDE_COMMAND = "--include-command";
const std::string CMD_INCLUDE_PATTERNS = "--include-patterns";
const std::string CMD_MOVE = "--move";
const std::string CMD_PATTERN_COMENTS = "--comments-pattern";
const std::string CMD_REPO_ORIGIN = "--repo-origin";
const std::string CMD_REPO_TARGET = "--repo-target";
const std::string CMD_REVERSE = "--reverse";
const std::string CMD_SHOW_MIGRATED = "--show-migrated";
const std::string CMD_SHOW_PDF = "--show-pdf";
const std::string CMD_VERBOSE = "--verbose";

const std::string HARD_CODED_EXCLUDES_REG = ",assert.h,stdio.h,stdarg.h,stddef.h,windows.h,dlfcn.h,stdint.h,\
stat.h,errno.h,string.h,stdlib.h,numeric_traits.h,functions.h,macros.h,gtest.h,ctype_inline.h,limits.h,\
math.h,cmath.h,allocator.h,alloc_traits.h,move.h,stl_pair.h,shared_ptr.h,gmock.h,stl.h,embed.h,eval.h,\
c++config.h,os_defines.h,cpu_defines.h,features.h,lfts_config.h,mfl_pub.h,define.h,norm.h,mat_pub.h";


/// The command line parser.
class InputParser
{
  public:
    InputParser(const InputParser& other)
        : InputParser(other.argc_, other.argv_)
    {
    }

    InputParser(int& argc, char** argv)
        : argc_(argc)
        , argv_(argv)
    {
        for (int i = 1; i < argc; ++i)
        {
            this->tokens_.push_back(std::string(argv[i]));
        }
    }

    const std::string& getCmdOption(const std::string& option) const
    {
        std::vector<std::string>::const_iterator itr;
        itr = std::find(this->tokens_.begin(), this->tokens_.end(), option);

        if (itr != this->tokens_.end() && ++itr != this->tokens_.end())
        {
            return *itr;
        }

        static const std::string empty_string("");

        if (defaults_.find(option) != defaults_.end())
        {
            return defaults_.at(option);
        }

        return empty_string;
    }

    bool cmdOptionExists(const std::string& option) const
    {
        return defaults_.find(option) != defaults_.end()
               || std::find(this->tokens_.begin(), this->tokens_.end(), option) != this->tokens_.end();
    }

    void addDefaultValue(const std::string& name, const std::string& default_value)
    {
        defaults_.insert({name, default_value});
    }

    void addDescription(const std::string& name, const std::string& description)
    {
        descriptions_.insert({name, description});
    }

    void triggerErrorIfArgumentMissing(const std::string& name) const
    {
        if (!cmdOptionExists(name))
        {
            std::cerr << "Please provide a " << getNameAndDesc(name) << " argument." << std::endl;
            error_ocurred_ = true;
        }
    }

    void triggerWarningIfParameterNameUnknown() const
    {
        for (int i = 1; i < argc_; i = i + 2)
        {
            std::string name = argv_[i];

            if (!allowed_options_.count(name))
            {
                std::cerr << "WARNING: Unknown parameter name '" << name << "'. I will skip it..." << std::endl;
            }
        }
    }

    void printArgumentFull(const std::string& name, const std::string& additional_values = "") const
    {
        allowed_options_.insert(name);
        triggerErrorIfArgumentMissing(name);

        std::cout << "Value of parameter " << getNameAndDesc(name) << ": '" << getCmdOption(name) << "'." << std::endl;

        if (!additional_values.empty())
        {
            std::cout << "   This argument is extended by this additional string: '" + additional_values + "'\n";
        }
    }

    /// TODO: this is a temporary hack that will be resolved ASAP after the main grepdep.cpp has been cut
    mutable bool error_ocurred_{false};

  private:
    int& argc_;
    char** argv_;
    std::vector<std::string> tokens_;
    std::map<std::string, std::string> defaults_;
    std::map<std::string, std::string> descriptions_;
    mutable std::set<std::string> allowed_options_;

    std::string getNameAndDesc(const std::string& name) const
    {
        return std::string("'") + name + "' (" + getDescriptionIfAny(name) + ")";
    }

    std::string getDescriptionIfAny(const std::string& name) const
    {
        return descriptions_.find(name) != descriptions_.end() ? descriptions_.at(name) : "NO-DESCRIPTION";
    }
};

InputParser createInputParser(int argc, char* argv[])
{
    InputParser input(argc, argv);

    input.addDefaultValue(CMD_CACHE_FILENAME, "migrated_mapping.cache");
    input.addDefaultValue(CMD_CACHE_MIGRATED, "true");
    input.addDefaultValue(CMD_COPY, "false");
    input.addDefaultValue(CMD_CREATE_PDF, "true");
    input.addDefaultValue(CMD_EXCLUDE_DIRS, "git,build,*deprecated*");
    input.addDefaultValue(CMD_EXCLUDE_PATTERNS, "");
    input.addDefaultValue(CMD_EXCLUDE_REG, "daddy,vfc,asmjit");
    input.addDefaultValue(CMD_FOLDER_INTERMEDIATE, "external/dc_temp");
    input.addDefaultValue(CMD_FOLDERS_MIGRATED, "components,staging");
    input.addDefaultValue(CMD_INCLUDE_COMMAND, "#include");
    input.addDefaultValue(CMD_INCLUDE_PATTERNS, "*.h,*.hpp,*.inl,*.cpp");
    input.addDefaultValue(CMD_MOVE, "false");
    input.addDefaultValue(CMD_PATTERN_COMENTS, "//");
    input.addDefaultValue(CMD_REPO_ORIGIN, "../pj-dc_stakeholder");
    input.addDefaultValue(CMD_REPO_TARGET, ".");
    input.addDefaultValue(CMD_REVERSE, "false");
    input.addDefaultValue(CMD_SHOW_MIGRATED, "true");
    input.addDefaultValue(CMD_SHOW_PDF, "true");
    input.addDefaultValue(CMD_VERBOSE, "false");

    input.addDescription(CMD_FILES,
                         "path to one or more comma-separated root header files to start dependency search with");
    input.addDescription(CMD_CACHE_FILENAME, "filename of cache of already migrated files");
    input.addDescription(CMD_CACHE_MIGRATED, "update cache of already migrated files");
    input.addDescription(CMD_COPY, "{true, false}; copy files to folder given by " + CMD_FOLDER_INTERMEDIATE);
    input.addDescription(CMD_CREATE_PDF, "{true, false}; create a .pdf file from .dot file");
    input.addDescription(CMD_EXCLUDE_DIRS, "directories to exclude");
    input.addDescription(CMD_EXCLUDE_PATTERNS, "filename patterns to exclude");
    input.addDescription(CMD_EXCLUDE_REG, "filename patterns to exclude (regular mode only; NO REGEX!)");
    input.addDescription(CMD_FOLDER_INTERMEDIATE, "path to pace dc_temp folder, relative to '" + CMD_REPO_TARGET + "'");
    input.addDescription(CMD_FOLDERS_MIGRATED,
                         "paths in '" + CMD_REPO_TARGET + "' to look for already migrated files, relative to '"
                             + CMD_REPO_TARGET + "'");
    input.addDescription(CMD_INCLUDE_COMMAND, "the command in the respective language for inclusion of other files");
    input.addDescription(CMD_INCLUDE_PATTERNS, "filename patterns to include");
    input.addDescription(CMD_MOVE,
                         "{true, false}; move files to folder given by " + CMD_FOLDER_INTERMEDIATE
                             + "(note that --move takes precedence over --copy)");
    input.addDescription(CMD_PATTERN_COMENTS, "string denoting a commented line (will be ignored in the search)");
    input.addDescription(CMD_REPO_ORIGIN, "path to DC folder to search in");
    input.addDescription(CMD_REPO_TARGET, "path to pace GIT repo");
    input.addDescription(CMD_REVERSE,
                         "{true, false}; creates the reverse dependency graph, who depends on me, rather than the "
                         "regular, who do I depend on");
    input.addDescription(CMD_SHOW_MIGRATED,
                         "{true,false}; when searching for dependencies, re-visit already known migrated files");
    input.addDescription(CMD_SHOW_PDF,
                         "{true, false}; show the pdf after creation; implies '" + CMD_CREATE_PDF + " true'");
    input.addDescription(CMD_VERBOSE, "{true, false}; show debug output");

    std::cout << "These are the parameters I am going to use:" << std::endl;
    std::cout << "---" << std::endl;
    input.printArgumentFull(CMD_FILES);
    input.printArgumentFull(CMD_REPO_ORIGIN);
    input.printArgumentFull(CMD_REPO_TARGET);
    input.printArgumentFull(CMD_FOLDER_INTERMEDIATE);
    input.printArgumentFull(CMD_FOLDERS_MIGRATED);
    input.printArgumentFull(CMD_COPY);
    input.printArgumentFull(CMD_CREATE_PDF);
    input.printArgumentFull(CMD_SHOW_PDF);
    input.printArgumentFull(CMD_SHOW_MIGRATED);
    input.printArgumentFull(CMD_CACHE_MIGRATED);
    input.printArgumentFull(CMD_CACHE_FILENAME);
    input.printArgumentFull(CMD_EXCLUDE_DIRS);
    input.printArgumentFull(CMD_EXCLUDE_PATTERNS);
    input.printArgumentFull(CMD_EXCLUDE_REG, HARD_CODED_EXCLUDES_REG);
    input.printArgumentFull(CMD_INCLUDE_COMMAND);
    input.printArgumentFull(CMD_INCLUDE_PATTERNS);
    input.printArgumentFull(CMD_MOVE);
    input.printArgumentFull(CMD_PATTERN_COMENTS);
    input.printArgumentFull(CMD_REVERSE);
    input.printArgumentFull(CMD_VERBOSE);
    std::cout << "---" << std::endl;

    input.triggerWarningIfParameterNameUnknown();

    return input;
}

#endif // MIGRATION_GREPDEP_INPUT_PARSER_HPP
