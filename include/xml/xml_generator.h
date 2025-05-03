//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2025 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file
#pragma once
#include "simplification/code_block.h"
#include "model_checking/mc_types.h"

namespace vfm::xml {

class CodeXML : public code_block::CodeBlockCustom
{
public:
   CodeXML(const std::string& tag);                                                                // "</tag>" // End tag.
   CodeXML(const std::string& tag, const VarVals& pars);                                           // "<tag par0="val0" par1="val1" ... />"
   CodeXML(const std::string& tag, const VarVals& pars, const std::string& content);               // "<tag par0="val0" par1="val1" ... > content </tag>"
   CodeXML(const std::string& tag, const VarVals& pars, const std::shared_ptr<CodeXML> content);   // "<tag par0="val0" par1="val1" ... > *xml sub-content* </tag>"

   std::string serializeSingleLine() const override;
   std::string serializeSingleLine(const int indent, const std::string& formula_name) const override;

   static std::shared_ptr<CodeXML> beginXML(const VarVals& pars = { { "version", "1.0" } });
   static inline std::shared_ptr<CodeXML> retrieveEndTag(const std::string& tag) { return std::make_shared<CodeXML>(tag); }
   static inline std::shared_ptr<CodeXML> retrieveParsOnlyElement(const std::string& tag, const VarVals& pars) { return std::make_shared<CodeXML>(tag, pars); }
   static inline std::shared_ptr<CodeXML> retrieveElementWithSimpleContent(const std::string& tag, const VarVals& pars, const std::string& content) { return std::make_shared<CodeXML>(tag, pars, content); }
   static inline std::shared_ptr<CodeXML> retrieveElementWithXMLContent(const std::string& tag, const VarVals& pars, const std::shared_ptr<CodeXML> content) { return std::make_shared<CodeXML>(tag, pars, content); }

private:
   std::string tag_{};
   VarVals pars_{};
   std::shared_ptr<CodeXML> complex_content_{ nullptr };
   std::string simple_content{};
};

} // vfm::xml
