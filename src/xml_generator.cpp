#include "xml/xml_generator.h"


using namespace vfm;
using namespace xml;


std::string convertParsToString(const vfm::VarVals& pars)
{
   std::string res{};

   for (const auto& par : pars) {
      res += " " + par.first + "=\"" + par.second + "\"";
   }

   return res;
}

vfm::xml::CodeXML::CodeXML(const std::string& tag, const VarVals& pars)
   : CodeBlockCustom("<" + tag + convertParsToString(pars) + " />",
      "", "<!--", "-->", "#ERROR-NO-INLINE-COMMENTS-IN-XML"), tag_{ tag }, pars_{ pars } {}

vfm::xml::CodeXML::CodeXML(const std::string& tag, const vfm::VarVals& pars, const std::string& content)
   : CodeBlockCustom("<" + tag + convertParsToString(pars) + ">" + content + "</" + tag + ">",
      "", "<!--", "-->", "#ERROR-NO-INLINE-COMMENTS-IN-XML"), tag_{ tag }, pars_{ pars }, simple_content{ content } {}

vfm::xml::CodeXML::CodeXML(const std::string& tag, const VarVals& pars, const std::shared_ptr<CodeXML> content)
   : CodeBlockCustom("<" + tag + convertParsToString(pars) + ">",
      "", "<!--", "-->", "#ERROR-NO-INLINE-COMMENTS-IN-XML"), tag_{ tag }, pars_{ pars }, complex_content_{ content } {}

std::string vfm::xml::CodeXML::serializeSingleLine() const
{
   return serializeSingleLine(0, "formula");
}

std::string vfm::xml::CodeXML::serializeSingleLine(const int indent, const std::string& formula_name) const
{
   if (tag_ == EMPTY_XML_DENOTER) return "";

   std::string s = indentation(indent) + getContent() + "\n";

   if (complex_content_) {
      s += complex_content_->serializeBlock(indent + 3, formula_name, MarkerMode::never);
      s += indentation(indent) + "</" + tag_ + ">\n";
   }

   return s;
}

std::shared_ptr<CodeXML> vfm::xml::CodeXML::beginXML(const VarVals& pars)
{
   auto res = std::make_shared<CodeXML>("MALFORMED", pars);

   res->setContent("<?xml" + convertParsToString(pars) + "?>");

   return res;
}

std::shared_ptr<CodeXML> vfm::xml::CodeXML::emptyXML()
{
   auto res = std::make_shared<CodeXML>(EMPTY_XML_DENOTER, VarVals{});
   res->tag_ = EMPTY_XML_DENOTER;
   return res;
}
