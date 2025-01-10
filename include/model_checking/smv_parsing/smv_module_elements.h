//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file
#pragma once

#include "failable.h"
#include <variant>
#include <string>

namespace vfm {
namespace mc {
namespace smv {

const std::string NAME_NULL_ELEMENT   = "";
const std::string NAME_VAR            = "VAR";            // var_declaration
const std::string NAME_IVAR           = "IVAR";           // ivar_declaration
const std::string NAME_FROZENVAR      = "FROZENVAR";      // frozenvar_declaration
const std::string NAME_DEFINE         = "DEFINE";         // define_declaration
const std::string NAME_CONSTANTS      = "CONSTANTS";      // constants_declaration
const std::string NAME_ASSIGN         = "ASSIGN";         // assign_constraint
const std::string NAME_TRANS          = "TRANS";          // trans_constraint
const std::string NAME_INIT           = "INIT";           // init_constraint
const std::string NAME_INVAR          = "INVAR";          // invar_constraint
const std::string NAME_FAIRNESS       = "FAIRNESS";       // fairness_constraint
const std::string NAME_JUSTICE        = "JUSTICE";        // fairness_constraint
const std::string NAME_COMPASSION     = "COMPASSION";     // fairness_constraint
const std::string NAME_CTLSPEC        = "CTLSPEC";        // ctl_specification
const std::string NAME_SPEC           = "SPEC";           // ctl_specification
const std::string NAME_CTLSPEC_NAME   = "CTLSPEC NAME";   // ctl_specification
const std::string NAME_SPEC_NAME      = "SPEC NAME";      // ctl_specification
const std::string NAME_INVARSPEC      = "INVARSPEC";      // invar_specification
const std::string NAME_INVARSPEC_NAME = "INVARSPEC NAME"; // invar_specification
const std::string NAME_LTLSPEC        = "LTLSPEC";        // ltl_specification
const std::string NAME_LTLSPEC_NAME   = "LTLSPEC NAME";   // ltl_specification
const std::string NAME_PSLSPEC        = "PSLSPEC";        // _declaration
const std::string NAME_PSLSPEC_NAME   = "PSLSPEC NAME";   // pslspec_specification
const std::string NAME_COMPUTE        = "COMPUTE";        // compute_specification
const std::string NAME_COMPUTE_NAME   = "COMPUTE NAME";   // compute_specification
const std::string NAME_PARSYNTH       = "PARSYNTH";       // parameter_synth_problem
const std::string NAME_ISA            = "ISA";            // isa_declaration
const std::string NAME_PRED           = "PRED";           // pred_declaration
const std::string NAME_MIRROR         = "MIRROR";         // mirror_declaration

class ModuleElementBase : public Failable {
public:
   std::string getModuleElementName() const;
   std::string getRawCode() const;
   void setCode(const std::string& to);
   void appendCodeLine(const std::string& additional_code);
   std::string serialize() const;

protected:
   ModuleElementBase(const std::string& name);

private:
   std::string raw_code_{};
};

class NullElement : public ModuleElementBase { public: NullElement() : ModuleElementBase(NAME_NULL_ELEMENT) {};};
class VarSection : public ModuleElementBase { public: VarSection() : ModuleElementBase(NAME_VAR) {};};
class IVarSection : public ModuleElementBase { public: IVarSection() : ModuleElementBase(NAME_IVAR) {};};
class FrozenVarSection : public ModuleElementBase { public: FrozenVarSection() : ModuleElementBase(NAME_FROZENVAR) {};};
class DefineSection : public ModuleElementBase { public: DefineSection() : ModuleElementBase(NAME_DEFINE) {};};
class ConstantsSection : public ModuleElementBase { public: ConstantsSection() : ModuleElementBase(NAME_CONSTANTS) {};};
class AssignSection : public ModuleElementBase { public: AssignSection() : ModuleElementBase(NAME_ASSIGN) {};};
class TransSection : public ModuleElementBase { public: TransSection() : ModuleElementBase(NAME_TRANS) {};};
class InitSection : public ModuleElementBase { public: InitSection() : ModuleElementBase(NAME_INIT) {};};
class InvarSection : public ModuleElementBase { public: InvarSection() : ModuleElementBase(NAME_INVAR) {};};
class FairnessSection : public ModuleElementBase { public: FairnessSection() : ModuleElementBase(NAME_FAIRNESS) {};};
class JusticeSection : public ModuleElementBase { public: JusticeSection() : ModuleElementBase(NAME_JUSTICE) {};};
class CompassionSection : public ModuleElementBase { public: CompassionSection() : ModuleElementBase(NAME_COMPASSION) {};};
class CtlspecSection : public ModuleElementBase { public: CtlspecSection() : ModuleElementBase(NAME_CTLSPEC) {};};
class SpecSection : public ModuleElementBase { public: SpecSection() : ModuleElementBase(NAME_SPEC) {};};
class CtlspecNameSection : public ModuleElementBase { public: CtlspecNameSection() : ModuleElementBase(NAME_CTLSPEC_NAME) {};};
class SpecNameSection : public ModuleElementBase { public: SpecNameSection() : ModuleElementBase(NAME_SPEC_NAME) {};};
class InvarspecSection : public ModuleElementBase { public: InvarspecSection() : ModuleElementBase(NAME_INVARSPEC) {};};
class InvarspecNameSection : public ModuleElementBase { public: InvarspecNameSection() : ModuleElementBase(NAME_INVARSPEC_NAME) {};};
class LtlspecSection : public ModuleElementBase { public: LtlspecSection() : ModuleElementBase(NAME_LTLSPEC) {};};
class LtlspecNameSection : public ModuleElementBase { public: LtlspecNameSection() : ModuleElementBase(NAME_LTLSPEC_NAME) {};};
class PslspecSection : public ModuleElementBase { public: PslspecSection() : ModuleElementBase(NAME_PSLSPEC) {};};
class PslspecNameSection : public ModuleElementBase { public: PslspecNameSection() : ModuleElementBase(NAME_PSLSPEC_NAME) {};};
class ComputeSection : public ModuleElementBase { public: ComputeSection() : ModuleElementBase(NAME_COMPUTE) {};};
class ComputeNameSection : public ModuleElementBase { public: ComputeNameSection() : ModuleElementBase(NAME_COMPUTE_NAME) {};};
class ParsynthSection : public ModuleElementBase { public: ParsynthSection() : ModuleElementBase(NAME_PARSYNTH) {};};
class IsaSection : public ModuleElementBase { public: IsaSection() : ModuleElementBase(NAME_ISA) {};};
class PredSection : public ModuleElementBase { public: PredSection() : ModuleElementBase(NAME_PRED) {};};
class MirrorSection : public ModuleElementBase { public: MirrorSection() : ModuleElementBase(NAME_MIRROR) {};};

using ModuleElement = std::variant<
   NullElement, 
   VarSection,
   IVarSection,
   FrozenVarSection,
   DefineSection,
   ConstantsSection,
   AssignSection,
   TransSection,
   InitSection,
   InvarSection,
   FairnessSection,
   JusticeSection,
   CompassionSection,
   CtlspecSection,
   SpecSection,
   CtlspecNameSection,
   SpecNameSection,
   InvarspecSection,
   InvarspecNameSection,
   LtlspecSection,
   LtlspecNameSection,
   PslspecSection,
   PslspecNameSection,
   ComputeSection,
   ComputeNameSection,
   ParsynthSection,
   IsaSection,
   PredSection,
   MirrorSection>;

using ModuleElements = std::vector<ModuleElement>;

} // smv
} // mc
} // vfm
