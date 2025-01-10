//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2019 Robert Bosch GmbH.
//
// The reproduction, distribution and utilization of this file as
// well as the communication of its contents to others without express
// authorization is prohibited. Offenders will be held liable for the
// payment of damages. All rights reserved in the event of the grant
// of a patent, utility model or design.
//============================================================================================================

#ifndef FCTENUMDEFINITIONS_HPP_INCLUDED
#define FCTENUMDEFINITIONS_HPP_INCLUDED

#include "enum_wrapper.h"
#include "geometry/images.h"
#include "fsm.h"
#include <memory>
#include <map>

using namespace vfm;
using namespace vfm::fsm;

namespace Fct
{
// ENUMS definition stateMachine 
// enum class HmiCommandEnum {deactivate, activate};                                                         // used for fct HMI
enum class LaneChangeDirectionEnum {left, right, undefined};                                                     // used for fct state machine
enum class HmiSpeechMessageIdEnum {none, notification, warning, safestop};

// ENUMS definition Driver Monitor
enum class fovioDELEnum {FOVIO_DEL_UNKNOWN, FOVIO_DEL_ATTENTIVE, FOVIO_DEL_INATTENTIVE, FOVIO_DEL_DISTRACTED, FOVIO_DEL_DISENGAGED, FOVIO_DEL_INVALID};     /*!< There is no result yet, the feature has not yet been executed or completed execution */
enum class fovioDDetectModeEnum {FOVIO_DDETECT_MODE_INVALID, FOVIO_DDETECT_MODE_INACTIVE, FOVIO_DDETECT_MODE_AUTO};                                         /*!< No face tracking information available. */
enum class fovioDDetectStatusEnum {FOVIO_DDETECT_STATUS_INVALID, FOVIO_DDETECT_IDLE, FOVIO_DDETECT_DETECTING, FOVIO_DDETECT_MONITORING};                    /*!< No real driver in the seat. Face detected may be of a printed face or other spoofing attacks. */
enum class fovioDDetectResultEnum {FOVIO_DDETECT_RESULT_INVALID, FOVIO_DDETECT_NO_DRIVER_FOUND, FOVIO_DDETECT_FAKE_DRIVER_FOUND, FOVIO_DDETECT_REAL_DRIVER_FOUND, FOVIO_DDETECT_REAL_DRIVER_FAKE_EYES_FOUND};  /*!< Human driver detected*/         

enum class FctStateEnum {initState, passiveState, activeState, warningState, safeStopState, laneChangeState, undefinedState};
const std::map<FctStateEnum, std::string> stateEnumMap
{
    { FctStateEnum::initState, "initState" },
    { FctStateEnum::passiveState, "passiveState" },
    { FctStateEnum::activeState, "activeState" },
    { FctStateEnum::warningState, "warningState" },
    { FctStateEnum::safeStopState, "safeStopState" },
    { FctStateEnum::laneChangeState, "laneChangeState" },
    { FctStateEnum::undefinedState, "undefinedState" },
};

class ExternalState : public vfm::fsm::EnumWrapper<FctStateEnum>
{
public:
   explicit ExternalState(const FctStateEnum& enum_val) : EnumWrapper("externalState", stateEnumMap, enum_val) {}
   explicit ExternalState(const int enum_val_as_num) : EnumWrapper("externalState", stateEnumMap, enum_val_as_num) {}
   explicit ExternalState() : EnumWrapper("externalState", stateEnumMap, FctStateEnum::undefinedState) {}
};

enum class StateTransitionEnum {noTransition, newStateEntry};
const std::map<StateTransitionEnum, std::string> stateTransitionEnumMap
{
    { StateTransitionEnum::noTransition, "noTransition" },
    { StateTransitionEnum::newStateEntry, "newStateEntry" },
};

class StateTransition : public vfm::fsm::EnumWrapper<StateTransitionEnum>
{
public:
   explicit StateTransition(const StateTransitionEnum& enum_val) : EnumWrapper("stateTransition", stateTransitionEnumMap, enum_val) {}
   explicit StateTransition(const int enum_val_as_num) : EnumWrapper("stateTransition", stateTransitionEnumMap, enum_val_as_num) {}
   explicit StateTransition() : EnumWrapper("stateTransition", stateTransitionEnumMap, StateTransitionEnum::noTransition) {}
};

enum class IndicatorLeverStateEnum {off, left, right};
const std::map<IndicatorLeverStateEnum, std::string> indicatorLeverStateEnumMap
{
    { IndicatorLeverStateEnum::off, "off" },
    { IndicatorLeverStateEnum::left, "left" },
    { IndicatorLeverStateEnum::right, "right" },
};

class IndicatorLeverState : public vfm::fsm::EnumWrapper<IndicatorLeverStateEnum>
{
public:
   explicit IndicatorLeverState(const IndicatorLeverStateEnum& enum_val) : EnumWrapper("indicatorLeverState", indicatorLeverStateEnumMap, enum_val) {}
   explicit IndicatorLeverState(const int enum_val_as_num) : EnumWrapper("indicatorLeverState", indicatorLeverStateEnumMap, enum_val_as_num) {}
   explicit IndicatorLeverState() : EnumWrapper("indicatorLeverState", indicatorLeverStateEnumMap, IndicatorLeverStateEnum::off) {}
};

enum class DmsGazeZoneEnum {
   undefined = 0, 
   onRoadTargetZone = 1, 
   hmiDisplayTablet = 13, 
   centralRearMirror = 17, 
   leftSideMirror = 18, 
   rightSideMirror = 19, 
   driversLab = 20, 
   centralInfotainment = 1010
};

const std::map<DmsGazeZoneEnum, std::string> dmsGazeZoneEnumMap
{
    { DmsGazeZoneEnum::undefined, "undefined" },
    { DmsGazeZoneEnum::onRoadTargetZone, "onRoadTargetZone" },
    { DmsGazeZoneEnum::hmiDisplayTablet, "hmiDisplayTablet" },
    { DmsGazeZoneEnum::centralRearMirror, "centralRearMirror" },
    { DmsGazeZoneEnum::leftSideMirror, "leftSideMirror" },
    { DmsGazeZoneEnum::rightSideMirror, "rightSideMirror" },
    { DmsGazeZoneEnum::driversLab, "driversLab" },
    { DmsGazeZoneEnum::centralInfotainment, "centralInfotainment" }
};

class DmsGazeZone : public vfm::fsm::EnumWrapper<DmsGazeZoneEnum>
{
public:
   explicit DmsGazeZone(const DmsGazeZoneEnum& enum_val) : EnumWrapper("dmsGazeZone", dmsGazeZoneEnumMap, enum_val) {}
   explicit DmsGazeZone(const int enum_val_as_num) : EnumWrapper("dmsGazeZone", dmsGazeZoneEnumMap, enum_val_as_num) {}
   explicit DmsGazeZone() : EnumWrapper("dmsGazeZone", dmsGazeZoneEnumMap, DmsGazeZoneEnum::undefined) {}
};

enum class DriverInterruptEnum {noInterrupt, interruptByBraking, interruptBySteering, interruptByAccelerating};
const std::map<DriverInterruptEnum, std::string> driverInterrupt_enum_map
{
    { DriverInterruptEnum::noInterrupt, "noInterrupt" },
    { DriverInterruptEnum::interruptByBraking, "interruptByBraking" },
    { DriverInterruptEnum::interruptBySteering, "interruptBySteering" },
    { DriverInterruptEnum::interruptByAccelerating, "interruptByAccelerating" }
};

class DriverInterrupt : public vfm::fsm::EnumWrapper<DriverInterruptEnum>
{
public:
   explicit DriverInterrupt(const DriverInterruptEnum& enum_val) : EnumWrapper("driverInterrupt", driverInterrupt_enum_map, enum_val) {}
   explicit DriverInterrupt(const int enum_val_as_num) : EnumWrapper("driverInterrupt", driverInterrupt_enum_map, enum_val_as_num) {}
   explicit DriverInterrupt() : EnumWrapper("driverInterrupt", driverInterrupt_enum_map, DriverInterruptEnum::noInterrupt) {}
};

enum class EscalationLevelEnum {Level0, Level1, Level2, Level3};
const std::map<EscalationLevelEnum, std::string> escalation_enum_map
{
    { EscalationLevelEnum::Level0, "zero" },
    { EscalationLevelEnum::Level1, "one" },
    { EscalationLevelEnum::Level2, "two" },
    { EscalationLevelEnum::Level3, "three" }
};

class EscalationLevel : public vfm::fsm::EnumWrapper<EscalationLevelEnum>
{
public:
   explicit EscalationLevel(const EscalationLevelEnum& enum_val) : EnumWrapper("escalationLevel", escalation_enum_map, enum_val) {}
   explicit EscalationLevel(const int enum_val_as_num) : EnumWrapper("escalationLevel", escalation_enum_map, enum_val_as_num) {}
   explicit EscalationLevel() : EnumWrapper("escalationLevel", escalation_enum_map, EscalationLevelEnum::Level0) {}
};

// ENUMS definition FCT-HMI and state machine
enum class HmiCommandEnum 
{
    deactivate,
    activate,
    invalid_value
}; 

const std::map<HmiCommandEnum, std::string> hmi_enum_map
{
    { HmiCommandEnum::deactivate, "deactivate" },
    { HmiCommandEnum::activate, "activate" },
    { HmiCommandEnum::invalid_value, "invalid_value" }
};

/// \brief EnumWrapper for all enums which are supposed to be an input to the vfm::fsm 
/// framework. It provides a fixed integer representation of the enum and a non-changing reference
/// to it. Use the setValue method to change the value of an enum (rather than constructing a 
/// new one).
///
/// Use it as a converter enum <=> int <=> string:
/// - Convert from num (1) to enum:        HmiCommand(1).getEnum(); // ==> HmiCommandEnum::activate
/// - Convert from enum (activate) to num: HmiCommand(HmiCommandEnum::activate).getEnumAsInt(); // ==> 1
///
/// Set a new value via: 
/// - hmi_command.setValue(1)                        // Set via num.
/// - hmi_command.setValue(HmiCommandEnum::activate) // Set via enum.
/// - hmi_command.setValue(other_hmi_command)        // Set via other EnumWrapper.
class HmiCommand : public vfm::fsm::EnumWrapper<HmiCommandEnum>
{
public:
   explicit HmiCommand(const HmiCommandEnum& enum_val) : EnumWrapper("hmiCommand", hmi_enum_map, enum_val) {}
   explicit HmiCommand(const int enum_val_as_num) : EnumWrapper("hmiCommand", hmi_enum_map, enum_val_as_num) {}
   explicit HmiCommand() : EnumWrapper("hmiCommand", hmi_enum_map, HmiCommandEnum::invalid_value) {}
};

enum class HmiSymbolColorEnum 
{
    notVisible, 
    green, 
    yellow, 
    red, 
    grey_outline, 
    grey, 
    grey_dashed, 
    noSetSpeedSymbol
};

const std::map<HmiSymbolColorEnum, std::string> hmi_symbol_color_enum_map
{
    { HmiSymbolColorEnum::notVisible, "notVisible" },
    { HmiSymbolColorEnum::green, "green" },
    { HmiSymbolColorEnum::yellow, "yellow" },
    { HmiSymbolColorEnum::red, "red" },
    { HmiSymbolColorEnum::grey_outline, "grey_outline" },
    { HmiSymbolColorEnum::grey, "grey" },
    { HmiSymbolColorEnum::grey_dashed, "grey_dashed" },
    { HmiSymbolColorEnum::noSetSpeedSymbol, "noSetSpeedSymbol" }
};

class HmiSymbolColor : public vfm::fsm::EnumWrapper<HmiSymbolColorEnum>
{
public:
   explicit HmiSymbolColor(const HmiSymbolColorEnum& enum_val) : EnumWrapper("hmiSymbolColor", hmi_symbol_color_enum_map, enum_val) {}
   explicit HmiSymbolColor(const int enum_val_as_num) : EnumWrapper("hmiSymbolColor", hmi_symbol_color_enum_map, enum_val_as_num) {}
   explicit HmiSymbolColor() : EnumWrapper("hmiSymbolColor", hmi_symbol_color_enum_map, HmiSymbolColorEnum::notVisible) {}
};

enum class ActuatorStateEnum 
{
    failure, 
    inhibit, 
    standby, 
    active
};

const std::map<ActuatorStateEnum, std::string> actuator_state_enum_map
{
    { ActuatorStateEnum::failure, "failure" },
    { ActuatorStateEnum::inhibit, "inhibit" },
    { ActuatorStateEnum::standby, "standby" },
    { ActuatorStateEnum::active, "active" },
};

class ActuatorState : public vfm::fsm::EnumWrapper<ActuatorStateEnum>
{
public:
   explicit ActuatorState(const ActuatorStateEnum& enum_val) : EnumWrapper("actuatorState", actuator_state_enum_map, enum_val) {}
   explicit ActuatorState(const int enum_val_as_num) : EnumWrapper("actuatorState", actuator_state_enum_map, enum_val_as_num) {}
   explicit ActuatorState() : EnumWrapper("actuatorState", actuator_state_enum_map, ActuatorStateEnum::inhibit) {}
};

enum class LedStateEnum 
{ 
    white, 
    blue, 
    green, 
    yellow,
    orange,
    red,
    purple,
    greenBreathing, 
    greenFlashing,
    blueBreathing,
    blueFlashing
};

const std::map<LedStateEnum, std::string> led_state_enum_map
{
    { LedStateEnum::white, "white" },
    { LedStateEnum::blue, "blue" },
    { LedStateEnum::green, "green" },
    { LedStateEnum::yellow, "yellow" },
    { LedStateEnum::orange, "orange" },
    { LedStateEnum::red, "red" },
    { LedStateEnum::purple, "purple" },
    { LedStateEnum::greenBreathing, "greenBreathing" },
    { LedStateEnum::greenFlashing, "greenFlashing" },
    { LedStateEnum::blueBreathing, "blueBreathing" },
    { LedStateEnum::blueFlashing, "blueFlashing" },
};

class LedState : public vfm::fsm::EnumWrapper<LedStateEnum>
{
public:
   explicit LedState(const LedStateEnum& enum_val) : EnumWrapper("ledState", led_state_enum_map, enum_val) {}
   explicit LedState(const int enum_val_as_num) : EnumWrapper("ledState", led_state_enum_map, enum_val_as_num) {}
   explicit LedState() : EnumWrapper("ledState", led_state_enum_map, LedStateEnum::white) {}
};

static bool m_accButtonMainIsPressed;
static bool m_accButtonSetIsPressed;
static bool m_accButtonResumeIsPressed;
static bool m_accButtonSetPlusIsPressed;
static bool m_accButtonSetMinusIsPressed;
static bool m_accButtonDistanceToggleIsPressed;
static bool m_accButtonModeIsPressed;

static bool m_conditionsOk;
static bool m_handsOnSteeringWheelDetected;
static bool m_driverInterrupt;
static Fct::IndicatorLeverState m_indicatorLeverState;
static Fct::ActuatorState m_actuatorStateLong;
static Fct::ActuatorState m_actuatorStateLat;
static bool m_vehicleSpeedValid;
static float m_vehicleSpeed;
static float m_vehicleSpeedDisplayed;
static float m_hmiVehicleSpeed;
static float m_hmiSetSpeedToRpm;
static float m_hmiResumeSpeed; 

static bool m_leadingVehiclePresent; 

static float m_hmiSetSpeed;

static Fct::ExternalState m_fctStateMachineState;
static Fct::StateTransition m_fctStateMachineStateTransition;
static Fct::LaneChangeDirectionEnum m_laneChangeDirection;
static float m_vehicleStandStillTimer; 
static bool m_vehicleStandStill;

static bool m_hmiMainButtonRisingEdge;
static bool m_hmiSetButtonRisingEdge;
static bool m_hmiResumeButtonRisingEdge;
static bool m_hmiSetPlusButtonRisingEdge;
static bool m_hmiSetMinusButtonRisingEdge;
static bool m_hmiDistanceToggleRisingEdge; 
static bool m_hmiModeButtonRisingEdge;
static bool m_hwaReadyToActivate;
static bool m_conditionsDriverMonitoringOk;
static bool m_conditionsGeofencingOk;
static float m_safeStopTimer;
static bool m_safeStopRequested;
static bool m_isCenteredInEgoLane; 
static bool m_isLaneChangeActive; 
static bool m_indicatorLeftHasBeenSet;

static Fct::EscalationLevel m_driverMonitoringEscalationState;
static Fct::EscalationLevel m_vehicleStandStillEscalationState; 
static Fct::EscalationLevel m_geofencingEscalationState; 
static Fct::EscalationLevel m_unforseeableSystemBoundaryEscalationState;
static Fct::EscalationLevel m_driverInteractionEscalationState;
static Fct::EscalationLevel m_degradationEscalationState;

static float m_expectedFctCycleTime;
static bool m_fctHandsOnDetected;
static float m_numberOfToleratedActuatorStandbyCycles;
static bool m_isDriverHandsOnSteeringWheelIndication; 

static bool m_isActuatorStandbyTolerated;
static float m_tolerateActuatorStandbyCounter;
static bool m_indicatorSet;
static bool m_indicatorHasBeenSet;
static float m_indicatorLeverStateTimeCounter;
static float m_failureToleranceCycleCounter;
static Fct::IndicatorLeverState m_lastIndicatorDirection;

static std::vector<PainterVariableDescription> my_vars_for_painter = {
   {"conditionsOk", {0, std::numeric_limits<float>::infinity()}},
   {"mainButton", {0, std::numeric_limits<float>::infinity()}},
   {"mainButtonRising", {0, std::numeric_limits<float>::infinity()}},
   {"setButton", {0, std::numeric_limits<float>::infinity()}},
   {"setButtonRising", {0, std::numeric_limits<float>::infinity()}},
   {"speedMinusButton", {0, std::numeric_limits<float>::infinity()}},
   {"speedMinusButtonRising", {0, std::numeric_limits<float>::infinity()}},
   {"hmiVehicleSpeed", {3, std::numeric_limits<float>::infinity()}},
   {"hmiSetSpeed", {3, std::numeric_limits<float>::infinity()}},
   {".CurrentState", {3, std::numeric_limits<float>::infinity()}},      // TODO: Won't work with future traces.
   {"Array.CurrentState", {3, std::numeric_limits<float>::infinity()}}, // TODO: Does not work with current implementation.
   {"mcState", {3, std::numeric_limits<float>::infinity()}},
   {"mcLoop", {0, std::numeric_limits<float>::infinity()}},
}; // 0 = bool, 1 = float, 2 = enum, 3 = int.

const Color REG_BUTTON_COLOR = Color(200, 100, 100, 255);
const Color REG_BUTTON_FRAME_COLOR = BLACK;

static std::vector<PainterButtonDescription> my_buttons = {
   {{"mainButton", "main"}, {{410, 131, 65, 50}, {REG_BUTTON_COLOR, REG_BUTTON_FRAME_COLOR}}},
   {{"setButton", "set"}, {{280, 228, 65, 50}, {REG_BUTTON_COLOR, REG_BUTTON_FRAME_COLOR}}},
   {{"resetButton", "res"}, {{320, 130, 65, 50}, {REG_BUTTON_COLOR, REG_BUTTON_FRAME_COLOR}}},
   {{"speedPlusButton", "+"}, {{432, 190, 35, 50}, {REG_BUTTON_COLOR, REG_BUTTON_FRAME_COLOR}}},
   {{"speedMinusButton", "-"}, {{338, 185, 35, 50}, {REG_BUTTON_COLOR, REG_BUTTON_FRAME_COLOR}}},
   {{"mainButtonRising", "MAIN"}, {{410, 131, 65, 50}, {REG_BUTTON_COLOR, REG_BUTTON_FRAME_COLOR}}},
   {{"setButtonRising", "SET"}, {{280, 228, 65, 50}, {REG_BUTTON_COLOR, REG_BUTTON_FRAME_COLOR}}},
   {{"resetButtonRising", "RES"}, {{320, 130, 65, 50}, {REG_BUTTON_COLOR, REG_BUTTON_FRAME_COLOR}}},
   {{"speedPlusButtonRising", "+"}, {{432, 190, 35, 50}, {REG_BUTTON_COLOR, REG_BUTTON_FRAME_COLOR}}},
   {{"speedMinusButtonRising", "-"}, {{338, 185, 35, 50}, {REG_BUTTON_COLOR, REG_BUTTON_FRAME_COLOR}}},
};

static void associateVariables(const std::shared_ptr<vfm::fsm::FSMs> m)
{
   m->initAndAssociateVariableToExternalBool("speedValid", &m_vehicleSpeedValid);
   m->initAndAssociateVariableToExternalBool("driverMonitoringOk", &m_conditionsDriverMonitoringOk);
   m->initAndAssociateVariableToExternalBool("geofencingOk", &m_conditionsGeofencingOk);
   m->initAndAssociateVariableToExternalBool("driverInterrupt", &m_driverInterrupt);
   m->initAndAssociateVariableToExternalChar("actuatorStateLong", m_actuatorStateLong.getPtrToNumValue());
   m->initAndAssociateVariableToExternalChar("actuatorStateLat", m_actuatorStateLat.getPtrToNumValue());
   m->initAndAssociateVariableToExternalChar("escalationLevelDriverInteraction", m_driverInteractionEscalationState.getPtrToNumValue());
   m->initAndAssociateVariableToExternalChar("escalationLevelVehicleStandStill", m_vehicleStandStillEscalationState.getPtrToNumValue());
   m->initAndAssociateVariableToExternalChar("escalationLevelUnforseeableSystemBoundary", m_unforseeableSystemBoundaryEscalationState.getPtrToNumValue());
   m->initAndAssociateVariableToExternalBool("isLaneChangeActive", &m_isLaneChangeActive);
   m->initAndAssociateVariableToExternalBool("isActuatorStandbyTolerated", &m_isActuatorStandbyTolerated);
   m->initAndAssociateVariableToExternalBool("mainButton", &m_accButtonMainIsPressed);
   m->initAndAssociateVariableToExternalBool("modeButton", &m_accButtonModeIsPressed);
   m->initAndAssociateVariableToExternalBool("conditionsOk", &m_conditionsOk);
   m->initAndAssociateVariableToExternalBool("setButton", &m_accButtonSetIsPressed);
   m->initAndAssociateVariableToExternalBool("resumeButton", &m_accButtonResumeIsPressed);
   m->initAndAssociateVariableToExternalFloat("hmiVehicleSpeed", &m_vehicleSpeedDisplayed);
   m->initAndAssociateVariableToExternalFloat("hmiResumeSpeed", &m_hmiResumeSpeed);
   m->initAndAssociateVariableToExternalFloat("hmiSetSpeed", &m_hmiSetSpeed);
   m->initAndAssociateVariableToExternalBool("speedPlusButton", &m_accButtonSetPlusIsPressed);
   m->initAndAssociateVariableToExternalBool("speedMinusButton", &m_accButtonSetMinusIsPressed);
   m->initAndAssociateVariableToExternalChar("escalationLeveldriverMonitoring", m_driverMonitoringEscalationState.getPtrToNumValue());
   m->initAndAssociateVariableToExternalChar("escalationLevelGeoFencing", m_geofencingEscalationState.getPtrToNumValue());
   m->initAndAssociateVariableToExternalChar("escalationLevelDegradation", m_degradationEscalationState.getPtrToNumValue());
   m->initAndAssociateVariableToExternalBool("safeStopRequested", &m_safeStopRequested);
   m->initAndAssociateVariableToExternalBool("handsOn", &m_isDriverHandsOnSteeringWheelIndication);
   m->initAndAssociateVariableToExternalBool("egoCenteredInLane", &m_isCenteredInEgoLane);
   m->initAndAssociateVariableToExternalChar("indicatorLeverState", m_indicatorLeverState.getPtrToNumValue());
   m->initAndAssociateVariableToExternalChar("hasMachineTransitioned", m_fctStateMachineStateTransition.getPtrToNumValue());
   m->initAndAssociateVariableToExternalChar("extFctState", m_fctStateMachineState.getPtrToNumValue());
   m->initAndAssociateVariableToExternalFloat("failureToleranceCycleCounter", &m_failureToleranceCycleCounter);
   m->initAndAssociateVariableToExternalBool("hwaReadyToActivate", &m_hwaReadyToActivate);
   m->initAndAssociateVariableToExternalBool("indicatorHasBeenSet", &m_indicatorHasBeenSet);
   m->initAndAssociateVariableToExternalBool("indicatorSet", &m_indicatorSet);
   m->initAndAssociateVariableToExternalFloat("tolerateActuatorStandbyCounter", &m_tolerateActuatorStandbyCounter);
   m->initAndAssociateVariableToExternalFloat("numberOfToleratedActuatorStandbyCycles", &m_numberOfToleratedActuatorStandbyCycles);
   m->initAndAssociateVariableToExternalFloat("indicatorLeverStateTimeCounter", &m_indicatorLeverStateTimeCounter);
   m->initAndAssociateVariableToExternalFloat("expectedFctCycleTime", &m_expectedFctCycleTime);
   m->initAndAssociateVariableToExternalChar("lastIndicatorDirection", m_lastIndicatorDirection.getPtrToNumValue());
   m->initAndAssociateVariableToExternalFloat("hasyVehicleSpeed", &m_vehicleSpeed);
   m->initAndAssociateVariableToExternalBool("vehicleStandStill", &m_vehicleStandStill);
   m->initAndAssociateVariableToExternalFloat("vehicleStandStillTimer", &m_vehicleStandStillTimer);
   m->initAndAssociateVariableToExternalFloat("safeStopTimer", &m_safeStopTimer);

   Fct::HmiCommand hmiEnumType;
   Fct::LedState ledStateEnumType;
   Fct::HmiSymbolColor hmiSymbolColorEnumType;
   Fct::ActuatorState actuatorStateEnumType;
   Fct::EscalationLevel escalationLevelEnumType;
   Fct::IndicatorLeverState indicatorLeverEnumType;
   Fct::StateTransition stateTransitionType;
   Fct::ExternalState externalStateType;

   m->addEnumType(hmiEnumType);
   m->addEnumType(ledStateEnumType);
   m->addEnumType(hmiSymbolColorEnumType);
   m->addEnumType(actuatorStateEnumType);
   m->addEnumType(escalationLevelEnumType);
   m->addEnumType(indicatorLeverEnumType);
   m->addEnumType(stateTransitionType);
   m->addEnumType(externalStateType);

   m->associateVarToEnum("actuatorStateLat", actuatorStateEnumType.getEnumName());
   m->associateVarToEnum("actuatorStateLong", actuatorStateEnumType.getEnumName());
   m->associateVarToEnum("escalationLevelVehicleStandStill", escalationLevelEnumType.getEnumName());
   m->associateVarToEnum("escalationLeveldriverMonitoring", escalationLevelEnumType.getEnumName());
   m->associateVarToEnum("escalationLevelGeoFencing", escalationLevelEnumType.getEnumName());
   m->associateVarToEnum("escalationLevelDegradation", escalationLevelEnumType.getEnumName());
   m->associateVarToEnum("escalationLevelDriverInteraction", escalationLevelEnumType.getEnumName());
   m->associateVarToEnum("escalationLevelUnforseeableSystemBoundary", escalationLevelEnumType.getEnumName());
   m->associateVarToEnum("indicatorLeverState", indicatorLeverEnumType.getEnumName());
   m->associateVarToEnum("hasMachineTransitioned", stateTransitionType.getEnumName());
   m->associateVarToEnum("extFctState", externalStateType.getEnumName());
   m->associateVarToEnum("lastIndicatorDirection", indicatorLeverEnumType.getEnumName());
}

}  // namespace Fct

#endif
