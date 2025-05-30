//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file
#pragma once

#include "static_helper.h"

namespace vfm {
namespace mc {
namespace trajectory_generator {

using DataPackTrace = std::vector<DataPackPtr>;

constexpr double LANE_WIDTH = 3.5; // meter
constexpr double LANE_WIDTH_HALF = LANE_WIDTH / 2.0;

constexpr double STATE_INVALID = 0;
constexpr double STATE_ENVIRONMENT_MODEL = 1;
constexpr double STATE_TACTICAL_PLANNER = 2;

	// Predefinitions
	struct InterpretableKeyInterface;
	class MCinterpretedTrace;

	enum class PossibleParameter {
		pos_x, pos_y, pos_z,
		vel_x, vel_y, vel_z,
		acc_x, acc_y, acc_z,
		heading, yawrate, jerk,
		special_current_state, // STATE_INVALID/STATE_ENVIRONMENT_MODEL/STATE_TACTICAL_PLANNER
		do_lane_change, // intending a lane change (signalizing)
		change_lane_now, // performing the lane change
		turn_signal_left, // Turn signal left and right (cannot use one variable because of interpolation)
		turn_signal_right,
		gap_0_i_agent_front, gap_1_i_agent_front, gap_2_i_agent_front,
		gap_0_i_agent_rear, gap_1_i_agent_rear, gap_2_i_agent_rear,
      on_straight_section, traversion_from, traversion_to
	};
	// NOTE: When you add more parameters here and access them every state, make sure to add them to required_parameters in the config.

   class Interpolation : public Failable 
   {
   public:
      Interpolation(const std::string& failable_name);
      virtual double interpolate(const double current_value, const double next_value, const double factor) const = 0;
      virtual void addAdditionalData(const std::vector<double>& additional_data);
   };

   class LinearInterpolation : public Interpolation
   {
   public:
      LinearInterpolation();
      double interpolate(const double current_value, const double next_value, const double factor) const override;
   };

   class OnetimeSwitchInterpolation : public Interpolation
   {
   public:
      OnetimeSwitchInterpolation();
      double interpolate(const double current_value, const double next_value, const double factor) const override;
      void addAdditionalData(const std::vector<double>& additional_data) override;

   private:
      bool has_switch_occurred_{ false };
   };

   class LinearInterpolationWithCorrection : public Interpolation 
   {
   public:
      LinearInterpolationWithCorrection();
      double interpolate(const double current_value, const double next_value, const double factor) const override;
      void addAdditionalData(const std::vector<double>& additional_data) override;

   private:
      double correction_{ -1 };
   };

   class NoInterpolation : public Interpolation 
   {
   public:
      NoInterpolation();
      double interpolate(const double current_value, const double next_value, const double factor) const override;
   };

	// A representation for a parameter that should be provided in the trajectory output. The method will be used for stage where the value has not been provided.
	class ParameterDetails
	{
   public:
      ParameterDetails(
         const PossibleParameter identifier,
         const std::string& name,
         const bool needed_for_osc_trajectory,
         const std::shared_ptr<Interpolation> interpolation_method = std::make_shared<LinearInterpolation>(),
         const std::function<double(double)> provide_if_missing = provideZeroIfNan);

      static double provideZeroIfNan(double value)
		{
			return std::isnan(value) ? 0 : value;
		}

   public: // TODO: Make private
      PossibleParameter identifier_{};
		std::string name_{};
		bool needed_for_osc_trajectory_{false};
      std::shared_ptr<Interpolation> interpolation_method_{};
		std::function<double(double)> provide_if_missing_{ provideZeroIfNan };
	};

	typedef std::map<std::string, std::shared_ptr<InterpretableKeyInterface>> InterpretationCommands;

	// A position on the trajectory consisting of a timestamp and list of parameters.
	typedef std::map<PossibleParameter, double> ParameterMap;

	// A data point on a trajectory associated with a timestamp.
	typedef std::pair<double, ParameterMap> TrajectoryPosition;

	// A data point on a trajectory associated with a timestamp.
	typedef std::vector<TrajectoryPosition> FullTrajectory;
	

	// warning for a specific step
	typedef std::pair<std::string, std::string> TraceStepWarning;


	struct InterpretationConfiguration
	{
		InterpretationConfiguration(
			std::string ego_vehicle_name,
			InterpretationCommands general_interpretations,
			InterpretationCommands vehicle_interpretations,
			std::vector<std::string> possible_vehicle_keys,
			std::vector<ParameterDetails> required_parameters,
			std::function<bool(std::vector<ParameterMap>&)>check_state_end,
			std::function<void(MCinterpretedTrace&)> preprocessing,
			std::function<void(MCinterpretedTrace&)> postprocessing,
			double default_step_time,
			char key_separator) :
			m_ego_vehicle_name(ego_vehicle_name),
			m_general_interpretations(general_interpretations),
			m_vehicle_interpretations(vehicle_interpretations),
			m_possible_vehicle_keys(possible_vehicle_keys),
			m_required_parameters(required_parameters),
			m_check_state_end(check_state_end),
			m_preprocessing(preprocessing),
			m_postprocessing(postprocessing),
			m_default_step_time(default_step_time),
			m_key_separator(key_separator)
      {
      };

		const std::string m_ego_vehicle_name;
		const InterpretationCommands m_general_interpretations;
		const InterpretationCommands m_vehicle_interpretations;
		const std::vector<std::string> m_possible_vehicle_keys;
		const std::vector<ParameterDetails> m_required_parameters;
		const std::function<bool(std::vector<ParameterMap>&)>m_check_state_end;
		const std::function<void(MCinterpretedTrace&)> m_preprocessing;
		const std::function<void(MCinterpretedTrace&)> m_postprocessing;
		const double m_default_step_time = 1.0; // in s
		const char m_key_separator;


		static InterpretationConfiguration getLaneChangeConfiguration();

	private:
		static std::vector<ParameterDetails> getStandardRequiredParameters();

	};


	struct InterpretableKeyInterface {
		virtual void interpret(MCinterpretedTrace& osc, std::vector<std::string> key_elements, std::string value_string) = 0;
	};

	template <class T>
	struct InterpretableKey : public InterpretableKeyInterface
	{
		InterpretableKey() {};

		InterpretableKey(std::function<void(MCinterpretedTrace&, std::vector<std::string>, T)> interpreter) :
			m_interpreter(interpreter) {};

		std::function<void(MCinterpretedTrace&, std::vector<std::string>, T)> m_interpreter;

		void interpret(MCinterpretedTrace& osc, std::vector<std::string> key_elements, std::string value_string)
		{
			T value{};
			get_value(value_string, value);
			m_interpreter(osc, key_elements, value);
		}

	private:
		void get_value(std::string value_string, double& out)
		{
			out = std::stod(value_string);
		}
		void get_value(std::string value_string, int& out)
		{
			out = std::stoi(value_string);
		}
		void get_value(std::string value_string, std::string& out)
		{
			out = value_string;
		}
		void get_value(std::string value_string, bool& out)
		{
			out = StaticHelper::isBooleanTrue(value_string);
		}
		// Add other types here
	};

} // trajectory_generator
} // mc
} // vfm
