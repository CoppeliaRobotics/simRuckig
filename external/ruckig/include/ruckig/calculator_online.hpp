#pragma once

#include <algorithm>
#include <array>
#include <chrono>
#include <iostream>
#include <limits>
#include <tuple>
#include <vector>

#include <httplib/httplib.h>
#include <json/json.hpp>

#include <ruckig/input_parameter.hpp>
#include <ruckig/profile.hpp>
#include <ruckig/trajectory.hpp>


namespace ruckig {

//! Calculation class for a trajectory along waypoints.
template<size_t DOFs, template<class, size_t> class CustomVector = StandardVector>
class WaypointsCalculator {
    httplib::Client cli {"http://api.ruckig.com"};

public:
    size_t degrees_of_freedom;

    template <size_t D = DOFs, typename std::enable_if<D >= 1, int>::type = 0>
    explicit WaypointsCalculator(): degrees_of_freedom(DOFs) { }

    template <size_t D = DOFs, typename std::enable_if<D >= 1, int>::type = 0>
    explicit WaypointsCalculator(size_t): degrees_of_freedom(DOFs) { }

    template <size_t D = DOFs, typename std::enable_if<D == 0, int>::type = 0>
    explicit WaypointsCalculator(size_t dofs): degrees_of_freedom(dofs) { }

    template <size_t D = DOFs, typename std::enable_if<D == 0, int>::type = 0>
    explicit WaypointsCalculator(size_t dofs, size_t): degrees_of_freedom(dofs) { }

    template<bool throw_error>
    Result calculate(const InputParameter<DOFs, CustomVector>& input, Trajectory<DOFs, CustomVector>& traj, double, bool& was_interrupted) {
        std::cout << "[ruckig] calculate trajectory via online API server." << std::endl;

        nlohmann::json params;
        params["degrees_of_freedom"] = input.degrees_of_freedom;
        params["current_position"] = input.current_position;
        params["current_velocity"] = input.current_velocity;
        params["current_acceleration"] = input.current_acceleration;
        params["target_position"] = input.target_position;
        params["target_velocity"] = input.target_velocity;
        params["target_acceleration"] = input.target_acceleration;
        params["max_velocity"] = input.max_velocity;
        params["max_acceleration"] = input.max_acceleration;
        params["max_jerk"] = input.max_jerk;
        if (input.min_velocity) {
            params["min_velocity"] = *input.min_velocity;
        }
        if (input.min_acceleration) {
            params["min_acceleration"] = *input.min_acceleration;
        }
        if (!input.intermediate_positions.empty()) {
            params["intermediate_positions"] = input.intermediate_positions;
        }
        if (input.per_section_max_velocity) {
            params["per_section_max_velocity"] = *input.per_section_max_velocity;
        }
        if (input.per_section_max_acceleration) {
            params["per_section_max_acceleration"] = *input.per_section_max_acceleration;
        }
        if (input.per_section_max_jerk) {
            params["per_section_max_jerk"] = *input.per_section_max_jerk;
        }
        if (input.per_section_min_velocity) {
            params["per_section_min_velocity"] = *input.per_section_min_velocity;
        }
        if (input.per_section_min_acceleration) {
            params["per_section_min_acceleration"] = *input.per_section_min_acceleration;
        }
        if (input.max_position) {
            params["max_position"] = *input.max_position;
        }
        if (input.min_position) {
            params["min_position"] = *input.min_position;
        }
        params["enabled"] = input.enabled;
        params["control_interface"] = input.control_interface;
        params["synchronization"] = input.synchronization;
        params["duration_discretization"] = input.duration_discretization;
        if (input.per_dof_control_interface) {
            params["per_dof_control_interface"] = *input.per_dof_control_interface;
        }
        if (input.per_dof_synchronization) {
            params["per_dof_synchronization"] = *input.per_dof_synchronization;
        }
        if (input.minimum_duration) {
            params["minimum_duration"] = *input.minimum_duration;
        }
        if (input.per_section_minimum_duration) {
            params["per_section_minimum_duration"] = *input.per_section_minimum_duration;
        }

        auto res = cli.Post("/calculate", params.dump(), "application/json");
        if (res->status != 200) {
            if constexpr (throw_error) {
                throw std::runtime_error("[ruckig] could not reach online API server, error code: " + std::to_string(res->status) + " " + res->body);
            } else {
                std::cout << "[ruckig] could not reach online API server, error code: " << res->status << " " << res->body << std::endl;
                return Result::Error;
            }
        }
        
        auto result = nlohmann::json::parse(res->body);

        was_interrupted = false;
        traj.degrees_of_freedom = input.degrees_of_freedom;
        traj.resize(result["profiles"].size() - 1);

        traj.continue_calculation_counter = 0;
        traj.duration = result["duration"].template get<double>();
        traj.cumulative_times = result["cumulative_times"].template get<std::vector<double>>();

        if (!result["message"].empty()) {
            std::cout << "[ruckig] " << result["message"] << std::endl;
        }
        
        if (result["result"] == "Result.Error") {
            return Result::Error; 
        
        } else if (result["result"] == "Result.ErrorInvalidInput") {
            return Result::ErrorInvalidInput; 
        }

        for (size_t i = 0; i < result["profiles"].size(); ++i) {
            for (size_t dof = 0; dof < traj.degrees_of_freedom; ++dof) {
                auto& p = result["profiles"][i][dof];
                traj.profiles[i][dof] = p.template get<Profile>();
            }
        }

        return Result::Working;
    }

    template<bool throw_error>
    Result continue_calculation(const InputParameter<DOFs, CustomVector>&, Trajectory<DOFs, CustomVector>&, double, bool&) {
        if constexpr (throw_error) {
            throw std::runtime_error("[ruckig] continue calculation not available in Ruckig Community Version.");
        } else {
            return Result::Error;
        }
    }
};

} // namespace ruckig_pro
