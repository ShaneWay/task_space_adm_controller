#include <iostream>
#include <string>
#include <vector>
#include <math.h>
#include <chrono>
#include <thread>
#include <future>

#include <KDetailedException.h>
#include <BaseClientRpc.h>
#include <BaseCyclicClientRpc.h>
#include <ActuatorConfigClientRpc.h>
#include <SessionClientRpc.h>
#include <SessionManager.h>
#include <RouterClient.h>
#include <TransportClientUdp.h>
#include <TransportClientTcp.h>
#include "utilities.h"

// Pinocchio & Eigen
#include <Eigen/Dense>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include "BaseParam.h"

namespace k_api = Kinova::Api;
constexpr auto TIMEOUT_DURATION = std::chrono::seconds{20};
#define PORT 10000
#define PORT_REAL_TIME 10001

float TIME_DURATION = 10.0f; // Duration of the gravity compensation test (seconds)
constexpr auto TIMEOUT_PROMISE_DURATION = std::chrono::seconds{20};

// Get system time in microseconds
int64_t GetTickUs() {
    struct timespec start;
    clock_gettime(CLOCK_MONOTONIC, &start);
    return (start.tv_sec * 1000000LLU) + (start.tv_nsec / 1000);
}

// Action completion callback listener
std::function<void(k_api::Base::ActionNotification)> 
create_event_listener_by_promise(std::promise<k_api::Base::ActionEvent>& finish_promise) {
    return [&finish_promise] (k_api::Base::ActionNotification notification) {
        const auto action_event = notification.action_event();
        if (action_event == k_api::Base::ActionEvent::ACTION_END || 
            action_event == k_api::Base::ActionEvent::ACTION_ABORT) {
            finish_promise.set_value(action_event);
        }
    };
}

bool example_angular_action_movement(k_api::Base::BaseClient* base) 
{
    std::cout << "Starting angular action movement ..." << std::endl;

    auto action = k_api::Base::Action();
    action.set_name("Example angular action movement");
    action.set_application_data("");

    auto reach_joint_angles = action.mutable_reach_joint_angles();
    auto joint_angles = reach_joint_angles->mutable_joint_angles();

    auto actuator_count = base->GetActuatorCount();

    float exp_start_angle[] = {0.073, 28.099, 179.942, 281.407, 359.992, 286.364, 180.197};
    // Arm straight up
    for (size_t i = 0; i < actuator_count.count(); ++i) 
    {
        auto joint_angle = joint_angles->add_joint_angles();
        joint_angle->set_joint_identifier(i);
        joint_angle->set_value(exp_start_angle[i]);
    }

    // Connect to notification action topic
    // (Promise alternative)
    // See cartesian examples for Reference alternative
    std::promise<k_api::Base::ActionEvent> finish_promise;
    auto finish_future = finish_promise.get_future();
    auto promise_notification_handle = base->OnNotificationActionTopic(
        create_event_listener_by_promise(finish_promise),
        k_api::Common::NotificationOptions()
    );

    std::cout << "Executing action" << std::endl;
    base->ExecuteAction(action);

    std::cout << "Waiting for movement to finish ..." << std::endl;

    // Wait for future value from promise
    // (Promise alternative)
    // See cartesian examples for Reference alternative
    const auto status = finish_future.wait_for(TIMEOUT_DURATION);
    base->Unsubscribe(promise_notification_handle);

    if(status != std::future_status::ready)
    {
        std::cout << "Timeout on action notification wait" << std::endl;
        return false;
    }
    const auto promise_event = finish_future.get();

    std::cout << "Angular movement completed" << std::endl;
    std::cout << "Promise value : " << k_api::Base::ActionEvent_Name(promise_event) << std::endl; 

    return true;
}

// Move to a safe initial position (Home)
bool example_move_to_home_position(k_api::Base::BaseClient* base) {
    auto servoingMode = k_api::Base::ServoingModeInformation();
    servoingMode.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
    base->SetServoingMode(servoingMode);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    std::cout << "Moving the arm to a safe (Home) position..." << std::endl;
    auto action_type = k_api::Base::RequestedActionType();
    action_type.set_action_type(k_api::Base::REACH_JOINT_ANGLES);
    auto action_list = base->ReadAllActions(action_type);
    auto action_handle = k_api::Base::ActionHandle();
    action_handle.set_identifier(0);
    
    for (auto action : action_list.action_list()) {
        if (action.name() == "Home") {
            action_handle = action.handle();
        }
    }

    if (action_handle.identifier() == 0) {
        std::cout << "Can't reach safe position, exiting." << std::endl;
        return false;
    } else {
        std::promise<k_api::Base::ActionEvent> finish_promise;
        auto finish_future = finish_promise.get_future();
        auto promise_notification_handle = base->OnNotificationActionTopic(
            create_event_listener_by_promise(finish_promise),
            k_api::Common::NotificationOptions()
        );

        base->ExecuteActionFromReference(action_handle);
        const auto status = finish_future.wait_for(TIMEOUT_PROMISE_DURATION);
        base->Unsubscribe(promise_notification_handle);

        if(status != std::future_status::ready) {
            std::cout << "Timeout on action notification wait." << std::endl;
            return false;
        }
        std::cout << "Move to Home completed." << std::endl;
        return true;
    }
}

// Core control logic: Gravity Compensation
bool example_gravity_compensation(k_api::Base::BaseClient* base, 
                                  k_api::BaseCyclic::BaseCyclicClient* base_cyclic, 
                                  k_api::ActuatorConfig::ActuatorConfigClient* actuator_config) {
    bool return_status = true;
    unsigned int actuator_count = base->GetActuatorCount().count();

    // 1. Load parameters and Pinocchio model
    BaseParam params;
    std::string yaml_path = "../Parameters/config.yaml"; // Please ensure the path is correct
    if (!params.loadFromYaml(yaml_path)) {
        std::cerr << "[ERROR] Could not load YAML configuration." << std::endl;
        return false;
    }

    pinocchio::Model model;
    try {
        pinocchio::urdf::buildModel(params.urdf_path, model);
        std::cout << "[SUCCESS] Pinocchio Model Loaded from: " << params.urdf_path << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "[ERROR] Failed to load URDF: " << e.what() << std::endl;
        return false;
    }
    pinocchio::Data data(model);
    Eigen::VectorXd q_rad = Eigen::VectorXd::Zero(model.nv);
    Eigen::VectorXd tau_gravity = Eigen::VectorXd::Zero(model.nv);

    // Clear robot fault states
    try {
        base->ClearFaults();
    } catch(...) {
        std::cout << "Unable to clear robot faults." << std::endl;
        return false;
    }

    k_api::BaseCyclic::Feedback base_feedback;
    k_api::BaseCyclic::Command  base_command;
    auto servoing_mode = k_api::Base::ServoingModeInformation();

    std::cout << "\nInitializing the arm for Gravity Compensation..." << std::endl;
    try {
        // Switch to low-level 1ms servoing mode
        servoing_mode.set_servoing_mode(k_api::Base::ServoingMode::LOW_LEVEL_SERVOING);
        base->SetServoingMode(servoing_mode);
        base_feedback = base_cyclic->RefreshFeedback();

        // Prevent following error: send current position first
        for (unsigned int i = 0; i < actuator_count; i++) {
            base_command.add_actuators()->set_position(base_feedback.actuators(i).position());
        }
        base_feedback = base_cyclic->Refresh(base_command);
        
        // Crucial step: Switch all joints to TORQUE mode
        auto control_mode_message = k_api::ActuatorConfig::ControlModeInformation();
        control_mode_message.set_control_mode(k_api::ActuatorConfig::ControlMode::TORQUE);
        for (unsigned int i = 0; i < actuator_count; i++) {
            int actuator_device_id = i + 1; // Joint IDs start from 1
            actuator_config->SetControlMode(control_mode_message, actuator_device_id);
        }

        std::cout << "\n=======================================================" << std::endl;
        std::cout << " [WARNING] GRAVITY COMPENSATION ACTIVE!" << std::endl;
        std::cout << " [WARNING] PLEASE HOLD THE ROBOT ARM TO PREVENT DROPPING!" << std::endl;
        std::cout << "=======================================================\n" << std::endl;

        int timer_count = 0;
        int64_t now = 0;
        int64_t last = GetTickUs();

        // 1kHz real-time control loop
        while (timer_count < (TIME_DURATION * 1000)) {
            now = GetTickUs();

            if (now - last >= 1000) {
                // A. Extract angles and convert: Kinova Degree -> Pinocchio Radian
                for (unsigned int i = 0; i < actuator_count; ++i) {
                    q_rad(i) = base_feedback.actuators(i).position() * (M_PI / 180.0);
                }

                // B. Compute gravity compensation torque via Pinocchio
                tau_gravity = pinocchio::computeGeneralizedGravity(model, data, q_rad);

                // C. Construct control command
                base_command.set_frame_id(base_command.frame_id() + 1);
                if (base_command.frame_id() > 65535) base_command.set_frame_id(0);

                for (unsigned int i = 0; i < actuator_count; i++) {
                    base_command.mutable_actuators(i)->set_command_id(base_command.frame_id());
                    // Continuously send current position to bypass low-level following error detection
                    base_command.mutable_actuators(i)->set_position(base_feedback.actuators(i).position());
                    // Send computed gravity compensation torque
                    base_command.mutable_actuators(i)->set_torque_joint(tau_gravity(i));
                }

                // D. Send command and refresh feedback
                try {
                    base_feedback = base_cyclic->Refresh(base_command, 0);
                } catch (k_api::KDetailedException& ex) {
                    std::cout << "Kortex exception: " << ex.what() << std::endl;
                }
                
                timer_count++;
                last = now;
            }
        }

        std::cout << "\nGravity Compensation Test Completed." << std::endl;

        // Test completed, safely restore all joints to POSITION mode
        control_mode_message.set_control_mode(k_api::ActuatorConfig::ControlMode::POSITION);
        for (unsigned int i = 0; i < actuator_count; i++) {
            int actuator_device_id = i + 1;
            actuator_config->SetControlMode(control_mode_message, actuator_device_id);
        }
        std::cout << "Actuators restored to POSITION mode." << std::endl;

    } catch (k_api::KDetailedException& ex) {
        std::cout << "API error: " << ex.what() << std::endl;
        return_status = false;
    } catch (std::runtime_error& ex2) {
        std::cout << "Error: " << ex2.what() << std::endl;
        return_status = false;
    }
    
    // Restore to single-level servoing mode
    servoing_mode.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
    base->SetServoingMode(servoing_mode);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    return return_status;
}

// Main function entry: Network connection and initialization
int main(int argc, char **argv) {
    auto parsed_args = ParseExampleArguments(argc, argv);
    auto error_callback = [](k_api::KError err){ std::cout << "_________ callback error _________" << err.toString(); };
    
    std::cout << "Creating transport objects" << std::endl;
    auto transport = new k_api::TransportClientTcp();
    auto router = new k_api::RouterClient(transport, error_callback);
    transport->connect(parsed_args.ip_address, PORT);

    auto transport_real_time = new k_api::TransportClientUdp();
    auto router_real_time = new k_api::RouterClient(transport_real_time, error_callback);
    transport_real_time->connect(parsed_args.ip_address, PORT_REAL_TIME);

    auto create_session_info = k_api::Session::CreateSessionInfo();
    create_session_info.set_username("admin");
    create_session_info.set_password("admin");
    create_session_info.set_session_inactivity_timeout(60000);   
    create_session_info.set_connection_inactivity_timeout(2000); 

    auto session_manager = new k_api::SessionManager(router);
    session_manager->CreateSession(create_session_info);
    auto session_manager_real_time = new k_api::SessionManager(router_real_time);
    session_manager_real_time->CreateSession(create_session_info);

    auto base = new k_api::Base::BaseClient(router);
    auto base_cyclic = new k_api::BaseCyclic::BaseCyclicClient(router_real_time);
    auto actuator_config = new k_api::ActuatorConfig::ActuatorConfigClient(router);

    bool success = true;
    success &= example_move_to_home_position(base);
    success &= example_angular_action_movement(base);
    success &= example_gravity_compensation(base, base_cyclic, actuator_config);

    if (!success) {
        std::cout << "There has been an unexpected error." << std::endl;
    }

    // Disconnect and clean up memory
    session_manager->CloseSession();
    session_manager_real_time->CloseSession();

    router->SetActivationStatus(false);
    transport->disconnect();
    router_real_time->SetActivationStatus(false);
    transport_real_time->disconnect();

    delete base; delete base_cyclic; delete actuator_config;
    delete session_manager; delete session_manager_real_time;
    delete router; delete router_real_time;
    delete transport; delete transport_real_time;

    return success ? 0 : 1;
}