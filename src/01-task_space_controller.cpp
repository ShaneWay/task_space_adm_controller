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

// Includes for the Admittance Controller
#include <Eigen/Dense>
#include "BaseParam.h"
#include "ControlState.h"
#include "Controller.h"
#include "ControlLogger.h"

namespace k_api = Kinova::Api;

#define PORT 10000
#define PORT_REAL_TIME 10001
constexpr auto TIMEOUT_DURATION = std::chrono::seconds{20};

float TIME_DURATION = 20.0f; // Duration of the control loop in seconds
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

// Core control logic: Task-Space Admittance Control
bool example_admittance_control(k_api::Base::BaseClient* base, 
                                k_api::BaseCyclic::BaseCyclicClient* base_cyclic, 
                                k_api::ActuatorConfig::ActuatorConfigClient* actuator_config) {
    bool return_status = true;
    unsigned int actuator_count = base->GetActuatorCount().count();

    // ==========================================================
    // 1. Initialize Parameters and the Controller
    // ==========================================================
    BaseParam params;
    std::string yaml_path = "../Parameters/config.yaml"; // Ensure path is correct relative to execution directory
    if (!params.loadFromYaml(yaml_path)) {
        std::cerr << "[ERROR] Could not load YAML configuration." << std::endl;
        return false;
    }

    AdmittanceController controller(params);
    ControlState state; // Empty state object, will be populated by controller.init()
    state.initSizes(params.nv);

    ControlLogger logger;
    logger.init(params.nv);
    logger.setLogDirectory(params.config_name);

    // ==========================================================
    // 2. Clear Faults and Prepare Hardware
    // ==========================================================
    try {
        base->ClearFaults();
    } catch(...) {
        std::cout << "Unable to clear robot faults." << std::endl;
        return false;
    }

    k_api::BaseCyclic::Feedback base_feedback;
    k_api::BaseCyclic::Command  base_command;
    auto servoing_mode = k_api::Base::ServoingModeInformation();

    std::cout << "\nInitializing Admittance Controller..." << std::endl;
    try {
        // Switch to low-level 1ms servoing mode
        servoing_mode.set_servoing_mode(k_api::Base::ServoingMode::LOW_LEVEL_SERVOING);
        base->SetServoingMode(servoing_mode);
        
        // Fetch current feedback to initialize state variables safely
        base_feedback = base_cyclic->RefreshFeedback();
        Eigen::VectorXd q_init = Eigen::VectorXd::Zero(params.nv);
        Eigen::VectorXd u_init = Eigen::VectorXd::Zero(params.nv);
        Eigen::VectorXd tau_s_init = Eigen::VectorXd::Zero(params.nv);

        for (unsigned int i = 0; i < actuator_count; i++) {
            // Send current position back to prevent following error
            base_command.add_actuators()->set_position(base_feedback.actuators(i).position());
            
            // Extract initial position and velocity in radians
            q_init(i) = base_feedback.actuators(i).position() * (M_PI / 180.0);
            u_init(i) = base_feedback.actuators(i).velocity() * (M_PI / 180.0);
            tau_s_init(i) = -base_feedback.actuators(i).torque(); 
        }
        
        // Fully encapsulated state initialization via the controller
        controller.init(state, q_init, u_init, tau_s_init);
        base_feedback = base_cyclic->Refresh(base_command);

        std::cout << "\n=======================================================" << std::endl;
        std::cout << " [WARNING] ADMITTANCE CONTROL ACTIVE!" << std::endl;
        std::cout << " [WARNING] BE READY TO INTERVENE IF INSTABILITY OCCURS!" << std::endl;
        std::cout << "=======================================================\n" << std::endl;

        std::cout << "=======================================================\n" << std::endl;
        std::cout << " Please enter a [SPACE] and press [ENTER] to continue (any other input will cancel): ";
        std::string user_input;
        std::getline(std::cin, user_input);
        if (user_input != " ") {
            std::cout << "Space not detected, operation cancelled. Exiting safely." << std::endl;
            return false; // Safely exit the current function
        }
        std::cout << "Confirmation successful, continuing execution..." << std::endl;
        std::cout << "=======================================================\n" << std::endl;

        // Switch ALL joints to TORQUE mode
        auto control_mode_message = k_api::ActuatorConfig::ControlModeInformation();
        control_mode_message.set_control_mode(k_api::ActuatorConfig::ControlMode::TORQUE);
        for (unsigned int i = 0; i < actuator_count; i++) {
            actuator_config->SetControlMode(control_mode_message, i + 1);
        }

        int timer_count = 0;
        int64_t now = 0;
        int64_t last = GetTickUs();

        // Initialize previous position and filtered velocity outside the loop
        Eigen::VectorXd q_s_prev = Eigen::VectorXd::Zero(params.nv);
        Eigen::VectorXd u_s_filtered = Eigen::VectorXd::Zero(params.nv);
        bool is_first_loop = true;

        // Filter coefficient: Range 0~1.
        // Smaller values (e.g., 0.02) yield smoother velocity and quieter torque, but larger delay (feels sluggish/        viscous).
        // Larger values (e.g., 0.1) yield smaller delay, but may have slight chattering. Recommended to start tuning       from 0.05.
        double alpha_v = 0.05;

        // ==========================================================
        // 3. Real-Time Control Loop (1 kHz)
        // ==========================================================

        std::vector<int64_t> loop_exec_times;
        loop_exec_times.reserve(TIME_DURATION * 1000);

        while (timer_count < (TIME_DURATION * 1000)) {
            now = GetTickUs();

            if (now - last >= 1000) {
                int64_t t_start = GetTickUs();
                // A. Update States from Sensor Feedback
                for (unsigned int i = 0; i < actuator_count; ++i) {
                    // 1. Read current actual position
                    state.q_s(i) = base_feedback.actuators(i).position() * (M_PI / 180.0);

                    // 2. Read measured torque
                    state.tau_s(i) = base_feedback.actuators(i).torque(); 
                
                    // 3. Calculate finite difference velocity and apply filter
                    if (is_first_loop) {
                        u_s_filtered(i) = 0.0;
                        q_s_prev(i) = state.q_s(i);
                    } else {
                        // Calculate raw finite difference
                        double delta_q = state.q_s(i) - q_s_prev(i);

                        // [CRITICAL] Prevent 2*PI jump spikes when joint crosses 360° -> 0°
                        if (delta_q > M_PI) delta_q -= 2.0 * M_PI;
                        if (delta_q < -M_PI) delta_q += 2.0 * M_PI;

                        double u_s_raw = delta_q / params.T; // T = 0.001s
                    
                        // Exponential Moving Average (EMA) first-order low-pass filter
                        u_s_filtered(i) = alpha_v * u_s_raw + (1.0 - alpha_v) * u_s_filtered(i);
                    }

                    // Assign the calculated and filtered velocity to the system state
                    state.u_s(i) = u_s_filtered(i);

                    // Update previous position for the next frame
                    q_s_prev(i) = state.q_s(i);
                }
                is_first_loop = false; // Mark the end of the first loop
                // A. Update States from Sensor Feedback
                // for (unsigned int i = 0; i < actuator_count; ++i) {
                //     // Convert position and velocity to radians and rad/s
                //     state.q_s(i) = base_feedback.actuators(i).position() * (M_PI / 180.0);
                //     state.u_s(i) = base_feedback.actuators(i).velocity() * (M_PI / 180.0);
                    
                //     // NOTE: Kinova's torque sensor reading is reversed compared to motor torque direction
                //     state.tau_s(i) = -base_feedback.actuators(i).torque(); 
                // }

                // B. Core Algorithm: Calculate commanded torque
                Eigen::VectorXd tau_cmd = controller.update(state);

                double time_sec = timer_count / 1000.0;
                logger.log(time_sec, state);

                // C. Construct hardware command
                base_command.set_frame_id(base_command.frame_id() + 1);
                if (base_command.frame_id() > 65535) base_command.set_frame_id(0);

                for (unsigned int i = 0; i < actuator_count; i++) {
                    base_command.mutable_actuators(i)->set_command_id(base_command.frame_id());
                    // Constantly feedback current position to bypass internal following errors
                    base_command.mutable_actuators(i)->set_position(base_feedback.actuators(i).position());
                    // Send the computed torque to the joint
                    base_command.mutable_actuators(i)->set_torque_joint(tau_cmd(i));
                }

                // D. Send command and refresh sensor feedback
                try {
                    base_feedback = base_cyclic->Refresh(base_command, 0);
                } catch (k_api::KDetailedException& ex) {
                    std::cout << "Kortex exception: " << ex.what() << std::endl;
                }

                int64_t t_end = GetTickUs();
                loop_exec_times.push_back(t_end - t_start);

                timer_count++;
                last = now;
            }
        }

        std::cout << "\nAdmittance Control Test Completed." << std::endl;
        logger.saveToTxt("control_log_data.csv");
        logger.saveToTxt("control_log_data.txt");

        // Restore actuators to POSITION mode
        control_mode_message.set_control_mode(k_api::ActuatorConfig::ControlMode::POSITION);
        for (unsigned int i = 0; i < actuator_count; i++) {
            actuator_config->SetControlMode(control_mode_message, i + 1);
        }
        std::cout << "Actuators safely restored to POSITION mode." << std::endl;

        if (!loop_exec_times.empty()) {
            int64_t max_time = 0;
            int64_t min_time = 9999999;
            int64_t sum_time = 0;
            int violations = 0;

            for (auto t : loop_exec_times) {
                if (t > max_time) max_time = t;
                if (t < min_time) min_time = t;
                sum_time += t;
                if (t > 1000) violations++; // 记录执行时间超过 1ms 的次数
            }

            std::cout << "\n--- Real-Time Performance Statistics ---" << std::endl;
            std::cout << " Total Frames Executed: " << loop_exec_times.size() << std::endl;
            std::cout << " Average Execution Time: " << (double)sum_time / loop_exec_times.size() << " us" << std::endl;
            std::cout << " Minimum Execution Time: " << min_time << " us" << std::endl;
            std::cout << " Maximum Execution Time: " << max_time << " us" << std::endl;
            std::cout << " Deadline Violations (>1000us): " << violations << " (" 
                      << ((double)violations / loop_exec_times.size() * 100.0) << "%)" << std::endl;
            std::cout << "----------------------------------------\n" << std::endl;
        }
        logger.plot();

    } catch (k_api::KDetailedException& ex) {
        std::cout << "API error: " << ex.what() << std::endl;
        return_status = false;
    } catch (std::runtime_error& ex2) {
        std::cout << "Error: " << ex2.what() << std::endl;
        return_status = false;
    }
    
    // Restore Servoing Mode
    servoing_mode.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
    base->SetServoingMode(servoing_mode);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    return return_status;
}

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
    success &= example_admittance_control(base, base_cyclic, actuator_config);

    if (!success) {
        std::cout << "There has been an unexpected error." << std::endl;
    }

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