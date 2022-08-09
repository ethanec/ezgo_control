#include <pthread.h>
#include <ros/ros.h>
#include <signal.h>
#include <unistd.h>
#include <iostream>
#include <string>
#include <vector>
#include <math.h>

#include <autoware_msgs/AccelCmd.h>
#include <autoware_msgs/BrakeCmd.h>
#include <autoware_msgs/SteerCmd.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Float32.h>
#include <tablet_socket_msgs/gear_cmd.h>
#include <tablet_socket_msgs/mode_cmd.h>

#include <autoware_can_msgs/CANInfo.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <tablet_socket_msgs/mode_info.h>

#define RESET "\033[0m"
#define BLACK "\033[30m"   /* Black */
#define RED "\033[31m"     /* Red */
#define GREEN "\033[32m"   /* Green */
#define YELLOW "\033[33m"  /* Yellow */
#define BLUE "\033[34m"    /* Blue */
#define MAGENTA "\033[35m" /* Magenta */
#define CYAN "\033[36m"    /* Cyan */
#define WHITE "\033[37m"   /* White */

enum Light { OFF, ON };
enum Shift { FORWARD, REVERSE };
enum Mode { MANUAL, AUTONOMOUS };

typedef struct vehicle_config {
    bool is_valid;
    int vendor_ID;
    int car_ID;
    int CAN_dev_ID;
    double SPEED_LIMIT;

    // steering
    double STEERING_ANGLE_MAX;
    double TURNING_RADIUS_MIN;
    double WHEEL_BASE;
    double WHEEL_ANGLE_MAX;

    // accel
    double _K_ACCEL_P_UNTIL30;
    double _K_ACCEL_I_UNTIL30;
    double _K_ACCEL_D_UNTIL30;
    double _K_ACCEL_P_UNTIL15;
    double _K_ACCEL_I_UNTIL15;
    double _K_ACCEL_D_UNTIL15;

    double _K_ACCEL_I_GAIN;
    double _K_ACCEL_OFFSET;
    double _ACCEL_MAX_I;
    double _ACCEL_PEDAL_MAX;
    double _ACCEL_PEDAL_MIN;

    // brake
    double _K_BRAKE_P;
    double _K_BRAKE_I;
    double _K_BRAKE_D;
    int _K_BRAKE_I_CYCLES;
    double _BRAKE_MAX_I;
    double _BRAKE_PEDAL_MAX;
    double _BRAKE_PEDAL_MIN;
    double _BRAKE_PEDAL_STOPPING_MAX;
    double _BRAKE_PEDAL_STOPPING_MED;
    double _BRAKE_PEDAL_OFFSET;

} vehicle_config_t;

typedef struct vehicle_info {
    int throttle;
    int brake;
    float steering_angle;
    float velocity;
    bool control_mode;  // 0 : manual mode, 1 : autonomous mode
    bool shift;         // 0 : Forward, 1 : Reverse
    bool light;         // 0 : OFF, 1 : ON
} vehicle_info_t;

typedef struct vehicle_cmd {
    double linear_x;
    double angular_z;
    int modeValue;  // 0 : manual, 1 : auto pilot, 2 : UI direct control
    int shift;      // 0 : Forward, 1 : Reverse
    int accel_stroke;
    int brake_stroke;
    int steering_angle;
    char light;
} vehicle_cmd_t;

extern vehicle_cmd_t vehicle_cmd;
extern vehicle_info_t vehicle_info;
extern vehicle_config_t vehicle_config;
extern int willExit;


inline void cmd_reset()
{
    vehicle_cmd.linear_x = 0;
    vehicle_cmd.angular_z = 0;
    vehicle_cmd.modeValue = 0;
    vehicle_cmd.shift = 0;
    vehicle_cmd.accel_stroke = 0;
    vehicle_cmd.brake_stroke = 0;
    vehicle_cmd.steering_angle = 0;
    vehicle_cmd.light = 0;
}

inline void modeCMDCallback(const tablet_socket_msgs::mode_cmd &mode)
{   
    if (mode.mode == -1 || mode.mode == 0) {
        cmd_reset();
    }
    vehicle_cmd.modeValue = mode.mode;
    std::cout << "In modeCMDCallback, mode = " << vehicle_cmd.modeValue << std::endl;
}

inline void gearCMDCallback(const tablet_socket_msgs::gear_cmd &gear)
{
    vehicle_cmd.shift = gear.gear;
    std::cout << "In gearCMDCallback, mode = " <<  vehicle_cmd.shift << std::endl;
}

inline void twistCMDCallback(const geometry_msgs::TwistStamped &msg)
{
    vehicle_cmd.linear_x = msg.twist.linear.x;
    vehicle_cmd.angular_z = msg.twist.angular.z;
}

inline void steerCMDCallback(const autoware_msgs::SteerCmd &steer)
{
    vehicle_cmd.steering_angle = steer.steer;
}

inline void accellCMDCallback(const autoware_msgs::AccelCmd &accell)
{
    vehicle_cmd.accel_stroke = accell.accel;
}

inline void brakeCMDCallback(const autoware_msgs::BrakeCmd &brake)
{
    vehicle_cmd.brake_stroke = brake.brake;
}

void set_drv_stroke(double accel_stroke)
{
    if(accel_stroke < 0)
        vehicle_cmd.accel_stroke = 0;
    else if(accel_stroke > 255)
        vehicle_cmd.accel_stroke = 255;
    else
        vehicle_cmd.accel_stroke = accel_stroke;
}

void set_break_stroke(double brake_stroke)
{
    if(brake_stroke < 0)
        vehicle_cmd.brake_stroke = 0;
    else if(brake_stroke > 255)
        vehicle_cmd.brake_stroke = 255;
    else
        vehicle_cmd.brake_stroke = brake_stroke;
}


inline double accel_pid_control(double current_vel, double cmd_vel)
{
    double e, ret;
    double prop, inter, diff;
    double target_accel_stroke;
    static double e_prev = 0;

    e = cmd_vel - current_vel;

    if(cmd_vel > 15){
        prop = e * vehicle_config._K_ACCEL_P_UNTIL30;
        inter += e_prev * vehicle_config._K_ACCEL_I_UNTIL30;
        diff = (e - e_prev) * vehicle_config._K_ACCEL_D_UNTIL30;
    }else{
        prop = e * vehicle_config._K_ACCEL_P_UNTIL15;
        inter += e_prev * vehicle_config._K_ACCEL_I_UNTIL15;
        diff = (e - e_prev) * vehicle_config._K_ACCEL_D_UNTIL15;
    }
    target_accel_stroke = prop + inter + diff;

    if(target_accel_stroke > vehicle_config._ACCEL_PEDAL_MAX){
        target_accel_stroke = vehicle_config._ACCEL_PEDAL_MAX;
    }else if(target_accel_stroke < vehicle_config._ACCEL_PEDAL_MIN){
        target_accel_stroke = vehicle_config._ACCEL_PEDAL_MIN;
    }

    e_prev = e;
    ret = target_accel_stroke;

    return ret;
}

inline double brake_pid_control(double current_vel, double cmd_vel)
{
    double e, ret;
    double prop, inter, diff;
    double target_brake_stroke;
    static double e_prev = 0;

    e = -1 * (cmd_vel - current_vel);
    if (e > 0 && e <= 1)
         e = 0;

    prop = e * vehicle_config._K_BRAKE_P;
    inter += e_prev * vehicle_config._K_BRAKE_I;
    diff = (e - e_prev) * vehicle_config._K_BRAKE_D;

    target_brake_stroke = prop + inter + diff;

    if(target_brake_stroke > vehicle_config._BRAKE_PEDAL_MAX){
        target_brake_stroke = vehicle_config._BRAKE_PEDAL_MAX;
    }else if(target_brake_stroke < vehicle_config._BRAKE_PEDAL_MIN){
        target_brake_stroke = vehicle_config._BRAKE_PEDAL_MIN;
    }

    e_prev = e;
    ret = target_brake_stroke;

    return ret;
}

inline void vehicle_control(double current_vel, double cmd_vel)
{
    double accel_stroke = 0;
    double brake_stroke = 0;
    int vel_offset = 1;

    if(fabs(cmd_vel) > vehicle_config.SPEED_LIMIT)
        cmd_vel = vehicle_config.SPEED_LIMIT;

    // accel
    if(fabs(cmd_vel) + vel_offset > current_vel && fabs(current_vel) != 0){
        std::cout << "[Pedal Accelerate] current_vel: " << current_vel << ", cmd_vel: " << cmd_vel << std::endl;
        accel_stroke  = accel_pid_control(current_vel, cmd_vel);
        if(accel_stroke > 0){
            set_drv_stroke(accel_stroke);
            set_break_stroke(0);
        }else{
            set_drv_stroke(0);
            set_break_stroke(-accel_stroke);
        }

    // brake
    }else if(fabs(cmd_vel) + vel_offset < current_vel && fabs(current_vel) != 0){
        std::cout << "[Pedal Brake] current_vel: " << current_vel << ", cmd_vel: " << cmd_vel << std::endl;
        brake_stroke = brake_pid_control(current_vel, cmd_vel);
        if(brake_stroke > 0){
            set_drv_stroke(0);
            set_break_stroke(brake_stroke);
        }else{
            set_drv_stroke(-brake_stroke);
            set_break_stroke(0);
        }

    // stop
    }else if(current_vel == 0. && cmd_vel == 0.){
        std::cout << "[Pedal Stop] current_vel: " << current_vel << ", cmd_vel: " << cmd_vel << std::endl;
        if(current_vel < 4.0){
            brake_stroke = vehicle_config._BRAKE_PEDAL_STOPPING_MED;
            set_drv_stroke(0);
            set_break_stroke(brake_stroke);
        }else{
            brake_stroke = brake_pid_control(current_vel, cmd_vel);
            if(brake_stroke > 0){
                set_drv_stroke(0);
                set_break_stroke(brake_stroke);
            }else{
                set_drv_stroke(-brake_stroke);
                set_break_stroke(0);
            }
        }

    // continuous
    }else if(current_vel == cmd_vel){
        std::cout << "[Pedal Continuous] current_vel: " << current_vel << ", cmd_vel: " << cmd_vel << std::endl;
        if(current_vel == 0.){
            brake_stroke = vehicle_config._BRAKE_PEDAL_STOPPING_MED;
            set_drv_stroke(0);
            set_break_stroke(brake_stroke);
        }else{
            accel_stroke  = accel_pid_control(current_vel, cmd_vel);
            set_drv_stroke(accel_stroke);
            set_break_stroke(0);
        }
    }else{
        std::cout << "[Pedal Unknown] current_vel: " << current_vel << ", cmd_vel: " << cmd_vel << std::endl;
    }
}

inline void showVehicleInfo()
{
    if (vehicle_info.control_mode == MANUAL) {
        std::cout << GREEN << "------ manual mode ------" << RESET << std::endl;
        std::cout << "Velocity : " << vehicle_info.velocity << std::endl;
    }

    else if (vehicle_info.control_mode == AUTONOMOUS) {
        std::cout << YELLOW << "------ autonomous mode ------" << RESET << std::endl;
        std::cout << "Throttle : " << vehicle_info.throttle << std::endl;
        std::cout << "Brake : " << vehicle_info.brake << std::endl;
        std::cout << "Velocity : " << vehicle_info.velocity << std::endl;
    }
    if (vehicle_info.light == ON)
        std::cout << "Light : ON" << std::endl;
    else
        std::cout << "Light : OFF" << std::endl;

    if (vehicle_info.shift == FORWARD)
        std::cout << "Shift : Forward" << std::endl;
    else
        std::cout << "Shift : Reverse" << std::endl;
    return;
}

inline char app_sigaltstack[SIGSTKSZ];

inline void app_signal_handler(int sig_num)
{
    willExit = 1;
    ros::shutdown();
}

inline int app_setup_signals()
{
    stack_t sigstack;
    struct sigaction sa;
    int ret = -1;

    sigstack.ss_sp = app_sigaltstack;
    sigstack.ss_size = SIGSTKSZ;
    sigstack.ss_flags = 0;
    if (sigaltstack(&sigstack, NULL) == -1) {
        perror("signalstack()");
        goto END;
    }

    sa.sa_handler = app_signal_handler;
    sa.sa_flags = SA_ONSTACK;
    if (sigaction(SIGINT, &sa, NULL) != 0) {
        perror("sigaction");
        goto END;
    }
    if (sigaction(SIGTERM, &sa, NULL) != 0) {
        perror("sigaction");
        goto END;
    }

    ret = 0;
END:
    return ret;
}
