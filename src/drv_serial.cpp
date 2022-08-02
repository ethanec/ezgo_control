#include <ezgo_control/SerialPort.hpp>
#include <ezgo_control/ezgo_vehicle.hpp>


double accel_pid_control(double current_vel, double cmd_vel)
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

double brake_pid_control(double current_vel, double cmd_vel)
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

double stop_control(double current_vel)
{
    double ret;
    ret = vehicle_config._BRAKE_PEDAL_STOPPING_MED;
    return ret;
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


void PedalControl(double current_vel, double cmd_vel)
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
            brake_stroke = stop_control(current_vel);
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
            brake_stroke = stop_control(current_vel);
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
