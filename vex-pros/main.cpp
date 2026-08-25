#include "main.h"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"
#include "pros/misc.hpp"
#include <fstream>
#include <cmath>
#include <iomanip>
#include <sstream>

pros::Motor motor(1, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_COUNTS);
pros::Controller master(pros::E_CONTROLLER_MASTER);

static constexpr double DT = 0.01;
static constexpr int MAX_MV = 6000;          // 50% of 12 V
static constexpr double MAX_RUN_S = 8.0;
static constexpr double MAX_VEL = 2000.0;    // encoder units/s; tune for your mechanism
static constexpr double MIN_BATTERY_MV = 7000.0;

std::ofstream out;
bool abortRequested=false;

void writeHeader() {
  out << "{\n"
      << "  \"schema\":\"robotune.characterization.v1\",\n"
      << "  \"metadata\":{\"platform\":\"VEX\",\"mechanism\":\"velocity\","
      << "\"units\":{\"time\":\"s\",\"voltage\":\"mV\",\"position\":\"ticks\","
      << "\"velocity\":\"ticks/s\"},\"sequence\":\"safe-velocity-v1\"},\n"
      << "  \"samples\":[\n";
}
bool firstSample=true;
void logSample(double t, const char* phase, int mv) {
  if(!firstSample) out << ",\n"; firstSample=false;
  out << std::fixed << std::setprecision(4)
      << "    {\"time\":" << t
      << ",\"voltage\":" << mv/1000.0
      << ",\"position\":" << motor.get_position()
      << ",\"velocity\":" << motor.get_actual_velocity()
      << ",\"current\":" << motor.get_current_draw()
      << ",\"batteryVoltage\":" << pros::battery::get_voltage()
      << ",\"phase\":\"" << phase << "\"}";
}
bool safe(double elapsed) {
  if(elapsed > MAX_RUN_S) return false;
  if(pros::battery::get_voltage() < MIN_BATTERY_MV) return false;
  if(std::abs(motor.get_actual_velocity()) > MAX_VEL) return false;
  if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) return false;
  return true;
}
void runCommand(int mv, double seconds, const char* phase) {
  const double start=pros::millis()/1000.0;
  while(true) {
    double now=pros::millis()/1000.0, t=now-start;
    if(t>=seconds || !safe(t)) break;
    motor.move_voltage(mv);
    logSample(t,phase,mv);
    pros::delay(10);
  }
  motor.move_voltage(0);
  pros::delay(250);
}
void ramp(int maxMv, double seconds, const char* phase) {
  const double start=pros::millis()/1000.0;
  while(true) {
    double now=pros::millis()/1000.0,t=now-start;
    if(t>=seconds || !safe(t)) break;
    int mv=(int)(maxMv*t/seconds);
    motor.move_voltage(mv);
    logSample(t,phase,mv);
    pros::delay(10);
  }
  motor.move_voltage(0);
  pros::delay(300);
}
void characterize() {
  if(pros::battery::get_voltage()<MIN_BATTERY_MV) return;
  out.open("/usd/robotune.json");
  if(!out.is_open()) return;
  writeHeader();

  // Operator must press A to begin; B aborts.
  while(!master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
    master.print(0,0,"A=start B=abort");
    pros::delay(20);
    if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) { out.close(); return; }
  }

  motor.tare_position();

  // Very low-power static friction search.
  for(int mv=300; mv<=MAX_MV; mv+=150) {
    runCommand(mv,0.20,"static_ramp");
    if(motor.get_actual_velocity()>50) break;
  }

  // Progressive ramps.
  ramp((int)(MAX_MV*.25),1.5,"ramp_25");
  ramp((int)(MAX_MV*.50),1.5,"ramp_50");
  ramp((int)(MAX_MV*.75),1.5,"ramp_75");

  // Step responses.
  runCommand((int)(MAX_MV*.25),1.0,"step_25");
  runCommand((int)(MAX_MV*.50),1.0,"step_50");
  runCommand((int)(MAX_MV*.75),1.0,"step_75");

  // Coast-down with zero command.
  motor.move_voltage(0);
  for(int i=0;i<200;i++) {
    if(!safe(i*DT)) break;
    logSample(i*DT,"coastdown",0);
    pros::delay(10);
  }

  motor.move_voltage(0);
  out << "\n  ]\n}\n";
  out.close();
}
void initialize(){ pros::lcd::initialize(); }
void disabled(){}
void competition_initialize(){}
void autonomous(){}
void opcontrol(){ characterize(); }
