/**
 * RoboTune mecanum characterization — VEX PROS template
 *
 * Pure-axis sequence: static → forward → strafe → rotate → coast.
 * Adjust motor ports and gearing. Prefer lifted wheels for first run.
 * Output: robotune_mecanum.json on the microSD / brain filesystem.
 */
#include "main.h"
#include <fstream>
#include <cmath>

using namespace pros;

Motor fl(1, E_MOTOR_GEARSET_18, false);
Motor fr(2, E_MOTOR_GEARSET_18, true);
Motor bl(3, E_MOTOR_GEARSET_18, false);
Motor br(4, E_MOTOR_GEARSET_18, true);
Controller master(E_CONTROLLER_MASTER);

static constexpr double MAX_PCT = 45.0;
static constexpr double MAX_VEL = 200.0; // rpm-ish; tune to your setup
static bool first = true;
static std::ofstream out;

static void write_sample(double t, const char* phase,
                         double pFl, double pFr, double pBl, double pBr) {
  if (!first) out << ",";
  first = false;
  out << "\n{\"time\":" << t << ",\"phase\":\"" << phase << "\",\"batteryVoltage\":"
      << battery::get_voltage() / 1000.0 << ","
      << "\"fl\":{\"voltage\":" << (pFl / 100.0 * 12.0)
      << ",\"position\":" << fl.get_position()
      << ",\"velocity\":" << fl.get_actual_velocity() << ",\"current\":0},"
      << "\"fr\":{\"voltage\":" << (pFr / 100.0 * 12.0)
      << ",\"position\":" << fr.get_position()
      << ",\"velocity\":" << fr.get_actual_velocity() << ",\"current\":0},"
      << "\"bl\":{\"voltage\":" << (pBl / 100.0 * 12.0)
      << ",\"position\":" << bl.get_position()
      << ",\"velocity\":" << bl.get_actual_velocity() << ",\"current\":0},"
      << "\"br\":{\"voltage\":" << (pBr / 100.0 * 12.0)
      << ",\"position\":" << br.get_position()
      << ",\"velocity\":" << br.get_actual_velocity() << ",\"current\":0}}";
}

static bool safe(double elapsed) {
  if (elapsed > 14.0) return false;
  double maxV = std::max(
      std::max(std::fabs(fl.get_actual_velocity()), std::fabs(fr.get_actual_velocity())),
      std::max(std::fabs(bl.get_actual_velocity()), std::fabs(br.get_actual_velocity())));
  return maxV < MAX_VEL && !master.get_digital(E_CONTROLLER_DIGITAL_B);
}

static void set_pct(double pFl, double pFr, double pBl, double pBr) {
  fl.move_velocity(0); // ensure velocity mode not fighting
  fl.move(pFl); fr.move(pFr); bl.move(pBl); br.move(pBr);
}

static void stop_all() { set_pct(0, 0, 0, 0); }

static void command(double pFl, double pFr, double pBl, double pBr,
                    double seconds, const char* phase) {
  uint32_t start = millis();
  while (true) {
    double t = (millis() - start) / 1000.0;
    if (t >= seconds || !safe(t)) break;
    set_pct(pFl, pFr, pBl, pBr);
    write_sample((millis()) / 1000.0, phase, pFl, pFr, pBl, pBr);
    delay(10);
  }
  stop_all();
  delay(250);
}

static void ramp(double maxMag, double sFl, double sFr, double sBl, double sBr,
                 double seconds, const char* phase) {
  uint32_t start = millis();
  while (true) {
    double t = (millis() - start) / 1000.0;
    if (t >= seconds || !safe(t)) break;
    double m = maxMag * t / seconds;
    double pFl = m * sFl, pFr = m * sFr, pBl = m * sBl, pBr = m * sBr;
    set_pct(pFl, pFr, pBl, pBr);
    write_sample((millis()) / 1000.0, phase, pFl, pFr, pBl, pBr);
    delay(10);
  }
  stop_all();
  delay(300);
}

void initialize() {
  fl.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
  fr.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
  bl.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
  br.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
}

void opcontrol() {
  lcd::print(0, "RoboTune Mecanum — A start, B abort");
  while (true) {
    if (master.get_digital_new_press(E_CONTROLLER_DIGITAL_A)) {
      out.open("/usd/robotune_mecanum.json");
      first = true;
      out << "{\n\"schema\":\"robotune.characterization.v1\","
          << "\"metadata\":{\"platform\":\"VEX\",\"mechanism\":\"mecanum\","
          << "\"units\":{\"time\":\"s\",\"voltage\":\"V\",\"position\":\"deg\",\"velocity\":\"rpm\"},"
          << "\"sequence\":\"safe-mecanum-v1\"},\"samples\":[";

      for (double p = 2; p <= MAX_PCT; p += 2) {
        command(p, p, p, p, 0.18, "static_ramp");
        double avg = (std::fabs(fl.get_actual_velocity()) + std::fabs(fr.get_actual_velocity())
                      + std::fabs(bl.get_actual_velocity()) + std::fabs(br.get_actual_velocity())) / 4.0;
        if (avg > 15) break;
      }

      ramp(MAX_PCT * 0.35, 1, 1, 1, 1, 1.4, "forward_ramp");
      ramp(MAX_PCT * 0.55, 1, 1, 1, 1, 1.4, "forward_ramp");
      command(MAX_PCT * 0.3, MAX_PCT * 0.3, MAX_PCT * 0.3, MAX_PCT * 0.3, 0.9, "forward_step");
      command(MAX_PCT * 0.5, MAX_PCT * 0.5, MAX_PCT * 0.5, MAX_PCT * 0.5, 0.9, "forward_step");

      ramp(MAX_PCT * 0.35, 1, -1, -1, 1, 1.4, "strafe_ramp");
      ramp(MAX_PCT * 0.50, 1, -1, -1, 1, 1.4, "strafe_ramp");
      command(MAX_PCT * 0.3, -MAX_PCT * 0.3, -MAX_PCT * 0.3, MAX_PCT * 0.3, 0.9, "strafe_step");
      command(MAX_PCT * 0.45, -MAX_PCT * 0.45, -MAX_PCT * 0.45, MAX_PCT * 0.45, 0.9, "strafe_step");

      ramp(MAX_PCT * 0.35, 1, -1, 1, -1, 1.4, "rotate_ramp");
      ramp(MAX_PCT * 0.50, 1, -1, 1, -1, 1.4, "rotate_ramp");
      command(MAX_PCT * 0.3, -MAX_PCT * 0.3, MAX_PCT * 0.3, -MAX_PCT * 0.3, 0.9, "rotate_step");
      command(MAX_PCT * 0.45, -MAX_PCT * 0.45, MAX_PCT * 0.45, -MAX_PCT * 0.45, 0.9, "rotate_step");

      uint32_t coast0 = millis();
      while ((millis() - coast0) < 2000 && safe((millis() - coast0) / 1000.0)) {
        stop_all();
        write_sample(millis() / 1000.0, "coastdown", 0, 0, 0, 0);
        delay(10);
      }
      stop_all();
      out << "\n]}\n";
      out.close();
      lcd::print(1, "Wrote /usd/robotune_mecanum.json");
    }
    if (master.get_digital(E_CONTROLLER_DIGITAL_B)) stop_all();
    delay(20);
  }
}
