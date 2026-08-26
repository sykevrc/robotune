package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.DriverStation;

import java.io.FileWriter;
import java.io.IOException;
import java.util.Locale;

/**
 * RoboTune mecanum characterization template (FRC / WPILib).
 * Replace motor controllers and encoders with your hardware.
 * Sequence: static → forward → strafe → rotate → coast.
 * A starts, B aborts. Prefer lifted wheels for first runs.
 */
public class RoboTuneCharacterization extends TimedRobot {
    private final PWMSparkMax fl = new PWMSparkMax(0);
    private final PWMSparkMax fr = new PWMSparkMax(1);
    private final PWMSparkMax bl = new PWMSparkMax(2);
    private final PWMSparkMax br = new PWMSparkMax(3);
    private final Encoder flEnc = new Encoder(0, 1);
    private final Encoder frEnc = new Encoder(2, 3);
    private final Encoder blEnc = new Encoder(4, 5);
    private final Encoder brEnc = new Encoder(6, 7);
    private final XboxController driver = new XboxController(0);

    private static final double MAX_OUTPUT = 0.45;
    private static final double MAX_VEL = 4000;
    private static final double MAX_TEST = 14.0;
    private FileWriter out;
    private boolean first = true;
    private double t0;

    private void write(String s) {
        try { out.write(s); } catch (IOException e) { abort(); }
    }

    private void sample(String phase, double pFl, double pFr, double pBl, double pBr) {
        double t = Timer.getFPGATimestamp() - t0;
        if (!first) write(",");
        first = false;
        write(String.format(Locale.US,
            "\n{\"time\":%.4f,\"phase\":\"%s\",\"batteryVoltage\":12.0," +
            "\"fl\":{\"voltage\":%.4f,\"position\":%.3f,\"velocity\":%.3f,\"current\":0}," +
            "\"fr\":{\"voltage\":%.4f,\"position\":%.3f,\"velocity\":%.3f,\"current\":0}," +
            "\"bl\":{\"voltage\":%.4f,\"position\":%.3f,\"velocity\":%.3f,\"current\":0}," +
            "\"br\":{\"voltage\":%.4f,\"position\":%.3f,\"velocity\":%.3f,\"current\":0}}",
            t, phase,
            pFl * 12.0, flEnc.getDistance(), flEnc.getRate(),
            pFr * 12.0, frEnc.getDistance(), frEnc.getRate(),
            pBl * 12.0, blEnc.getDistance(), blEnc.getRate(),
            pBr * 12.0, brEnc.getDistance(), brEnc.getRate()
        ));
    }

    private boolean safe(double elapsed) {
        double maxV = Math.max(
            Math.max(Math.abs(flEnc.getRate()), Math.abs(frEnc.getRate())),
            Math.max(Math.abs(blEnc.getRate()), Math.abs(brEnc.getRate()))
        );
        return elapsed < MAX_TEST && maxV < MAX_VEL && !driver.getBButton();
    }

    private void set(double pFl, double pFr, double pBl, double pBr) {
        fl.set(pFl); fr.set(pFr); bl.set(pBl); br.set(pBr);
    }

    private void abort() {
        set(0, 0, 0, 0);
    }

    private void command(double pFl, double pFr, double pBl, double pBr, double seconds, String phase) {
        Timer timer = new Timer();
        timer.start();
        while (timer.get() < seconds && safe(timer.get())) {
            set(pFl, pFr, pBl, pBr);
            sample(phase, pFl, pFr, pBl, pBr);
            Timer.delay(0.01);
        }
        abort();
        Timer.delay(0.25);
    }

    private void ramp(double maxMag, double sFl, double sFr, double sBl, double sBr,
                      double seconds, String phase) {
        Timer timer = new Timer();
        timer.start();
        while (timer.get() < seconds && safe(timer.get())) {
            double m = maxMag * timer.get() / seconds;
            double pFl = m * sFl, pFr = m * sFr, pBl = m * sBl, pBr = m * sBr;
            set(pFl, pFr, pBl, pBr);
            sample(phase, pFl, pFr, pBl, pBr);
            Timer.delay(0.01);
        }
        abort();
        Timer.delay(0.30);
    }

    @Override
    public void robotInit() {
        try {
            out = new FileWriter("/home/lvuser/robotune_mecanum.json", false);
            write("{\n\"schema\":\"robotune.characterization.v1\",");
            write("\"metadata\":{\"platform\":\"FRC\",\"mechanism\":\"mecanum\",");
            write("\"units\":{\"time\":\"s\",\"voltage\":\"V\",\"position\":\"encoder_units\",\"velocity\":\"encoder_units/s\"},");
            write("\"sequence\":\"safe-mecanum-v1\"},\"samples\":[");
        } catch (IOException e) {
            DriverStation.reportError("RoboTune file open failed", e.getStackTrace());
        }
    }

    @Override
    public void teleopPeriodic() {
        if (driver.getAButtonPressed()) {
            t0 = Timer.getFPGATimestamp();
            first = true;
            flEnc.reset(); frEnc.reset(); blEnc.reset(); brEnc.reset();

            for (double p = 0.02; p <= MAX_OUTPUT; p += 0.02) {
                command(p, p, p, p, 0.18, "static_ramp");
                double avg = (Math.abs(flEnc.getRate()) + Math.abs(frEnc.getRate())
                    + Math.abs(blEnc.getRate()) + Math.abs(brEnc.getRate())) / 4.0;
                if (avg > 40) break;
            }

            ramp(MAX_OUTPUT * 0.35, 1, 1, 1, 1, 1.4, "forward_ramp");
            ramp(MAX_OUTPUT * 0.55, 1, 1, 1, 1, 1.4, "forward_ramp");
            command(MAX_OUTPUT * 0.3, MAX_OUTPUT * 0.3, MAX_OUTPUT * 0.3, MAX_OUTPUT * 0.3, 0.9, "forward_step");
            command(MAX_OUTPUT * 0.5, MAX_OUTPUT * 0.5, MAX_OUTPUT * 0.5, MAX_OUTPUT * 0.5, 0.9, "forward_step");

            ramp(MAX_OUTPUT * 0.35, 1, -1, -1, 1, 1.4, "strafe_ramp");
            ramp(MAX_OUTPUT * 0.50, 1, -1, -1, 1, 1.4, "strafe_ramp");
            command(MAX_OUTPUT * 0.3, -MAX_OUTPUT * 0.3, -MAX_OUTPUT * 0.3, MAX_OUTPUT * 0.3, 0.9, "strafe_step");
            command(MAX_OUTPUT * 0.45, -MAX_OUTPUT * 0.45, -MAX_OUTPUT * 0.45, MAX_OUTPUT * 0.45, 0.9, "strafe_step");

            ramp(MAX_OUTPUT * 0.35, 1, -1, 1, -1, 1.4, "rotate_ramp");
            ramp(MAX_OUTPUT * 0.50, 1, -1, 1, -1, 1.4, "rotate_ramp");
            command(MAX_OUTPUT * 0.3, -MAX_OUTPUT * 0.3, MAX_OUTPUT * 0.3, -MAX_OUTPUT * 0.3, 0.9, "rotate_step");
            command(MAX_OUTPUT * 0.45, -MAX_OUTPUT * 0.45, MAX_OUTPUT * 0.45, -MAX_OUTPUT * 0.45, 0.9, "rotate_step");

            Timer timer = new Timer();
            timer.start();
            while (timer.get() < 2.0 && safe(timer.get())) {
                abort();
                sample("coastdown", 0, 0, 0, 0);
                Timer.delay(0.01);
            }
            abort();
            write("\n]}\n");
            try { out.close(); } catch (IOException ignored) {}
        }
        if (driver.getBButtonPressed()) abort();
    }
}
