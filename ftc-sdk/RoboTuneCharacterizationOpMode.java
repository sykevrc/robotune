package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Locale;

/**
 * RoboTune mecanum characterization.
 *
 * Hardware map names (change to match your config):
 *   frontLeft, frontRight, backLeft, backRight
 *
 * Sequence (pure-axis, safe limits):
 *   1. Static friction ramp (all wheels same sign)
 *   2. Forward ramps + steps
 *   3. Strafe ramps + steps  (mecanum signs)
 *   4. Rotate ramps + steps
 *   5. Coast-down
 *
 * Safety: START begins, STOP aborts. Keep MAX_POWER ≤ 0.5 until envelope is known.
 * Prefer first runs with wheels lifted.
 */
@TeleOp(name = "RoboTune Mecanum Characterization", group = "RoboTune")
public class RoboTuneCharacterizationOpMode extends LinearOpMode {
    private DcMotorEx fl, fr, bl, br;
    private static final double DT = 0.01;
    private static final double MAX_POWER = 0.45;
    private static final double MAX_SECONDS = 12.0;
    private static final double MAX_TICKS_PER_SEC = 2800.0;
    private FileWriter out;
    private boolean first = true;
    private long globalT0;

    private void write(String s) {
        try { out.write(s); } catch (IOException e) { requestOpModeStop(); }
    }

    private void sample(String phase, double pFl, double pFr, double pBl, double pBr) {
        double t = (System.nanoTime() - globalT0) / 1e9;
        if (!first) write(",");
        first = false;
        write(String.format(Locale.US,
            "\n{\"time\":%.4f,\"phase\":\"%s\",\"batteryVoltage\":0," +
            "\"fl\":{\"voltage\":%.4f,\"position\":%.1f,\"velocity\":%.2f,\"current\":0}," +
            "\"fr\":{\"voltage\":%.4f,\"position\":%.1f,\"velocity\":%.2f,\"current\":0}," +
            "\"bl\":{\"voltage\":%.4f,\"position\":%.1f,\"velocity\":%.2f,\"current\":0}," +
            "\"br\":{\"voltage\":%.4f,\"position\":%.1f,\"velocity\":%.2f,\"current\":0}}",
            t, phase,
            pFl * 12.0, (double) fl.getCurrentPosition(), fl.getVelocity(),
            pFr * 12.0, (double) fr.getCurrentPosition(), fr.getVelocity(),
            pBl * 12.0, (double) bl.getCurrentPosition(), bl.getVelocity(),
            pBr * 12.0, (double) br.getCurrentPosition(), br.getVelocity()
        ));
    }

    private boolean safe(double t) {
        if (!opModeIsActive() || t > MAX_SECONDS) return false;
        double maxV = Math.max(
            Math.max(Math.abs(fl.getVelocity()), Math.abs(fr.getVelocity())),
            Math.max(Math.abs(bl.getVelocity()), Math.abs(br.getVelocity()))
        );
        return maxV < MAX_TICKS_PER_SEC;
    }

    private void setPowers(double pFl, double pFr, double pBl, double pBr) {
        fl.setPower(pFl);
        fr.setPower(pFr);
        bl.setPower(pBl);
        br.setPower(pBr);
    }

    private void stopAll() {
        setPowers(0, 0, 0, 0);
    }

    /** Hold constant wheel powers for `seconds`. */
    private void command(double pFl, double pFr, double pBl, double pBr, double seconds, String phase) {
        long start = System.nanoTime();
        while (opModeIsActive()) {
            double t = (System.nanoTime() - start) / 1e9;
            if (t >= seconds || !safe(t)) break;
            setPowers(pFl, pFr, pBl, pBr);
            sample(phase, pFl, pFr, pBl, pBr);
            sleep(10);
        }
        stopAll();
        sleep(250);
    }

    /** Linear ramp of magnitude from 0 → max over `seconds`. */
    private void ramp(double maxMag, double sFl, double sFr, double sBl, double sBr,
                      double seconds, String phase) {
        long start = System.nanoTime();
        while (opModeIsActive()) {
            double t = (System.nanoTime() - start) / 1e9;
            if (t >= seconds || !safe(t)) break;
            double m = maxMag * t / seconds;
            double pFl = m * sFl, pFr = m * sFr, pBl = m * sBl, pBr = m * sBr;
            setPowers(pFl, pFr, pBl, pBr);
            sample(phase, pFl, pFr, pBl, pBr);
            sleep(10);
        }
        stopAll();
        sleep(300);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        fl = hardwareMap.get(DcMotorEx.class, "frontLeft");
        fr = hardwareMap.get(DcMotorEx.class, "frontRight");
        bl = hardwareMap.get(DcMotorEx.class, "backLeft");
        br = hardwareMap.get(DcMotorEx.class, "backRight");

        // Adjust directions for your robot so +power = forward for all when testing forward axis
        fl.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.REVERSE);
        fr.setDirection(DcMotor.Direction.FORWARD);
        br.setDirection(DcMotor.Direction.FORWARD);

        for (DcMotorEx m : new DcMotorEx[]{fl, fr, bl, br}) {
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        File file = new File(hardwareMap.appContext.getFilesDir(), "robotune_mecanum.json");
        try {
            out = new FileWriter(file, false);
            write("{\n\"schema\":\"robotune.characterization.v1\",");
            write("\"metadata\":{\"platform\":\"FTC\",\"mechanism\":\"mecanum\",");
            write("\"units\":{\"time\":\"s\",\"voltage\":\"V\",\"position\":\"ticks\",\"velocity\":\"ticks/s\"},");
            write("\"sequence\":\"safe-mecanum-v1\"},\"samples\":[");
        } catch (IOException e) {
            telemetry.addLine("Cannot open output file");
            telemetry.update();
            return;
        }

        telemetry.addLine("ROBOTUNE MECANUM");
        telemetry.addLine("Prefer wheels lifted for first run.");
        telemetry.addLine("Pure axes: forward → strafe → rotate");
        telemetry.addLine("Press START to begin; STOP aborts.");
        telemetry.update();
        waitForStart();
        if (isStopRequested()) return;

        globalT0 = System.nanoTime();

        // 1) Static friction — forward polarity
        for (double p = 0.02; p <= MAX_POWER; p += 0.02) {
            command(p, p, p, p, 0.18, "static_ramp");
            double avg = (Math.abs(fl.getVelocity()) + Math.abs(fr.getVelocity())
                + Math.abs(bl.getVelocity()) + Math.abs(br.getVelocity())) / 4.0;
            if (avg > 40) break;
        }

        // 2) Forward
        ramp(MAX_POWER * 0.35, 1, 1, 1, 1, 1.4, "forward_ramp");
        ramp(MAX_POWER * 0.55, 1, 1, 1, 1, 1.4, "forward_ramp");
        command(MAX_POWER * 0.30, MAX_POWER * 0.30, MAX_POWER * 0.30, MAX_POWER * 0.30, 0.9, "forward_step");
        command(MAX_POWER * 0.50, MAX_POWER * 0.50, MAX_POWER * 0.50, MAX_POWER * 0.50, 0.9, "forward_step");

        // 3) Strafe (classic mecanum: FL+, FR-, BL-, BR+)
        ramp(MAX_POWER * 0.35, 1, -1, -1, 1, 1.4, "strafe_ramp");
        ramp(MAX_POWER * 0.50, 1, -1, -1, 1, 1.4, "strafe_ramp");
        command(MAX_POWER * 0.30, -MAX_POWER * 0.30, -MAX_POWER * 0.30, MAX_POWER * 0.30, 0.9, "strafe_step");
        command(MAX_POWER * 0.45, -MAX_POWER * 0.45, -MAX_POWER * 0.45, MAX_POWER * 0.45, 0.9, "strafe_step");

        // 4) Rotate (left+, right-)
        ramp(MAX_POWER * 0.35, 1, -1, 1, -1, 1.4, "rotate_ramp");
        ramp(MAX_POWER * 0.50, 1, -1, 1, -1, 1.4, "rotate_ramp");
        command(MAX_POWER * 0.30, -MAX_POWER * 0.30, MAX_POWER * 0.30, -MAX_POWER * 0.30, 0.9, "rotate_step");
        command(MAX_POWER * 0.45, -MAX_POWER * 0.45, MAX_POWER * 0.45, -MAX_POWER * 0.45, 0.9, "rotate_step");

        // 5) Coast-down
        long coastStart = System.nanoTime();
        while (opModeIsActive()) {
            double t = (System.nanoTime() - coastStart) / 1e9;
            if (t > 2.0 || !safe(t)) break;
            stopAll();
            sample("coastdown", 0, 0, 0, 0);
            sleep(10);
        }

        stopAll();
        write("\n]}\n");
        try { out.close(); } catch (IOException ignored) {}
        telemetry.addLine("Done. File: robotune_mecanum.json");
        telemetry.addLine("Pull via Device File Explorer or adb.");
        telemetry.update();
        while (opModeIsActive()) sleep(50);
    }
}
