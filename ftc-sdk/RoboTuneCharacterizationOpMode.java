package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Locale;

@TeleOp(name="RoboTune Characterization", group="RoboTune")
public class RoboTuneCharacterizationOpMode extends LinearOpMode {
    private DcMotorEx motor;
    private static final double DT = 0.01;
    private static final double MAX_POWER = 0.50;
    private static final double MAX_SECONDS = 8.0;
    private static final double MAX_TICKS_PER_SEC = 5000.0;
    private FileWriter out;
    private boolean first = true;

    private void sample(long t0, String phase, double power) {
        double t=(System.nanoTime()-t0)/1e9;
        double pos=motor.getCurrentPosition();
        double vel=motor.getVelocity();
        if(!first) write(",");
        first=false;
        write(String.format(Locale.US,
            "\n{\"time\":%.4f,\"voltage\":%.4f,\"position\":%.3f,\"velocity\":%.3f,\"current\":%.3f,\"batteryVoltage\":0,\"phase\":\"%s\"}",
            t,power*12.0,pos,vel,phase));
    }
    private void write(String s) { try { out.write(s); } catch(IOException e) { requestOpModeStop(); } }

    private boolean safe(double t) {
        return t < MAX_SECONDS && Math.abs(motor.getVelocity()) < MAX_TICKS_PER_SEC && opModeIsActive();
    }

    private void command(double power,double seconds,String phase) {
        long start=System.nanoTime();
        while(opModeIsActive()) {
            double t=(System.nanoTime()-start)/1e9;
            if(t>=seconds || !safe(t)) break;
            motor.setPower(power);
            sample(start,phase,power);
            sleep(10);
        }
        motor.setPower(0);
        sleep(250);
    }

    private void ramp(double maxPower,double seconds,String phase) {
        long start=System.nanoTime();
        while(opModeIsActive()) {
            double t=(System.nanoTime()-start)/1e9;
            if(t>=seconds || !safe(t)) break;
            double p=maxPower*t/seconds;
            motor.setPower(p);
            sample(start,phase,p);
            sleep(10);
        }
        motor.setPower(0); sleep(300);
    }

    @Override public void runOpMode() throws InterruptedException {
        motor=hardwareMap.get(DcMotorEx.class,"tuneMotor");
        motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        File file=new File(hardwareMap.appContext.getFilesDir(),"robotune.json");
        try {
            out=new FileWriter(file,false);
            write("{\n\"schema\":\"robotune.characterization.v1\",");
            write("\"metadata\":{\"platform\":\"FTC\",\"mechanism\":\"velocity\",");
            write("\"units\":{\"time\":\"s\",\"voltage\":\"V\",\"position\":\"ticks\",\"velocity\":\"ticks/s\"},");
            write("\"sequence\":\"safe-velocity-v1\"},\"samples\":[");
        } catch(IOException e) { telemetry.addLine("Cannot open output"); telemetry.update(); return; }

        telemetry.addLine("ROBOTUNE: verify stand, limits, and E-stop.");
        telemetry.addLine("Press START to begin; STOP aborts.");
        telemetry.update();
        waitForStart();
        if(isStopRequested()) return;

        motor.setPower(0);
        motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        for(double p=.025;p<=MAX_POWER;p+=.0125) {
            command(p,.20,"static_ramp");
            if(Math.abs(motor.getVelocity())>50) break;
        }
        ramp(MAX_POWER*.25,1.5,"ramp_25");
        ramp(MAX_POWER*.50,1.5,"ramp_50");
        ramp(MAX_POWER*.75,1.5,"ramp_75");
        command(MAX_POWER*.25,1.0,"step_25");
        command(MAX_POWER*.50,1.0,"step_50");
        command(MAX_POWER*.75,1.0,"step_75");

        motor.setPower(0);
        long coast=System.nanoTime();
        for(int i=0;i<200 && safe(i*DT);i++) {
            sample(coast,"coastdown",0);
            sleep(10);
        }

        motor.setPower(0);
        write("\n]}\n");
        try { out.close(); } catch(IOException ignored) {}
        telemetry.addData("Saved",file.getAbsolutePath());
        telemetry.update();
        sleep(2000);
    }
}
