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

public class RoboTuneCharacterization extends TimedRobot {
    private final PWMSparkMax motor = new PWMSparkMax(0); // replace with your controller
    private final Encoder encoder = new Encoder(0,1);     // replace/configure for your encoder
    private final XboxController driver = new XboxController(0);

    private static final double MAX_OUTPUT=.50;
    private static final double MAX_VEL=5000;
    private static final double MAX_TEST=8.0;
    private FileWriter out;
    private boolean first=true;

    private void write(String s){try{out.write(s);}catch(IOException e){abort();}}
    private void sample(double t,String phase,double output){
        if(!first) write(",");
        first=false;
        double pos=encoder.getDistance(), vel=encoder.getRate();
        write(String.format(Locale.US,
            "\n{\"time\":%.4f,\"voltage\":%.4f,\"position\":%.5f,\"velocity\":%.5f,\"current\":0,\"batteryVoltage\":%.4f,\"phase\":\"%s\"}",
            t,output*12.0,pos,vel,12.0,phase));
    }
    private boolean safe(double elapsed){
        return elapsed<MAX_TEST && Math.abs(encoder.getRate())<MAX_VEL && !driver.getBButton();
    }
    private void abort(){motor.stopMotor();}

    private void command(double output,double seconds,String phase){
        Timer timer=new Timer(); timer.start();
        while(timer.get()<seconds && safe(timer.get())){
            motor.set(output);
            sample(timer.get(),phase,output);
            Timer.delay(.01);
        }
        motor.stopMotor(); Timer.delay(.25);
    }
    private void ramp(double max,double seconds,String phase){
        Timer timer=new Timer(); timer.start();
        while(timer.get()<seconds && safe(timer.get())){
            double p=max*timer.get()/seconds;
            motor.set(p);
            sample(timer.get(),phase,p);
            Timer.delay(.01);
        }
        motor.stopMotor(); Timer.delay(.30);
    }

    @Override public void robotInit(){
        encoder.reset();
        try{
            out=new FileWriter("/home/lvuser/robotune.json",false);
            write("{\n\"schema\":\"robotune.characterization.v1\",");
            write("\"metadata\":{\"platform\":\"FRC\",\"mechanism\":\"velocity\",");
            write("\"units\":{\"time\":\"s\",\"voltage\":\"V\",\"position\":\"encoder_units\",\"velocity\":\"encoder_units/s\"},");
            write("\"sequence\":\"safe-velocity-v1\"},\"samples\":[");
        }catch(IOException e){ DriverStation.reportError("RoboTune file open failed",e.getStackTrace());}
    }

    @Override public void teleopPeriodic(){
        // A starts the sequence; B aborts. Keep this test program out of match code.
        if(driver.getAButtonPressed()){
            encoder.reset();
            for(double p=.025;p<=MAX_OUTPUT;p+=.0125){ command(p,.20,"static_ramp"); if(Math.abs(encoder.getRate())>50)break; }
            ramp(MAX_OUTPUT*.25,1.5,"ramp_25");
            ramp(MAX_OUTPUT*.50,1.5,"ramp_50");
            ramp(MAX_OUTPUT*.75,1.5,"ramp_75");
            command(MAX_OUTPUT*.25,1.0,"step_25");
            command(MAX_OUTPUT*.50,1.0,"step_50");
            command(MAX_OUTPUT*.75,1.0,"step_75");
            motor.stopMotor();
            Timer timer=new Timer(); timer.start();
            while(timer.get()<2.0 && safe(timer.get())){sample(timer.get(),"coastdown",0);Timer.delay(.01);}
            motor.stopMotor();
            write("\n]}\n");
            try{out.close();}catch(IOException ignored){}
        }
        if(driver.getBButtonPressed()) abort();
    }
}
