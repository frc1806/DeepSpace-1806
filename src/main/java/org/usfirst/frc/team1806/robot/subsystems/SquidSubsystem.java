package org.usfirst.frc.team1806.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.SensorUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.usfirst.frc.team1806.robot.RobotMap;
import org.usfirst.frc.team1806.robot.loop.Looper;

public class SquidSubsystem implements Subsystem {


    private static SquidSubsystem mSquidSubsystem = new SquidSubsystem();
    private DoubleSolenoid mSquidSolenoid;
    private DoubleSolenoid mSquidExtender;
    private DigitalInput mHatchDetector;

    public static SquidSubsystem getInstance(){
        return mSquidSubsystem;
    }

    private SquidSubsystem(){
            mSquidSolenoid = new DoubleSolenoid(RobotMap.squidOpenPort, RobotMap.squidClosePort);
            mSquidExtender = new DoubleSolenoid(RobotMap.squidExtendForward, RobotMap.squidExtendBackward);
            mHatchDetector = new DigitalInput(RobotMap.hatchDetector);
    }

    public void writeToLog(){

    }

    public void outputToSmartDashboard(){
    SmartDashboard.putString("Squid State", mSquidSolenoid.get().name());
    SmartDashboard.putString("Squid Extend" , mSquidExtender.get().name());

    }

    public void stop(){

    }

    public void zeroSensors(){

    }

    public boolean isOpen() {
        return mSquidSolenoid.get() == DoubleSolenoid.Value.kForward;
    }

    public boolean isDetectingHatch(){
        return mHatchDetector.get();
    }


    public void registerEnabledLoops(Looper enabledLooper){

    }
    public void openSquid(){
        mSquidSolenoid.set(DoubleSolenoid.Value.kForward);

    }
    public void closeSquid(){
        mSquidSolenoid.set(DoubleSolenoid.Value.kReverse);
    }

    public void extendSquid() {
        mSquidExtender.set(DoubleSolenoid.Value.kForward);
    }

    public void retractSquid () {
        mSquidExtender.set(DoubleSolenoid.Value.kReverse);
    }
    public boolean isExtended() {
        return mSquidExtender.get() == DoubleSolenoid.Value.kForward;
    }

    public void openSquidIfHatch(){
        if (isDetectingHatch()){
            openSquid();
        } else if (isOpen()){
            closeSquid();
        }
    }
}

