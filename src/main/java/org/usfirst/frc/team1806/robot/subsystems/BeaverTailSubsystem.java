package org.usfirst.frc.team1806.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.usfirst.frc.team1806.robot.RobotMap;
import org.usfirst.frc.team1806.robot.loop.Looper;

public class BeaverTailSubsystem implements Subsystem {


    public static BeaverTailSubsystem mBeaverTailSubsystem = new BeaverTailSubsystem();
    public static  BeaverTailSubsystem getInstance(){
        return mBeaverTailSubsystem;
    }
    private DoubleSolenoid mBeaverTailFlipper;
    private DoubleSolenoid mBeaverTailEject;

    private BeaverTailSubsystem(){
        mBeaverTailFlipper = new DoubleSolenoid(RobotMap.beaverTailFlipperExtend, RobotMap.beaverTailFlipperRetract);
        mBeaverTailEject = new DoubleSolenoid(RobotMap.beaverTailEjectExtend, RobotMap.beaverTailEjectRetract);
    }

    public void writeToLog(){

    }

    public void outputToSmartDashboard(){
        SmartDashboard.putString("Beaver Tail Flipper State", mBeaverTailFlipper.get().name());
        SmartDashboard.putString("Beaver Tail Eject State", mBeaverTailEject.get().name());
    }

    public void stop(){
        mBeaverTailEject.set(DoubleSolenoid.Value.kOff);
        mBeaverTailFlipper.set(DoubleSolenoid.Value.kOff);
    }

    public void zeroSensors(){

    }

    public void registerEnabledLoops(Looper enabledLooper){

    }

    public boolean isOut(){
        return mBeaverTailFlipper.get() == DoubleSolenoid.Value.kForward;
    }

    public void extendBeaverTail(){
        mBeaverTailFlipper.set(DoubleSolenoid.Value.kForward);
    }

    public void retractBeaverTail(){
        mBeaverTailFlipper.set(DoubleSolenoid.Value.kReverse);
    }

    public boolean isEjected(){
        return mBeaverTailEject.get() == DoubleSolenoid.Value.kForward;
    }

    public void extendEjector(){
        mBeaverTailEject.set(DoubleSolenoid.Value.kForward);
    }

    public void retractEjector(){
        mBeaverTailEject.set(DoubleSolenoid.Value.kReverse);
    }
}
