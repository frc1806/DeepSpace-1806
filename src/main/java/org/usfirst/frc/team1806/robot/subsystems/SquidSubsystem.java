package org.usfirst.frc.team1806.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.usfirst.frc.team1806.robot.RobotMap;
import org.usfirst.frc.team1806.robot.loop.Looper;

public class SquidSubsystem implements Subsystem {


    private static SquidSubsystem mSquidSubsystem = new SquidSubsystem();
    private DoubleSolenoid mSquidSolenoid;


    public static SquidSubsystem getInstance(){
        return mSquidSubsystem;
    }

    private SquidSubsystem(){
            mSquidSolenoid = new DoubleSolenoid(RobotMap.squidopenport, RobotMap.squidcloseport);

    }

    public void writeToLog(){

    }

    public void outputToSmartDashboard(){
    SmartDashboard.putString("SquidState", mSquidSolenoid.get().name());
    }

    public void stop(){

    }

    public void zeroSensors(){

    }

    public boolean isOpen() {
       return mSquidSolenoid.get() == DoubleSolenoid.Value.kForward;
    }


    public void registerEnabledLoops(Looper enabledLooper){

    }
    public void openSquid(){
        mSquidSolenoid.set(DoubleSolenoid.Value.kForward);

    }
    public void closeSquid(){
        mSquidSolenoid.set(DoubleSolenoid.Value.kReverse);
    }
}
