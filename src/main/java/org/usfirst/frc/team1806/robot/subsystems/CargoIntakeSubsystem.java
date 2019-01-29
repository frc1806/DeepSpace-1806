package org.usfirst.frc.team1806.robot.subsystems;

import org.usfirst.frc.team1806.robot.Constants;
import org.usfirst.frc.team1806.robot.RobotMap;
import org.usfirst.frc.team1806.robot.loop.Looper;
import edu.wpi.first.wpilibj.DoubleSolenoid;

public class CargoIntakeSubsystem implements Subsystem {

    private static CargoIntakeSubsystem mCargoIntakeSubsystem = new CargoIntakeSubsystem();
    private DoubleSolenoid extensionSolenoid;
    private IntakeSubsystem innerIntake;
    private IntakeSubsystem outerIntake;

    public static CargoIntakeSubsystem getInstance(){
        return mCargoIntakeSubsystem;
    }

    private CargoIntakeSubsystem(){
        extensionSolenoid = new DoubleSolenoid(RobotMap.cargoIntakeExtend, RobotMap.cargoIntakeRetract);
        innerIntake = new IntakeSubsystem(Constants.kInnerIntakingSpeed, RobotMap.leftInnerIntake, RobotMap.rightInnerIntake);
        outerIntake = new IntakeSubsystem(Constants.kOuterIntakingSpeed, RobotMap.leftOuterIntake, RobotMap.rightOuterIntake);

    }


    public void writeToLog(){

    }

    public void outputToSmartDashboard(){

    }

    public void stop(){

    }

    public void zeroSensors(){

    }

    public void registerEnabledLoops(Looper enabledLooper){

    }

    public void extendOuterIntake () { extensionSolenoid.set(DoubleSolenoid.Value.kForward);
    }

    public void retractOuterIntake () { extensionSolenoid.set(DoubleSolenoid.Value.kReverse);}

    public boolean isOuterIntakeExtended() {
        return extensionSolenoid.get() == DoubleSolenoid.Value.kForward;
    }
}
