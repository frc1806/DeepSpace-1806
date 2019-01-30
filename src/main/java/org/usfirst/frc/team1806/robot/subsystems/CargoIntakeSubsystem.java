package org.usfirst.frc.team1806.robot.subsystems;

import jdk.internal.org.objectweb.asm.tree.InnerClassNode;
import org.usfirst.frc.team1806.robot.Constants;
import org.usfirst.frc.team1806.robot.RobotMap;
import org.usfirst.frc.team1806.robot.loop.Looper;
import edu.wpi.first.wpilibj.DoubleSolenoid;

public class CargoIntakeSubsystem implements Subsystem {

    private static CargoIntakeSubsystem mCargoIntakeSubsystem = new CargoIntakeSubsystem();
    private DoubleSolenoid extensionSolenoid;
    private IntakeSubsystem innerIntake;
    private IntakeSubsystem outerIntake;

    public enum ScoringPower {
        SLOW(.2),
        MEDIUM(.4),
        FAST(.6),
        IRRESPONSIBLE(.8),
        PLAID(1.0);


        Double power;

        ScoringPower(Double power) {
            this.power = power;
        }

        Double getPower() {
            return power;
        }
    }

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
        //TODO
    }

    public void stop(){
        innerIntake.stop();
        outerIntake.stop();
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

    public void intakeCargo(){
    innerIntake.intakeLeftSide(Constants.kInnerIntakingSpeed);
    innerIntake.intakeRightSide(Constants.kInnerIntakingSpeed);
    outerIntake.intakeLeftSide(Constants.kOuterIntakingSpeed);
    outerIntake.intakeRightSide(Constants.kOuterIntakingSpeed);
    }

    public void scoreCargo(ScoringPower power){
    innerIntake.outtaking(power.getPower());
    outerIntake.stop();

    }
}
