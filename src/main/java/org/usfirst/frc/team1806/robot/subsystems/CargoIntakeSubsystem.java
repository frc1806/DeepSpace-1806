package org.usfirst.frc.team1806.robot.subsystems;

//import jdk.internal.org.objectweb.asm.tree.InnerClassNode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.usfirst.frc.team1806.robot.Constants;
import org.usfirst.frc.team1806.robot.RobotMap;
import org.usfirst.frc.team1806.robot.loop.Looper;
import edu.wpi.first.wpilibj.DoubleSolenoid;

public class CargoIntakeSubsystem implements Subsystem {

    private static CargoIntakeSubsystem mCargoIntakeSubsystem = new CargoIntakeSubsystem();
    private LiftSubsystem liftSubsystem;
    private DoubleSolenoid extensionSolenoid;
    private IntakeSubsystem innerIntake;
    //private IntakeSubsystem outerIntake;

    /**
     * Different power percentages for scoring a cargo
     */
    public enum ScoringPower {
        SLOW(.2),
        MEDIUM(.4),
        FAST(.6),
        IRRESPONSIBLE(.8),
        PLAID(1.0);


        Double power;

        /** Constructs a ScoringPower
         *
         * @param power this is the power value to use while using the Intake outwards to score the cargo
          (in percentages from 0.0 to 1.0)
         */
        ScoringPower(Double power) {
            this.power = power;
        }

        /**
         *
         * @return the power of scoring the cargo
         */
        Double getPower() {
            return power;
        }
    }

    public static CargoIntakeSubsystem getInstance(){
        return mCargoIntakeSubsystem;
    }
    
    private CargoIntakeSubsystem(){

        extensionSolenoid = new DoubleSolenoid(RobotMap.cargoIntakeExtend, RobotMap.cargoIntakeRetract);
        innerIntake = new IntakeSubsystem(Constants.kInnerIntakingSpeed, RobotMap.leftInnerIntake, RobotMap.rightInnerIntake, false, false);
        //outerIntake = new IntakeSubsystem(Constants.kOuterIntakingSpeed, RobotMap.leftOuterIntake, RobotMap.rightOuterIntake, false, false);
        liftSubsystem = LiftSubsystem.getInstance();
    }


    public void writeToLog(){

    }

    public void outputToSmartDashboard(){
        SmartDashboard.putBoolean("OuterIntakeExtend", isOuterIntakeExtended());
    }


    public void stop(){
        innerIntake.stop();
       // outerIntake.stop();
    }

    public void zeroSensors(){

    }

    public void registerEnabledLoops(Looper enabledLooper){

    }

    public void extendOuterIntake () {
        extensionSolenoid.set(DoubleSolenoid.Value.kForward);

    }

    public void retractOuterIntake () {
        extensionSolenoid.set(DoubleSolenoid.Value.kReverse);}

    public boolean isOuterIntakeExtended() {
        return extensionSolenoid.get() == DoubleSolenoid.Value.kForward;
    }

    /**
     * function to enable the intake system
     */
    public void intakeCargo(){
    innerIntake.intakeLeftSide(Constants.kInnerIntakingSpeed);
    innerIntake.intakeRightSide(Constants.kInnerIntakingSpeed);
    if(isOuterIntakeExtended()){
        //outerIntake.intakeLeftSide(Constants.kOuterIntakingSpeed);
       // outerIntake.intakeRightSide(Constants.kOuterIntakingSpeed);
    }
    else{
        //outerIntake.stop();
    }

    }

    /**
     * Reverses the inner intake to shoot the ball.
     * @param power how much power should be used to shoot the ball.
     */
    public void scoreCargo(ScoringPower power){
    innerIntake.outtaking(power.getPower());
    //outerIntake.stop();

    }

    /**
     * when HatchMode is enabled it retracts outer intake
     */
    public void goToHatchMode(){
            retractOuterIntake();
    }

    public void goToCargoMode(){
        //TODO
    }

    public void retractAll() {
        retractOuterIntake(); //lift will handle making sure there's no problem doing this
    }
}