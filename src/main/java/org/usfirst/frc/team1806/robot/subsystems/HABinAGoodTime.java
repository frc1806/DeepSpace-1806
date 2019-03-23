package org.usfirst.frc.team1806.robot.subsystems;

import com.revrobotics.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.usfirst.frc.team1806.robot.Constants;
import org.usfirst.frc.team1806.robot.RobotMap;
import org.usfirst.frc.team1806.robot.loop.Loop;
import org.usfirst.frc.team1806.robot.loop.Looper;
//import com.revrobotics.CANSparkMax;

public class HABinAGoodTime implements Subsystem {

    private static HABinAGoodTime mHabANiceDay = new HABinAGoodTime();

    private CANSparkMax leftHABArm, rightHABArm;
    private DriveTrainSubsystem mDriveTrainSubsystem;

    private ClimbPosition mClimbPosition;
    private ClimbStates mClimbStates;

    private double avgClimberHeight;
    private boolean isManual = false;

    public static HABinAGoodTime getInstance(){
        return mHabANiceDay;
    }

    /** instantiating the left and right HABArm
     *  sets default states for the HABClimber
     */
    private HABinAGoodTime(){
        leftHABArm = new CANSparkMax(RobotMap.HABLiftLeft, CANSparkMaxLowLevel.MotorType.kBrushless);
        rightHABArm = new CANSparkMax(RobotMap.HABLiftRight, CANSparkMaxLowLevel.MotorType.kBrushless);

        mClimbStates = ClimbStates.IDLE;
        mClimbPosition = ClimbPosition.RETRACTION_LIMIT;
        mDriveTrainSubsystem = DriveTrainSubsystem.getInstance();
        reloadGains();

    }

    /** Different presets for the climber (climber heights)
     *
     */
    public enum ClimbPosition {

        RETRACTION_LIMIT(0),
        EXTENSION_LIMIT(70),
        LOW_POS(50),
        HIGH_POS(30);

        int height;

        /**
         * Constructing an instance of climbHeight
         * @param climbHeight height of the set point that is being creating
         */
        ClimbPosition(int climbHeight){
            height = climbHeight;
        }

        /**
         * returns the height of the ClimbPosition
         * @return
         */
        int getHeight(){
            return height;

        }
    }

    /**
     * Different states of control of the HABClimber
     */
    public enum ClimbStates {
        POSITION_CONTROL,
        RESET_RETRACT,
        RESET_EXTEND,
        HOLD_POSITION,
        MANUAL_CONTROL,
        IDLE
    }

    public void writeToLog(){
        //TODO
    }
    public void manualHandler(boolean manualReq, double left, double right) {
        isManual = manualReq;
        if (manualReq) {
            leftHABArm.getPIDController().setReference(left, ControlType.kDutyCycle);
            rightHABArm.getPIDController().setReference(right, ControlType.kDutyCycle);
            mClimbStates = ClimbStates.MANUAL_CONTROL;
        }
        else if(!manualReq && mClimbStates == ClimbStates.MANUAL_CONTROL) {
            mClimbStates = ClimbStates.IDLE;
        }
    }

    public void outputToSmartDashboard(){
        SmartDashboard.putNumber("Climb rightVel", rightHABArm.getEncoder().getVelocity());
        SmartDashboard.putNumber("Climb leftVel", leftHABArm.getEncoder().getVelocity());
        SmartDashboard.putNumber("Climb right encoder pos", rightHABArm.getEncoder().getPosition());
        SmartDashboard.putNumber("Climb left encoder pos", leftHABArm.getEncoder().getPosition());
        SmartDashboard.putNumber("Climb angleOffset", mDriveTrainSubsystem.getGyroRoll());
        SmartDashboard.putString("Climb Position" , mClimbPosition.toString());
        SmartDashboard.putString("Climb States" , mClimbStates.toString());
    }

    public void stop(){
        rightHABArm.stopMotor();
        leftHABArm.stopMotor();
        zeroSensors();
    }

    public void zeroSensors(){
        mDriveTrainSubsystem.zeroGyroRoll();
        rightHABArm.getEncoder().setPosition(0);
        leftHABArm.getEncoder().setPosition(0);
    }
    public synchronized void goToSetpoint(ClimbPosition setpoint) {
        mClimbStates = ClimbStates.POSITION_CONTROL;
        mClimbPosition = setpoint;
        leftHABArm.getPIDController().setReference(mClimbPosition.getHeight(), ControlType.kPosition);
        rightHABArm.getPIDController().setReference(mClimbPosition.getHeight(), ControlType.kPosition);
    }



    public void registerEnabledLoops(Looper enabledLooper){
        enabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
                zeroSensors();
                mClimbPosition = ClimbPosition.RETRACTION_LIMIT;
                mClimbStates = ClimbStates.IDLE;
            }
            double leftPower = 0;
            double rightPower = 0;
            double kPAngleCorrection = 0.02;
            double error = 0;
            @Override
            public void onLoop(double timestamp) {
                avgClimberHeight = (Math.abs(rightHABArm.getEncoder().getPosition()) + Math.abs(leftHABArm.getEncoder().getPosition()))/2;
                error = mClimbPosition.getHeight() - avgClimberHeight;
                switch (mClimbStates) {
                    case IDLE:
                        stop();
                    case HOLD_POSITION:
                        stop(); //TODO create dumb hold position loop IF needed
                    case POSITION_CONTROL:
                        if(Math.abs(error) < 1) {
                            mClimbStates = ClimbStates.HOLD_POSITION;
                        }
                }
            }

            @Override
            public void onStop(double timestamp) {

            }
        });
    }


    public void reloadGains(){
        reloadHABArmGains(leftHABArm);
        reloadHABArmGains(rightHABArm);
    }

    public void reloadHABArmGains(CANSparkMax controllerToReload){
        CANPIDController pidControllerToSet = controllerToReload.getPIDController();
        pidControllerToSet.setP(Constants.kHABClimbPositionKp);
        pidControllerToSet.setI(Constants.kHABClimbPositionKi);
        pidControllerToSet.setD(Constants.kHABClimbPositionKd);
        pidControllerToSet.setFF(Constants.kHABClimbPositionkf);
        pidControllerToSet.setIZone(Constants.kHABClimbPositionKiZone);
    }


    public void goToHatchMode(){
        //nothing to do here
    }

    public void goToCargoMode(){
        //nothing to do here
    }

    public void retractAll() {
        //TODO
    }

    public ClimbStates getmClimbStates(){
        return mClimbStates;
    }
}
