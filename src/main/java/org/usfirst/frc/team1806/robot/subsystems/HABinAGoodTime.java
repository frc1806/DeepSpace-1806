package org.usfirst.frc.team1806.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.usfirst.frc.team1806.robot.Constants;
import org.usfirst.frc.team1806.robot.RobotMap;
import org.usfirst.frc.team1806.robot.loop.Loop;
import org.usfirst.frc.team1806.robot.loop.Looper;
import org.usfirst.frc.team1806.robot.util.SparkMaxMechanismSynchronizer;
//import com.revrobotics.CANSparkMax;

public class HABinAGoodTime implements Subsystem {
    boolean debug = false;
    private static HABinAGoodTime mHabANiceDay = new HABinAGoodTime();

    private TalonSRX leftClimbDrive, rightClimbDrive;
    private CANSparkMax leftHABArm, rightHABArm;
    private SparkMaxMechanismSynchronizer mSynchronizer;
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
        mSynchronizer = new SparkMaxMechanismSynchronizer(leftHABArm, rightHABArm);
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
        HIGH_POS(30),
        LEVEL_TWO(20);

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
        if(debug) {
            SmartDashboard.putNumber("Climb rightVel", rightHABArm.getEncoder().getVelocity());
            SmartDashboard.putNumber("Climb leftVel", leftHABArm.getEncoder().getVelocity());
            SmartDashboard.putNumber("Climb right encoder pos", rightHABArm.getEncoder().getPosition());
            SmartDashboard.putNumber("Climb left encoder pos", leftHABArm.getEncoder().getPosition());
            SmartDashboard.putNumber("Climb angleOffset", mDriveTrainSubsystem.getGyroRoll());
            SmartDashboard.putString("Climb Position", mClimbPosition.toString());
            SmartDashboard.putString("Climb States", mClimbStates.toString());
            SmartDashboard.putString("Climb synchronizer state", mSynchronizer.getState().name());
        }
    }

    public void stop(){
        mSynchronizer.stop();
    }

    public void zeroSensors(){
        mDriveTrainSubsystem.zeroGyroRoll();
        rightHABArm.getEncoder().setPosition(0);
        leftHABArm.getEncoder().setPosition(0);
    }
    public synchronized void goToSetpoint(ClimbPosition setpoint) {
        mClimbStates = ClimbStates.POSITION_CONTROL;
        mClimbPosition = setpoint;
        mSynchronizer.setWantedMovement(setpoint.getHeight(), Constants.kHabClimbTargetSpeed);
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

                if(mClimbStates != ClimbStates.IDLE) {
                    leftClimbDrive.set(ControlMode.PercentOutput, mDriveTrainSubsystem.getLeftVoltage());
                    rightClimbDrive.set(ControlMode.PercentOutput, mDriveTrainSubsystem.getRightVoltage());
                }
                switch (mClimbStates) {
                    case MANUAL_CONTROL:
                        mSynchronizer.forceToManual();
                        break;
                    case IDLE:
                        stop();
                        break;
                    case HOLD_POSITION:
                        stop(); //TODO create dumb hold position loop IF needed
                        break;
                    case POSITION_CONTROL:
                        mSynchronizer.update();
                        if(mSynchronizer.getState() == SparkMaxMechanismSynchronizer.SynchronizerState.COMPLETE) {
                            mClimbStates = ClimbStates.HOLD_POSITION;
                        }
                        break;

                }
            }

            @Override
            public void onStop(double timestamp) {

            }
        });
    }

    @Override
    public void setDebug(boolean _debug) {
        debug = _debug;
    }

    public void setWantStop() {
        mClimbStates = ClimbStates.IDLE;
        stop();
    }


    public void reloadGains(){
        mSynchronizer.configureSynchronizerParameters(Constants.kHABClimbSyncKp, Constants.kHABClimbSyncThrottleLetoffDistance, Constants.kHABClimbSyncVelocityTolerance, Constants.kHABClimbSyncPositionTolerance);
        mSynchronizer.reloadBothMotorGains(Constants.kHABClimbVelocityKp, Constants.kHABClimbVelocityKi, Constants.kHABClimbVelocityKd, Constants.kHABClimbVelocityKf, Constants.kHABClimbVelocityKiZone);
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
