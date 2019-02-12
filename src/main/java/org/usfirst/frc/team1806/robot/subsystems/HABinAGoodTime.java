package org.usfirst.frc.team1806.robot.subsystems;

import com.revrobotics.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

    public static HABinAGoodTime getInstance(){
        return mHabANiceDay;
    }

    private HABinAGoodTime(){
        leftHABArm = new CANSparkMax(RobotMap.HABLiftLeft, CANSparkMaxLowLevel.MotorType.kBrushless);
        rightHABArm = new CANSparkMax(RobotMap.HABLiftRight, CANSparkMaxLowLevel.MotorType.kBrushless);

        leftHABArm.setInverted(true); //TODO verify this is the correct motor to reverse

        mClimbStates = ClimbStates.IDLE;
        mClimbPosition = ClimbPosition.RETRACTION_LIMIT;
        mDriveTrainSubsystem = DriveTrainSubsystem.getInstance();

    }

    public enum ClimbPosition {

        RETRACTION_LIMIT(0),
        EXTENSION_LIMIT(300),
        LOW_POS(100),
        HIGH_POS(200);

        int height;
        ClimbPosition(int climbHeight){
            height = climbHeight;
        }

        int getHeight(){
            return height;
        }

        ClimbPosition setHeight(int liftHeight) {
            height = liftHeight;
            return this;
        }
    }
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
    private boolean isManual;
    public boolean manualHandler(boolean manualReq, double left, double right) {
        if (manualReq) {
            leftHABArm.getPIDController().setReference(left, ControlType.kDutyCycle);
            rightHABArm.getPIDController().setReference(right, ControlType.kDutyCycle);
            mClimbStates = ClimbStates.MANUAL_CONTROL;
            return true;
        }
        else {
            return false;
        }
    }

    public void outputToSmartDashboard(){
        SmartDashboard.putNumber("Climb rightVel", rightHABArm.getEncoder().getVelocity());
        SmartDashboard.putNumber("Climb leftVel", leftHABArm.getEncoder().getVelocity());
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
    }



    public void registerEnabledLoops(Looper enabledLooper){
        enabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
                zeroSensors();
                mClimbPosition = ClimbPosition.RETRACTION_LIMIT;
                mClimbStates = ClimbStates.IDLE;
            }

            @Override
            public void onLoop(double timestamp) {

                switch (mClimbStates) {
                    case IDLE:
                        stop();
                    case HOLD_POSITION:
                        stop(); //TODO create dumb hold position loop IF needed
                    case MANUAL_CONTROL:

                }
            }

            @Override
            public void onStop(double timestamp) {

            }
        });
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
}
