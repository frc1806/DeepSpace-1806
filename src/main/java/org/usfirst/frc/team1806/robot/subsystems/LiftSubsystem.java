package org.usfirst.frc.team1806.robot.subsystems;
import com.revrobotics.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.usfirst.frc.team1806.robot.Constants;
import org.usfirst.frc.team1806.robot.Robot;
import org.usfirst.frc.team1806.robot.RobotMap;
import org.usfirst.frc.team1806.robot.loop.Loop;
import org.usfirst.frc.team1806.robot.loop.Looper;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANEncoder.*;
import edu.wpi.first.wpilibj.DigitalInput;

/**
 * Whew, this subsystem is for our liftactions, we will later implement this into a superstructure
 * so we can interact with our intake for doing things like shooting!
 */
public class LiftSubsystem  implements Subsystem {
	public enum LiftStates {
		POSITION_CONTROL,
		RESET_TO_BOTTOM,
		RESET_TO_TOP,
		HOLD_POSITION,
		MANUAL_CONTROL,
		IDLE
	}
	public enum LiftPosition {
		BOTTOM_LIMIT(0),
        TOP_LIMIT(16000),
		TELEOP_HOLD(600),
		SHIP_CARGO(7000),
		ROCKET_CARGO_LOW(6000),
		ROCKET_CARGO_MID(10000),
		ROCKET_CARGO_HIGH(16000),
		ROCKET_HATCH_LOW(4000),
		ROCKET_HATCH_MID(8000),
		ROCKET_HATCH_HIGH(12000),
		TEMP_HOLD_POS(0);

		int height;
		LiftPosition(int liftHeight){
			height = liftHeight;
		}

		int getHeight(){
			return height;
		}

		LiftPosition setHeight(int liftHeight) {
			height = liftHeight;
			return this;
		}
	}

	private CANSparkMax liftLead, liftFollow; //gotta have the power
	private CANPIDController liftLeadController, liftFollowController;
	public DigitalInput bottomLimit, topLimit;

	private boolean isBrakeMode = false;
	private boolean mIsOnTarget = false;

	private LiftStates mLiftStates;
	private LiftPosition mLiftPosition;

	private CANPIDController canpidController;

	private static LiftSubsystem mLiftSubsystem = new LiftSubsystem(); //only ever 1 lift

	public LiftSubsystem() {
		liftLead = new CANSparkMax(RobotMap.liftLead, CANSparkMaxLowLevel.MotorType.kBrushless);
		liftLeadController = new CANPIDController(liftLead);
		liftFollow.follow(liftLead, true);
		liftLead.setSmartCurrentLimit(130, 80);
		liftFollow.setSmartCurrentLimit(130, 80);
		bottomLimit = new DigitalInput(RobotMap.liftBottomLimit);
		topLimit = new DigitalInput(RobotMap.liftHighLimit);
		mLiftStates = LiftStates.IDLE;
		mLiftPosition = LiftPosition.BOTTOM_LIMIT;
 		reloadGains();
 		canpidController = new CANPIDController(liftLead);
	}


	@Override
	public void outputToSmartDashboard() {
        SmartDashboard.putString("Lift State: ", returnLiftStates().toString());
        SmartDashboard.putString("Lift Position", returnLiftPosition().toString());
		SmartDashboard.putNumber("Lift Encoder Position", liftLead.getEncoder().getPosition());
		SmartDashboard.putNumber("Lift Leader Power Sending", liftLead.getAppliedOutput());
		SmartDashboard.putNumber("Lift Follow Power Sending", liftFollow.getAppliedOutput());
		SmartDashboard.putBoolean("Lift Bottom limit triggered", areWeAtBottomLimit());
        SmartDashboard.putNumber("Lift Wanted Height", mLiftPosition.getHeight());
    }

	@Override
	public void stop() {
        mLiftStates = LiftStates.IDLE;
	}

	@Override
	public synchronized void zeroSensors() {
		liftLead.getEncoder();
	}


    public synchronized void zeroSensorsAtTop(){
        //liftLead.setSelectedSensorPosition(Constants.kLiftTopLimitSwitchPosition, 0, 10); TODO re-add once CanSpark api gets fixed
        mLiftStates = LiftStates.POSITION_CONTROL;
    }
    public synchronized void zeroSensorsAtBottom(){
        //liftLead.setSelectedSensorPosition(0, 0, 10); TODO re-add once CanSpark api gets fixed
        mLiftStates = LiftStates.POSITION_CONTROL;
    }

	@Override
	public void registerEnabledLoops(Looper enabledLooper) {
        enabledLooper.register(new Loop() {
            @Override
            public void onStop(double timestamp) {
                mLiftStates = LiftStates.IDLE;
            }

            @Override
            public void onStart(double timestamp) {
				if(Robot.needToPositionControlInTele){
					setLiftHoldPosition();
				} else {
					mLiftStates = LiftStates.IDLE;
				}
            }

            @Override
            public void onLoop(double timestamp) {
            	synchronized (LiftSubsystem.this){
            		//not moving and not manual
					if(isAtPosition() && mLiftStates != LiftStates.MANUAL_CONTROL){
						//not moving, at bottom
						if(mLiftStates == LiftStates.RESET_TO_BOTTOM
								|| areWeAtBottomLimit()
								){
							mLiftStates = LiftStates.IDLE;
						}
						//not moving, not at bottom
						else{
							mLiftStates = LiftStates.HOLD_POSITION;
							holdPosition();
						}
					}

					cubePositionLoop();
					cubeLiftStateLoop();
				}
            }

            private void cubePositionLoop(){
            	switch (mLiftPosition){
					case BOTTOM_LIMIT:
						return;
					case TOP_LIMIT:
						return;
				}
			}

			private void cubeLiftStateLoop(){
				switch(mLiftStates) {
					case POSITION_CONTROL:
						setBrakeMode();
						return;
					case RESET_TO_BOTTOM:
						mIsOnTarget = false;
						return;
					case RESET_TO_TOP:
						mIsOnTarget = false;
						return;
					case HOLD_POSITION:
						liftLead.set(Constants.kLiftHoldPercentOutput +
								( mLiftPosition.getHeight() - liftLead.getEncoder().getPosition()) * Constants.kLiftHoldkPGain);
						if(Math.abs(getHeightInCounts() - mLiftPosition.getHeight()) < Constants.kLiftPositionTolerance) {
							mLiftStates = LiftStates.POSITION_CONTROL;
							goToSetpoint(mLiftPosition);
						}
						return;
					case MANUAL_CONTROL:
						return;
					case IDLE:
						liftLeadController.setReference(0, ControlType.kVoltage);
						return;
					default:
						return;

				}
			}
        });
	}

	@Override
	public void writeToLog() {

	}

	public static LiftSubsystem getInstance() {
		return mLiftSubsystem;
	}

	public synchronized void goToSetpoint(LiftPosition setpoint) {
		mLiftStates = LiftStates.POSITION_CONTROL;
		mLiftPosition = setpoint;
		setBrakeMode();
		liftLeadController.setReference(mLiftPosition.getHeight(), ControlType.kPosition);
		//System.out.println(mLiftWantedPosition + "  " + isReadyForSetpoint());
	}
	public synchronized void goToSetpoint(int setpoint) {
		mLiftStates = LiftStates.POSITION_CONTROL;
		mLiftPosition.TEMP_HOLD_POS.setHeight(setpoint);
		mLiftPosition = LiftPosition.TEMP_HOLD_POS;
		setBrakeMode();
		liftLeadController.setReference(mLiftPosition.getHeight(), ControlType.kPosition);
		//System.out.println(mLiftWantedPosition + "  " + isReadyForSetpoint());
	}

	public synchronized void zeroOnBottom() {
		// TODO Auto-generated method stub
		if(mLiftStates != LiftStates.RESET_TO_BOTTOM) {
			mLiftStates = LiftStates.RESET_TO_BOTTOM;
		}
	}

	public synchronized void goToTop() {
		// TODO Auto-generated method stub

	}

	public double getHeightInInches() {
		return getHeightInCounts() / Constants.kCountsPerInch;
	}

	public double getHeightInCounts() {
		return liftLead.getEncoder().getPosition();
	}

	public boolean isOnTarget() {
		return mIsOnTarget;
	}
	public void setBrakeMode(){

		liftLead.setIdleMode(CANSparkMax.IdleMode.kBrake);
		liftFollow.setIdleMode(CANSparkMax.IdleMode.kBrake);
		isBrakeMode = true;

	}
	public void setCoastMode() {
		liftLead.setIdleMode(CANSparkMax.IdleMode.kCoast);
		liftFollow.setIdleMode(CANSparkMax.IdleMode.kCoast);
		isBrakeMode = false;
	}
	public boolean isInBrakeMode() {
		return isBrakeMode;
	}
	public void reloadGains() {
		/*
		liftLead.setParameter(CANSparkMaxLowLevel.ConfigParameter.kP_0, Constants.kLiftPositionkP);
		liftLead.setParameter(CANSparkMaxLowLevel.ConfigParameter.kI_0, Constants.kLiftPositionkI);
		liftLead.setParameter(CANSparkMaxLowLevel.ConfigParameter.kD_0, Constants.kLiftPositionkD);
		liftLead.setParameter(CANSparkMaxLowLevel.ConfigParameter.kF_0, Constants.kLiftPositionkF);
		liftLead.setParameter(CANSparkMaxLowLevel.ConfigParameter.kIZone_0, Constants.kLiftPositionIZone);
		liftLead.setParameter(CANSparkMaxLowLevel.ConfigParameter.kRampRate, Constants.kLiftPositionRampRate);
		*/
		liftLeadController.setP(Constants.kLiftPositionkP);
		liftLeadController.setI(Constants.kLiftPositionkI);
		liftLeadController.setD(Constants.kLiftPositionkD);
		liftLeadController.setFF(Constants.kLiftPositionkF);
		liftLeadController.setIZone(Constants.kLiftPositionIZone);

	}

	public synchronized void resetToBottom() {
		if(!areWeAtBottomLimit() || Math.abs(liftLead.getEncoder().getPosition()) < Constants.kBottomLimitTolerance) {
		        mLiftStates = LiftStates.RESET_TO_BOTTOM;
		        mLiftPosition = LiftPosition.BOTTOM_LIMIT;
				goToSetpoint(LiftPosition.BOTTOM_LIMIT);
		}
	}

	public synchronized void resetToTop() {
        if(!topLimit.get()) {
            if(mLiftStates != LiftStates.RESET_TO_TOP){
                mLiftStates = LiftStates.RESET_TO_TOP;
				mLiftPosition = LiftPosition.TOP_LIMIT;
			}
            liftLeadController.setReference(Constants.liftSpeed, ControlType.kDutyCycle);
        } else {
            zeroSensorsAtTop();
        }
	}

	/**
	 * @return
	 * Returns whether or not the liftactions is ready to be held at position for a cube to be deposited
	 */
	public synchronized boolean isAtPosition() {
		if(mLiftStates == LiftStates.IDLE){
			return false;
		}
		return Math.abs(mLiftPosition.getHeight() - liftLead.getEncoder().getPosition()) < Constants.kLiftPositionTolerance &&
				Math.abs(liftLead.getEncoder().getVelocity()) < Constants.kLiftVelocityTolerance;
	}
	/**
	 *
	 * @return
	 * returns current state of cube
	 */
	public synchronized LiftStates returnLiftStates() {
		return mLiftStates;
	}

    /**
     * @return
     * returns the position of where the liftactions is eg: moving, scale
     */
	public synchronized LiftPosition returnLiftPosition() {
		return mLiftPosition;
	}

    /**
     * Used to stop the manipulator from running. mostly ran on stop or when first setting the liftactions up
     */
	public synchronized void setLiftIdle(){
	    mLiftStates = LiftStates.IDLE;
        liftLeadController.setReference(0, ControlType.kDutyCycle); //DutyCycle just means voltage on scale of -1 to 1
    }
	public synchronized  void setLiftHoldPosition(){
		mLiftStates = LiftStates.POSITION_CONTROL;
		goToSetpoint(mLiftPosition.TEMP_HOLD_POS.getHeight());
	}
    /**
     * Sets up the robot to accept position setpoints
     */
    public synchronized void updatePositionControl(){
		mLiftStates = LiftStates.POSITION_CONTROL;
        setBrakeMode();
    }
	public synchronized void goToTeleOpHold(){
    	mLiftPosition = LiftPosition.TELEOP_HOLD;
    	mLiftStates = LiftStates.POSITION_CONTROL;
    	if(isReadyForSetpoint()){
    		goToSetpoint(Constants.kTeleOpHoldHeight);
		}
	}
	private synchronized boolean isReadyForSetpoint(){
    	return (isCurrentModesReady());
	}
	private synchronized boolean isCurrentModesReady(){
    	return mLiftStates == LiftStates.POSITION_CONTROL ||
				mLiftStates == LiftStates.IDLE ||
				mLiftStates == LiftStates.HOLD_POSITION ||
				mLiftStates == LiftStates.RESET_TO_BOTTOM;
	}

	public synchronized void manualMode(double power){
    	mLiftStates = LiftStates.MANUAL_CONTROL;
    	liftLeadController.setReference(power, ControlType.kDutyCycle);
	}
	public void setupForManualMode(){

	}

	/**
	 * Used to hold the cube when it is ready to be spat out
	 */
	public synchronized void holdPosition(){
    	liftLeadController.setReference(Constants.kLiftHoldPercentOutput +
				( mLiftPosition.getHeight() - liftLead.getEncoder().getPosition()) * Constants.kLiftHoldkPGain, ControlType.kDutyCycle);
    	if(Math.abs(getHeightInCounts() - mLiftPosition.getHeight()) < Constants.kLiftPositionTolerance){
    		mLiftStates = LiftStates.POSITION_CONTROL;
    		goToSetpoint(mLiftPosition);
		}
	}

	/**
	 * @return
	 * are we at the bottom limit??
	 */
	public boolean areWeAtBottomLimit(){
		return !bottomLimit.get();
	}
	public double returnLiftHeight(){
		return liftLead.getEncoder().getPosition();
	}
	public synchronized boolean bumpHeightUp(){
		if(mLiftStates == LiftStates.POSITION_CONTROL || mLiftStates == LiftStates.HOLD_POSITION) {
			mLiftPosition = LiftPosition.TEMP_HOLD_POS.setHeight(mLiftPosition.getHeight() + Constants.kBumpEncoderPosition);
			goToSetpoint(mLiftPosition);
			return true;
		}
		return false;
	}
	public synchronized boolean bumpHeightDown(){
		if(mLiftStates == LiftStates.POSITION_CONTROL || mLiftStates == LiftStates.HOLD_POSITION ){
			mLiftPosition = LiftPosition.TEMP_HOLD_POS.setHeight(mLiftPosition.getHeight() - Constants.kBumpEncoderPosition);
			goToSetpoint(mLiftPosition);
			return true;
		}
		return false;
	}
}
