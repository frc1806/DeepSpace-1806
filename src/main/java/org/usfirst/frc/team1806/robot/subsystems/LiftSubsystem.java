package org.usfirst.frc.team1806.robot.subsystems;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.usfirst.frc.team1806.robot.Constants;
import org.usfirst.frc.team1806.robot.Robot;
import org.usfirst.frc.team1806.robot.RobotMap;
import org.usfirst.frc.team1806.robot.loop.Loop;
import org.usfirst.frc.team1806.robot.loop.Looper;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

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
        TOP_LIMIT(0),
		TELEOP_HOLD(100),
		SHIP_CARGO(0),
		ROCKET_CARGO_LOW(0),
		ROCKET_CARGO_MID(0),
		ROCKET_CARGO_HIGH(0),
		ROCKET_HATCH_LOW(0),
		ROCKET_HATCH_MID(0),
		ROCKET_HATCH_HIGH(0),
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

	private TalonSRX liftLead, liftFollow; //gotta have the power
	public DigitalInput bottomLimit, topLimit;

	private boolean isBrakeMode = false;
	private boolean mIsOnTarget = false;

	private LiftStates mLiftStates;
	private LiftPosition mLiftPosition;

	private static LiftSubsystem mLiftSubsystem = new LiftSubsystem();
	public LiftSubsystem() {
		liftLead = new TalonSRX(RobotMap.liftLead);
		liftFollow = new TalonSRX(RobotMap.liftFollow);
		liftFollow.follow(liftLead);
		liftLead.configContinuousCurrentLimit(130, 10);
		bottomLimit = new DigitalInput(RobotMap.liftBottomLimit);
		topLimit = new DigitalInput(RobotMap.liftHighLimit);
		mLiftStates = LiftStates.IDLE;
		mLiftPosition = LiftPosition.BOTTOM_LIMIT;
		liftLead.setSensorPhase(false);
		liftLead.setInverted(true);
		liftFollow.setInverted(true);
		liftLead.configPeakOutputReverse(-.4, 10);
 		reloadGains();
	}

	@Override
	public void outputToSmartDashboard() {
		SmartDashboard.putString("Wanted Lift Position: ", mLiftPosition.toString());
        SmartDashboard.putString("Lift State: ", returnLiftStates().toString());
        SmartDashboard.putString("Lift Position", returnCubePosition().toString());
		SmartDashboard.putNumber("Lift Encoder Position", liftLead.getSelectedSensorPosition(0));
		SmartDashboard.putNumber("Lift Power Sending", liftLead.getMotorOutputPercent());
		SmartDashboard.putBoolean("Bottom limit triggered", areWeAtBottomLimit());
        SmartDashboard.putNumber("Lift Wanted Height", mLiftPosition.getHeight());
    }

	@Override
	public void stop() {
        mLiftStates = LiftStates.IDLE;
	}

	@Override
	public synchronized void zeroSensors() {
		liftLead.setSelectedSensorPosition(0,0,10);
	}


    public synchronized void zeroSensorsAtTop(){
        liftLead.setSelectedSensorPosition(Constants.kLiftTopLimitSwitchPosition, 0, 10);
        mLiftStates = LiftStates.POSITION_CONTROL;
    }
    public synchronized void zeroSensorsAtBottom(){
        liftLead.setSelectedSensorPosition(0, 0, 10);
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
						liftLead.set(ControlMode.PercentOutput, Constants.kLiftHoldPercentOutput +
								( mLiftPosition.getHeight() - liftLead.getSelectedSensorPosition(0)) * Constants.kLiftHoldkPGain);
						if(Math.abs(getHeightInCounts() - mLiftPosition.getHeight()) < Constants.kLiftPositionTolerance) {
							mLiftStates = LiftStates.POSITION_CONTROL;
							goToSetpoint(mLiftPosition);
						}
						return;
					case MANUAL_CONTROL:
						return;
					case IDLE:
						liftLead.set(ControlMode.PercentOutput, 0);
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
		liftLead.set(ControlMode.Position, mLiftPosition.getHeight());
		//System.out.println(mLiftWantedPosition + "  " + isReadyForSetpoint());
	}
	public synchronized void goToSetpoint(int setpoint) {
		mLiftStates = LiftStates.POSITION_CONTROL;
		mLiftPosition.TEMP_HOLD_POS.setHeight(setpoint);
		mLiftPosition = LiftPosition.TEMP_HOLD_POS;
		setBrakeMode();
		liftLead.set(ControlMode.Position, mLiftPosition.getHeight());
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

	public int getHeightInCounts() {
		return liftLead.getSelectedSensorPosition(0);
	}

	public boolean isOnTarget() {
		return mIsOnTarget;
	}
	public void setBrakeMode(){
		liftLead.setNeutralMode(NeutralMode.Brake);
		liftFollow.setNeutralMode(NeutralMode.Brake);
		isBrakeMode = true;

	}
	public void setCoastMode() {
		liftLead.setNeutralMode(NeutralMode.Coast);
		liftFollow.setNeutralMode(NeutralMode.Coast);
	}
	public boolean isInBrakeMode() {
		return isBrakeMode;
	}
	public void reloadGains() {
		liftLead.config_kP(Constants.kLiftPositionControlPIDSlot, Constants.kLiftPositionkP, Constants.kLiftPositionPIDTimeout);
		liftLead.config_kI(Constants.kLiftPositionControlPIDSlot, Constants.kLiftPositionkI, Constants.kLiftPositionPIDTimeout);
		liftLead.config_kD(Constants.kLiftPositionControlPIDSlot, Constants.kLiftPositionkD, Constants.kLiftPositionPIDTimeout);
		liftLead.config_kF(Constants.kLiftPositionControlPIDSlot, Constants.kLiftPositionkF, Constants.kLiftPositionPIDTimeout);
		liftLead.config_IntegralZone(Constants.kLiftPositionControlPIDSlot, Constants.kLiftPositionIZone, Constants.kLiftPositionPIDTimeout);
		liftLead.configClosedloopRamp(Constants.kLiftPositionRampRate, Constants.kLiftPositionPIDTimeout);
	}

	public synchronized void resetToBottom() {
		if(!areWeAtBottomLimit() || Math.abs(liftLead.getSelectedSensorPosition(0)) < Constants.kBottomLimitTolerance) {
		        mLiftStates = LiftStates.RESET_TO_BOTTOM;
		        mLiftPosition = LiftPosition.BOTTOM_LIMIT;
				goToSetpoint(0);
		}
	}

	public synchronized void resetToTop() {
        if(!topLimit.get()) {
            if(mLiftStates != LiftStates.RESET_TO_TOP){
                mLiftStates = LiftStates.RESET_TO_TOP;
				mLiftPosition = LiftPosition.TOP_LIMIT;
			}
            liftLead.set(ControlMode.PercentOutput, Constants.liftSpeed);
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
		return Math.abs(mLiftPosition.getHeight() - liftLead.getSelectedSensorPosition(0)) < Constants.kLiftPositionTolerance &&
				Math.abs(liftLead.getSelectedSensorVelocity(0)) < Constants.kLiftVelocityTolerance;
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
	public synchronized LiftPosition returnCubePosition() {
		return mLiftPosition;
	}

    /**
     * Used to stop the manipulator from running. mostly ran on stop or when first setting the liftactions up
     */
	public synchronized void setLiftIdle(){
	    mLiftStates = LiftStates.IDLE;
        liftLead.set(ControlMode.PercentOutput, 0);
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
    	liftLead.set(ControlMode.PercentOutput, power);
	}
	public void setupForManualMode(){

	}

	/**
	 * Used to hold the cube when it is ready to be spat out
	 */
	public synchronized void holdPosition(){
    	liftLead.set(ControlMode.PercentOutput, Constants.kLiftHoldPercentOutput +
				( mLiftPosition.getHeight() - liftLead.getSelectedSensorPosition(0)) * Constants.kLiftHoldkPGain);
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
	public int returnLiftHeight(){
		return liftLead.getSelectedSensorPosition(0);
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
