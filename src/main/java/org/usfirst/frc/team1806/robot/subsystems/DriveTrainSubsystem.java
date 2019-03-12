package org.usfirst.frc.team1806.robot.subsystems;

//import org.omg.CORBA.PRIVATE_MEMBER;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.ControlType;
import org.usfirst.frc.team1806.robot.Constants;
import org.usfirst.frc.team1806.robot.Kinematics;
import org.usfirst.frc.team1806.robot.RobotMap;
import org.usfirst.frc.team1806.robot.RobotState;
import org.usfirst.frc.team1806.robot.loop.Loop;
import org.usfirst.frc.team1806.robot.loop.Looper;
import org.usfirst.frc.team1806.robot.path.Path;
import org.usfirst.frc.team1806.robot.path.PathFollower;
import org.usfirst.frc.team1806.robot.util.DriveSignal;
import org.usfirst.frc.team1806.robot.util.Lookahead;
import org.usfirst.frc.team1806.robot.util.NavX;
import org.usfirst.frc.team1806.robot.util.RigidTransform2d;
import org.usfirst.frc.team1806.robot.util.Rotation2d;
import org.usfirst.frc.team1806.robot.util.Twist2d;

import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.SPI;

/**
 * The DrivetrainSubsystem deals with all of the drivetrain
 * code used on the robot.
 */
public class DriveTrainSubsystem implements Subsystem {

	public enum DriveStates {
		DRIVING, // Ya old normal dirivng
		CREEP, // Creep for percise movement
		VISION, // Vision tracking?
		TURN_TO_HEADING, // Turn using PID
		DRIVE_TO_POSITION, // Drive to Position using SRX PID
		PATH_FOLLOWING,
		VELOCITY_SETPOINT,
		NOTHING // Used on init
	}

	private static DriveTrainSubsystem mDriveTrainSubsystem = new DriveTrainSubsystem(); //Only ever 1 instance of drivetrain.
	private static final int kLowGearPositionControlSlot = 0;
	private static final int kHighGearVelocityControlSlot = 1;

	public static DriveTrainSubsystem getInstance() {
		return mDriveTrainSubsystem;
	}

	private static double inchesPerSecondToRPM(double inches_per_second) {
		return (inches_per_second / Constants.kDriveInchesPerRevolution) * 60;
	}


	private static double inchesPerSecondToRpm(double inches_per_second) {
		return inchesToRotations(inches_per_second) * 60;
	}

	private static double inchesToRPM(double inches) {
		return inches / Constants.kDriveInchesPerRevolution;
	}

	private static double inchesToRotations(double inches) {
		return inches / (Constants.kDriveWheelDiameterInches * Math.PI);
	}

	/**
	 * Check if the drive talons are configured for position control
	 */
	protected static boolean usesTalonPositionControl(DriveStates state) {
		if (state == DriveStates.DRIVE_TO_POSITION ||
				state == DriveStates.TURN_TO_HEADING) {
			return true;
		}
		return false;
	}

	protected static boolean usesTalonVelocityControl(DriveStates state) {
		if (state == DriveStates.VELOCITY_SETPOINT || state == DriveStates.PATH_FOLLOWING) {
			return true;
		}
		return false;
	}

	//Initialize all of the drive motors
	private CANSparkMax masterLeft, masterRight, leftA, rightC;
	//    private DoubleSolenoid shifter;
	private NavX navx;
	private PathFollower mPathFollower;
	private Rotation2d mTargetHeading = new Rotation2d();
	private boolean mIsOnTarget = false;
	//TODO:Remove these
	private double leftLowGearMaxVel = 0;
	private double rightLowGearMaxVel = 0;
	private double leftLastVel = 0;
	private double rightLastVel = 0;
	private double leftMaxAccel = 0;
	private double rightMaxAccel = 0;
	private double leftHighGearMaxVel = 0;
	private double rightHighGearMaxVel = 0;
	private double currentTimeStamp;
	private double lastTimeStamp;
	// State Control
	private DriveStates mDriveStates;
	private RobotState mRobotState = RobotState.getInstance();
	private Path mCurrentPath = null;
	private boolean mIsHighGear = false;
	public static boolean isWantedLowPID = false;
	private boolean mIsBrakeMode = false;

	private Loop mLoop = new Loop() {

		@Override
		public synchronized void onLoop(double timestamp) {
			// TODO Auto-generated method stub
			lastTimeStamp = currentTimeStamp;
			currentTimeStamp = timestamp;
			synchronized (DriveTrainSubsystem.this) {
				switch (mDriveStates) {
					case CREEP:
						return;
					case DRIVE_TO_POSITION:
						return;
					case DRIVING:
						return;
					case NOTHING:
						return;
					case PATH_FOLLOWING:
						if (mPathFollower != null) {
							updatePathFollower(timestamp);
						}
						return;
					case TURN_TO_HEADING:
						updateTurnToHeading(timestamp);
						return;
					case VELOCITY_SETPOINT:
						return;
					case VISION:
						return;
					default:
						return;

				}
			}

		}

		@Override
		public synchronized void onStart(double timestamp) {
			synchronized (DriveTrainSubsystem.this) {
				setOpenLoop(DriveSignal.NEUTRAL);
				setNeutralMode(false);
				navx.reset();
				setMaxDrivePower(1);
			}
		}

		@Override
		public synchronized void onStop(double timestamp) {
			setOpenLoop(DriveSignal.NEUTRAL);

		}
	};

	public DriveTrainSubsystem() {
		//init the all of the motor controllers
		masterLeft = new CANSparkMax(RobotMap.masterLeft, CANSparkMaxLowLevel.MotorType.kBrushless);
		masterRight = new CANSparkMax(RobotMap.masterRight, CANSparkMaxLowLevel.MotorType.kBrushless);

		leftA = new CANSparkMax(RobotMap.leftC, CANSparkMaxLowLevel.MotorType.kBrushless);
		rightC = new CANSparkMax(RobotMap.rightC, CANSparkMaxLowLevel.MotorType.kBrushless);

		//Follow for right side
        rightC.follow(masterRight);

		// Follow for left side
        leftA.follow(masterLeft);

        masterLeft.setSmartCurrentLimit(85);
        leftA.setSmartCurrentLimit(85);
        masterRight.setSmartCurrentLimit(85);
        rightC.setSmartCurrentLimit(85);

		//Set Encoders for each side of the talon
        //TODO: configure after REV updates their software. https://www.chiefdelphi.com/t/connecting-external-encoders-to-spark-max/345039
		/*masterLeft.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
		masterRight.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
		masterRight.setSensorPhase(true);
		masterLeft.setSensorPhase(true);
		*/
		masterLeft.setInverted(false);
		leftA.setInverted(false);

//		//Invert the right side
		masterRight.setInverted(true);
		rightC.setInverted(true);



		// init solenoids
//		shifter = new DoubleSolenoid(RobotMap.shiftLow, RobotMap.shiftHigh);
//		init navx
		navx = new NavX(SPI.Port.kMXP);

		reloadGains();
		mDriveStates = DriveStates.NOTHING;
		setMaxDrivePower(1);
	}

	private synchronized void configureTalonsForPositionControl() {
		if (usesTalonPositionControl(mDriveStates)) {
			setMaxDrivePower(1);
			// We entered a position control state.
			System.out.println("Configuring position control");
			masterLeft.setIAccum(0);
			masterRight.setIAccum(0);
			setBrakeMode();
		} else {
			System.out.println("Oh no! DIdn't set Position control");
		}
	}

	private synchronized void configureTalonsForSpeedControl() {
		if (!usesTalonVelocityControl(mDriveStates)) {
			// We entered a velocity control state.
			setMaxDrivePower(1);
			System.out.println("Configuring speed control");
			setBrakeMode();
		}
	}

	public synchronized void forceDoneWithPath() {
		if (mDriveStates == DriveStates.PATH_FOLLOWING && mPathFollower != null) {
			mPathFollower.forceFinish();
		} else {
			System.out.println("Robot is not in path following mode");
		}
	}

	public synchronized Rotation2d getGyroYaw() {
		return navx.getYaw();
		//return new Rotation2d();
	}

	private double zeroRoll = 0;
	public synchronized double getGyroRoll() {
		return navx.getRoll() - zeroRoll;
	}
	public void zeroGyroRoll() {
		zeroRoll = navx.getRoll();
	}

	public double getLeftDistanceInches() {
		return masterLeft.getEncoder().getPosition() * Constants.kDriveInchesPerRevolution;
	}

	public double getLeftVelocityInchesPerSec() {
		return masterLeft.getEncoder().getVelocity() * Constants.kDriveInchesPerRevolution / 60;
	}

	public double getRightDistanceInches() {
		return masterRight.getEncoder().getPosition() * Constants.kDriveInchesPerRevolution;
	}

	public double getRightVelocityInchesPerSec() {
		return masterRight.getEncoder().getVelocity() * Constants.kDriveInchesPerRevolution / 60;
	}

	public boolean isCreeping() {
		return mDriveStates == DriveStates.CREEP;
	}

	public synchronized boolean isDoneWithPath() {
		if (mDriveStates == DriveStates.PATH_FOLLOWING && mPathFollower != null) {
			return mPathFollower.isFinished();
		} else {
			System.out.println("Robot is not in path following mode");
			return true;
		}
	}

	public synchronized boolean isDoneWithTurn() {
		if (mDriveStates == DriveStates.TURN_TO_HEADING) {
			return mIsOnTarget;
		} else {
			System.out.println("Robot is not in turn to heading mode");
			return false;
		}
	}

	public boolean isHighGear() {
		return mIsHighGear;
	}

	public boolean isPositionControl(DriveStates state) {
		if (state == DriveStates.DRIVE_TO_POSITION ||
				state == DriveStates.TURN_TO_HEADING) {
			return true;
		} else {
			return false;
		}
	}

	public void leftDrive(double output) {
	    masterLeft.getPIDController().setReference(output, ControlType.kDutyCycle);
	}

	@Override
	public void outputToSmartDashboard() {
		SmartDashboard.putBoolean("HighGear?", isHighGear());
		SmartDashboard.putNumber("driveLeftPosition", getLeftDistanceInches());
		SmartDashboard.putNumber("driveRightPosition", getRightDistanceInches());
		SmartDashboard.putNumber("driveLeftVelocity", getLeftVelocityInchesPerSec());
		SmartDashboard.putNumber("driveRightVelocity", getRightVelocityInchesPerSec());
		SmartDashboard.putNumber("Left Side", masterLeft.getEncoder().getPosition());
		SmartDashboard.putNumber("Right Side: ", masterRight.getEncoder().getPosition());
		SmartDashboard.putNumber("Current Acceleration Value", navx.getWorldLinearAccelZ());
		SmartDashboard.putNumber("LeftA", masterLeft.get());
		SmartDashboard.putNumber("LeftB", leftA.get());
		SmartDashboard.putNumber("RightA", masterRight.get());
		SmartDashboard.putNumber("RightB", rightC.get());

		SmartDashboard.putNumber("Right Motor Percent Output", masterRight.get());
		SmartDashboard.putNumber("Left Motor Percent Output", masterLeft.get());
		SmartDashboard.putString("Drive State", returnDriveState());
		SmartDashboard.putNumber("NavX", getGyroYaw().getDegrees());
		SmartDashboard.putBoolean("Are we in brake mode", mIsBrakeMode);
		SmartDashboard.putNumber("Main Left Drive Temp", masterLeft.getMotorTemperature());
		SmartDashboard.putNumber("Main LeftA Drive Temp", leftA.getMotorTemperature());
		SmartDashboard.putNumber("Main Right Drive Temp", masterRight.getMotorTemperature());
		SmartDashboard.putNumber("Main RightC Drive Temp", rightC.getMotorTemperature());
		SmartDashboard.putNumber("Drive Left Main Amps", masterLeft.getOutputCurrent());
		SmartDashboard.putNumber("Drive Left Follow Amps", leftA.getOutputCurrent());
		SmartDashboard.putNumber("Drive Right Main Amps", masterRight.getOutputCurrent());
		SmartDashboard.putNumber("Drive Right Follow Amps", rightC.getOutputCurrent());
	}


	@Override
	public synchronized void registerEnabledLoops(Looper enabledLooper) {
		enabledLooper.register(mLoop);

	}

	public synchronized void reloadGains() {
		reloadLowGearPositionGains();
		reloadHighGearVelocityGains();
	}

	public synchronized void reloadHighGearPositionGainsForController(CANSparkMax motorController) {
	    motorController.getPIDController().setP(Constants.kDriveHighGearVelocityKp, kHighGearVelocityControlSlot);
        motorController.getPIDController().setI(Constants.kDriveHighGearVelocityKi, kHighGearVelocityControlSlot);
        motorController.getPIDController().setD(Constants.kDriveHighGearVelocityKd, kHighGearVelocityControlSlot);
        motorController.getPIDController().setFF(Constants.kDriveHighGearVelocityKf, kHighGearVelocityControlSlot);
        motorController.getPIDController().setIZone(Constants.kDriveHighGearVelocityIZone, kHighGearVelocityControlSlot);

        /*TODO: Do we need this?
		motorController.configClosedloopRamp(Constants.kDriveHighGearVelocityRampRate, Constants.kDriveTrainPIDSetTimeout);
         */
	}

	public synchronized void reloadHighGearPositionGainsForControllerLowPID(CANSparkMax motorController) {
        motorController.getPIDController().setP(Constants.kDriveHighGearVelocityLowKp, kHighGearVelocityControlSlot);
        motorController.getPIDController().setI(Constants.kDriveHighGearVelocityLowKi, kHighGearVelocityControlSlot);
        motorController.getPIDController().setD(Constants.kDriveHighGearVelocityLowKd, kHighGearVelocityControlSlot);
        motorController.getPIDController().setFF(Constants.kDriveHighGearVelocityLowKf, kHighGearVelocityControlSlot);
        motorController.getPIDController().setIZone(Constants.kDriveHighGearVelocityLowIZone, kHighGearVelocityControlSlot);

                /*TODO: Do we need this?
		motorController.configClosedloopRamp(Constants.kDriveHighGearVelocityLowRampRate, Constants.kDriveTrainPIDSetTimeout);
         */
	}

	public synchronized void reloadHighGearVelocityGains() {
		if (isWantedLowPID) {
			System.out.println("low PID");
			reloadHighGearPositionGainsForControllerLowPID(masterLeft);
			reloadHighGearPositionGainsForControllerLowPID(masterRight);
		} else {
			System.out.println("high PID");
			reloadHighGearPositionGainsForController(masterLeft);
			reloadHighGearPositionGainsForController(masterRight);
		}
	}

	public synchronized void reloadLowGearPositionGains() {
		reloadLowGearPositionGainsForController(masterLeft);
		reloadLowGearPositionGainsForController(masterRight);
	}

	public synchronized void reloadLowGearPositionGainsForController(CANSparkMax motorController) {
        motorController.getPIDController().setP(Constants.kDriveLowGearPositionKp,kLowGearPositionControlSlot);
        motorController.getPIDController().setI(Constants.kDriveLowGearPositionKi,kLowGearPositionControlSlot);
        motorController.getPIDController().setD(Constants.kDriveLowGearPositionKd,kLowGearPositionControlSlot);
        motorController.getPIDController().setFF(Constants.kDriveLowGearPositionKf,kLowGearPositionControlSlot);
        motorController.getPIDController().setIZone(Constants.kDriveLowGearPositionIZone,kLowGearPositionControlSlot);
        motorController.getPIDController().setSmartMotionMaxVelocity(Constants.kDriveLowGearMaxVelocity,kLowGearPositionControlSlot);
        motorController.getPIDController().setSmartMotionMaxAccel(Constants.kDriveLowGearMaxAccel, Constants.kLiftPositionControlPIDSlot);
        /*TODO:DO we need this?
		motorController.configClosedloopRamp(Constants.kDriveLowGearPositionRampRate, Constants.kDriveTrainPIDSetTimeout);
		*/

	}

	public synchronized void resetNavx() {
		navx.reset();
	}

	public synchronized void resetYaw() {
		navx.zeroYaw();
	}

	public String returnDriveState() {
		return mDriveStates.toString();
	}

	/**
	 * Drives only the right side at a percent
	 *
	 * @param output Wanted percent
	 */
	public void rightDrive(double output) {
	    masterRight.getPIDController().setReference(output, ControlType.kDutyCycle);
	}

	/**
	 * Sets the talons for brake mode
	 */
	public synchronized void setBrakeMode() {
		//set for auto
        masterLeft.setIdleMode(CANSparkMax.IdleMode.kBrake);
        masterRight.setIdleMode(CANSparkMax.IdleMode.kBrake);
	}

	/**
	 * Sets the talons up for coast mode
	 */
	public synchronized void setCoastMode() {
		// set for driving
        masterLeft.setIdleMode(CANSparkMax.IdleMode.kCoast);
        masterRight.setIdleMode(CANSparkMax.IdleMode.kCoast);
	}

	/**
	 * Used in OI to set the robot up for creep mode
	 *
	 * @param signal Our left and right drivetrain speed
	 */
	public synchronized void setCreepMode(DriveSignal signal) {
		if (mDriveStates != DriveStates.CREEP) {
			mDriveStates = DriveStates.CREEP;
			System.out.println("CREEP");
		}
		masterLeft.getPIDController().setReference(signal.getLeft() / 2, ControlType.kDutyCycle);
		masterRight.getPIDController().setReference(signal.getRight() / 2, ControlType.kDutyCycle);
	}

	//////

	/**
	 * Used to change the robot heading when needed
	 *
	 * @param angle Wanted angle
	 */
	public synchronized void setGyroAngle(Rotation2d angle) {
		navx.reset();
		navx.setAngleAdjustment(angle);
	}

	/**
	 * Used to set highgear
	 * @param wantsHighGear
	 * it's a boolean saying if you want it or not
	 */
//	public synchronized void setHighGear(boolean wantsHighGear) {
//        if (wantsHighGear != mIsHighGear) {
//            mIsHighGear = wantsHighGear;
//            shifter.set(wantsHighGear ? Value.kForward : Value.kReverse);
//        }
//    }

	/**
	 * Sets the neutral mode for the drive train.
	 *
	 * @param brake if 1, the drive train will go into brake mode, 0 will put it into coast mode
	 */
	public synchronized void setNeutralMode(boolean brake) {
		mIsBrakeMode = brake;
		CANSparkMax.IdleMode currentMode = brake ? CANSparkMax.IdleMode.kBrake : CANSparkMax.IdleMode.kCoast;
		masterRight.setIdleMode(currentMode);
		rightC.setIdleMode(currentMode);
		masterLeft.setIdleMode(currentMode);
		leftA.setIdleMode(currentMode);
	}

	/**
	 * Used to control robot in OI
	 *
	 * @param signal Signal is our left drivetrain and right drivetrian power
	 */
	public synchronized void setOpenLoop(DriveSignal signal) {
		if (mDriveStates != DriveStates.DRIVING) {
			mDriveStates = DriveStates.DRIVING;
			setNeutralMode(false);
		}
        masterLeft.getPIDController().setReference(signal.getLeft(), ControlType.kDutyCycle);
		masterRight.getPIDController().setReference(signal.getRight(), ControlType.kDutyCycle);
	}

	/**
	 * Used to update velocity setpoint and set state
	 *
	 * @param left_inches_per_sec  Left inches per second
	 * @param right_inches_per_sec Right inches per second
	 */
	public synchronized void setVelocitySetpoint(double left_inches_per_sec, double right_inches_per_sec) {
		configureTalonsForSpeedControl();
		mDriveStates = DriveStates.VELOCITY_SETPOINT;
		updateVelocitySetpoint(left_inches_per_sec, right_inches_per_sec);
	}

	/**
	 * Used to setup our pure pursuit controller for auto
	 *
	 * @param path     This is the wanted path that we want to drive on
	 * @param reversed If the robot is reversed or not
	 */
	public synchronized void setWantDrivePath(Path path, boolean reversed) {
		reloadHighGearVelocityGains();
		if (mCurrentPath != path || mDriveStates != DriveStates.PATH_FOLLOWING) {
			System.out.println("Setting Path_Following");
			configureTalonsForSpeedControl();
			RobotState.getInstance().resetDistanceDriven();
			mPathFollower = new PathFollower(path, reversed,
					new PathFollower.Parameters(
							new Lookahead(Constants.kMinLookAhead, Constants.kMaxLookAhead,
									Constants.kMinLookAheadSpeed, Constants.kMaxLookAheadSpeed),
							Constants.kInertiaSteeringGain, Constants.kPathFollowingProfileKp,
							Constants.kPathFollowingProfileKi, Constants.kPathFollowingProfileKv,
							Constants.kPathFollowingProfileKffv, Constants.kPathFollowingProfileKffa,
							Constants.kPathFollowingMaxVel, Constants.kPathFollowingMaxAccel,
							Constants.kPathFollowingGoalPosTolerance, Constants.kPathFollowingGoalVelTolerance,
							Constants.kPathStopSteeringDistance));
			mDriveStates = DriveStates.PATH_FOLLOWING;
			mCurrentPath = path;
		} else {
			System.out.println("setting velocity to 0");
			setVelocitySetpoint(0, 0);
		}
	}

	/**
	 * Configures the drivebase to turn to a desired heading
	 */
	public synchronized void setWantTurnToHeading(Rotation2d heading) {
		if (mDriveStates != DriveStates.TURN_TO_HEADING) {
			mDriveStates = DriveStates.TURN_TO_HEADING;
			configureTalonsForPositionControl();
			setMaxDrivePower(Constants.kDriveTurnMaxPower);
			updatePositionSetpoint(getLeftDistanceInches(), getRightDistanceInches());
		}
		if (Math.abs(heading.inverse().rotateBy(mTargetHeading).getDegrees()) > 1E-3) {
			mTargetHeading = heading;
			mIsOnTarget = false;
		}
	}

	@Override
	public synchronized void stop() {
		stopDrive();

	}

	/**
	 * Stops the drivetrain completely
	 */
	public synchronized void stopDrive() {
		if (mDriveStates != DriveStates.DRIVING) {
			mDriveStates = DriveStates.DRIVING;
		}
		masterLeft.getPIDController().setReference(0, ControlType.kDutyCycle);
		masterRight.getPIDController().setReference(0, ControlType.kDutyCycle);
	}


	/**
	 * Called periodically when the robot is in path following mode. Updates the path follower with the robots latest
	 * pose, distance driven, and velocity, the updates the wheel velocity setpoints.
	 */
	private synchronized void updatePathFollower(double timestamp) {
		RigidTransform2d robot_pose = mRobotState.getLatestFieldToVehicle().getValue();
		Twist2d command = mPathFollower.update(timestamp, robot_pose,
				RobotState.getInstance().getDistanceDriven(), RobotState.getInstance().getPredictedVelocity().dx);
		if (!mPathFollower.isFinished()) {
			Kinematics.DriveVelocity setpoint = Kinematics.inverseKinematics(command);
			updateVelocitySetpoint(setpoint.left, setpoint.right);
			SmartDashboard.putNumber("Left Side Setpoint: ", setpoint.left);
			SmartDashboard.putNumber("Right Side Setpoint: ", setpoint.right);
		} else {
			updateVelocitySetpoint(0, 0);
		}
	}

	/**
	 * Updates the talons to what position it will go to
	 *
	 * @param left_position_inches  Inches wanted
	 * @param right_position_inches Inches wanted
	 */
	private synchronized void updatePositionSetpoint(double left_position_inches, double right_position_inches) {
		if (usesTalonPositionControl(mDriveStates)) {
		    masterLeft.getPIDController().setReference(inchesToRPM(left_position_inches), ControlType.kPosition, kLowGearPositionControlSlot);
			masterRight.getPIDController().setReference(inchesToRPM(right_position_inches), ControlType.kPosition, kLowGearPositionControlSlot);
		} else {
			System.out.println("Hit a bad position control state");
            masterLeft.getPIDController().setReference(0, ControlType.kDutyCycle);
            masterRight.getPIDController().setReference(0, ControlType.kDutyCycle);
		}
	}

	/**
	 * Updates motor speed when turning to an angle
	 *
	 * @param timestamp Current timestamp, don't worry chris it's not used
	 */
	private synchronized void updateTurnToHeading(double timestamp) {
		final Rotation2d field_to_robot = mRobotState.getLatestFieldToVehicle().getValue().getRotation();
		// Figure out the rotation necessary to turn to face the goal.
		final Rotation2d robot_to_target = field_to_robot.inverse().rotateBy(mTargetHeading);

		// Check if we are on target
		final double kGoalPosTolerance = 10; // degrees
		final double kGoalVelTolerance = 5.0; // inches per second
		if (Math.abs(robot_to_target.getDegrees()) < kGoalPosTolerance
				&& Math.abs(getLeftVelocityInchesPerSec()) < kGoalVelTolerance
				&& Math.abs(getRightVelocityInchesPerSec()) < kGoalVelTolerance) {
			// Use the current setpoint and base lock.
			mIsOnTarget = true;
			updatePositionSetpoint(getLeftDistanceInches(), getRightDistanceInches());
			return;
		}

		Kinematics.DriveVelocity wheel_delta = Kinematics
				.inverseKinematics(new Twist2d(0, 0, robot_to_target.getRadians()));
		updatePositionSetpoint(wheel_delta.left + getLeftDistanceInches(),
				wheel_delta.right + getRightDistanceInches());

		SmartDashboard.putNumber("Wanted Right Turn To Heading: ", wheel_delta.right + getRightDistanceInches());
		SmartDashboard.putNumber("Wanted Left Turn to Heading", wheel_delta.left + getLeftDistanceInches());
		SmartDashboard.putNumber("Current Right Turn To Heading", getRightDistanceInches());
		SmartDashboard.putNumber("Current Left Turn To Heading", getLeftDistanceInches());
	}

	/**
	 * Update velocity setpoint is used to send over our desired velocity from pure pursuit control
	 *
	 * @param left_inches_per_sec  Left side inches per second
	 * @param right_inches_per_sec right side inches per second
	 */
	private synchronized void updateVelocitySetpoint(double left_inches_per_sec, double right_inches_per_sec) {
		if (usesTalonVelocityControl(mDriveStates)) {
			final double max_desired = Math.max(Math.abs(left_inches_per_sec), Math.abs(right_inches_per_sec));
			final double scale = max_desired > Constants.kDriveHighGearMaxSetpoint
					? Constants.kDriveHighGearMaxSetpoint / max_desired : 1.0;
			masterLeft.getPIDController().setReference(inchesPerSecondToRPM(left_inches_per_sec * scale), ControlType.kVelocity, kHighGearVelocityControlSlot);
			masterRight.getPIDController().setReference(inchesPerSecondToRPM(right_inches_per_sec * scale), ControlType.kVelocity, kHighGearVelocityControlSlot);
            
           /* System.out.println("Left Side Velocity : "+ left_inches_per_sec+ "  " + 
            		"Right Side Veloctiy: "+ right_inches_per_sec);*/


			SmartDashboard.putNumber("Left Side Velocity", getLeftVelocityInchesPerSec());
			SmartDashboard.putNumber("Right Side Velocity", getRightVelocityInchesPerSec());
		} else {
			System.out.println("Hit a bad velocity control state");
            masterLeft.getPIDController().setReference(0, ControlType.kDutyCycle);
            masterRight.getPIDController().setReference(0, ControlType.kDutyCycle);
		}
	}

	/**
	 * This method zeros the encoders of both sides of the drivetrain
	 */
	@Override
	public synchronized void zeroSensors() {
//		System.out.println("Zeroing drivetrain sensors...");
        masterLeft.getEncoder().setPosition(0);
        masterRight.getEncoder().setPosition(0);
		navx.zeroYaw();
//   	 System.out.println("Drivetrain sensors zeroed!");
	}

	@Override
	public void writeToLog() {
		// TODO Auto-generated method stub

	}

	/**
	 * sets up a parking brake for climbing
	 * <p>
	 * TODO: Do it chris
	 */
	public void setParkingBrakeMode() {
		setBrakeMode();
	}

	/**
	 * sets the max power that the drivetrain can go
	 *
	 * @param power
	 */
	public void setMaxDrivePower(double power) {
	    masterLeft.getPIDController().setOutputRange(-power, power);
	    masterRight.getPIDController().setOutputRange(-power, power);

	}

	public float getWorldLinearAccelX() {
		return navx.getWorldLinearAccelX();
	}

	public float getWorldLinearAccelY() {
		return navx.getWorldLinearAccelY();
	}

	public float getWorldLinearAccelZ() {
		return navx.getWorldLinearAccelZ();
	}


	public void goToHatchMode(){
		//nothing to do here
	}

	public void goToCargoMode(){
		//nothing to do here
	}

	public void retractAll() {
		//nothing to do here
	}
}


