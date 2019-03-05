package org.usfirst.frc.team1806.robot;

import org.usfirst.frc.team1806.robot.util.Translation2d;

public class Constants {
    public final static boolean enableAutoInTeleOp = false;
    public final static double kLooperDt = 0.005;
    public final static double kDriveWheelDiameterInches = 4;
    public final static double kTrackWidthInches = 27.5;
    public final static double kTrackScrubFactor = .978;
    

    public final static int kDriveTrainPIDSetTimeout = 30;
    public final static double kCountsPerInch = 323.8833166666667;
    public final static double kDriveInchesPerRevolution = 0.0030875316774318;

    ///Motion
    public final static double kMinLookAhead = 9; // inches
    public final static double kMinLookAheadSpeed = 9.0; // inches per second
    public final static double kMaxLookAhead = 42; // inches
    public final static double kMaxLookAheadSpeed = 120.0; // inches per second
    public final static double kDeltaLookAhead = kMaxLookAhead - kMinLookAhead;
    public final static double kDeltaLookAheadSpeed = kMaxLookAheadSpeed - kMinLookAheadSpeed;

    public final static double kInertiaSteeringGain = 0.0; // angular velocity command is multiplied by this gain *
    // our speed
    // in inches per sec
    public final static double kSegmentCompletionTolerance = 0.75; // inches
    public final static double kPathFollowingMaxAccel = 144; // inches per second^2
    public final static double kPathFollowingMaxVel = 144; // inches per second
    public final static double kPathFollowingProfileKp = 1.15;
    public final static double kPathFollowingProfileKi = 0.05;
    public final static double kPathFollowingProfileKv = 0.00002;
    public final static double kPathFollowingProfileKffv = 1.2;
    public final static double kPathFollowingProfileKffa = 0.05;
    public final static double kPathFollowingGoalPosTolerance = 0.75;
    public final static double kPathFollowingGoalVelTolerance = 18.0;
    public final static double kPathStopSteeringDistance = 9.0;

    //
    // PID gains for drive velocity loop (HIGH GEAR)
    // Units: setpoint, error, and output are in counts per tenth of a second
    public final static double kDriveHighGearVelocityKp = 1.01; // 1.2/1500;
    public final static double kDriveHighGearVelocityKi = 0.0000000; //0.0;
    public final static double kDriveHighGearVelocityKd =  7.8; //0.0001; //6.0/1500;
    public final static double kDriveHighGearVelocityKf = 0.21; //.025;
    public final static int kDriveHighGearVelocityIZone = 0;
    public final static double kDriveHighGearVelocityRampRate = .1;
    public final static double kDriveHighGearNominalOutput = 0.25;
    public final static double kDriveHighGearMaxSetpoint = 12 * 12; //FPS

    // PID gains for drive velocity loop (HIGH GEAR)
    // Units: setpoint, error, and output are in counts per tenth of a second
    public final static double kDriveHighGearVelocityLowKp = .082; // 1.2/1500;
    public final static double kDriveHighGearVelocityLowKi = 0.0000000; //0.0;
    public final static double kDriveHighGearVelocityLowKd = .42; //0.0001; //6.0/1500;
    public final static double kDriveHighGearVelocityLowKf = 0.21; //.025;
    public final static int kDriveHighGearVelocityLowIZone = 0;
    public final static double kDriveHighGearVelocityLowRampRate = .1;
    public final static double kDriveHighGearLowNominalOutput = 0.25;
    public final static double kDriveHighGearLowMaxSetpoint = 10.5 * 12; //FPS

    // PID gains for drive velocity loop (LOW GEAR)
    // Units: setpoint, error, and output are in counts
    public final static double kDriveLowGearPositionKp = .15;
    public final static double kDriveLowGearPositionKi = 0.00;
    public final static double kDriveLowGearPositionKd = 0;
    public final static double kDriveLowGearPositionKf = 0.0;
    public final static int kDriveLowGearPositionIZone = 250;
    public final static double kDriveLowGearPositionRampRate = 240.0; // V/s
    public final static double kDriveLowGearNominalOutput = 0.5; // V
    public final static int kDriveLowGearMaxVelocity = 20146; // Counts
    public final static int kDriveLowGearMaxAccel = 20146; // Counts
    public final static double kDriveTurnMaxPower = .6;

    public final static double kLiftHoldPercentOutput = .17;
    // Encoder constants used by Rocket Elevator system
    public final static int kCreepModeLiftHeight = 13000;

    public final static int kLiftPositionControlPIDSlot = 10;
    public final static int kLiftPositionPIDTimeout = 10;
    public final static double kLiftPositionkP = 1;
    public final static double kLiftPositionkI = 0.002;
    public final static double kLiftPositionkD = 0.0015;
    public final static double kLiftPositionkF = 1 / 20000;
    public final static int kLiftPositionIZone = 800;
    public final static double kLiftPositionRampRate = 0;
    public final static int kBottomLimitTolerance = 50;

    public final static int kLiftPositionTolerance = 500;
    public final static int kLiftVelocityTolerance = 100;

    public final static double liftSpeed = .2;


    public final static double kLiftHoldkPGain = .00005;
    public final static int kBumpEncoderPosition = 1500;

    public final static int kLiftTopLimitSwitchPosition = 500;

    public final static int kTeleOpHoldHeight = 1100;



    //Compressor Control Constants
    public final static int kPressureSensorSamplingLoops = 100; //each loop is 1/200 of a second
    public final static double kPressureAverageMinimumToStart = 80; //PSI

    public final static int kBatteryVoltageSamplingLoops = 100; //each loop is 1/200 of a second
    public final static double kBatteryVoltageCompressorShutoffThreshold = 11.5; //volts
    public final static double kBatteryVoltageAbsoluteCompressorShutoffThreshold = 8.0; //volts

    public final static int kRobotDemandAmpsSamplingLoops = 100; // each loop is 1/200 of a second
    public final static double kAverageAmpDemandToShutOffCompressor = 140; //Amps
    public final static double kAbsoluteRobotCompressorShutOffAmps = 300; //Amps

    public final static double kExpectedCompressorMaxCurrentDraw = 100;
    public final static double kExpectedCompressorVoltageDrop = 1.5;


    //Intake Constants
    public final static double kInnerIntakingSpeed = .8;
    public final static double kOuterIntakingSpeed = .8;

    //Battery State Of Charge
    public final static double kFullChargeBatteryCoulombCount = 25200;
    public final static double kBatteryFullChargeVoltage = 12.7;
    public final static double kBatteryDepletedVoltage = 11.8;
    public final static double kLowAmpLoad = 10;

    //Accelerometer Constants
    public final static double habDropAccelerationThreshold = 1.0; //g force


    //Lift interference avoidence
    public final static double kLiftWaitForExtendIntake = 0.25;
    public final static int kMaxLiftHeightToNeedToExtendIntake = 1000;
    public final static int kSafeLiftHeightOffsetToNotHitIntake = 500; // total of this and the line above will be the setpoint

    //controls
    public final static double kTriggerThreshold = .2;

}
