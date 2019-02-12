package org.usfirst.frc.team1806.robot.subsystems;

import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.usfirst.frc.team1806.robot.Robot;
import org.usfirst.frc.team1806.robot.RobotMap;
import org.usfirst.frc.team1806.robot.loop.Looper;
import org.usfirst.frc.team1806.robot.util.RigidTransform2d;
import org.usfirst.frc.team1806.robot.util.Rotation2d;
import org.usfirst.frc.team1806.robot.util.Translation2d;

public class SonarPositioningSubsystem implements Subsystem{

    private static SonarPositioningSubsystem mSonarPositioningSubsystem = new SonarPositioningSubsystem();
    private Ultrasonic rearLeft, rearRight, left, right;
    public static SonarPositioningSubsystem getInstance() {return mSonarPositioningSubsystem;}

    private SonarPositioningSubsystem(){
        //frontLeft = new Ultrasonic(RobotMap.frontLeftTrigger, RobotMap.frontLeftResponse);
        //frontRight = new Ultrasonic(RobotMap.frontRightTrigger, RobotMap.frontRightResponse);
        rearLeft = new Ultrasonic(RobotMap.rearLeftTrigger, RobotMap.rearLeftResponse);
        rearRight = new Ultrasonic(RobotMap.rearRightTrigger, RobotMap.rearRightResponse);
        left = new Ultrasonic(RobotMap.leftTrigger,RobotMap.leftResponse);
        right = new Ultrasonic(RobotMap.rightTrigger, RobotMap.rightResponse);
        rearLeft.setAutomaticMode(true);
        rearLeft.ping();
    }
    public RigidTransform2d getRightFowardsStartPose(){
        return new RigidTransform2d (new Translation2d(0,0), Rotation2d.fromDegrees(0));
    }
    public RigidTransform2d getRightBackwardsStartPose(){
        return new RigidTransform2d (new Translation2d(0,0), Rotation2d.fromDegrees(0));
    }
    public RigidTransform2d getLeftFowardsStartPose(){
        return new RigidTransform2d (new Translation2d(0,0), Rotation2d.fromDegrees(0));
    }
    public RigidTransform2d getLeftBackwardsStartPose(){
        return new RigidTransform2d (new Translation2d(0,0), Rotation2d.fromDegrees(0));
    }
    public double getAverageBackOfRobotDistance(){
        return (rearLeft.getRangeInches() + rearRight.getRangeInches()) / 2;
    }
    final double rearOffset = 10;
    final double sideOffset = 11;
    final double rearSeperation = 16;
    public RigidTransform2d getRobotPoseFromUltrasonic() {
        double robotAngle = Math.atan((rearLeft.getRangeInches() - rearRight.getRangeInches())/rearSeperation);
        return new RigidTransform2d (new Translation2d(0,0), Rotation2d.fromDegrees(0));
    }

   /* public double rawSonarVal() {

    }*/


    public void writeToLog() {

    }

    public void outputToSmartDashboard(){

        SmartDashboard.putNumber("Ultrasonic Left", left.getRangeInches());
        SmartDashboard.putNumber("Ultrasonic Right", right.getRangeInches());
        SmartDashboard.putNumber("Ultrasonic Rear Left", rearLeft.getRangeInches());
        SmartDashboard.putNumber("Ultrasonic Rear Right", rearRight.getRangeInches());
    }

    public void stop(){

    }

    public void zeroSensors(){

    }

    public void registerEnabledLoops(Looper enabledLooper){

    }


    public void goToHatchMode(){
        //sonars don't care about game piece mode
    }

    public void goToCargoMode(){
        //sonars don't care about game piece mode
    }

    public void retractAll() {
        //sonars don't need retracting
    }
}