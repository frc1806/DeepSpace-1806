package org.usfirst.frc.team1806.robot.subsystems;

import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.usfirst.frc.team1806.robot.RobotMap;
import org.usfirst.frc.team1806.robot.loop.Loop;
import org.usfirst.frc.team1806.robot.loop.Looper;
import org.usfirst.frc.team1806.robot.util.RigidTransform2d;
import org.usfirst.frc.team1806.robot.util.Rotation2d;
import org.usfirst.frc.team1806.robot.util.Translation2d;

import java.util.ArrayList;
import java.util.List;

public class SonarPositioningSubsystem implements Subsystem{

    private static SonarPositioningSubsystem mSonarPositioningSubsystem = new SonarPositioningSubsystem();
    private Ultrasonic rearLeftSonar, rearRightSonar, leftSonar, rightSonar;
    private double lastPing;
    private int sonarIndex;
    public static SonarPositioningSubsystem getInstance() {return mSonarPositioningSubsystem;}

    private SonarPositioningSubsystem(){
        rearLeftSonar = new Ultrasonic(RobotMap.rearLeftTrigger, RobotMap.rearLeftResponse);
        rearRightSonar = new Ultrasonic(RobotMap.rearRightTrigger, RobotMap.rearRightResponse);
        //leftSonar = new Ultrasonic(RobotMap.leftTrigger,RobotMap.leftResponse);
        rightSonar = new Ultrasonic(RobotMap.rightTrigger, RobotMap.rightResponse);
       /* rearLeftSonar.setAutomaticMode(true);
        rearRightSonar.setAutomaticMode(true);
        rearRightSonar.setAutomaticMode(true);
        */
       lastPing = 0;
       sonarIndex = 0;
    }

    /**
     * @return average range of the rear left and the rear right sonars
     */
    public double getAverageBackOfRobotDistance(){
        return (rearLeftSonar.getRangeInches() + rearRightSonar.getRangeInches()) / 2;
    }

    /**
     * @return sonar calculated angle off of the perpendicular projection of the alliance wall
     */
    public double getAngle() {
        return Math.atan((rearLeftSonar.getRangeInches() - rearRightSonar.getRangeInches()) / rearSeperation);
    }

    final double rearToCenterX = 10;
    final double rightToCenterX = 10;
    final double rightToCenterY = 10;
    final double leftToCenterX = 10;
    final double leftToCenterY = 10;

    final double sideOffset = 11;
    final double rearSeperation = 14.3;

    double xDisplacementToBack = -1;
    double yDisplacementToSide = -1;
    double xDisplacementToCenter = -1;
    double yDisplacementToCenter = -1;
    double theta = -1;
    double distBack = -1;
    double distSide = -1;


    private Loop mLoop = new Loop() {
        @Override
        public synchronized void onLoop(double timestamp) {
            synchronized (SonarPositioningSubsystem.this) {
                if(timestamp - lastPing > .2) {
                    switch (sonarIndex) {
                        case 1:
                        default:
                            rearRightSonar.ping();
                            //leftSonar.ping()
                            sonarIndex = 2;
                            break;
                        case 2:
                            rearLeftSonar.ping();
                            rightSonar.ping();
                            sonarIndex = 1;
                            break;

                    }

                    lastPing = timestamp;

                    }
                }

            }



        @Override
        public synchronized void onStart(double timestamp) {
            synchronized (SonarPositioningSubsystem.this) {
                lastPing = timestamp;
            }
        }

        @Override
        public synchronized void onStop(double timestamp) {


        }
    };
    /**
     * Stores sensor values in method vars
     * Calculates dist from alliance wall to center of back bumper, then snatches range from right sonar to side wall
     * converts to global X and Y displacements to these same locations
     * Finds the Xs and the Ys to the center of the robot
     *
     * @return X and Y global coordinates of the robot with a sonar calculated angle
     */
    public RigidTransform2d getRobotPoseFromUltrasonic() {
        //save sensor data so that it doesn't change between calculations
        theta = getAngle();

        distBack = getAverageBackOfRobotDistance();
        distSide = rightSonar.getRangeInches();

        //use calculated angle to find x and y of the robot
        xDisplacementToBack = Math.cos(theta) * distBack;
        yDisplacementToSide = Math.cos(theta) * distSide;

        //get global coordinates with respect to the center of the robot
        xDisplacementToCenter = xDisplacementToBack + rearToCenterX * Math.cos(theta);
        yDisplacementToCenter = Math.cos(getAngle()) * rightToCenterY - Math.sin(theta) * rightToCenterX;

        //TODO add functionality for the left position
        //TODO be able to mathematically normalize cooridinates to bottom left of field (on cheesy path)

        return new RigidTransform2d (new Translation2d(xDisplacementToCenter, yDisplacementToCenter), Rotation2d.fromDegrees(theta));
    }

    public void writeToLog() {
    }


    public void outputToSmartDashboard(){
        SmartDashboard.putNumber("Ultrasonic angle", Math.toDegrees(getAngle()));

        SmartDashboard.putNumber("Ultrasonic Rear Left", rearLeftSonar.getRangeInches());
        SmartDashboard.putNumber("Ultrasonic Rear Right", rearRightSonar.getRangeInches());
        //SmartDashboard.putNumber("Ultrasonic Left", leftSonar.getRangeInches());
        SmartDashboard.putNumber("Ultrasonic Right", rightSonar.getRangeInches());

        SmartDashboard.putNumber("Ultrasonic back dist", getAverageBackOfRobotDistance());

        SmartDashboard.putNumber("Ultrasonic calc xDisplacementToBack" , xDisplacementToBack);
        SmartDashboard.putNumber("Ultrasonic calc yDisplacementToSide" , yDisplacementToSide);
        SmartDashboard.putNumber("Ultrasonic calc xDisplacementToCenter" , xDisplacementToCenter);
        SmartDashboard.putNumber("Ultrasonic calc yDisplacementToCenter" , yDisplacementToCenter);
        SmartDashboard.putNumber("Ultrasonic calc theta" , theta);

    }

    /**
     * closes the sonar objects
     */
    public void stop(){
        rearRightSonar.close();
        rearLeftSonar.close();
        //leftSonar.close();
        rightSonar.close();

    }

    /**
     * reinitializes sonars for now
     */
    public void zeroSensors(){
        rearLeftSonar.setDistanceUnits(Ultrasonic.Unit.kInches);
        rearRightSonar.setDistanceUnits(Ultrasonic.Unit.kInches);
        //leftSonar.setDistanceUnits(Ultrasonic.Unit.kInches);
        rightSonar.setDistanceUnits(Ultrasonic.Unit.kInches);
/*
        rearLeftSonar.setAutomaticMode(true);
        rearRightSonar.setAutomaticMode(true);
        //leftSonar.setAutomaticMode(true);
        //rightSonar.setAutomaticMode(true);
        */
    }

    /**
     * nothing because loops are automatic for the time being
     * @param enabledLooper
     */
    public void registerEnabledLoops(Looper enabledLooper){
        enabledLooper.register(mLoop);
    }

    /**
     * sonars don't care about game piece mode
     */
    public void goToHatchMode(){}

    /**
     * sonars don't care about game piece mode
     */
    public void goToCargoMode(){}

    /**
     * sonars don't need retracting
     */
    public void retractAll() {}
}