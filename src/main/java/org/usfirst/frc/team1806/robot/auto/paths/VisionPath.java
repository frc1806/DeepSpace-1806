package org.usfirst.frc.team1806.robot.auto.paths;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.usfirst.frc.team1806.robot.RobotState;
import org.usfirst.frc.team1806.robot.path.Path;
import org.usfirst.frc.team1806.robot.path.PathBuilder;
import org.usfirst.frc.team1806.robot.path.PathContainer;
import org.usfirst.frc.team1806.robot.util.RigidTransform2d;
import org.usfirst.frc.team1806.robot.util.Rotation2d;
import org.usfirst.frc.team1806.robot.util.Translation2d;

import java.util.ArrayList;

public class VisionPath implements PathContainer {
    public enum BayLocation {
        CARGO_SHIP_SIDE_1_AUDIANCE(new RigidTransform2d(new Translation2d(0,0), Rotation2d.fromDegrees(0))),
        CARGO_SHIP_SIDE_2_AUDIANCE(new RigidTransform2d(new Translation2d(), new Rotation2d())),
        CARGO_SHIP_SIDE_3_AUDIANCE(new RigidTransform2d(new Translation2d(), new Rotation2d())),
        CARGO_SHIP_SIDE_1_NONAUDIANCE(new RigidTransform2d(new Translation2d(), new Rotation2d())),
        CARGO_SHIP_SIDE_2_NONAUDIANCE(new RigidTransform2d(new Translation2d(), new Rotation2d())),
        CARGO_SHIP_SIDE_3_NONAUDIANCE(new RigidTransform2d(new Translation2d(), new Rotation2d())),
        CARGO_SHIP_FRONT_AUDIANCE(new RigidTransform2d(new Translation2d(), new Rotation2d())),
        CARGO_SHIP_FRONT_NONAUDIANCE(new RigidTransform2d(new Translation2d(), new Rotation2d()));
        public RigidTransform2d getBayCoordinates() {
            return bayCoordinates;
        }

        private RigidTransform2d bayCoordinates;
        BayLocation(RigidTransform2d bayCoordinates) {
            this.bayCoordinates = bayCoordinates;
        }
    }

    BayLocation trackedBay = BayLocation.CARGO_SHIP_FRONT_AUDIANCE;
    RigidTransform2d odometry;
    public final int speed = 40;
    public VisionPath(RigidTransform2d odo) {
        odometry = odo;
    }
    @Override
    public Path buildPath() {
        double fps = 1;
        ArrayList<PathBuilder.Waypoint> sWaypoints = new ArrayList<PathBuilder.Waypoint>();
        if(false) {
            RigidTransform2d roboPose = generateTemporaryVisionPose();
            sWaypoints.add(new PathBuilder.Waypoint(roboPose.getTranslation().x(), roboPose.getTranslation().y(), 0, speed));
            sWaypoints.add(new PathBuilder.Waypoint(0, roboPose.getTranslation().y()- 4, 0, speed));
            sWaypoints.add(new PathBuilder.Waypoint(0, 20, 0, speed));
            sWaypoints.add(new PathBuilder.Waypoint(0, 18, 0, speed));
        }
        else {
            fps = SmartDashboard.getNumber("Vfps", 0);

            RigidTransform2d roboPose = RobotState.getInstance().getLatestFieldToVehicle().getValue();
            System.out.println("Field to vehicle: (" + roboPose.getTranslation().x() + ", " + roboPose.getTranslation().y() + ")");
            //RigidTransform2d bayyPose = generateBayVisionPoseFromODO();
            RigidTransform2d bayyPose = new RigidTransform2d(new Translation2d(roboPose.getTranslation().x() - odometry.getTranslation().x(), roboPose.getTranslation().y() + odometry.getTranslation().y()), Rotation2d.fromDegrees(roboPose.getRotation().getDegrees() + odometry.getRotation().getDegrees()));
            sWaypoints.add(new PathBuilder.Waypoint(interpolateAlongLine(roboPose, 0, roboPose.getRotation().getRadians()), 0, speed));
            System.out.println("(" + interpolateAlongLine(roboPose, 0, roboPose.getRotation().getRadians()).x() + ", " + interpolateAlongLine(roboPose, 0, roboPose.getRotation().getRadians()).y() + ")");
            sWaypoints.add(new PathBuilder.Waypoint(interpolateAlongLine(roboPose, 0.5, roboPose.getRotation().getRadians()), 0, speed));
            System.out.println("(" + interpolateAlongLine(roboPose, 0.5, roboPose.getRotation().getRadians()).x() + ", " + interpolateAlongLine(roboPose, 0.5, roboPose.getRotation().getRadians()).y() + ")");
            sWaypoints.add(new PathBuilder.Waypoint(interpolateAlongLine(bayyPose, -30, bayyPose.getRotation().getRadians()), 0, speed));
            sWaypoints.add(new PathBuilder.Waypoint(interpolateAlongLine(bayyPose, -26, bayyPose.getRotation().getRadians()), 0, speed));
            sWaypoints.add(new PathBuilder.Waypoint(interpolateAlongLine(bayyPose, -23, bayyPose.getRotation().getRadians()), 0, speed));
            System.out.println("(" + interpolateAlongLine(bayyPose, 25, bayyPose.getRotation().getRadians()).x() + ", " + interpolateAlongLine(bayyPose, 25, bayyPose.getRotation().getRadians()).y() + ")");
            sWaypoints.add(new PathBuilder.Waypoint(interpolateAlongLine(bayyPose, -21, bayyPose.getRotation().getRadians()), 0, speed));
            System.out.println("(" + interpolateAlongLine(bayyPose, 23, bayyPose.getRotation().getRadians()).x() + ", " + interpolateAlongLine(bayyPose, 23, bayyPose.getRotation().getRadians()).y() + ")");
            sWaypoints.add(new PathBuilder.Waypoint(interpolateAlongLine(bayyPose, -18, bayyPose.getRotation().getRadians()), 0, speed));
            System.out.println("(" + interpolateAlongLine(bayyPose, 21, bayyPose.getRotation().getRadians()).x() + ", " + interpolateAlongLine(bayyPose, 21, bayyPose.getRotation().getRadians()).y() + ")");
        }
        return PathBuilder.buildPathFromWaypoints(sWaypoints);

    }
    public RigidTransform2d generateTemporaryVisionPose() {
        double dist = SmartDashboard.getNumber("Vdistance", 0);
        double angle= SmartDashboard.getNumber("Vangle", 0);

        double x = dist * Math.sin(angle);
        double y = dist * Math.cos(angle);

        return new RigidTransform2d(new Translation2d(x,y), Rotation2d.fromDegrees(angle));
    }
    public RigidTransform2d generateBayVisionPoseFromODO() {
        double fps  = SmartDashboard.getNumber("Vfps", 0);
        double dist = SmartDashboard.getNumber("Vdistance", 0);
        double correctionAngle= SmartDashboard.getNumber("Vangle", 0);

        RigidTransform2d robotPose = RobotState.getInstance().getPredictedFieldToVehicle(1/fps);
        double goalHeading = robotPose.getRotation().getDegrees() - correctionAngle;
        Translation2d bayPose = interpolateAlongLine(robotPose, dist, goalHeading);

        return new RigidTransform2d(new Translation2d(bayPose.x(),bayPose.y()), Rotation2d.fromDegrees(goalHeading));
    }

    public Translation2d interpolateAlongLine(RigidTransform2d point, double adjust, double heading) {
        double X = point.getTranslation().x() + adjust * Math.cos(heading);
        double Y = point.getTranslation().y() + adjust * Math.sin(heading);

        return new Translation2d(X,Y);

    }
    public RigidTransform2d interpolateAlongLine(RigidTransform2d point, double adjust, double heading, double heading2) {
        double X = point.getTranslation().x() + adjust * Math.cos(heading);
        double Y = point.getTranslation().y() + adjust * Math.sin(heading);

        return new RigidTransform2d(new Translation2d(X,Y), Rotation2d.fromRadians(heading2));

    }
    @Override
    public RigidTransform2d getStartPose() {
        return RobotState.getInstance().getLatestFieldToVehicle().getValue();
    }

    @Override
    public boolean isReversed() {
        return false;
    }
}
