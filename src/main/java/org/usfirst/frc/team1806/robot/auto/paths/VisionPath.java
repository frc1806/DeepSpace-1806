package org.usfirst.frc.team1806.robot.auto.paths;

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
    VisionPath(RigidTransform2d odo) {
        odometry = odo;
    }
    public final int speed = 30;
    @Override
    public Path buildPath() {
        ArrayList<PathBuilder.Waypoint> sWaypoints = new ArrayList<PathBuilder.Waypoint>();
        sWaypoints.add(new PathBuilder.Waypoint(trackedBay.getBayCoordinates().getTranslation().x() + 20 * Math.cos(trackedBay.bayCoordinates.getRotation().getRadians()),
                trackedBay.getBayCoordinates().getTranslation().y() + 20 * Math.sin(trackedBay.bayCoordinates.getRotation().getRadians()), 0, speed));

        return PathBuilder.buildPathFromWaypoints(sWaypoints);

    }

    @Override
    public RigidTransform2d getStartPose() {
        return odometry;
    }

    @Override
    public boolean isReversed() {
        return false;
    }
}
