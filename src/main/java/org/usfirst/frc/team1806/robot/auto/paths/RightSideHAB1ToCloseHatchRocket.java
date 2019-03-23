package org.usfirst.frc.team1806.robot.auto.paths;

import org.usfirst.frc.team1806.robot.RobotState;
import org.usfirst.frc.team1806.robot.path.Path;
import org.usfirst.frc.team1806.robot.path.PathBuilder;
import org.usfirst.frc.team1806.robot.path.PathContainer;
import org.usfirst.frc.team1806.robot.util.RigidTransform2d;
import org.usfirst.frc.team1806.robot.util.Rotation2d;
import org.usfirst.frc.team1806.robot.util.Translation2d;

import java.util.ArrayList;

public class RightSideHAB1ToCloseHatchRocket implements PathContainer {

    @Override
    public Path buildPath() {
        ArrayList<PathBuilder.Waypoint> sWaypoints = new ArrayList<PathBuilder.Waypoint>();

        sWaypoints.add(new PathBuilder.Waypoint(66,115,0,0));
        sWaypoints.add(new PathBuilder.Waypoint(67,115,0,10));
        sWaypoints.add(new PathBuilder.Waypoint(116,115,0,20));
        sWaypoints.add(new PathBuilder.Waypoint(136,74,10,50));
        sWaypoints.add(new PathBuilder.Waypoint(166,50,0,80));

        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }
    @Override

    public RigidTransform2d getStartPose() {
        return new RigidTransform2d(new Translation2d(66, 115), RobotState.getInstance().getLatestFieldToVehicle().getValue().getRotation());
    }

    @Override
    public boolean isReversed() {
        return false;
    }
    // WAYPOINT_DATA: [{"position":{"x":67,"y":115},"speed":0,"radius":0,"comment":""},{"position":{"x":125,"y":115},"speed":60,"radius":10,"comment":""},{"position":{"x":160,"y":65},"speed":60,"radius":0,"comment":""}]
    // IS_REVERSED: false
    // FILE_NAME: UntitledPath
}
