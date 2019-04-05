package org.usfirst.frc.team1806.robot.auto.paths;

import org.usfirst.frc.team1806.robot.RobotState;
import org.usfirst.frc.team1806.robot.path.Path;
import org.usfirst.frc.team1806.robot.path.PathBuilder;
import org.usfirst.frc.team1806.robot.path.PathContainer;
import org.usfirst.frc.team1806.robot.util.RigidTransform2d;
import org.usfirst.frc.team1806.robot.util.Translation2d;

import java.util.ArrayList;

public class RightSideHAB1ToCloseHatchRocketNoVision implements PathContainer {

    @Override
    public Path buildPath() {
        ArrayList<PathBuilder.Waypoint> sWaypoints = new ArrayList<PathBuilder.Waypoint>();

        sWaypoints.add(new PathBuilder.Waypoint(116,118,0,0));
        sWaypoints.add(new PathBuilder.Waypoint(117,118,0,60));
        sWaypoints.add(new PathBuilder.Waypoint(136,100,6,60));
        sWaypoints.add(new PathBuilder.Waypoint(156,47,6,50));
        sWaypoints.add(new PathBuilder.Waypoint(176,37,0,50));

        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }
    @Override

    public RigidTransform2d getStartPose() {
        return new RigidTransform2d(new Translation2d(116, 118), RobotState.getInstance().getLatestFieldToVehicle().getValue().getRotation());
    }

    @Override
    public boolean isReversed() {
        return false;
    }
    // WAYPOINT_DATA: [{"position":{"x":67,"y":115},"speed":0,"radius":0,"comment":""},{"position":{"x":125,"y":115},"speed":60,"radius":10,"comment":""},{"position":{"x":160,"y":65},"speed":60,"radius":0,"comment":""}]
    // IS_REVERSED: false
    // FILE_NAME: UntitledPath
}