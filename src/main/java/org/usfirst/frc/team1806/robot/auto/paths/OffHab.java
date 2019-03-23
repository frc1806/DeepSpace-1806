package org.usfirst.frc.team1806.robot.auto.paths;

import org.usfirst.frc.team1806.robot.path.Path;
import org.usfirst.frc.team1806.robot.path.PathBuilder;
import org.usfirst.frc.team1806.robot.path.PathContainer;
import org.usfirst.frc.team1806.robot.util.RigidTransform2d;

import java.util.ArrayList;

public class OffHab implements PathContainer {
    @Override
    public Path buildPath() {
        ArrayList<PathBuilder.Waypoint> sWaypoints = new ArrayList<PathBuilder.Waypoint>();
        sWaypoints.add(new PathBuilder.Waypoint(66,115,0,0));
        sWaypoints.add(new PathBuilder.Waypoint(67,115,0,10));
        sWaypoints.add(new PathBuilder.Waypoint(116,115,0,20));
        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }

    @Override
    public RigidTransform2d getStartPose() {
        return null;
    }

    @Override
    public boolean isReversed() {
        return false;
    }
}
