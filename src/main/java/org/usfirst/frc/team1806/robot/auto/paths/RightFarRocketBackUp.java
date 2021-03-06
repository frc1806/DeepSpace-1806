package org.usfirst.frc.team1806.robot.auto.paths;

import org.usfirst.frc.team1806.robot.path.Path;
import org.usfirst.frc.team1806.robot.path.PathBuilder;
import org.usfirst.frc.team1806.robot.path.PathBuilder.Waypoint;
import org.usfirst.frc.team1806.robot.path.PathContainer;
import org.usfirst.frc.team1806.robot.util.RigidTransform2d;
import org.usfirst.frc.team1806.robot.util.Rotation2d;
import org.usfirst.frc.team1806.robot.util.Translation2d;

import java.util.ArrayList;

public class RightFarRocketBackUp implements PathContainer {
    @Override
    public Path buildPath() {
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(265,29,0,0));
        sWaypoints.add(new Waypoint(290,44,0,120));

        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }
    @Override

    public RigidTransform2d getStartPose() {
        return new RigidTransform2d(new Translation2d(265, 44), Rotation2d.fromDegrees(180.0));
    }

    @Override
    public boolean isReversed() {
        return true;
    }
    // WAYPOINT_DATA: [{"position":{"x":265,"y":29},"speed":0,"radius":0,"comment":""},{"position":{"x":290,"y":44},"speed":110,"radius":15,"comment":""}]
    // IS_REVERSED: true
    // FILE_NAME: UntitledPath
}
