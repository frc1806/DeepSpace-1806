package org.usfirst.frc.team1806.robot.auto.paths;

import org.usfirst.frc.team1806.robot.path.Path;
import edu.wpi.first.wpilibj.DriverStation;
import org.usfirst.frc.team1806.robot.path.PathBuilder;
import org.usfirst.frc.team1806.robot.path.PathBuilder.Waypoint;
import org.usfirst.frc.team1806.robot.path.PathContainer;
import org.usfirst.frc.team1806.robot.util.RigidTransform2d;
import org.usfirst.frc.team1806.robot.util.Rotation2d;
import org.usfirst.frc.team1806.robot.util.Translation2d;

import java.util.ArrayList;



public class TrainingPath implements PathContainer{

    /*
    I completely redid this, you guys need to make your path classes implement path container
                -Dillon
     */
    @Override
    public Path buildPath() {
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(22,120,0,0));
        sWaypoints.add(new Waypoint(140,120,20,110));
        sWaypoints.add(new Waypoint(260,60,20,110));
        sWaypoints.add(new Waypoint(306,60,18,90));
        sWaypoints.add(new Waypoint(306,81,0,90));

        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }

    @Override
    public RigidTransform2d getStartPose() {
        return new RigidTransform2d(new Translation2d(22, 120), Rotation2d.fromDegrees(0));
    }


    @Override
    public boolean isReversed() {
        return false;
    }
    // WAYPOINT_DATA: [{"position":{"x":50,"y":50},"speed":0,"radius":0,"comment":""},{"position":{"x":100,"y":50},"speed":80,"radius":10,"comment":""},{"position":{"x":150,"y":90},"speed":80,"radius":10,"comment":""},{"position":{"x":150,"y":160},"speed":60,"radius":0,"comment":""}]
    // IS_REVERSED: false
    // FILE_NAME: UntitledPath

}
