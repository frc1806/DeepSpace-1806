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
        ArrayList<PathBuilder.Waypoint> sWaypoints = new ArrayList<PathBuilder.Waypoint>();
        sWaypoints.add(new PathBuilder.Waypoint(-11,-19,0,0));
        //hope that's the starting point^ TODO: That ain't it, chief.
        sWaypoints.add(new PathBuilder.Waypoint(-35,-4,0,50));
        sWaypoints.add(new PathBuilder.Waypoint(0,-4.5,0,50));
        //these puppies are gonna be adjusted once we get Mr. Roboto on the field
        sWaypoints.add(new PathBuilder.Waypoint(120,220,0,50));
        sWaypoints.add(new PathBuilder.Waypoint(110,200,0,50));

        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }

    @Override
    public RigidTransform2d getStartPose() {
        // TODO: Update startpoint, probably translate entire path based on that info.
        return new RigidTransform2d(new Translation2d(-11, -19), Rotation2d.fromDegrees(0.0));
    }

    @Override
    public boolean isReversed() {
        // TODO Auto-generated method stub
        return false;
    }
}
