package org.usfirst.frc.team1806.robot.auto.paths;

import java.util.ArrayList;

import org.usfirst.frc.team1806.robot.path.Path;
import org.usfirst.frc.team1806.robot.path.PathBuilder;
import org.usfirst.frc.team1806.robot.path.PathBuilder.Waypoint;
import org.usfirst.frc.team1806.robot.path.PathContainer;
import org.usfirst.frc.team1806.robot.util.RigidTransform2d;
import org.usfirst.frc.team1806.robot.util.Rotation2d;
import org.usfirst.frc.team1806.robot.util.Translation2d;

public class DumbMode implements PathContainer {

	@Override
	public Path buildPath() {
		double speed = 20;
		System.out.println("well you're sitting here");
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
		sWaypoints.add(new Waypoint(0,0,0,0));
		sWaypoints.add(new Waypoint(0.5,0,0, speed));
		sWaypoints.add(new Waypoint(12,0,0, speed));
		sWaypoints.add(new Waypoint(24,12,0, speed));
		sWaypoints.add(new Waypoint(36,12,0, speed));
		sWaypoints.add(new Waypoint(48,0,0, speed));
		sWaypoints.add(new Waypoint(60,0,0, speed));
		return PathBuilder.buildPathFromWaypoints(sWaypoints);
	}

	@Override
	public RigidTransform2d getStartPose() {
		// TODO Auto-generated method stub
		return new RigidTransform2d(new Translation2d(0, 0), Rotation2d.fromDegrees(0.0));
	}

	@Override
	public boolean isReversed() {
		// TODO Auto-generated method stub
		return false;
	}

}
