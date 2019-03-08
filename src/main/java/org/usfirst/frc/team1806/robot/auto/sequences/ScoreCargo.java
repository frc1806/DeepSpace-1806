package org.usfirst.frc.team1806.robot.auto.sequences;

import org.usfirst.frc.team1806.robot.auto.AutoModeBase;
import org.usfirst.frc.team1806.robot.auto.AutoModeEndedException;
import org.usfirst.frc.team1806.robot.auto.actions.Outtake;

import static org.usfirst.frc.team1806.robot.subsystems.CargoIntakeSubsystem.ScoringPower.FAST;

public class ScoreCargo extends AutoModeBase {

    @Override
    protected void routine() throws AutoModeEndedException {
        //Path from vision
        runAction(new Outtake(FAST, false));
    }
}
