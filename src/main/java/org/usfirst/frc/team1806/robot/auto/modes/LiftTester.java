package org.usfirst.frc.team1806.robot.auto.modes;

import org.usfirst.frc.team1806.robot.auto.AutoModeBase;
import org.usfirst.frc.team1806.robot.auto.AutoModeEndedException;
import org.usfirst.frc.team1806.robot.auto.actions.actionUtil.WaitAction;
import org.usfirst.frc.team1806.robot.auto.actions.LiftToHeight;
import org.usfirst.frc.team1806.robot.subsystems.LiftSubsystem;

/**
 * Lift Tester auto will be used to test our lift very quickly whenever we get to an event
 *
 * Self-Tester
 */
public class LiftTester extends AutoModeBase {
    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(new LiftToHeight(LiftSubsystem.LiftPosition.ROCKET_CARGO_HIGH, false));
        runAction(new WaitAction(1));
        runAction(new LiftToHeight(LiftSubsystem.LiftPosition.ROCKET_CARGO_LOW, false));
        runAction(new WaitAction(1));
        runAction(new LiftToHeight(LiftSubsystem.LiftPosition.ROCKET_CARGO_MID, false));
        runAction(new WaitAction(1));
        runAction(new LiftToHeight(LiftSubsystem.LiftPosition.TELEOP_HOLD, false));
        runAction(new WaitAction(15));
    }
}
