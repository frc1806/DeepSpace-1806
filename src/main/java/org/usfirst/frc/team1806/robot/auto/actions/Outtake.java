package org.usfirst.frc.team1806.robot.auto.actions;

import org.usfirst.frc.team1806.robot.auto.actions.actionUtil.Action;
import org.usfirst.frc.team1806.robot.subsystems.CargoIntakeSubsystem;

public class Outtake implements Action {

    private boolean instant;
    private CargoIntakeSubsystem.ScoringPower scoringPower;

    public Outtake(CargoIntakeSubsystem.ScoringPower _power, boolean _instant){
        instant = _instant;
        scoringPower = _power;
    }

    CargoIntakeSubsystem mCargoIntakeSubsystem = CargoIntakeSubsystem.getInstance();

    @Override
    public boolean isFinished() {
        return instant; //We need a sensor
    }

    @Override
    public void update() {

    }

    @Override
    public void done() {
        mCargoIntakeSubsystem.stop();
    }

    @Override
    public void start() {
        mCargoIntakeSubsystem.scoreCargo(scoringPower);
    }
}
