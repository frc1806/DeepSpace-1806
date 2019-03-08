package org.usfirst.frc.team1806.robot.auto.actions;

import org.usfirst.frc.team1806.robot.auto.actions.actionUtil.Action;
import org.usfirst.frc.team1806.robot.subsystems.CargoIntakeSubsystem;

public class Intake implements Action {

    private boolean instant;
    private CargoIntakeSubsystem.ScoringPower scoringPower;

    public Intake(CargoIntakeSubsystem.ScoringPower _power, boolean _instant){
        scoringPower = _power;
        instant = _instant;
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
        //IRRESPONSIBLE is normal intake speed
        mCargoIntakeSubsystem.intakeAtPower(scoringPower);
    }
}