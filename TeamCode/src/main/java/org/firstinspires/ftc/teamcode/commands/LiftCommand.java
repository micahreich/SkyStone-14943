package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.subsystems.Lift;

public class LiftCommand implements Command {

    private Lift m_lift;

    public LiftCommand(Lift lift) {
        m_lift = lift;
    }

    @Override
    public void execute() {
        m_lift.moveLiftUpToNextStoneLevel();
    }

    public void returnToBase() {
        m_lift.reset();
    }

    @Override
    public void loop() {
        execute();
    }

    public void loop(double power) {
        m_lift.driveLift(power);
    }

    @Override
    public void stop() {
        m_lift.driveLift(0);
    }

    @Override
    public void disable() {
        m_lift.disable();
    }

}
