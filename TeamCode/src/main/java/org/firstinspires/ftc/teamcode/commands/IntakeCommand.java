package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.subsystems.Intake;

public class IntakeCommand implements Command {

    private Intake m_intake;

    public IntakeCommand(Intake intake) {
        m_intake = intake;
    }

    @Override
    public void execute() {
        m_intake.runIntake(0.75);
    }

    @Override
    public void loop() {
        if(!m_intake.stoneInBay()) {
            execute();
        } else {
            stop();
        }
    }

    @Override
    public void stop() {
        m_intake.runIntake(0);
    }

    @Override
    public void disable() {
        m_intake.disable();
    }

}
