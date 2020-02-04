package org.firstinspires.ftc.teamcode.commands;

import java.util.function.DoubleSupplier;

import org.firstinspires.ftc.teamcode.subsystems.MecanumSubsystem;

public class DriveCommand implements Command {

  private Doubleupplier m_x, m_y, m_turn;
  private MecanumSubsystem m_drive;

  public ExampleCommand(MecanumSubsystem subsystem,
                        DoubleSupplier x,
                        DoubleSupplier y,
                        DoubleSupplier turn) {
    m_drive = subsystem;
    
    m_x = x;
    m_y = y;
    m_turn = turn;
  }
  
  @Override
  public void execute() {
    m_drive.driveRobotCentric(m_x.getAsDouble(), m_y.getAsDouble(), m_turn.getAsDouble());
  }
  
  @Override
  public void loop() {
    loop(false);
  }
  
  public void loop(boolean stopRequested) {
    if (!stopRequested) execute();
    else disable();
  }
  
  @Override
  public void stop() {
    m_drive.stopMotors();
  }
  
  @Override
  public void disable() {
    m_drive.disable();
  }

}
