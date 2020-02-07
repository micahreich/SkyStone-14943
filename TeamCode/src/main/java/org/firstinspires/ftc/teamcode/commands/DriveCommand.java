package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.subsystems.MecanumSubsystem;

public class DriveCommand implements Command {

  private MecanumSubsystem m_drive;

  public DriveCommand(MecanumSubsystem driveTrain) {
    m_drive = driveTrain;
  }

  @Override
  public void execute() {
    loop(0,0,0);
  }
  
  @Override
  public void loop() {
    execute();
  }
  
  public void loop(boolean stopRequested, double x, double y, double turn) {
    if (!stopRequested) loop(x, y, turn);
    else loop();
  }

  public void loop(double x, double y, double turn) {
    m_drive.driveRobotCentric(x, y, turn);
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
