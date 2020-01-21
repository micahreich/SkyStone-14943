package org.firstinspires.ftc.teamcode.commands;

import java.util.function.BooleanSupplier;

public class ExampleCommand implements Command {

  private BooleanSupplier m_triggerValue;
  private ExampleSubsystem m_subsystem;

  public ExampleCommand(ExampleSubsystem subsystem, BooleanSupplier triggerValue) {
    m_subsystem = subsystem;
    m_triggerValue = triggerValue;
  }
  
  @Override
  public void execute() {
    if (m_triggerValue.getAsBoolean()) m_subsystem.open();
    else m_subsystem.close();
  }
  
  @Override
  public void loop() {
    return;
  }
  
  @Override
  public void stop() {
    return;
  }
  
  @Override
  public void disable() {
    return;
  }

}
