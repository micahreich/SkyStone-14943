package org.firstinspires.ftc.teamcode.commands;

public interface Subsystem {

  void runCommand(Command command);
  
  Command getDefaultCommand();
  
  void setDefaultCommand(Command command);
  
  void runDefaultCommand();

}
