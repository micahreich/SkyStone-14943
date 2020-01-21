package org.firstinspires.ftc.teamcode.commands;

import com.qualcomm.robotcore.hardware.Servo;

public class ExampleSubsystem {

  private Servo m_servo;
  
  public ExampleSubsystem(Servo servo) {
    m_servo = servo;
  }
  
  public void open() {
    m_servo.setPosition(1);
  }
  
  public void close() {
    m_servo.setPosition(0);
  }

}
