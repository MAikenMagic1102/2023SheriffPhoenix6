// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class CubeKicker extends SubsystemBase {
  private DigitalServo lilLatch;
  /** Creates a new CubeKicker. */
  public CubeKicker() {
    lilLatch = new DigitalServo(Constants.CubeKicker.pwmPort);
  }

  public void home(){
    lilLatch.set(0.05);
    //hs-785hb spool home
  }

  public void fire(){
    lilLatch.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
