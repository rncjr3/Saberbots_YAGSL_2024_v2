// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


// URL links for Vendordeps
// https://dev.studica.com/releases/2024/NavX.json
// https://3015rangerrobotics.github.io/pathplannerlib/PathplannerLib.json
// https://maven.ctr-electronics.com/release/com/ctre/phoenix/Phoenix5-frc2024-latest.json
// https://maven.ctr-electronics.com/release/com/ctre/phoenix6/latest/Phoenix6-frc2024-latest.json
// https://maven.photonvision.org/repository/internal/org/photonvision/photonlib-json/1.0/photonlib-json-1.0.json
// https://frcsdk.reduxrobotics.com/ReduxLib_2024.json
// https://software-metadata.revrobotics.com/REVLib-2024.json
// 

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * Do NOT add any static variables to this class, or any initialization at all. Unless you know what you are doing, do
 * not modify this file except to change the parameter class to the startRobot call.
 */
public final class Main
{

  private Main()
  {
  }

  /**
   * Main initialization function. Do not perform any initialization here.
   *
   * <p>If you change your main robot class, change the parameter type.
   */
  public static void main(String... args)
  {
    RobotBase.startRobot(Robot::new);
  }
}
