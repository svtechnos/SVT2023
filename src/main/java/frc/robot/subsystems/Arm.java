// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;

public class Arm extends SubsystemBase {
  // Only one instance of this class will be created per robot

  // Initaite the joystick here
  Joystick armJoystick;
  PS4Controller examplePS4 = new PS4Controller(0); // 0 is the USB Port to be used as indicated on the Driver Station
  XboxController exampleXbox = new XboxController(1); // 0 is the USB Port to be used as indicated on the Driver Station

  // Initialize motors here

  // array of ints variable for each position

  // Current button pressed - target state of the arm expressed as a set of angles
  // for each arm

  // Constants to be defined here - move it to the Constants file later
  /*
   * Predefiend positions require a combination of motors to be in a specific
   * positions
   * Neutral
   * Grab cone from the kiosk
   * Drop cone on the tall pole int[]
   * Drop cone on the short pole int[]
   * Drop cube on the tall table int[]
   * Drop cube on the short table int[]
   */

  // Mapping between buttons and predefined positions
  // 4 numbers go in an order. 4 numbers for the values and 4 numbers for the
  // order. We will do 1 at a time

  /** Creates a new Arm. */
  public Arm(Joystick armJoystick) {
    this.armJoystick = armJoystick;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Did we press a new button? set it in the instance state
    // call the setMotor() on all the correspinding motors in a specfic order
    // handle the stae machine logic also here
  }
}
