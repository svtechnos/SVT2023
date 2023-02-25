// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.Pigeon2;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {

  private static final int mDeviceID[] = { 1, 2, 3, 4, 5, 6, 7, 8 };// even motors(ID) are turn
  private static final int tDeviceID[] = { 21, 23, 24, 22 };// turn-encoder device ids
  private static final double tOffset[] = { 90.97, 175.61, 308.057, 301.03 };// turn motor offsets
  private static final int gDeviceID = 11; // gyro device id
  private WPI_TalonSRX t[] = new WPI_TalonSRX[4];// encoders // rename as turn
  private CANSparkMax m[] = new CANSparkMax[8];// motors // rename as motor
  private RelativeEncoder e[] = new RelativeEncoder[4];// drive encoders // rename as encoders
  private Pigeon2 gyro; // pigeon motor encoder
  private Joystick joystick; // joystick

  public Drivetrain(Joystick joystick) {
    this.joystick = joystick;
    gyro = new Pigeon2(gDeviceID, "rio"); // initialize gyro-pigeon object
    for (int i = 0; i < 8; i++) { // "loop for initizalizing stuff" - Sepandar
      m[i] = new CANSparkMax(mDeviceID[i], MotorType.kBrushless); // making neos
      m[i].restoreFactoryDefaults(); // setting to factor default
      m[i].setIdleMode(IdleMode.kBrake); // setting idle mode to brake so it will brake
      if (i % 2 == 0) { // so only even loop numbers... (things we have 4 of like turn encoders)
        t[i / 2] = new WPI_TalonSRX(tDeviceID[i / 2]); // making abs turn encoder objects' canbus
        t[i / 2].configSelectedFeedbackSensor(FeedbackDevice.PulseWidthEncodedPosition); // setting configures the
                                                                                         // selected feeddback sensor to
                                                                                         // selected encoder positions
        m[i].setOpenLoopRampRate(Constants.dremp); // set ramp rate for drive motors to "dremp"
        e[i / 2] = m[i].getEncoder(); // setting relative encoders for drive motors
        e[i / 2].setPositionConversionFactor(1 / 20);
      } // setting conversion factor for every encoder turn is 1 meter
      else {
        m[i].setOpenLoopRampRate(Constants.tremp); // sets ramp rate for turn motors to tremp
      }
    }
  }

  /**
   * Get target angle and current angle
   * Goal is to find the most efficient way to go from current angle to target
   * angle
   * 
   * @param t
   * @param c
   * @return
   */
  public double[] deltaMod(double t, double c) { // "this is the deltaMod where the maths happen" - Sepandar
    double d = t - c; // gets delta angle between t and c
    double dir; // direction initialization
    d = (Math.abs(d) >= 180) ? -(((360 * d) / (Math.abs(d))) - d) : d; // "sets the delta angle to the correct stuff" -
                                                                       // Sepandar
    if (Math.abs(d) >= 90) { // "makes the direction the correct stuff" - Sepandar
      d = -(((180 * d) / (Math.abs(d))) - d);
      dir = -1;
    } else {
      dir = 1;
    }
    d = (Math.abs(d) < Constants.angleThresh) ? 0 : d;
    // System.out.println("c: "+c+" t: "+t+" d: "+d+" dir: "+dir);
    return new double[] { -d, dir }; // returns the delta angle you have to go at and the direction to turn
  }

  /**
   * Resets drive encoders
   */
  public void resetDEncoders() {
    e[0].setPosition(0);
    e[1].setPosition(0);
    e[2].setPosition(0);
    e[3].setPosition(0);
  }

  /**
   * gets turn encoder position
   * 
   * @param tEncNum
   * @return
   */
  public double getTEncoderPostion(int tEncNum) {
    return (((t[tEncNum].getSelectedSensorPosition()) * 360 / 4096) + (tOffset[(tEncNum)])) % 360;
  }

  /**
   * Gets same as the last one but adds gyro to make sure forward is always
   * forward
   * 
   * @param tEncNum
   * @return
   */
  public double getTEncoderPostionGyro(int tEncNum) {
    return (((t[tEncNum].getSelectedSensorPosition()) * 360 / 4096) + (tOffset[(tEncNum)]) + (gyro.getYaw())) % 360;
  }

  /**
   * Gets drive encoder position
   * 
   * @param dEncNum
   * @return
   */
  public double getDEncoderPosition(int dEncNum) {
    return e[dEncNum].getPosition();
  }

  /**
   * sets the motor speed
   * 
   * @param speed
   * @param motor
   */
  public void setSpeed(double speed, int motor) {
    m[motor].set(speed);
  }

  /**
   * Puts pitch to dashboard
   */
  public void gyroPutPitch() {
    SmartDashboard.putNumber("Pitch", gyro.getPitch());
  }

  /**
   * Puts roll to dashboard
   */
  public void gyroPutRoll() {
    SmartDashboard.putNumber("Roll", gyro.getRoll());
  }

  /**
   * Puts yaw to dashboard
   */
  public void gyroPutYaw() {
    SmartDashboard.putNumber("Yaw", gyro.getYaw());
  }

  /**
   * to do later
   * 
   * @return pitch
   */
  public double gyroGetPitch() {
    return gyro.getPitch();
  }

  // to do later
  public double gyroGetRoll() {
    return gyro.getRoll();
  }

  // to do later
  public double gyroGetYaw() {
    return gyro.getYaw();
  }

  // to do later
  public void gyroSetYaw(double angle) {
    gyro.setYaw(angle);
  }

  // to do later
  public void stopMotors() {
    for (int i = 0; i < 8; i++) {
      m[i].setOpenLoopRampRate(Constants.fremp);
      m[i].stopMotor();
      if (i % 2 == 0) {
        m[i].setOpenLoopRampRate(Constants.dremp);
      } else {
        m[i].setOpenLoopRampRate(Constants.tremp);
      }
    }
  }

  /**
   * Converts cartesian coordinates to polar coordinates
   * 
   * @param x
   * @param y
   * @return
   */
  public double[] cTp(double x, double y) {
    double magnitude = Math.sqrt(((x * x) + (y * y)) / 2);
    double degrees = Math.toDegrees(Math.atan2(y, x));
    if (degrees < 0) {
      degrees = 360 + degrees;
    }
    return new double[] { degrees, magnitude };
  }

  /**
   * Moves the whole robot
   * //NOT DONE YET
   * 
   * @param tAngle
   * @param dPower
   * @param jX
   * @param trigger
   * @param b2
   */
  public void RobotMove(double tAngle, double dPower, double jX, boolean trigger, boolean b2) {
    double fdF;
    double dAngle;
    double cAngle;
    double dir;
    fdF = (trigger) ? Constants.dF * 2 : Constants.dF;
    for (int i = 1; i < 8; i += 2) {
      cAngle = getTEncoderPostionGyro((i - 1) / 2);
      if (b2) {
        if (jX < 0) {
          cAngle = getTEncoderPostion((i - 1) / 2);
          dPower = (Constants.twF * (Math.abs(jX)));
          if (i == 1) {
            tAngle = 315;
          }
          if (i == 3) {
            tAngle = 45;
          }
          if (i == 5) {
            tAngle = 135;
          }
          if (i == 7) {
            tAngle = 225;
          }
        } else if (jX > 0) {
          cAngle = getTEncoderPostion((i - 1) / 2);
          dPower = (Constants.twF * (Math.abs(jX)));
          if (i == 1) {
            tAngle = 135;
          }
          if (i == 3) {
            tAngle = 225;
          }
          if (i == 5) {
            tAngle = 315;
          }
          if (i == 7) {
            tAngle = 45;
          }
        }
      }
      double[] deltaM = deltaMod(tAngle, cAngle);
      dAngle = deltaM[0];
      dir = deltaM[1];
      if (dPower < Constants.dPowerMin) {
        dAngle = 0;
        dPower = 0;
      }
      double tPower = Constants.tF * dAngle / 180;
      if (Math.abs(tPower) > Constants.mT) {
        tPower = Constants.mT * tPower / Math.abs(tPower);
      }
      setSpeed(tPower, i);
      if (Math.abs(dAngle) < Constants.turnInProgress) {
        setSpeed(fdF * dPower * dir, i - 1);
      } else {
        setSpeed(0, i - 1);
      }
    }
  }

  /**
   * Tests the CAN buses
   * //NOT DONE YET
   */
  public void CANtest() {
    if (joystick.getRawButton(1)) {
      setSpeed(0.1, 0);
    } else {
      setSpeed(0.1, 0);
    }
    if (joystick.getRawButton(2)) {
      setSpeed(0.1, 1);
      System.out.println("TurnMotor 1 encoder:" + getTEncoderPostion(0));
    } else {
      setSpeed(0.1, 1);
    }
    if (joystick.getRawButton(3)) {
      setSpeed(0.1, 2);
    } else {
      setSpeed(0.1, 2);
    }
    if (joystick.getRawButton(4)) {
      setSpeed(0.1, 3);
      System.out.println("TurnMotor 2 encoder:" + getTEncoderPostion(1));
    } else {
      setSpeed(0.1, 3);
    }
    if (joystick.getRawButton(5)) {
      setSpeed(0.1, 4);
    } else {
      setSpeed(0.1, 4);
    }
    if (joystick.getRawButton(6)) {
      setSpeed(0.1, 5);
      System.out.println("TurnMotor 3 encoder:" + getTEncoderPostion(2));
    } else {
      setSpeed(0.1, 5);
    }
    if (joystick.getRawButton(7)) {
      setSpeed(0.1, 6);
    } else {
      setSpeed(0.1, 6);
    }
    if (joystick.getRawButton(7)) {
      setSpeed(0.1, 7);
      System.out.println("TurnMotor 4 encoder:" + getTEncoderPostion(3));
    } else {
      setSpeed(0.1, 7);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
