// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Arm;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
//import com.arcrobotics.vision.Limelight;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in th
 * e TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  //private final Drivetrain drivetrain = new Drivetrain();
  private RobotContainer m_robotContainer;
  Drivetrain drivetrain;
  Joystick joystick;
  XboxController xboxController = new XboxController(0); // 0 is the USB Port to be used as indicated on the Driver Station
  Arm arm;
  public double jxArray[] = new double[750]; // joystick x array
  public double tAngleArray[] = new double[750]; // turn angle array
  public double dPowerArray[] = new double[750]; // drive power array
  public boolean triggerArray[] = new boolean[750]; // trigger array
  public boolean b2Array[] = new boolean[750]; // button 2 array
  public int idx = 750; // index to iterate through (like i in a for loop)
  public int idxr = 750; // another index
  public int flag; // flag for macro
  //private RobotContainer m_robotContainer;
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_robotContainer=new RobotContainer();
    this.drivetrain = m_robotContainer.drivetrain;
    this.joystick = m_robotContainer.joystick;
    this.arm = m_robotContainer.arm;
    drivetrain.resetDEncoders();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    drivetrain.gyroPutPitch();
    drivetrain.gyroPutRoll();
    drivetrain.gyroPutYaw();
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  public void setNeutral () {
    if (xboxController.getRawButton(5)) {
      arm.neutral();
    }
  }
  
  
  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  /**
   * Resets drive encoders
   * Schedules the autonomous commands
   * If no commands, it does the macro
   */
  public void autonomousInit() {
    drivetrain.resetDEncoders();
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
      }else{if(flag==1){idxr=0;flag=0;}}
  }

  /** This function is called periodically during autonomous. */
  @Override
  /**
   * if (the autonomous command == null and flag == 1) {iterates through the array which store the macro;}
   */
  public void autonomousPeriodic() {
    double jX=0;
    double tAngle=0;
    double dPower=0;
    boolean b2=false;
    boolean trigger=false;
    //Reading the arrays
    if(idxr<750){
      jX=jxArray[idxr];
      tAngle=tAngleArray[idxr];
      dPower=dPowerArray[idxr];
      trigger=triggerArray[idxr];
      b2=b2Array[idxr];
      idxr++;
      System.out.println(idxr);
    }
    //Moves the robot
    drivetrain.RobotMove(tAngle, dPower, jX, trigger, b2);
  }
  @Override
  /**
   * Sets gyro to 0
   */
  public void teleopInit() {
    drivetrain.gyroSetYaw(0);
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  // Gonna be changed a lot
  @Override
  public void teleopPeriodic() {
    double jX;
    double jY;
    double tAngle;
    double dPower;
    boolean b2;
    boolean trigger;
    if(joystick.getRawButton(13)){drivetrain.CANtest();}
    else{
      jX=joystick.getX();
      jY=joystick.getY()*-1;
      b2=joystick.getRawButton(2);
      trigger=joystick.getTrigger();
      double[] cTpResult = drivetrain.cTp(jX, jY);
      tAngle=cTpResult[0];
      dPower=cTpResult[1];
      if(joystick.getRawButton(11)){idx=0;}
      if(idx==749){flag=1;}
      if(idx<750){jxArray[idx]=jX;tAngleArray[idx]=tAngle;dPowerArray[idx]=dPower;triggerArray[idx]=trigger;b2Array[idx]=b2;idx++;System.out.println(idx);}
      drivetrain.RobotMove(tAngle, dPower, jX, trigger, b2);
    }
  }
  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
