// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.*;
import frc.controllers.*;
import frc.robot.mode.RobotMode;
import frc.robot.subsystems.swerve.*;
import frc.robot.vision.LimelightVision;
import frc.robot.vision.PhotonVision;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private NetworkTableComms nt = new NetworkTableComms();
  private IXboxController xboxController = new XboxControllerSCUF(0);
  private Joystick joystick = new Joystick(1);
  private final Solenoid sealingSolenoid = new Solenoid(PneumaticsModuleType.REVPH, 15);
  private final RobotMode mode = new RobotMode();
  private LimelightVision limelightVision;
  private PhotonVision photonVision;
  private Command m_autonomousCommand;
  private IGyro gyro;
  private SwerveSubsystem swerveSubsystem;
  private RobotContainer m_robotContainer;
  public static final double MAX_ACCELERATION_AUTO_G = 0.325;
  public static final double MAX_ACCELERATION_TELEOP_G = 1.5;
  public static final double SCAN_TIME_S = 0.020;
  public static final double MAX_ROTATION_RATE_RAD_S = Math.toRadians(300);
  public static final double MAX_SPEED_IN_SEC = 167.0; // L1: 125.0, COMP: 167

  public static final boolean ENABLE_SWERVE = true;

  public Robot() {
    super(SCAN_TIME_S);
  }

  @Override
  public void robotInit() {

    // this is for the upward facing camera
    var camera = CameraServer.startAutomaticCapture("Jump", 0);
    camera.setFPS(15);

    // The main NavX2 board's ScaleFactor is 0.9375
    // The alternate NavX2 board's ScaleFactor is 1.03125
    // If installing the NavX classic board, use the NavXMxpGyro class instead of the NavX2MxpGyro class
    gyro = new NavX2MxpGyro(0.9375); 
    
    gyro.setEnabled(true);
    this.limelightVision = new LimelightVision(mode);
    this.photonVision = new PhotonVision(mode);
    this.swerveSubsystem = new SwerveSubsystem(xboxController, gyro, mode, limelightVision, nt);

    m_robotContainer = new RobotContainer(
      this.xboxController,
      this.joystick,
      this.mode,
      this.swerveSubsystem,
      this.nt,
      this.photonVision);
  }

  @Override
  public void robotPeriodic() {

    this.nt.printDelay();

    this.nt.setGyroEnabled(this.gyro.getEnabled());
    //this.limelightVision.RunRobotPeriodic();
    this.photonVision.RunRobotPeriodic();
    this.m_robotContainer.RunRobotPeriodic();

    // allows swerve to home, and runs odometry, etc.
    if(ENABLE_SWERVE) {
      this.swerveSubsystem.RunRobotPeriodic();
    }

    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    this.xboxController.getXboxController().setRumble(RumbleType.kLeftRumble, 0);
  }

  @Override
  public void disabledPeriodic() {
    if(this.mode.GetAiming()) {
      this.mode.SetAiming(false);
    }

    // unseal vacuum when disabled (should happen anyway)
    this.sealingSolenoid.set(false);
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {

    this.photonVision.SetDriverMode(false);
  
    if(ENABLE_SWERVE) {
      // sets accel limits, etc.
      this.swerveSubsystem.autonomousInit();
      
      m_autonomousCommand = m_robotContainer.getAutonomousCommand();

      // schedule the autonomous command (example)
      if (m_autonomousCommand != null) {
        m_autonomousCommand.schedule();
      }
    }
    
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() { }

  @Override
  public void teleopInit() {
    
    this.photonVision.SetDriverMode(true);

    if(ENABLE_SWERVE) {
      // sets accel limits, etc.
      this.swerveSubsystem.teleopInit();
    }

    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    
    if(ENABLE_SWERVE) {
      this.swerveSubsystem.executeOperatorControl();
    }
    // always close this when we're in teleop
    this.sealingSolenoid.set(true);

    this.rumbleWhenInRange();
  }

  private void rumbleWhenInRange() {
      var aiming = this.mode.GetAiming();
      var targetData = this.limelightVision.GetTargetData();
      var inRange = aiming && targetData.ValidTargetData()
        && targetData.Y_deg() <= 18.0 // closest
        && targetData.Y_deg() >= 13.0; // furthest away
      var rumbleValue = inRange ? 1.0 : 0.0;
      this.xboxController.getXboxController().setRumble(RumbleType.kLeftRumble, rumbleValue);
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}


