// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.*;
import frc.controllers.*;
import frc.robot.automode.*;
import frc.robot.mode.*;
import frc.robot.mode.commands.*;
import frc.robot.subsystems.feeder.*;
import frc.robot.subsystems.flywheel.*;
import frc.robot.subsystems.flywheel.commands.*;
import frc.robot.subsystems.grappler.*;
import frc.robot.subsystems.intake.*;
import frc.robot.subsystems.release.*;
import frc.robot.subsystems.swerve.*;
import frc.robot.subsystems.swerve.commands.*;
import frc.robot.subsystems.vacuum.*;
import frc.robot.subsystems.winch.*;
import frc.robot.subsystems.winch.commands.*;
import frc.robot.teleop.WinchAndVacuumCommand;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.*;
import frc.robot.vision.PhotonVision;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  
  // Note: Only set at most one mode switch to true, for competition all should be false

  private static final boolean TUNE_FLYWHEEL_KF_MODE = false; // allows tuning flywheel motor kF, disable everything else

  private final IXboxController xboxController;
  private final RobotMode mode;
  private final JoystickButton unjamButton;
  private final JoystickButton aimToggleButton;
  private final JoystickButton shootHighButton;
  private final JoystickButton releaseButton1;
  private final JoystickButton releaseButton2;
  private final JoystickButton resetGyroButton;
  private final SwerveSubsystem swerveSubsystem;
  private final SimpleFlywheelSubsystem flywheelSubsystem;
  private final IntakeSubsystem intakeSubsystem;
  private final FeederSubsystem feederSubsystem;
  private final VacuumSubsystem vacuumSubsystem;
  private final WinchSubsystem winchSubsystem;
  private final ReleaseSubsystem releaseSubsystem;
  private final GrapplerSubsystem grapplerSubsystem;
  private final NetworkTableComms nt;
  private final PhotonVision photonVision;
  
  private final SendableChooser<Command> autoPathChooser = new SendableChooser<>();
  private final StopCommand stopCommand;
  private final ShootAndBackUp shootAndBackUp;
  private final FiveBall fiveBall;
  private final TwoBallPrimary twoBallPrimary;
  private final ThreeBall threeBall;
  private final TwoBallSecondary twoBallSecondary;
  private final TwoBallTertiary twoBallTertiary;
  private final Test test;
  public RobotContainer(
      IXboxController xboxController,
      Joystick joystick,
      RobotMode mode,
      SwerveSubsystem swerveSubsystem,
      NetworkTableComms nt,
      PhotonVision photonVision) {

    this.xboxController = xboxController;
    this.mode = mode;
    this.unjamButton = new JoystickButton(this.xboxController.getXboxController(), XboxController.Button.kA.value);
    this.aimToggleButton = new JoystickButton(this.xboxController.getXboxController(), XboxController.Button.kY.value);
    this.shootHighButton = new JoystickButton(this.xboxController.getXboxController(), XboxController.Button.kX.value);
    this.releaseButton1 = new JoystickButton(this.xboxController.getXboxController(), XboxController.Button.kStart.value);
    this.releaseButton2 = new JoystickButton(this.xboxController.getXboxController(), XboxController.Button.kB.value);
    this.resetGyroButton = new JoystickButton(this.xboxController.getXboxController(), XboxController.Button.kBack.value);
    this.swerveSubsystem = swerveSubsystem;
    this.nt = nt;
    this.photonVision = photonVision;
    this.flywheelSubsystem = new SimpleFlywheelSubsystem(nt); 
    this.intakeSubsystem = new IntakeSubsystem();
    this.feederSubsystem = new FeederSubsystem(nt);
    this.vacuumSubsystem = new VacuumSubsystem(nt);
    this.winchSubsystem = new WinchSubsystem(nt);
    this.releaseSubsystem = new ReleaseSubsystem();
    this.grapplerSubsystem = new GrapplerSubsystem(mode);

    this.stopCommand = new StopCommand(this.swerveSubsystem);
    this.shootAndBackUp = new ShootAndBackUp(mode, swerveSubsystem, flywheelSubsystem, feederSubsystem, releaseSubsystem);
    this.fiveBall = new FiveBall(mode, swerveSubsystem, flywheelSubsystem, feederSubsystem, intakeSubsystem, releaseSubsystem, photonVision);
    this.twoBallPrimary = new TwoBallPrimary(mode, swerveSubsystem, flywheelSubsystem, feederSubsystem, intakeSubsystem, releaseSubsystem);
    this.twoBallSecondary = new TwoBallSecondary(mode, swerveSubsystem, flywheelSubsystem, feederSubsystem, intakeSubsystem);
    this.twoBallTertiary = new TwoBallTertiary(mode, swerveSubsystem, flywheelSubsystem, feederSubsystem, intakeSubsystem);
    this.threeBall = new ThreeBall(mode, swerveSubsystem, flywheelSubsystem, feederSubsystem, intakeSubsystem, releaseSubsystem);
    this.test = new Test(mode, swerveSubsystem, flywheelSubsystem, feederSubsystem, intakeSubsystem, releaseSubsystem, photonVision);
  
    this.autoPathChooser.setDefaultOption("None", this.stopCommand);
    this.autoPathChooser.addOption("Shoot and Back Up", this.shootAndBackUp);
    this.autoPathChooser.addOption("Three Ball", this.threeBall);
    this.autoPathChooser.addOption("Five Ball", this.fiveBall);
    this.autoPathChooser.addOption("Two Ball Primary", this.twoBallPrimary);
    this.autoPathChooser.addOption("Two Ball Secondary", this.twoBallSecondary);
    this.autoPathChooser.addOption("Two Ball Tertiary", this.twoBallTertiary);
    this.autoPathChooser.addOption("Test", this.test);


    SmartDashboard.putData(this.autoPathChooser);

    initMode();
    initSwerve();
    initFeeder();
    initFlywheel();
    initIntake();
    initVacuumAndWinch();
    initRelease();
    initGrappler();
  }

  public void RunRobotPeriodic() {
    this.feederSubsystem.RunRobotPeriodic();
    this.flywheelSubsystem.RunRobotPeriodic();
    this.vacuumSubsystem.RunRobotPeriodic();
    this.winchSubsystem.RunRobotPeriodic();
  }

  private void initMode() {
    this.aimToggleButton.whenPressed(new ModeToggleCameraCommand(nt, mode));
  }

  private void initSwerve() {
    this.resetGyroButton.whenPressed(new GyroZeroCommand(swerveSubsystem));
    this.nt.initializeSwerveCommands(
      new GyroEnableCommand(this.swerveSubsystem), 
      new GyroDisableCommand(this.swerveSubsystem));
  }

  private void initFlywheel() {
    this.flywheelSubsystem.setDefaultCommand(new FlywheelDefaultCommand(mode, flywheelSubsystem));
  }

  private void initIntake() {
    var intakeUnjamCommand = new IntakeUnjamCommand(intakeSubsystem);
    this.intakeSubsystem.setDefaultCommand(new IntakeDefaultCommand(mode, intakeSubsystem));
    this.unjamButton.whileHeld(intakeUnjamCommand);
  }

  private void initFeeder() {
    var shootHighHeld = new Trigger(this.shootHighButton::get)
      .debounce(1.0, DebounceType.kRising);
    var feederUnjamCommand = new FeederUnjamCommand(feederSubsystem);
    var feederFireCommand = new FeederFireCommand(feederSubsystem, flywheelSubsystem);
    this.feederSubsystem.setDefaultCommand(new FeederDefaultCommand(mode, feederSubsystem));
    this.unjamButton.whileHeld(feederUnjamCommand);
    this.shootHighButton.whenPressed(feederFireCommand, false);
    shootHighHeld.whenActive(feederFireCommand, false); // fires a second ball if you hold it
  }

  private void initVacuumAndWinch() {
    this.nt.initializeVacuumAndWinchCommands(
      new VacuumRunCommand(vacuumSubsystem),
      new WinchAndVacuumCommand(mode, vacuumSubsystem, winchSubsystem));

    this.vacuumSubsystem.setDefaultCommand(new VacuumDefaultCommand(vacuumSubsystem, mode));
    this.winchSubsystem.setDefaultCommand(new WinchDefaultCommand(winchSubsystem));
  }

  private void initRelease() {
    var winchAtTop = new Trigger(winchSubsystem::isAtTop);
    this.releaseSubsystem.setDefaultCommand(new ReleaseDefaultCommand(releaseSubsystem));
    var releaseLaunchCommand = new ReleaseLaunchCommand(swerveSubsystem, releaseSubsystem);
    this.releaseButton1
      .and(this.releaseButton2)
      .and(winchAtTop)
      .whenActive(releaseLaunchCommand);
  }

  private void initGrappler() {
    this.grapplerSubsystem.setDefaultCommand(new GrapplerDefaultCommand(grapplerSubsystem, mode));

    this.nt.initializeGrapplerCommands(
      new GrapplerLockCommand(grapplerSubsystem),
      new GrapplerUnlockCommand(grapplerSubsystem)
      
      
    );
  }


  public Command getAutonomousCommand() {
    return this.autoPathChooser.getSelected();
  }
}
