package frc.robot.automode;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.mode.RobotMode;
import frc.robot.subsystems.feeder.*;
import frc.robot.subsystems.flywheel.*;
import frc.robot.subsystems.flywheel.commands.*;
import frc.robot.subsystems.release.ReleaseDefaultCommand;
import frc.robot.subsystems.release.ReleaseSubsystem;
import frc.robot.subsystems.swerve.*;
import frc.robot.subsystems.swerve.commands.*;

public class ShootAndBackUp extends SequentialCommandGroup {
    
    private final Vector2D startPosition = Vector2D.FromXY(0, 32);

    public ShootAndBackUp(
        RobotMode mode,
        SwerveSubsystem swerveSubsystem,
        SimpleFlywheelSubsystem flywheelSubsystem,
        FeederSubsystem feederSubsystem,
        ReleaseSubsystem releaseSubsystem) {
        
        addCommands(
            new SetRobotFieldPositionCommand(swerveSubsystem, startPosition.getX(), startPosition.getY(), Math.toRadians(0)),

            new ParallelDeadlineGroup( // index the ball into position to fire, wait for flywheel to spin up
                new WaitCommand(5.0), // parallel ends when this ends
                new FeederDefaultCommand(mode, feederSubsystem),
                new FlywheelDefaultCommand(mode, flywheelSubsystem),
                new ReleaseDefaultCommand(releaseSubsystem),
                new StopCommand(swerveSubsystem)
            ),

            new ParallelDeadlineGroup( // Shoot ball
                new FeederRunCommand(feederSubsystem, 0.4), // parallel ends when this ends
                new FlywheelDefaultCommand(mode, flywheelSubsystem),
                new ReleaseDefaultCommand(releaseSubsystem),
                new StopCommand(swerveSubsystem)
            ),

            new ParallelDeadlineGroup( // move back out of tarmac
                new MoveRelativeCommand(swerveSubsystem, -90.0, 0.0), // parallel ends when this ends
                new FeederDefaultCommand(mode, feederSubsystem),
                new FlywheelDefaultCommand(mode, flywheelSubsystem),
                new ReleaseDefaultCommand(releaseSubsystem)
            ),

            new StopCommand(swerveSubsystem)
        );
    }
}
