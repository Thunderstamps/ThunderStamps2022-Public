package frc.robot.automode;

import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.*;
import edu.wpi.first.math.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.mode.RobotMode;
import frc.robot.subsystems.feeder.*;
import frc.robot.subsystems.flywheel.SimpleFlywheelSubsystem;
import frc.robot.subsystems.flywheel.commands.FlywheelDefaultCommand;
import frc.robot.subsystems.intake.IntakeDefaultCommand;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.release.ReleaseDefaultCommand;
import frc.robot.subsystems.release.ReleaseSubsystem;
import frc.robot.subsystems.swerve.*;
import frc.robot.subsystems.swerve.commands.*;
import frc.robot.vision.PhotonVision;

public class Test extends SequentialCommandGroup{


    public Test(
        RobotMode mode,
        SwerveSubsystem swerveSubsystem,
        SimpleFlywheelSubsystem flywheelSubsystem,
        FeederSubsystem feederSubsystem,
        IntakeSubsystem intakeSubsystem,
        ReleaseSubsystem releaseSubsystem,
        PhotonVision photonVision) {
            
            
        var points = new ArrayList<Pose2d>();
        points.add(new Pose2d(0, 0, new Rotation2d(0)));
        points.add(new Pose2d(67.0, 0, new Rotation2d(0)));
        Trajectory trajectory = makeTrajectory(points);
        var startPosition = trajectory.getInitialPose();

        addCommands(
            new SetRobotFieldPositionCommand(swerveSubsystem, startPosition.getX(), startPosition.getY(), 0),

            new ParallelDeadlineGroup(
                new FollowTrajectoryCommandToBall(swerveSubsystem, trajectory, photonVision, 2.0), // Going to first ball
                new FeederDefaultCommand(mode, feederSubsystem),
                new IntakeDefaultCommand(mode, intakeSubsystem),
                new FlywheelDefaultCommand(mode, flywheelSubsystem),
                new ReleaseDefaultCommand(releaseSubsystem)
            ),

            new ParallelDeadlineGroup(
                new FeederDefaultCommand(mode, feederSubsystem),
                new IntakeDefaultCommand(mode, intakeSubsystem),
                new FlywheelDefaultCommand(mode, flywheelSubsystem),
                new ReleaseDefaultCommand(releaseSubsystem),
                new StopCommand(swerveSubsystem)
            ),
            new StopCommand(swerveSubsystem)
        );
    }

  
    private Trajectory makeTrajectory(ArrayList<Pose2d> waypoints) {
        var config = new TrajectoryConfig(125.0, 119.0);
        config.addConstraint(new CentripetalAccelerationConstraint(119.0));
        config.setEndVelocity(0.0);

        var result = TrajectoryGenerator.generateTrajectory(waypoints, config);
        return result;
    }
    
}
