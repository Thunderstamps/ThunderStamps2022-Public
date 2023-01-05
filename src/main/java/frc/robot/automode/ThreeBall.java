package frc.robot.automode;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.trajectory.*;
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

public class ThreeBall extends SequentialCommandGroup{


    public ThreeBall(
        RobotMode mode,
        SwerveSubsystem swerveSubsystem,
        SimpleFlywheelSubsystem flywheelSubsystem,
        FeederSubsystem feederSubsystem,
        IntakeSubsystem intakeSubsystem,
        ReleaseSubsystem releaseSubsystem) {
            var speed_in_s = 70;
        //53.5 - 42
        //-88x , 73y
        Trajectory trajectory = loadTrajectroy("BlueFiveBallMove1");    
        Trajectory trajectory2 = loadTrajectroy("BlueFiveBallMove2");
        Trajectory trajectory3 = loadTrajectroy("BlueFiveBallMove3");
        Trajectory trajectory4 = loadTrajectroy("BlueFiveBallMove4");
        var startPosition = trajectory.getInitialPose();

        addCommands(
            new SetRobotFieldPositionCommand(swerveSubsystem, startPosition.getX(), startPosition.getY(), Math.toRadians(-88.5)),

            new ParallelDeadlineGroup(
                new FollowTrajectoryCommand(swerveSubsystem, trajectory), // Going to first ball
                new FeederDefaultCommand(mode, feederSubsystem),
                new IntakeDefaultCommand(mode, intakeSubsystem),
                new FlywheelDefaultCommand(mode, flywheelSubsystem) 
            ),


            // Drive to shooting position and turn to goal.
            new ParallelDeadlineGroup( // Turn towards goal
                new FollowTrajectoryCommand(swerveSubsystem, trajectory2), // parallel ends when this ends
                new SequentialCommandGroup(
                    new WaitCommand(0.3), // delay until we get away from the wall
                    new TurnToFieldOrientationCommand(swerveSubsystem, Math.toRadians(60))
                ),
                new FeederDefaultCommand(mode, feederSubsystem),
                new IntakeDefaultCommand(mode, intakeSubsystem),
                new FlywheelDefaultCommand(mode, flywheelSubsystem),
                new ReleaseDefaultCommand(releaseSubsystem)
            ),


            new ParallelDeadlineGroup( // Shoot first ball
                new FeederFireCommand(feederSubsystem, flywheelSubsystem), // parallel ends when this ends -- Extra time (because 2 ball allows for it)
                new IntakeDefaultCommand(mode, intakeSubsystem),
                new FlywheelDefaultCommand(mode, flywheelSubsystem),
                new ReleaseDefaultCommand(releaseSubsystem),
                new StopCommand(swerveSubsystem)
            ),

            new ParallelDeadlineGroup( // index the ball into position to fire, wait for flywheel to spin up
                new WaitCommand(1.0), // parallel ends when this ends
                new FeederDefaultCommand(mode, feederSubsystem),
                new IntakeDefaultCommand(mode, intakeSubsystem),
                new FlywheelDefaultCommand(mode, flywheelSubsystem),
                new ReleaseDefaultCommand(releaseSubsystem),
                new StopCommand(swerveSubsystem)
            ),

            new ParallelDeadlineGroup( // Shoot everything (balls 1 and 2)
                new FeederRunCommand(feederSubsystem, 0.4), // parallel ends when this ends
                new IntakeDefaultCommand(mode, intakeSubsystem),
                new FlywheelDefaultCommand(mode, flywheelSubsystem),
                new ReleaseDefaultCommand(releaseSubsystem),
                new StopCommand(swerveSubsystem)
            ),

            new ParallelDeadlineGroup( // Move to third ball
                new FollowTrajectoryCommand(swerveSubsystem, trajectory3), // parallel ends when this ends
                new TurnToFieldOrientationCommand(swerveSubsystem, Math.toRadians(180+5)),
                new FeederDefaultCommand(mode, feederSubsystem),
                new IntakeDefaultCommand(mode, intakeSubsystem),
                new FlywheelDefaultCommand(mode, flywheelSubsystem),
                new ReleaseDefaultCommand(releaseSubsystem)
            ),

            new ParallelDeadlineGroup( // Move to shooting position & Turn towards goal
                new FollowTrajectoryCommand(swerveSubsystem, trajectory4), // parallel ends when this ends
                new TurnToFieldOrientationCommand(swerveSubsystem, Math.toRadians(60)),
                new FeederDefaultCommand(mode, feederSubsystem),
                new IntakeDefaultCommand(mode, intakeSubsystem),
                new FlywheelDefaultCommand(mode, flywheelSubsystem),
                new ReleaseDefaultCommand(releaseSubsystem)
            ),



            new ParallelDeadlineGroup( // Shoot ball 3
                new FeederRunCommand(feederSubsystem, 0.5), // parallel ends when this ends
                new IntakeDefaultCommand(mode, intakeSubsystem),
                new FlywheelDefaultCommand(mode, flywheelSubsystem),
                new ReleaseDefaultCommand(releaseSubsystem),
                new StopCommand(swerveSubsystem)
            ),

            
            new StopCommand(swerveSubsystem)
        );
    }

    private Trajectory loadTrajectroy(String pathname){
        String trajectoryJSON = ("paths/output/" + pathname + ".wpilib.json");
        Trajectory trajectory = new Trajectory();

        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
        }
        return trajectory;
    }
    
}
