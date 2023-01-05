package frc.robot.automode;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.trajectory.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.*;
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

public class FiveBall extends SequentialCommandGroup{


    public FiveBall(
        RobotMode mode,
        SwerveSubsystem swerveSubsystem,
        SimpleFlywheelSubsystem flywheelSubsystem,
        FeederSubsystem feederSubsystem,
        IntakeSubsystem intakeSubsystem,
        ReleaseSubsystem releaseSubsystem,
        PhotonVision photonVision) {
        
        Trajectory trajectory = loadTrajectroy("BlueFiveBallMove1");    
        Trajectory trajectory2 = loadTrajectroy("BlueFiveBallMove2");
        Trajectory trajectory3 = loadTrajectroy("BlueFiveBallMove3");
        Trajectory trajectory4 = loadTrajectroy("BlueFiveBallMove4");
        Trajectory trajectory5 = loadTrajectroy("BlueFiveBallMove5");
        Trajectory trajectory6 = loadTrajectroy("BlueFiveBallMove6");
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
                new FlywheelDefaultCommand(mode, flywheelSubsystem)
            ),

            new ParallelDeadlineGroup( // Shoot first ball
                new FeederFireCommand(feederSubsystem, flywheelSubsystem), // parallel ends when this ends -- Extra time (because 2 ball allows for it)
                new IntakeDefaultCommand(mode, intakeSubsystem),
                new FlywheelDefaultCommand(mode, flywheelSubsystem),
                new ReleaseDefaultCommand(releaseSubsystem),
                new StopCommand(swerveSubsystem)
            ), 

            new ParallelDeadlineGroup( // Shoot everything (balls 1 and 2)
                new FeederRunCommand(feederSubsystem, 0.3), // parallel ends when this ends
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

            new ParallelDeadlineGroup( // Move to fourth and fifth ball
                new FollowTrajectoryCommand(swerveSubsystem, trajectory5), // parallel ends when this ends
                new TurnToFieldOrientationCommand(swerveSubsystem, Math.toRadians(-135)),
                new FeederDefaultCommand(mode, feederSubsystem),
                new IntakeDefaultCommand(mode, intakeSubsystem),
                new FlywheelDefaultCommand(mode, flywheelSubsystem),
                new ReleaseDefaultCommand(releaseSubsystem)
                ),

            // wait for human player ball here!
            new ParallelDeadlineGroup(
                new WaitCommand(0.5), // parallel ends when this ends
                new FeederDefaultCommand(mode, feederSubsystem),
                new IntakeDefaultCommand(mode, intakeSubsystem),
                new FlywheelDefaultCommand(mode, flywheelSubsystem),
                new ReleaseDefaultCommand(releaseSubsystem),
                new StopCommand(swerveSubsystem)
            ),


            new ParallelDeadlineGroup( // Move to last shooting position & Turn towards goal
                new FollowTrajectoryCommand(swerveSubsystem, trajectory6), // parallel ends when this ends
                new SequentialCommandGroup(
                    new WaitCommand(0.7), // delay turning until we get away from the terminal
                    new TurnToFieldOrientationCommand(swerveSubsystem, Math.toRadians(60))),
                new FeederDefaultCommand(mode, feederSubsystem),
                new IntakeDefaultCommand(mode, intakeSubsystem),
                new FlywheelDefaultCommand(mode, flywheelSubsystem),
                new ReleaseDefaultCommand(releaseSubsystem)
            ),

            new ParallelDeadlineGroup( // Shoot 4th ball
                new FeederFireCommand(feederSubsystem, flywheelSubsystem), // parallel ends when this ends -- Extra time (because 2 ball allows for it)
                new IntakeDefaultCommand(mode, intakeSubsystem),
                new FlywheelDefaultCommand(mode, flywheelSubsystem),
                new ReleaseDefaultCommand(releaseSubsystem),
                new StopCommand(swerveSubsystem)
            ),
            new ParallelDeadlineGroup( // Shoot everything (balls 4 and 5)
                new FeederRunCommand(feederSubsystem, 1.0), // parallel ends when this ends
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
