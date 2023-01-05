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
import frc.robot.subsystems.intake.IntakeUnjamCommand;
import frc.robot.subsystems.intake.IntakeDefaultCommand;
import frc.robot.subsystems.release.ReleaseDefaultCommand;
import frc.robot.subsystems.release.ReleaseSubsystem;
import frc.robot.subsystems.swerve.*;
import frc.robot.subsystems.swerve.commands.*;

public class TwoBallPrimary extends SequentialCommandGroup{


    public TwoBallPrimary(
        RobotMode mode,
        SwerveSubsystem swerveSubsystem,
        SimpleFlywheelSubsystem flywheelSubsystem,
        FeederSubsystem feederSubsystem,
        IntakeSubsystem intakeSubsystem,
        ReleaseSubsystem releaseSubsystem) {
            var speed_in_s = 70.0;
        //53.5 - 42
        //-88x , 73y

        // var startPosition =

        addCommands(
            new SetRobotFieldPositionCommand(swerveSubsystem, 0, 0, Math.toRadians(136.5)),


            new ParallelDeadlineGroup(
                new MoveAbsoluteAndRotateCommand(swerveSubsystem,Vector2D.FromXY(-54, 54),Math.toRadians(136.5),speed_in_s), // Going to first ball
                new FeederDefaultCommand(mode, feederSubsystem),
                new IntakeDefaultCommand(mode, intakeSubsystem),
                new FlywheelDefaultCommand(mode, flywheelSubsystem),
                new ReleaseDefaultCommand(releaseSubsystem)
            ),

            new ParallelDeadlineGroup( // index the ball into position to fire, wait for flywheel to spin up
                new WaitCommand(0.5), // parallel ends when this ends
                new FeederDefaultCommand(mode, feederSubsystem),
                new IntakeDefaultCommand(mode, intakeSubsystem),
                new FlywheelDefaultCommand(mode, flywheelSubsystem),
                new ReleaseDefaultCommand(releaseSubsystem),
                new StopCommand(swerveSubsystem)
            ),

            new ParallelDeadlineGroup(
                new MoveAbsoluteAndRotateCommand(swerveSubsystem,Vector2D.FromXY(6, -6),Math.toRadians(-21),speed_in_s,5), // Going to shooting position
                new FeederDefaultCommand(mode, feederSubsystem),
                new IntakeDefaultCommand(mode, intakeSubsystem), // run backwards to prevent another ball from entering
                new FlywheelDefaultCommand(mode, flywheelSubsystem),
                new ReleaseDefaultCommand(releaseSubsystem) 
            ),
            new ParallelDeadlineGroup( // index the ball into position to fire, wait for flywheel to spin up
                new WaitCommand(2.0), // parallel ends when this ends
                new FeederDefaultCommand(mode, feederSubsystem),
                new IntakeDefaultCommand(mode, intakeSubsystem), // run backwards to prevent another ball from entering
                new FlywheelDefaultCommand(mode, flywheelSubsystem),
                new ReleaseDefaultCommand(releaseSubsystem),
                new StopCommand(swerveSubsystem)
            ),
            

            new ParallelDeadlineGroup( // Shoot first ball
                new FeederFireCommand(feederSubsystem, flywheelSubsystem), // parallel ends when this ends -- Extra time (because 2 ball allows for it)
                new IntakeDefaultCommand(mode, intakeSubsystem), // run backwards to prevent another ball from entering
                new FlywheelDefaultCommand(mode, flywheelSubsystem),
                new ReleaseDefaultCommand(releaseSubsystem),
                new StopCommand(swerveSubsystem)
            ),

            new ParallelDeadlineGroup( // index the ball into position to fire, wait for flywheel to spin up
                new WaitCommand(1.0), // parallel ends when this ends
                new FeederDefaultCommand(mode, feederSubsystem),
                new IntakeDefaultCommand(mode, intakeSubsystem), // run backwards to prevent another ball from entering
                new FlywheelDefaultCommand(mode, flywheelSubsystem),
                new ReleaseDefaultCommand(releaseSubsystem),
                new StopCommand(swerveSubsystem)
            ),
            
            new ParallelDeadlineGroup( // Shoot everything (balls 1 and 2)
                new FeederRunCommand(feederSubsystem, 1), // parallel ends when this ends -- Extra time (because 2 ball allows for it)
                new IntakeDefaultCommand(mode, intakeSubsystem), // run backwards to prevent another ball from entering
                new FlywheelDefaultCommand(mode, flywheelSubsystem),
                new ReleaseDefaultCommand(releaseSubsystem),
                new StopCommand(swerveSubsystem)
            ),
            new ParallelDeadlineGroup(
                new MoveAbsoluteAndRotateCommand(swerveSubsystem,Vector2D.FromXY(0, 100),Math.toRadians(90),speed_in_s,5), // Going to shooting position
                new FeederDefaultCommand(mode, feederSubsystem),
                new IntakeDefaultCommand(mode, intakeSubsystem), // run backwards to prevent another ball from entering
                new FlywheelDefaultCommand(mode, flywheelSubsystem),
                new ReleaseDefaultCommand(releaseSubsystem) 
            ),
            new ParallelDeadlineGroup(
                new MoveAbsoluteAndRotateCommand(swerveSubsystem,Vector2D.FromXY(0, 90),Math.toRadians(180),speed_in_s,5), // Going to shooting position
                new FeederDefaultCommand(mode, feederSubsystem),
                new IntakeDefaultCommand(mode, intakeSubsystem), // run backwards to prevent another ball from entering
                new FlywheelDefaultCommand(mode, flywheelSubsystem),
                new ReleaseDefaultCommand(releaseSubsystem) 
            ),
            new ParallelDeadlineGroup( // index the ball into position to fire, wait for flywheel to spin up
                new WaitCommand(1.5), // parallel ends when this ends
                new FeederUnjamCommand(feederSubsystem),
                new IntakeUnjamCommand(intakeSubsystem), // run backwards to prevent another ball from entering
                new FlywheelDefaultCommand(mode, flywheelSubsystem),
                new ReleaseDefaultCommand(releaseSubsystem),
                new StopCommand(swerveSubsystem)
            ),
            new ParallelDeadlineGroup(
                new MoveAbsoluteAndRotateCommand(swerveSubsystem,Vector2D.FromXY(0, 90),Math.toRadians(0),speed_in_s,5), // Going to shooting position
                new FeederDefaultCommand(mode, feederSubsystem),
                new IntakeDefaultCommand(mode, intakeSubsystem), // run backwards to prevent another ball from entering
                new FlywheelDefaultCommand(mode, flywheelSubsystem),
                new ReleaseDefaultCommand(releaseSubsystem) 
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
