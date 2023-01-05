package frc.robot.subsystems.release;

import java.time.Duration;
import java.time.Instant;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.swerve.*;

public class ReleaseLaunchCommand extends CommandBase {
    private final static int LAUNCH_TIME_MS = 2000;
    private final SwerveSubsystem swerveSubsystem;
    private final ReleaseSubsystem releaseSubsystem;
    private Instant launchTime = Instant.now();

    public ReleaseLaunchCommand(
            SwerveSubsystem swerveSubsystem,
            ReleaseSubsystem releaseSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        this.releaseSubsystem = releaseSubsystem;
        addRequirements(releaseSubsystem);
    }


    @Override
    public void initialize() {
        this.launchTime = Instant.now();
        this.swerveSubsystem.disableGyro();; // disables field oriented swerve so that it doesn't try to hold heading
    }


    @Override
    public void execute() {
        this.releaseSubsystem.executeRelease();
    }

    @Override
    public boolean isFinished() {
        var duration = Duration.between(this.launchTime, Instant.now());
        return duration.toMillis() >= LAUNCH_TIME_MS;
    }

}
