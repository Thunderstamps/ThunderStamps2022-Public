package frc.robot.subsystems.feeder;

import java.time.*;

import edu.wpi.first.wpilibj2.command.*;

public class FeederRunCommand extends CommandBase {
    
    private final FeederSubsystem feederSubsystem;
    private Instant startTime = Instant.now();
    private final double milliseconds;

    public FeederRunCommand(
            FeederSubsystem feederSubsystem,
            double seconds) {
        this.feederSubsystem = feederSubsystem;
        addRequirements(feederSubsystem);
        this.milliseconds = seconds * 1000;
    }
    
    @Override
    public void initialize() {
        this.startTime = Instant.now();
    }

    @Override
    public void execute() {
        this.feederSubsystem.execute(0.4);
    }

    @Override
    public boolean isFinished() {
        var duration = Duration.between(this.startTime, Instant.now());
        return duration.toMillis() >= this.milliseconds;
    }
}
