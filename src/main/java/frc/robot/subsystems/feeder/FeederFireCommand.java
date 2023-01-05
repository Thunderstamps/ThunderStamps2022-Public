package frc.robot.subsystems.feeder;

import java.time.*;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.flywheel.SimpleFlywheelSubsystem;

public class FeederFireCommand extends CommandBase {

    private final FeederSubsystem feederSubsystem;
    private final SimpleFlywheelSubsystem flywheelSubsystem;
    private Instant startTime;
    private boolean fireComplete;
    private Instant fireCompleteTime;

    public FeederFireCommand(
            FeederSubsystem feederSubsystem,
            SimpleFlywheelSubsystem flywheelSubsystem) {
        this.feederSubsystem = feederSubsystem;
        this.flywheelSubsystem = flywheelSubsystem;
        addRequirements(feederSubsystem);
    }
    
    @Override
    public void initialize() {
        this.startTime = Instant.now();
        this.fireComplete = false;
        this.feederSubsystem.ZeroPosition();
    }
    
    @Override
    public void execute() {
        var duration = Duration.between(this.startTime, Instant.now());
        if(!this.fireComplete && duration.toMillis() >= 50 && this.feederSubsystem.GetPositionRotations() >= 5.0) {
            this.fireComplete = true;
            this.fireCompleteTime = Instant.now();
        }

        if(!this.fireComplete) {
            this.feederSubsystem.executeSpeed(3000);
        }
        else {
            this.feederSubsystem.execute(0);
            this.feederSubsystem.SetBallPresent(false);
        }
    }

    @Override
    public boolean isFinished() {
        var now = Instant.now();
        if(this.fireComplete) {
            var sinceFireComplete = Duration.between(this.fireCompleteTime, now);
            if(sinceFireComplete.toMillis() >= 500) {
                return true;
            }
        }
        var duration = Duration.between(this.startTime, now);
        return duration.toMillis() >= 3000;
    }

    @Override
    public void end(boolean interrupted) {
        this.flywheelSubsystem.SetFullCurrent();
    }
}
