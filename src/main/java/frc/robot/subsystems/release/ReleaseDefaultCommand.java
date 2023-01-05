package frc.robot.subsystems.release;

import edu.wpi.first.wpilibj2.command.*;

public class ReleaseDefaultCommand extends CommandBase {
    
    private final ReleaseSubsystem releaseSubsystem;

    public ReleaseDefaultCommand(ReleaseSubsystem releaseSubsystem) {
        this.releaseSubsystem = releaseSubsystem;
        addRequirements(releaseSubsystem);
    }

    @Override
    public void execute() {
        this.releaseSubsystem.executeLock();
    }
}
