package frc.robot.subsystems.grappler;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class GrapplerLockCommand extends CommandBase {
    
    private final GrapplerSubsystem grapplerSubsystem;

    public GrapplerLockCommand(
            GrapplerSubsystem grapplerSubsystem) {
        this.grapplerSubsystem = grapplerSubsystem;
        addRequirements(grapplerSubsystem);
        setName("Lock Grappler");
    }

    @Override
    public void initialize() {
        this.grapplerSubsystem.Lock();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
