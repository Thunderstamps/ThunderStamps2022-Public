package frc.robot.subsystems.grappler;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class GrapplerUnlockCommand extends CommandBase {
    
    private final GrapplerSubsystem grapplerSubsystem;

    public GrapplerUnlockCommand(
            GrapplerSubsystem grapplerSubsystem) {
        this.grapplerSubsystem = grapplerSubsystem;
        addRequirements(grapplerSubsystem);
        setName("Unlock Grappler");
    }

    @Override
    public void initialize() {
        this.grapplerSubsystem.Unlock();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
