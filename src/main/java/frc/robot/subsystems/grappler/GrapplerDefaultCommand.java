package frc.robot.subsystems.grappler;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.mode.RobotMode;

public class GrapplerDefaultCommand extends CommandBase {
    
    private final GrapplerSubsystem grapplerSubsystem;
    private final RobotMode mode;

    public GrapplerDefaultCommand(
            GrapplerSubsystem grapplerSubsystem,
            RobotMode mode) {
        this.grapplerSubsystem = grapplerSubsystem;
        this.mode = mode;
        addRequirements(grapplerSubsystem);
    }

    @Override
    public void execute() {
        // this.grapplerSubsystem.Print();
        if(this.mode.GetWinchStarted()) {
            this.grapplerSubsystem.Unlock();
        }
    }
}
