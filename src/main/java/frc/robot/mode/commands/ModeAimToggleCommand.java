package frc.robot.mode.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.mode.RobotMode;

public class ModeAimToggleCommand extends CommandBase {
    
    private final RobotMode mode;
    private boolean initialValue;

    public ModeAimToggleCommand(
            RobotMode mode) {
        this.mode = mode;
    }

    @Override
    public void initialize() {
        this.initialValue = this.mode.GetAiming();
    }

    @Override
    public void execute() {
        this.mode.SetAiming(!this.initialValue);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
    
}
