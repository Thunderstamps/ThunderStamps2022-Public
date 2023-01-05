package frc.robot.mode.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.mode.RobotMode;

public class ModeAimOnCommand extends CommandBase {
    
    private final RobotMode mode;

    public ModeAimOnCommand(
            RobotMode mode) {
        this.mode = mode;
    }

    @Override
    public void initialize() {
        this.mode.SetAiming(true);
    }

    @Override
    public void execute() {
        this.mode.SetAiming(true);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
