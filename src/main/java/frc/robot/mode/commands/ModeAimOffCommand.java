package frc.robot.mode.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.mode.RobotMode;

public class ModeAimOffCommand extends CommandBase {
    
    private final RobotMode mode;

    public ModeAimOffCommand(
            RobotMode mode) {
        this.mode = mode;
    }

    @Override
    public void initialize() {
        this.mode.SetAiming(false);
    }

    @Override
    public void execute() {
        this.mode.SetAiming(false);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
