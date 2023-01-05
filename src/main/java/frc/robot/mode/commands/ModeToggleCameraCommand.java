package frc.robot.mode.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.NetworkTableComms;
import frc.robot.mode.RobotMode;

public class ModeToggleCameraCommand extends CommandBase {
    
    private final NetworkTableComms nt;
    private final RobotMode mode;
    private boolean initialValue;

    public ModeToggleCameraCommand(
            NetworkTableComms nt,
            RobotMode mode) {
        this.nt = nt;
        this.mode = mode;
    }

    @Override
    public void initialize() {
        this.initialValue = this.mode.GetJumpCamera();
    }

    @Override
    public void execute() {
        this.mode.SetJumpCamera(!this.initialValue);
        if(this.mode.GetJumpCamera()) {
            this.nt.setCameraJump();
        }
        else {
            this.nt.setCameraLimelight();
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
