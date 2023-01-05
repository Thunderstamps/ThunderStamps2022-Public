package frc.robot.subsystems.winch.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.mode.RobotMode;
import frc.robot.subsystems.winch.*;

public class WinchRunCommand extends CommandBase {
    
    private final RobotMode mode;
    private final WinchSubsystem winchSubsystem;
    private final Timer timer = new Timer();

    public WinchRunCommand(
            RobotMode mode,
            WinchSubsystem winchSubsystem) {
        this.mode = mode;
        this.winchSubsystem = winchSubsystem;
        addRequirements(winchSubsystem);
        this.setName("Activate Winch");
    }

    @Override
    public void initialize() {
        this.timer.reset();
        this.timer.start();
        this.mode.SetWinchStarted();
        this.mode.SetJumpCamera(true); // toggles to jump camera for driver (they can toggle back with Y button)
    }

    @Override
    public void execute() {
        var elapsedTime_s = this.timer.get();

        // implement a 1 second ramp from 0 to 1
        var ramp = 1.0;
        if(elapsedTime_s < 1.0) {
            ramp = elapsedTime_s;
        }

        this.winchSubsystem.execute(true, ramp);
    }

    @Override
    public boolean isFinished() {
        return this.winchSubsystem.isAtTop();
    }

    @Override
    public void end(boolean interrupted) {
        this.timer.stop();
    }
}
