package frc.robot.subsystems.winch.commands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.winch.*;

public class WinchDefaultCommand extends CommandBase {
    
    private final WinchSubsystem winchSubsystem;

    public WinchDefaultCommand(WinchSubsystem winchSubsystem) {
        this.winchSubsystem = winchSubsystem;
        addRequirements(winchSubsystem);
    }

    @Override
    public void execute() {
        this.winchSubsystem.execute(false, 1.0);
    }
}
