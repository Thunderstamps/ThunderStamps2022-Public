package frc.robot.subsystems.flywheel.commands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.mode.RobotMode;
import frc.robot.subsystems.flywheel.*;

public class FlywheelDefaultCommand extends CommandBase {

    private final RobotMode mode;
    private final SimpleFlywheelSubsystem flywheelSubsystem;

    public FlywheelDefaultCommand(
            RobotMode mode,
            SimpleFlywheelSubsystem flywheelSubsystem) {
        this.mode = mode;
        this.flywheelSubsystem = flywheelSubsystem;
        addRequirements(this.flywheelSubsystem);
    }

    @Override
    public void execute() {
        // Stop flywheel once we've started winching
        var speed = this.mode.GetWinchStarted() ? 0.0 : 1.0;
        this.flywheelSubsystem.execute(speed);
    }
}