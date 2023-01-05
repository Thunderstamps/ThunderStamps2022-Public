package frc.robot.subsystems.flywheel.commands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.flywheel.*;

// Commands the flywheel to run at a certain RPM until interrupted
public class FlywheelSpeedCommand extends CommandBase {
    
    private final IFlywheelSubsystem flywheelSubsystem;
    private final double rpm;

    public FlywheelSpeedCommand(IFlywheelSubsystem flywheelSubsystem, double rpm) {
        this.flywheelSubsystem = flywheelSubsystem;
        this.rpm = rpm;
        addRequirements(this.flywheelSubsystem);
    }

    @Override
    public void initialize() {
        this.flywheelSubsystem.executeVelocity(this.rpm);
    }

    @Override
    public void end(boolean interrupted) {
        this.flywheelSubsystem.executeVelocity(0.0);
    }
}
