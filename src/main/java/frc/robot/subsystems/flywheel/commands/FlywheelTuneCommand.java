package frc.robot.subsystems.flywheel.commands;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.flywheel.*;

public class FlywheelTuneCommand extends CommandBase {

    private final IFlywheelSubsystem flywheelSubsystem;
    private final Joystick joystick;

    public FlywheelTuneCommand(
            IFlywheelSubsystem flywheelSubsystem, 
            Joystick joystick) {
        this.flywheelSubsystem = flywheelSubsystem;
        this.joystick = joystick;
        this.addRequirements(this.flywheelSubsystem);
    }

    @Override
    public void execute() {
        var throttleMinusOneToOne = -this.joystick.getThrottle(); // throttle axis is inverted
        var throttle = (throttleMinusOneToOne + 1.0) * 0.5;
        this.flywheelSubsystem.tune_kF(throttle);
    }

    @Override
    public void end(boolean interrupted) {
        this.flywheelSubsystem.executeVelocity(0.0);
    }
}