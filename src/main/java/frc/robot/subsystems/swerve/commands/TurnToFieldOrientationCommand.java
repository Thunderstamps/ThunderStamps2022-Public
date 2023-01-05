package frc.robot.subsystems.swerve.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.swerve.*;

public class TurnToFieldOrientationCommand extends CommandBase {

    private final static double TOLERANCE_RAD = Math.toRadians(5.0);
    private final SwerveSubsystem swerveSubsystem;
    private final double targetFieldOrientation_rad;

    public TurnToFieldOrientationCommand(
            SwerveSubsystem swerveSubsystem,
            double targetFieldOrientation_rad) {
        this.swerveSubsystem = swerveSubsystem;
        this.targetFieldOrientation_rad = targetFieldOrientation_rad;
    }

    @Override
    public void initialize() {
        this.swerveSubsystem.setTargetFieldOrientation_rad(this.targetFieldOrientation_rad);
    }

    @Override
    public void execute() {
        this.swerveSubsystem.setTargetFieldOrientation_rad(this.targetFieldOrientation_rad);
    }

    @Override
    public boolean isFinished() {
        var diff_rad = SwerveUtil.limitAngleFromNegativePItoPI_rad(
            this.targetFieldOrientation_rad - this.swerveSubsystem.getFieldOrientation_rad()
        );
        return Math.abs(diff_rad) < TOLERANCE_RAD;
    }
}