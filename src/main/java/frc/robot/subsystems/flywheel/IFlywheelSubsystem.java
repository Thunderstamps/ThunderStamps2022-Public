package frc.robot.subsystems.flywheel;

import edu.wpi.first.wpilibj2.command.Subsystem;

public interface IFlywheelSubsystem extends Subsystem {
    void executeVelocity(double flywheelRPM);
    double getMotorRPM();
    double getFlywheelRPM();
    void tune_kF(double minusOneToOne);
    boolean atSpeed();

    void maxCurrent();
    void limitCurrent();
}