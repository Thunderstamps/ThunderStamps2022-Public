package frc.robot.subsystems.feeder;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.*;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.NetworkTableComms;


public class FeederSubsystem extends SubsystemBase {

    private final static double COUNTS_PER_MOTOR_REV = 2048.0;

    private final DigitalInput ballsensor = new DigitalInput(0);
    private final TalonFX feederMotor = new TalonFX(4, "Default Name");
    private final NetworkTableComms nt;
    private double feederSpeed;
    private double feederPosition;
    private boolean ballPresent = false;

        
    public FeederSubsystem(
            NetworkTableComms nt) {
        this.nt = nt;
        feederMotor.setInverted(false);
        feederMotor.configVoltageCompSaturation(10);
        feederMotor.enableVoltageCompensation(true);
        feederMotor.setNeutralMode(com.ctre.phoenix.motorcontrol.NeutralMode.Brake);
        feederMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 40, 0.0));
        
        feederMotor.selectProfileSlot(0, 0);
        feederMotor.config_kF(0, 0.0645, 10); // 0.28 * 1023 / 4450
        feederMotor.config_kP(0, 0.1, 10);
        feederMotor.config_kI(0, 0.0002, 10);
        feederMotor.config_kD(0, 0.0, 10);
        feederMotor.config_IntegralZone(0, 600);
    }

    public void RunRobotPeriodic() {
        this.feederSpeed = this.feederMotor.getSelectedSensorVelocity();
        this.feederPosition = this.feederMotor.getSelectedSensorPosition();
        this.nt.setFeederSpeed(this.feederSpeed);
    }
    
    public void execute(double speedPercent) {
        feederMotor.set(ControlMode.PercentOutput, speedPercent);
    }
    
    public void executeSpeed(double speedCountsPer100ms) {
        feederMotor.set(ControlMode.Velocity, speedCountsPer100ms);
    }

    public void ZeroPosition() {
        this.feederMotor.setSelectedSensorPosition(0.0);
        this.feederPosition = 0.0;
    }

    public double GetPositionRotations() {
        return this.feederPosition / COUNTS_PER_MOTOR_REV;
    }

    public boolean GetBallPresent() {
        return this.ballPresent;
    }

    public void SetBallPresent(boolean value) {
        this.ballPresent = value;
    }

    public boolean SensorDetectsBall() {
        return !this.ballsensor.get();
    }
}
