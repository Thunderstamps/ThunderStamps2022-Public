package frc.robot.subsystems.flywheel;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.NetworkTableComms;

public class DoubleFalconFlywheelSubsystem extends SubsystemBase implements IFlywheelSubsystem {

    private final static double MAX_MOTOR_RPM = 6000.0;
    private final static double COUNTS_PER_MOTOR_REV = 2048.0;
    private final static int MOTOR_PULLEY_TEETH = 16; 
    private final static int FLYWHEEL_PULLEY_TEETH = 24; 
    private final static double FLYWHEEL_RPM_PER_MOTOR_RPM = (double)MOTOR_PULLEY_TEETH / (double)FLYWHEEL_PULLEY_TEETH;
    private final static double MAX_CURRENT_A = 40.0; // per motor, max current during accel
    private final static double LIMITED_CURRENT_A = 11.0; // per motor, limited current during shot

    private final TalonFX talon1;
    private final TalonFX talon2;

    private final NetworkTableComms nt;

    private double motorRPM = 0.0;
    private double flywheelRPM = 0.0;
    private double lastTargetMotorRPM = 0.0;
    private boolean atSpeed = false;

    public DoubleFalconFlywheelSubsystem(
        int canBusAddress1, 
        int canBusAddress2,
        NetworkTableComms nt) {

        this.nt = nt;

        talon1 = new TalonFX(canBusAddress1, "Default Name");
        talon2 = new TalonFX(canBusAddress2, "Default Name");
        talon2.follow(talon1, FollowerType.PercentOutput);

        talon1.setInverted(false);
        talon2.setInverted(true);
        
        // neutral behavior
        talon1.setNeutralMode(com.ctre.phoenix.motorcontrol.NeutralMode.Coast);
        talon2.setNeutralMode(com.ctre.phoenix.motorcontrol.NeutralMode.Coast);

        // stop it
        talon1.set(ControlMode.PercentOutput, 0.0);
        
        talon1.selectProfileSlot(0, 0);
        talon1.config_kF(0, 0.0538, 10);
        talon1.config_kP(0, 0.3, 10);
        talon1.config_kI(0, 0.0008, 10);
        talon1.config_kD(0, 0.0, 10);
        talon1.config_IntegralZone(0, 600);
        
        talon1.configClosedloopRamp(1.0, 10);
        talon1.configOpenloopRamp(2.0, 10);
        talon2.configClosedloopRamp(1.0, 10);
        talon2.configOpenloopRamp(2.0, 10);
        
        this.maxCurrent();
    }

    public void executeVelocity(double flywheelRPM) {
        double targetMotorRPM = flywheelRPM / FLYWHEEL_RPM_PER_MOTOR_RPM;
        if(targetMotorRPM > MAX_MOTOR_RPM) {
            targetMotorRPM = MAX_MOTOR_RPM;
        }
        if(targetMotorRPM < -MAX_MOTOR_RPM) {
            targetMotorRPM = -MAX_MOTOR_RPM;
        }

        this.lastTargetMotorRPM = targetMotorRPM;

        if(Math.abs(targetMotorRPM) < 1.0) {
            this.executePercent(0.0);
        }
        else {
            double motorCountsPer100ms = COUNTS_PER_MOTOR_REV * targetMotorRPM / 600.0;
            talon1.set(ControlMode.Velocity, motorCountsPer100ms);
        }

        // System.out.print(targetMotorRPM + ", ");
        // this.printInputCurrentAndRPM();
    }

    @Override
    public void periodic() {
        var rawVelocityCountsPer100ms = talon1.getSelectedSensorVelocity();
        this.motorRPM = 600.0 * rawVelocityCountsPer100ms / COUNTS_PER_MOTOR_REV;
        this.flywheelRPM = getMotorRPM() * FLYWHEEL_RPM_PER_MOTOR_RPM;
        //this.nt.setFlywheelSpeed_rpm(this.getFlywheelRPM());
        var diffToTarget = Math.abs(getMotorRPM() - this.lastTargetMotorRPM);
        this.atSpeed = diffToTarget < 10.0 && this.lastTargetMotorRPM > 0.1;
        //this.nt.setFlywheelAtSpeed(this.atSpeed());
    }

    public double getMotorRPM() {
        return this.motorRPM;
    }

    public double getFlywheelRPM() {
        return this.flywheelRPM;
    }

    public void tune_kF(double minusOneToOne) {
        // run the motor at x% output and see how fast it goes
        this.executePercent(minusOneToOne);
        var rawVelocityCountsPer100ms = talon1.getSelectedSensorVelocity();

        // calculate what kF should be
        if(Math.abs(minusOneToOne) > 0.05 && Math.abs(rawVelocityCountsPer100ms) > 100) {
            double kF = (minusOneToOne * 1023.0) / rawVelocityCountsPer100ms;
            System.out.printf("%.2f", minusOneToOne); System.out.print(", ");
            System.out.print(rawVelocityCountsPer100ms); System.out.print(", ");
            System.out.printf("kF: %.6f", kF); System.out.print(", ");
            this.printInputCurrentAndRPM();
        }
    }

    private void printInputCurrentAndRPM() {
        System.out.printf("%.0f A", talon1.getSupplyCurrent()); System.out.print(", ");
        System.out.printf("%.0f A", talon2.getSupplyCurrent()); System.out.print(", ");
        System.out.printf("%.2f pct", talon1.getMotorOutputPercent()); System.out.print(", ");
        System.out.printf("%.2f pct", talon2.getMotorOutputPercent()); System.out.print(", ");
        System.out.printf("%.0f Mot RPM", this.getMotorRPM()); System.out.print(", ");
        System.out.printf("%.0f Fly RPM", this.getFlywheelRPM()); 
        System.out.println();
    }

    private void executePercent(double minusOneToOne) {
        talon1.set(ControlMode.PercentOutput, minusOneToOne);
    }

    @Override
    public boolean atSpeed() {
        return this.atSpeed;
    }

    @Override
    public void maxCurrent() {
        talon1.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, MAX_CURRENT_A, MAX_CURRENT_A, 0.0));
        talon2.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, MAX_CURRENT_A, MAX_CURRENT_A, 0.0));
    }

    @Override
    public void limitCurrent() {
        talon1.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, LIMITED_CURRENT_A, LIMITED_CURRENT_A, 0.0));
        talon2.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, LIMITED_CURRENT_A, LIMITED_CURRENT_A, 0.0));
    }
}