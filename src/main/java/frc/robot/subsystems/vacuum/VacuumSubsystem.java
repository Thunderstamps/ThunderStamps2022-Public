package frc.robot.subsystems.vacuum;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.*;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.NetworkTableComms;
import frc.robot.Robot;

public class VacuumSubsystem extends SubsystemBase {
    
    private static final double MAX_CURRENT_A = 20.0;
    private static final double MOTOR_PERCENT = 0.25; // don't go above 25% or we could damage the pump

    private final TalonSRX talon;
    private final AnalogInput vacSensor = new AnalogInput(0);
    private final NetworkTableComms nt;
    private double vacSensorValue_inHg = 0.0;
    private double vacuumTime_s = 0.0;

    public VacuumSubsystem(
            NetworkTableComms nt) {
        this.nt = nt;
        talon = new TalonSRX(31);

        talon.setInverted(false);
        
        // neutral behavior
        talon.setNeutralMode(com.ctre.phoenix.motorcontrol.NeutralMode.Coast);

        // stop it
        talon.set(ControlMode.PercentOutput, 0.0);
        
        talon.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, MAX_CURRENT_A, MAX_CURRENT_A, 0.0));
    }

    public void RunRobotPeriodic() {
        this.vacSensorValue_inHg = roundToOneDecimal(voltsToInchesMercury(this.vacSensor.getVoltage()));
        this.nt.setVacuumSensor(this.vacSensorValue_inHg);
    }

    private double voltsToInchesMercury(double volts) {
        return 14.553 * volts - 15.446;
    }

    private double roundToOneDecimal(double value) {
        return Math.round(value * 10.0) / 10.0;
    }

    public void execute(boolean runVacuumPump) {
        var speed = runVacuumPump ? MOTOR_PERCENT : 0.0;
        talon.set(ControlMode.PercentOutput, speed);

        if(runVacuumPump) {
            this.vacuumTime_s += Robot.SCAN_TIME_S;
        }
    }

    public double GetVacuum_inHg() {
        return this.vacSensorValue_inHg;
    }

    public boolean IsVacuumed() {
        return this.vacuumTime_s >= 2.0;
    }
}
