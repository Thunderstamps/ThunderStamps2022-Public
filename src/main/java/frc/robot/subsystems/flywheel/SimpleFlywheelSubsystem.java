package frc.robot.subsystems.flywheel;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.NetworkTableComms;

public class SimpleFlywheelSubsystem extends SubsystemBase {
    private static final double CURRENT_LIMIT_A = 50;

    private final TalonFX flywheel1 = new TalonFX(5, "Default Name");
    private final NetworkTableComms nt;
    private double flywheelSpeed;
    private boolean fullCurrent;

    public SimpleFlywheelSubsystem(
            NetworkTableComms nt) {
        this.nt = nt;

        this.flywheel1.configVoltageCompSaturation(6.0);
        this.flywheel1.enableVoltageCompensation(true);

        this.flywheel1.setNeutralMode(NeutralMode.Coast);

        this.SetFullCurrent();
        
        this.flywheel1.configOpenloopRamp(1.0, 10); 
    
        this.flywheel1.setInverted(false);
        
    }

    public void RunRobotPeriodic() {
        this.flywheelSpeed = this.flywheel1.getSelectedSensorVelocity();
        this.nt.setFlywheelSpeed(this.flywheelSpeed);
    }

    public void execute(double percent) {
        this.flywheel1.set(ControlMode.PercentOutput, percent);
    }

    public void SetFullCurrent() {
        if(!fullCurrent) {
            this.flywheel1.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, CURRENT_LIMIT_A, CURRENT_LIMIT_A, 0));
            this.fullCurrent = true;
        }
    }

    public void SetLowCurrent() {
        if(fullCurrent) {
            this.flywheel1.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 1.0, 1.0, 0));
            this.fullCurrent = false;
        }
    }
}
