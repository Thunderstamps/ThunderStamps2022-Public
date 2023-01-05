package frc.robot.subsystems.intake;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    
    private final TalonFX intakeMotor = new TalonFX(3, "Default Name");

    public IntakeSubsystem() {
        intakeMotor.setInverted(true);
        intakeMotor.configVoltageCompSaturation(10);
        intakeMotor.enableVoltageCompensation(true);
        intakeMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 10, 0.5));
    }

    public void execute(double speedPercent) {
        intakeMotor.set(ControlMode.PercentOutput, speedPercent);

    }


    
}
