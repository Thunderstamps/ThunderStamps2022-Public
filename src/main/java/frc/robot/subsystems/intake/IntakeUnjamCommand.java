package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class IntakeUnjamCommand extends CommandBase {
    private final IntakeSubsystem intakeSubsystem;
    public IntakeUnjamCommand(IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);

    }
    

    @Override
    public void execute() {
        this.intakeSubsystem.execute(-0.2);
    }

    
}
