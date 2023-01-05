package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.mode.RobotMode;

public class IntakeDefaultCommand extends CommandBase {

    private static final double INTAKE_SPEED = 0.4;
    private final RobotMode mode;
    private final IntakeSubsystem intakeSubsystem;

    public IntakeDefaultCommand(
            RobotMode mode,
            IntakeSubsystem intakeSubsystem) {
        this.mode = mode;
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);

    }
    
    @Override
    public void execute() {
        // Don't run intake after we've started winching.
        // Note that the Unjam Command still needs to work.
        var speed = this.mode.GetWinchStarted() ? 0.0 : INTAKE_SPEED;
        this.intakeSubsystem.execute(speed);
    }
}
