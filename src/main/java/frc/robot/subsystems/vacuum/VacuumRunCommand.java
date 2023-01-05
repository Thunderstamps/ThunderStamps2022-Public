package frc.robot.subsystems.vacuum;

import edu.wpi.first.wpilibj2.command.*;

public class VacuumRunCommand extends CommandBase {

    private final VacuumSubsystem vacuumSubsystem;

    public VacuumRunCommand(VacuumSubsystem vacuumSubsystem) {
        this.vacuumSubsystem = vacuumSubsystem;
        addRequirements(this.vacuumSubsystem);
        this.setName("Run Vacuum Pump");
    }

    @Override
    public void execute() {
        // This is initiated by a button on the dashboard.
        // It's terminated by clicking the button on the dashboard again.
        this.vacuumSubsystem.execute(true);
    }
}
