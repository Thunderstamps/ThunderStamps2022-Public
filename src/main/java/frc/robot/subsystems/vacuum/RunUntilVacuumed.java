package frc.robot.subsystems.vacuum;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class RunUntilVacuumed extends CommandBase {

    private final VacuumSubsystem vacuumSubsystem;

    public RunUntilVacuumed(VacuumSubsystem vacuumSubsystem) {
        this.vacuumSubsystem = vacuumSubsystem;
        addRequirements(this.vacuumSubsystem);
    }

    @Override
    public void execute() {
        this.vacuumSubsystem.execute(true);
    }
    
    @Override
    public boolean isFinished() {
        return this.vacuumSubsystem.IsVacuumed();
    }


}
