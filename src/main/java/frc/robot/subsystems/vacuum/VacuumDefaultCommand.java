package frc.robot.subsystems.vacuum;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.mode.RobotMode;

public class VacuumDefaultCommand extends CommandBase {

    private final VacuumSubsystem vacuumSubsystem;
    private final RobotMode mode;

    public VacuumDefaultCommand(
        VacuumSubsystem vacuumSubsystem,
        RobotMode mode) {
        this.vacuumSubsystem = vacuumSubsystem;
        this.mode = mode;
        addRequirements(this.vacuumSubsystem);
    }

    @Override
    public void execute() {
        var automaticallyVacuum = DriverStation.isTeleopEnabled() 
            && DriverStation.getMatchTime() <= 30.0
            && DriverStation.getMatchTime() > 1.0
            && !this.mode.GetWinchStarted();
        this.vacuumSubsystem.execute(automaticallyVacuum);
    }
}
