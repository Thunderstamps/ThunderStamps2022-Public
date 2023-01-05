package frc.robot.teleop;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.mode.RobotMode;
import frc.robot.subsystems.vacuum.*;
import frc.robot.subsystems.winch.*;
import frc.robot.subsystems.winch.commands.WinchRunCommand;


public class WinchAndVacuumCommand extends SequentialCommandGroup {

    public WinchAndVacuumCommand(
            RobotMode mode,
            VacuumSubsystem vacuumSubsystem, 
            WinchSubsystem winchSubsystem){

        this.setName("Vacuum and Winch");
    
        addCommands(
            new SequentialCommandGroup(
                new RunUntilVacuumed(vacuumSubsystem),
                new WinchRunCommand(mode, winchSubsystem)
            )
        );

    }
    
    
}
