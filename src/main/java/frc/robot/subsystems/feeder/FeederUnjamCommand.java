package frc.robot.subsystems.feeder;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class FeederUnjamCommand extends CommandBase{

    private final FeederSubsystem feederSubsystem;
    public FeederUnjamCommand(FeederSubsystem feederSubsystem) {
        this.feederSubsystem = feederSubsystem;
        addRequirements(feederSubsystem);

    }
    

    @Override
    public void execute() {
        this.feederSubsystem.execute(-0.2);
    }

    
}
