package frc.robot.subsystems.feeder;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.mode.RobotMode;

public class FeederDefaultCommand extends CommandBase {

    private final RobotMode mode;
    private final FeederSubsystem feederSubsystem;

    public FeederDefaultCommand(
            RobotMode mode,
            FeederSubsystem feederSubsystem) {
        this.mode = mode;
        this.feederSubsystem = feederSubsystem;
        addRequirements(feederSubsystem);

    }
    
    @Override
    public void initialize() {
        this.feederSubsystem.SetBallPresent(false);
    }

    @Override
    public void execute() {
        if(!this.feederSubsystem.GetBallPresent() && this.feederSubsystem.SensorDetectsBall()) {
            this.feederSubsystem.SetBallPresent(true);
        }

        // Don't intake once we've started winching, but remember the Unjam command still needs to work.
        if(this.feederSubsystem.GetBallPresent() || this.mode.GetWinchStarted()) {
            this.feederSubsystem.execute(0.0);
        }
        else {
            this.feederSubsystem.executeSpeed(3000);
        }
    }
    
}
