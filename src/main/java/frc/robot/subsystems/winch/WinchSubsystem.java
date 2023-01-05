package frc.robot.subsystems.winch;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.NetworkTableComms;

public class WinchSubsystem extends SubsystemBase {

    private final NetworkTableComms nt;

    // winch 2 must be inverted
    private final WinchSide winch32 = new WinchSide(32, false);
    private final WinchSide winch33 = new WinchSide(33, true);

    public WinchSubsystem(
            NetworkTableComms nt) {
        this.nt = nt;
    }

    public void RunRobotPeriodic() {

        this.winch32.RunRobotPeriodic();
        this.winch33.RunRobotPeriodic();

        this.nt.setWinchPositions(
            this.winch32.getWinchPosition_in(),
            this.winch32.getWinchAtTop(), 
            this.winch33.getWinchPosition_in(),
            this.winch33.getWinchAtTop());
        this.nt.setWinchPositionErrors(
            this.winch32.getPositionError(), 
            this.winch33.getPositionError());
    }

    public void execute(boolean winch, double ramp) {
        this.winch32.execute(winch, ramp);
        this.winch33.execute(winch, ramp);
    }

    public boolean isAtTop() {
        return this.winch32.getWinchAtTop()
            && this.winch33.getWinchAtTop();
    }
}
