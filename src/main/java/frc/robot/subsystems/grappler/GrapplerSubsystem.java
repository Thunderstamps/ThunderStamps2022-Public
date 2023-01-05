package frc.robot.subsystems.grappler;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.mode.RobotMode;

public class GrapplerSubsystem extends SubsystemBase {
    
    private final Servo unlockServo = new Servo(0);
    private final RobotMode mode;

    public GrapplerSubsystem(
            RobotMode mode) {
        this.mode = mode;
        this.unlockServo.set(0); // always start the match with it locked
    }

    public void Unlock() {
        this.unlockServo.set(0.28); // 0.6 would be 180 degrees, don't want to turn that far
    }

    public void Lock() {
        if(!this.mode.GetWinchStarted()) { // can only re-lock from Manual, and only if we haven't started winching
            this.unlockServo.set(0); 
        }
    }
    // public void Print() {
    //     System.out.println(this.unlockServo.getPosition());
    // }
}
