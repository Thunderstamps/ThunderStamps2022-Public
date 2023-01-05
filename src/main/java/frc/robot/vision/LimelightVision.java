package frc.robot.vision;

import frc.robot.mode.RobotMode;
import edu.wpi.first.networktables.*;

public class LimelightVision {
    
    private final NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    private final NetworkTableEntry tv = table.getEntry("tv");
    private final NetworkTableEntry tx = table.getEntry("tx");
    private final NetworkTableEntry ty = table.getEntry("ty");
    private final NetworkTableEntry ledMode = table.getEntry("ledMode");
    private final RobotMode mode;

    private TargetData targetData = new TargetData(false, 0.0, 0.0);

    public LimelightVision(
            RobotMode mode) {
        this.mode = mode;
    }

    public TargetData GetTargetData() {
        return this.targetData;
    }

    public void RunRobotPeriodic() {
        this.readTargetData();
        this.setLEDs();
    }

    private void readTargetData() {
        this.targetData = new TargetData(
            this.tv.getDouble(0.0) >= 0.5, // returns 0 or 1, where 1 = valid target
            this.tx.getDouble(0.0), 
            this.ty.getDouble(0.0));
    }

    private void setLEDs() {
        var aiming = this.mode.GetAiming();
        var led = aiming ? 3 : 1;  // 3 = force LEDs on, 1 = force LEDs off
        this.ledMode.setNumber(led);
    }
}
