package frc.robot.vision;

import frc.robot.mode.RobotMode;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.DriverStation;

public class PhotonVision {

    private final NetworkTable table = NetworkTableInstance.getDefault().getTable("photonvision/Microsoft_LifeCam_HD-3000");
    private final NetworkTableEntry hasTarget = table.getEntry("hasTarget");
    private final NetworkTableEntry targetX = table.getEntry("targetPixelsX");
    private final NetworkTableEntry targetY = table.getEntry("targetPixelsY");
    private final NetworkTableEntry driverMode = table.getEntry("driverMode");
    private final NetworkTableEntry pipelineIndex = table.getEntry("pipelineIndex");
    private final RobotMode mode;

    private BallTargetData nullTargetData = new BallTargetData(false, 0.0, 0.0);
    private BallTargetData targetData = new BallTargetData(false, 0.0, 0.0);

    public PhotonVision(
            RobotMode mode) {
        this.mode = mode;
    }

    public BallTargetData GetBallTargetData() {
        return this.targetData;
    }

    public void RunRobotPeriodic() {
        this.readTargetData();
    }

    public void SetDriverMode(boolean setToDriveMode) {
         this.driverMode.setBoolean(setToDriveMode);
         if(setToDriveMode) {
            this.pipelineIndex.setDouble(-1);
         }
         else {
             var blue = DriverStation.getAlliance() == DriverStation.Alliance.Blue;
            var p = blue ? 1.0 : 0.0;
            this.pipelineIndex.setDouble(p);
         }
    }

    private void readTargetData() {
        this.targetData = new BallTargetData(
                this.hasTarget.getBoolean(false),
                this.targetX.getDouble(0.0),
                this.targetY.getDouble(0.0));
        if(this.targetData.ValidTargetData() && this.targetData.X_pixels() < 20.0) {
            this.targetData = this.nullTargetData; 
        }
    }

}
