package frc.robot.mode;

public class RobotMode {
    private boolean aiming = false;
    private boolean winchStarted = false;
    private boolean jumpCamera = false;

    public void SetAiming(boolean value) {
        this.aiming = value;
    }

    public boolean GetAiming() {
        return this.aiming;
    }

    public void SetWinchStarted() {
        this.winchStarted = true;
    }

    public boolean GetWinchStarted() {
        return this.winchStarted;
    }

    public void SetJumpCamera(boolean value) {
        this.jumpCamera = value;
    }

    public boolean GetJumpCamera() {
        return this.jumpCamera;
    }
}
