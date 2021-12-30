package frc.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.common.hardware.VisionLEDMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.util.multithreading.Threaded;

public class VisionManager extends Threaded{
    static VisionManager instance;
    PhotonCamera camera = new PhotonCamera("limelight");

    VisionState visionState =VisionState.ON;

    double yaw; 

    public enum VisionState{
        OFF,
        ON
    }


    public synchronized double getTargetYaw(){
        var result = camera.getLatestResult();
        if(result.hasTargets())
        return result.getBestTarget().getYaw();
        else
            return 0;
            //return RobotTracker.getInstance().getOdometry().getRotation().getDegrees();
    }

    public static VisionManager getInstance(){
        if(instance == null)
            instance = new VisionManager();
            
        return instance;
    }

    @Override
    public void update() {
        VisionState snapVisionState;
        synchronized(this){
            snapVisionState = visionState;
        }

        switch(snapVisionState){
            case OFF:
                SmartDashboard.putString("vision state", "off");
            case ON:
                SmartDashboard.putString("vision state", "on");
        }
    }
    

    public synchronized void setOff(){
        camera.setLED(VisionLEDMode.kOff);
        camera.setDriverMode(true);
    }

    public synchronized void setOn(){
        camera.setDriverMode(false);
        camera.setLED(VisionLEDMode.kOn);
    }
}
