package frc.subsystems.simulation;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import frc.robot.Constants;

public class DriveSim {
    
    WPI_TalonSRX simLeftMaster;
    WPI_TalonSRX simLeftSlave;
    WPI_TalonSRX simRightMaster;
    WPI_TalonSRX simRightSlave;

    public DriveSim(){
        simLeftMaster = new WPI_TalonSRX(Constants.DriveConstants.DEVICE_ID_LEFT_MASTER);
        simLeftSlave = new WPI_TalonSRX(Constants.DriveConstants.DEVICE_ID_LEFT_SLAVE);
        simRightMaster = new WPI_TalonSRX(Constants.DriveConstants.DEVICE_ID_RIGHT_MASTER);
        simRightSlave = new WPI_TalonSRX(Constants.DriveConstants.DEVICE_ID_RIGHT_SLAVE);
    }
}
