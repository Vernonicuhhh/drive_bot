package frc.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.util.CommonConversions;
import frc.util.multithreading.Threaded;

public class Drive extends Threaded {
    TalonFX leftMaster;
    TalonFX leftSlave;
    TalonFX rightMaster;
    TalonFX rightSlave;
    PIDController turnPID;

    Rotation2d desiredHeading;
    
    DriveState driveState = DriveState.TELEOP;

    static Drive instance;

    AHRS gyro = new AHRS(SPI.Port.kMXP);

    public Drive(){
        leftMaster = new TalonFX(Constants.DriveConstants.DEVICE_ID_LEFT_MASTER);
        leftSlave = new TalonFX(Constants.DriveConstants.DEVICE_ID_LEFT_SLAVE);
        rightMaster = new TalonFX(Constants.DriveConstants.DEVICE_ID_RIGHT_MASTER);
        rightSlave = new TalonFX(Constants.DriveConstants.DEVICE_ID_RIGHT_SLAVE);
        
        
    leftMaster.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 10);
    rightMaster.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 10);

        TalonFXConfiguration motorConfig = new TalonFXConfiguration();
        motorConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
        motorConfig.neutralDeadband = Constants.DriveConstants.STICK_DEADBAND;
        motorConfig.slot0.kP = Constants.DriveConstants.kP;
        motorConfig.slot0.closedLoopPeakOutput = 1.0;
        motorConfig.closedloopRamp = Constants.DriveConstants.CLOSED_LOOP_RAMP;
        motorConfig.openloopRamp = Constants.DriveConstants.OPEN_LOOP_RAMP;

        leftMaster.configAllSettings(motorConfig);
        leftSlave.configAllSettings(motorConfig);
        rightMaster.configAllSettings(motorConfig);
        rightSlave.configAllSettings(motorConfig);

        rightMaster.overrideLimitSwitchesEnable(false);
        leftMaster.overrideLimitSwitchesEnable(false);

        leftSlave.follow(leftMaster);
        rightSlave.follow(rightMaster);

        leftMaster.setInverted(true);
        rightMaster.setInverted(true);
        leftSlave.setInverted(true);
        rightSlave.setInverted(true);

        zeroEncoders();
    }

    public enum DriveState{
        TELEOP,
        AUTO,
        TURN,
        TUNING,
        DONE
    }
    @Override
    public void update() {
        DriveState snapDriveState;
        synchronized(this){
            snapDriveState = driveState;    
        }

        switch(snapDriveState){
            case TELEOP:
                updateTeleop();
                break;
            case TURN:
                updateTurn();
                break;
            case AUTO:

                break;
            case TUNING:

                break;
        }
        
    }

    public static Drive getInstance(){
        if(instance==null)
            instance = new Drive();
        return instance;
    }

    private void updateTeleop(){
        tankDriveTeleOp(Robot.leftStick.getY(), Robot.rightStick.getY());
    }

    private void updateTurn(){
        double deltaSpeed = turnPID.calculate(RobotTracker.getInstance().getOdometry().getRotation().getDegrees(), desiredHeading.getDegrees());
        deltaSpeed = MathUtil.clamp(deltaSpeed, -1, 1);
        tankDriveVelocity(deltaSpeed, -deltaSpeed);

        if(Math.abs(deltaSpeed)<1E-3){
            synchronized(this){
                driveState = DriveState.TELEOP;
            }
        }
    }

    private void tankDriveTeleOp(double leftSpeed, double rightSpeed){
        if(Math.abs(leftSpeed)<=Constants.DriveConstants.STICK_DEADBAND){
            leftSpeed = 0;
        }

        if(Math.abs(rightSpeed)<=Constants.DriveConstants.STICK_DEADBAND){
            rightSpeed = 0;
        }

        tankDrive(leftSpeed, rightSpeed,false,true);
    }

    private void tankDrive(double l, double r, boolean squaredInputs, boolean velocity){
        if(velocity){
            double leftMPS =  l *Constants.DriveConstants.MAX_SPEED_TELE;
            double rightMPS =  r *Constants.DriveConstants.MAX_SPEED_TELE;
            if(squaredInputs){
                leftMPS = Math.copySign(Math.pow(leftMPS, 4), leftMPS);
                rightMPS = Math.copySign(Math.pow(rightMPS, 4), rightMPS);
            }
            tankDriveVelocity(leftMPS, rightMPS);
        }
    
        else{
            if(squaredInputs){
                l = Math.copySign(Math.pow(l, 4), l);
                r = Math.copySign(Math.pow(r, 4), r);
                }
            setRawSpeeds(l, r);
        }
    }

 /**
   * controls the drivetrain at a given velocity during teleoperated control
   * @param left left velocity of robot in meters/sec
   * @param right right velocity of robot in meters/sec
   */
    private void tankDriveVelocity(double leftSetpoint,double rightSetpoint){

        //get actual velocities of the drivetrain, parameters are setpoints
        double actualLeft = getLeftVelocity();
        double actualRight = getRightVelocity();

        //find acceleration setpoint using: a = dv/dt
        //in this case dt is .2 because this runs every 20ms
        double leftAccelSetpoint = (leftSetpoint - actualLeft)/.2;
        double rightAccelSetpoint = (rightSetpoint-actualRight)/.2;

        // calculate the voltage needed to get to velocity and acceleration setpoint
        double leftFF = Constants.DriveConstants.VELOCITY_FEED_FORWARD.calculate(leftSetpoint,leftAccelSetpoint);
        double rightFF = Constants.DriveConstants.VELOCITY_FEED_FORWARD.calculate(rightSetpoint, rightAccelSetpoint);

        if(leftSetpoint == 0)
            leftMaster.set(ControlMode.PercentOutput, 0);
        else
            leftMaster.set(ControlMode.Velocity,CommonConversions.metersPerSecToStepsPerDecisec(leftSetpoint), DemandType.ArbitraryFeedForward,leftFF/12); //divide by 12 bc feedforward returns voltage input to motor, dividing by 12 would make it in range of [-1,1]

        if(rightSetpoint == 0)
            rightMaster.set(ControlMode.PercentOutput, 0);
        else
            rightMaster.set(ControlMode.Velocity,CommonConversions.metersPerSecToStepsPerDecisec(rightSetpoint), DemandType.ArbitraryFeedForward,rightFF/12);
    }

    /**
     * 
     * turns the robot to desired angle
     * @param angle - desired angle in degrees
     */
    public synchronized void turnToAngle(double angle){
        synchronized(this){
        driveState = DriveState.TURN;
        desiredHeading = new Rotation2d(Units.degreesToRadians(angle));
        }
    }

    /**
     * 
     * @param leftSpeed percent output of left motors on [-1,1]
     * @param rightSpeed percent output of right motors on [-1,1]
     */
    public void setRawSpeeds(double leftSpeed, double rightSpeed){
        leftMaster.set(ControlMode.PercentOutput, leftSpeed);
        rightMaster.set(ControlMode.PercentOutput, rightSpeed);
    }
    
    public synchronized void editPIDGains(double kPInc, double kIInc, double kDInc){
    }

    /**
     * @return the velocity of the left side of the drivetrain in meters per second
     * 
     */
    private double getLeftVelocity(){
        return CommonConversions.stepsPerDecisecToMetersPerSec(leftMaster.getSelectedSensorVelocity());
    }

    /**
     * @return the velocity of the right side of the drivetrain in meters per second
     * 
     */
    private double getRightVelocity(){
        return CommonConversions.stepsPerDecisecToMetersPerSec(rightMaster.getSelectedSensorVelocity());
    }

    /**
     * 
     * @return distance of the left side of the drivetrain in meters
     */
    public double getLeftDistance(){
        return CommonConversions.stepsToMeters(leftMaster.getSelectedSensorPosition());
    }

    /**
     * 
     * @return distance of the right side of the drivetrain in meters
     */
    public double getRightDistance(){
        return CommonConversions.stepsToMeters(rightMaster.getSelectedSensorPosition());
    }

    public void zeroEncoders(){
        rightMaster.setSelectedSensorPosition(0);
        leftMaster.setSelectedSensorPosition(0);
    }

    /**
     * 
     * @return the heading of the drivetrain as a Rotation2d object
     */
    public Rotation2d getRotation2d(){
        return gyro.getRotation2d();
    }

    /**
     * 
     * @return heading of the drivetrain from [-180,180] degrees
     */
    public double getHeading(){
        return Math.IEEEremainder(gyro.getAngle(), 360.0d) * -1.0d; 
    }

    public void resetGyro(){
        gyro.reset();
    }

    public void zeroSensors(){
        gyro.zeroYaw();
        zeroEncoders();
    } 
}