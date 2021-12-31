package frc.subsystems;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpiutil.math.VecBuilder;
import frc.auto.PathTrigger;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.util.CommonConversions;
import frc.util.OrangeUtility;
import frc.util.SynchronousPID;
import frc.util.multithreading.Threaded;


public class Drive extends Threaded {
    private TalonFX leftMaster;
    private TalonFX leftSlave;
    private TalonFX rightMaster;
    private TalonFX rightSlave;
    private AHRS gyro = new AHRS(SPI.Port.kMXP);

    private Rotation2d desiredHeading;
    private DriveState driveState = DriveState.TELEOP;
    private static Drive instance;

    private ArrayList <PathTrigger> triggers = new ArrayList<>();
    private Trajectory currentTrajectory;
    private Timer ramseteTimer;
    private RamseteController ramseteController;
    private double previousTime;

    private PIDController turnPID;

    DifferentialDrivetrainSim driveSim;

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
        rightMaster.setInverted(false);
        leftSlave.setInverted(InvertType.FollowMaster);
        rightSlave.setInverted(InvertType.FollowMaster);

        leftMaster.setNeutralMode(NeutralMode.Coast);
        leftSlave.setNeutralMode(NeutralMode.Coast);
        rightMaster.setNeutralMode(NeutralMode.Coast);
        rightSlave.setNeutralMode(NeutralMode.Coast);

        //turnPID = new SynchronousPID(-.0001, 0, 0, 0);
        //turnPID = new PIDController(.0250033,0,.000725,.01); //.0250033 //.000615
        turnPID = new PIDController(.01862,0,.003863555105);

        ramseteController = new RamseteController();
        ramseteTimer = new Timer();

        zeroEncoders();

        driveSim = new DifferentialDrivetrainSim(
            LinearSystemId.identifyDrivetrainSystem(Constants.DriveConstants.kV, Constants.DriveConstants.kA, Constants.DriveConstants.kVAngular, Constants.DriveConstants.kAAngular),
            DCMotor.getFalcon500(2),
            Constants.PhysicalConstants.GEAR_RATIO,
            Constants.PhysicalConstants.TRACK_WIDTH_METERS,
            Constants.PhysicalConstants.WHEEL_DIAMETER_METERS,
            VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005)
        );


    }

    public enum DriveState{
        TELEOP,
        AUTO,
        TURN,
        VISION_TRACKING,
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
                SmartDashboard.putString("Drive State", "teleop");
                break;
            case TURN:
                updateTurn();
                SmartDashboard.putString("Drive State", "turn");
                break;
            case AUTO:
                updatePathController();
                SmartDashboard.putString("Drive State", "auto");
                break;
            case VISION_TRACKING:
                updateVisionTracking();
                SmartDashboard.putString("Drive State", "vision tracking");
                break;
            case DONE:
                SmartDashboard.putString("Drive State", "done");
                break;
        }
        
    }

    public static Drive getInstance(){
        if(instance==null)
            instance = new Drive();
        return instance;
    }

    public void setTeleop(){
        synchronized(this){
        driveState = DriveState.TELEOP;
        }
    }

    public synchronized void setAutoPath(Trajectory desiredTrajectory,ArrayList<PathTrigger>actions){
        ramseteTimer.reset();
        ramseteTimer.start();
        currentTrajectory = desiredTrajectory;
        driveState = DriveState.AUTO;
        updatePathController();
        triggers = actions;
    }

    public synchronized void setAutoPath(Trajectory desiredTrajectory){
        ramseteTimer.reset();
        ramseteTimer.start();
        currentTrajectory = desiredTrajectory;
        driveState = DriveState.AUTO;
        updatePathController();
    }


    public synchronized void setTracking(){
        driveState = DriveState.VISION_TRACKING;
    }

    private void updateTeleop(){
        tankDriveTeleOp(-Robot.leftStick.getY(), -Robot.rightStick.getY());
    }

    private void updatePathController(){
        double currentTime = ramseteTimer.get();
        SmartDashboard.putBoolean("finished",isFinished());
        while (!triggers.isEmpty()) {
			if (triggers.get(0).getPercentage() <= getPathPercentage()) {
        triggers.remove(0).playTrigger();
			} else {
				break;
			}
		}

        Trajectory.State desiredPose = currentTrajectory.sample(currentTime);
        ChassisSpeeds speedSetpoint = ramseteController.calculate(RobotTracker.getInstance().getOdometry(), desiredPose);

        DifferentialDriveWheelSpeeds wheelSpeeds = Constants.DriveConstants.DRIVE_KINEMATICS.toWheelSpeeds(speedSetpoint);

        double leftSetpoint = wheelSpeeds.leftMetersPerSecond;
        double rightSetpoint = wheelSpeeds.rightMetersPerSecond;
        double dt = (currentTime-previousTime)*10;
        boolean finished = ramseteTimer.hasPeriodPassed(currentTrajectory.getTotalTimeSeconds());

        if(finished){
            synchronized(this){
                driveState = DriveState.DONE;
            }
            ramseteTimer.stop();
        }

        tankDriveVelocity(leftSetpoint, rightSetpoint, dt);
        previousTime = currentTime;
    }

    private void updateTurn(){
     double error = desiredHeading.getDegrees()-getHeading();
     SmartDashboard.putNumber("error", error);
     double deltaSpeed = turnPID.calculate(getHeading(), desiredHeading.getDegrees());
     SmartDashboard.putNumber("heading", getHeading());
     SmartDashboard.putNumber("desired heading", desiredHeading.getDegrees());
     SmartDashboard.putNumber("deltaSpeed", deltaSpeed);
     

        if(Math.abs(deltaSpeed)<1E-1){
            /*synchronized(this){
                driveState = DriveState.TELEOP;
            }*/
            setRawSpeeds(.3, .3);
        }

        else
        setRawSpeeds(deltaSpeed, -deltaSpeed);
        //tankDriveVelocity(deltaSpeed*Constants.DriveConstants.MAX_SPEED_TELE, -deltaSpeed*Constants.DriveConstants.MAX_SPEED_TELE);

     /*double error = wantedHeading.rotateBy(RobotTracker.getInstance().getOdometry().getRotation().unaryMinus()).getDegrees();
		double deltaSpeed; 
		deltaSpeed = turnPID.update(error);
		deltaSpeed = Math.copySign(
		OrangeUtility.coercedNormalize(Math.abs(deltaSpeed), 0, 180, 0, TrajectoryConstants.MAX_SPEED_AUTO), deltaSpeed);
		if (Math.abs(error) < 10 && deltaSpeed < 0.2) {
			tankDriveVelocity(0, 0);
			synchronized (this) {
				driveState = DriveState.DONE;
			} 
		} else {
			tankDriveVelocity(-deltaSpeed, deltaSpeed);
		}*/
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

        SmartDashboard.putNumber("left Veloctiy", actualLeft);
        SmartDashboard.putNumber("left setpoint", leftSetpoint);

        //find acceleration setpoint using: a = dv/dt
        //in this case dt is .2 because this runs every 20ms
        double leftAccelSetpoint = (leftSetpoint - actualLeft)/.2;
        double rightAccelSetpoint = (rightSetpoint-actualRight)/.2;
        SmartDashboard.putNumber("left accel", leftAccelSetpoint);

        // calculate the voltage needed to get to velocity and acceleration setpoint
        double leftFF = Constants.DriveConstants.VELOCITY_FEED_FORWARD.calculate(leftSetpoint,leftAccelSetpoint);
        double rightFF = Constants.DriveConstants.VELOCITY_FEED_FORWARD.calculate(rightSetpoint, rightAccelSetpoint);

        SmartDashboard.putNumber("error", Math.abs(actualLeft-leftSetpoint));

        SmartDashboard.putNumber("voltage", leftFF);

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
   * controls the drivetrain at a given velocity during teleoperated control
   * @param left left velocity of robot in meters/sec
   * @param right right velocity of robot in meters/sec
   * @param dt change in time
   */
    private void tankDriveVelocity(double leftSetpoint,double rightSetpoint,double dt){
        //get actual velocities of the drivetrain, parameters are setpoints
        double actualLeft = getLeftVelocity();
        double actualRight = getRightVelocity();

        SmartDashboard.putNumber("left Veloctiy", actualLeft);
        SmartDashboard.putNumber("left setpoint", leftSetpoint);

        //find acceleration setpoint using: a = dv/dt
        //in this case dt is .2 because this runs every 20ms
        double leftAccelSetpoint = (leftSetpoint - actualLeft)/dt;
        double rightAccelSetpoint = (rightSetpoint-actualRight)/dt;

        // calculate the voltage needed to get to velocity and acceleration setpoint
        double leftFF = Constants.DriveConstants.VELOCITY_FEED_FORWARD.calculate(leftSetpoint,leftAccelSetpoint);
        double rightFF = Constants.DriveConstants.VELOCITY_FEED_FORWARD.calculate(rightSetpoint, rightAccelSetpoint);

        SmartDashboard.putNumber("error", Math.abs(actualLeft-leftSetpoint));

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

    private void updateVisionTracking(){
        double deltaSpeed = turnPID.calculate(-VisionManager.getInstance().getTargetYaw(), 0);
        setRawSpeeds(deltaSpeed, -deltaSpeed);
    }

    private double getPathPercentage(){
        return ramseteTimer.get()/currentTrajectory.getTotalTimeSeconds();
    }
    private boolean isFinished(){
        return driveState == DriveState.DONE;
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
        turnPID.setP(turnPID.getP()+kPInc);
        turnPID.setI(turnPID.getI()+kIInc);
        turnPID.setD(turnPID.getD()+kDInc);
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

    public synchronized DifferentialDriveWheelSpeeds getWheelSpeeds(){
        DifferentialDriveWheelSpeeds speeds = new DifferentialDriveWheelSpeeds(getLeftDistance(), getRightVelocity());
        return speeds;
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