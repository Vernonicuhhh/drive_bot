package frc.robot;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.util.Units;

public class Constants {

    public static class PhysicalConstants{

        public static final double GEAR_RATIO = 6.88888888889;
        
        public static final double WHEEL_DIAMETER_INCHES = 6d;
        public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(WHEEL_DIAMETER_INCHES);
        public static final double WHEEL_CIRCUMFERENCE_INCHES = 2* WHEEL_DIAMETER_INCHES*Math.PI;
        public static final double WHEEL_CIRCUMFERENCE_METERS = Units.inchesToMeters(WHEEL_DIAMETER_INCHES)*Math.PI;

        public static final double TRACK_WIDTH_METERS = .2159;

    }

    public static class DriveConstants{
        public static final double STICK_DEADBAND = .15;
        public static final double MAX_SPEED_TELE = 0;

        // voltage needed to overcome the motor's static friction
        public static final double kS = 0;

        //voltage needed to hold (cruise) at a given velocity.
        public static final double kA = 0;

        // voltage needed to induce a given acceleration at the motor shaft
        public static final double kV = 0;

        public static final SimpleMotorFeedforward VELOCITY_FEED_FORWARD = new SimpleMotorFeedforward(kS, kV, kA);

        public static final double kP = 0;

        public static final int DEVICE_ID_LEFT_MASTER = 8;
        public static final int DEVICE_ID_LEFT_SLAVE = 7;
        public static final int DEVICE_ID_RIGHT_MASTER = 10;
        public static final int DEVICE_ID_RIGHT_SLAVE = 9;

        // seconds taken for motor to ramp up from 0 to setpoint
        public static final double CLOSED_LOOP_RAMP = .2;
        public static final double OPEN_LOOP_RAMP = .25;



        public static final double MAX_SPEED_TELEOP = 3.25; 
        public static final double MAX_ANGULAR_VELOCITY = 3.25;  //MUST BE IN RAD/S

        public static final double TURN_MULTIPLIER = .05;
        public static final double WHEEL_NONLINEARITY = .05;

        public static final double TURN_P = .02;
        public static final double TURN_I = 0;
        public static final double TURN_D = 0;
    }
}