package org.team1619.behavior;

import org.uacr.models.behavior.Behavior;
import org.uacr.shared.abstractions.InputValues;
import org.uacr.shared.abstractions.OutputValues;
import org.uacr.shared.abstractions.RobotConfiguration;
import org.uacr.utilities.Config;
import org.uacr.utilities.logging.LogManager;
import org.uacr.utilities.logging.Logger;
import org.uacr.utilities.Timer;

import java.util.*;

/**
 * Example behavior to copy for other behaviors
 */

public class Behavior_Drivetrain_Swerve implements Behavior {

	private static final Logger sLogger = LogManager.getLogger(Behavior_Drivetrain_Swerve.class);
	private static final Set<String> sSubsystems = Set.of("ss_drivetrain");

	private final InputValues fSharedInputValues;
	private final OutputValues fSharedOutputValues;
	private final String fXAxis_left_js;
	private final String fYAxis_left_js;
	private final String fXAxis_right_js;
	private final String fYAxis_right_js;

	private final double fRobotLength;
	private final double fRobotWidth;
	private final double fDiameter;

	private final String fNavx;
	private Map<String, Double> fNavxValues = new HashMap<>();

	public Behavior_Drivetrain_Swerve(InputValues inputValues, OutputValues outputValues, Config config, RobotConfiguration robotConfiguration) {
		fSharedInputValues = inputValues;
		fSharedOutputValues = outputValues;

		fXAxis_left_js = robotConfiguration.getString("global_drivetrain", "x_left_js");
		fYAxis_left_js = robotConfiguration.getString("global_drivetrain", "y_left_js");
		fXAxis_right_js = robotConfiguration.getString("global_drivetrain", "x_right_js");
		fYAxis_right_js = robotConfiguration.getString("global_drivetrain", "y_right_js");

		fRobotLength = robotConfiguration.getDouble("global_drivetrain", "robot_length");
		fRobotWidth = robotConfiguration.getDouble("global_drivetrain", "robot_width");
		fDiameter = Math.sqrt ((fRobotLength * fRobotLength) + (fRobotWidth * fRobotWidth));

		fNavx = robotConfiguration.getString("global_drivetrain", "navx");
	}

	@Override
	public void initialize(String stateName, Config config) {
		sLogger.debug("Entering state {}", stateName);
	}

	@Override
	public void update() {
		// Read joysticks
		double leftJs_yAxis = fSharedInputValues.getNumeric(fYAxis_left_js);
		double leftJs_xAxis = fSharedInputValues.getNumeric(fXAxis_left_js);
		double rightJs_xAxis = fSharedInputValues.getNumeric(fXAxis_right_js);
		double rightJs_yAxis = fSharedInputValues.getNumeric(fYAxis_right_js);

		// Define forward, strafe, point and rotate
		double forward = leftJs_yAxis;
		double strafe = -1 * leftJs_xAxis;
		double point = rightJs_yAxis;
		double rotate = -1 * rightJs_xAxis;

		// Get heading from the Navx
		fNavxValues = fSharedInputValues.getVector(fNavx);
		double heading = fNavxValues.getOrDefault("angle", 0.0) - 90;
		fSharedInputValues.setNumeric("opn_swerve_navx_heading", heading);

		// Field centric steering - adjust joysticks based on Navx heading
		if (fSharedInputValues.getBooleanRisingEdge("ipb_driver_start")) {
			fSharedInputValues.setBoolean("ipb_swerve_field_centric", !fSharedInputValues.getBoolean("ipb_swerve_field_centric"));
		}
		if (fSharedInputValues.getBoolean("ipb_swerve_field_centric")) {
			double radHeading = heading * Math.PI / 180;
			double temp = forward * Math.cos(radHeading) + strafe * Math.sin(radHeading);
			strafe = - forward * Math.sin(radHeading) + strafe * Math.cos(radHeading);
			forward = temp;
		}

		// Use the right joystick to point the robot in a specific direction instead of spinning continuously
		if (fSharedInputValues.getBoolean("ipb_driver_right_stick_button")){
			// Calculate the direction the joystick is pointing
			double rightJoystickDirection = Math.atan2(rightJs_xAxis, rightJs_yAxis) * 180 / Math.PI;
			fSharedInputValues.setNumeric("opn_swerve_right_joystick_direction", rightJoystickDirection);
			// Adjust rotation based on how far it needs to spin to get to the correct orientation
			double headingDiff = heading - rightJoystickDirection;
			fSharedInputValues.setNumeric("opn_swerve_heading_difference", headingDiff);
			rotate = headingDiff / 180;
			//todo - need way to increase roation value when close to zero to cause movement
		} else{
			fSharedInputValues.setNumeric("opn_swerve_right_joystick_direction", -9999);
			fSharedInputValues.setNumeric("opn_swerve_heading_difference", -9999);
		}

		// Output values for debugging
		fSharedInputValues.setNumeric("opn_swerve_forward", forward);
		fSharedInputValues.setNumeric("opn_swerve_strafe", strafe);
		fSharedInputValues.setNumeric("opn_swerve_rotate", rotate);


		double a = strafe - rotate * (fRobotLength / fDiameter);
		double b = strafe + rotate * (fRobotLength / fDiameter);
		double c = forward - rotate * (fRobotWidth / fDiameter);
		double d = forward + rotate * (fRobotWidth / fDiameter);

		// Calculate the wheel speed
		double frontRightMotorSpeed = Math.sqrt ((b * b) + (d * d));
		double frontLeftMotorSpeed = Math.sqrt ((b * b) + (c * c));
		double backLeftMotorSpeed = Math.sqrt ((a * a) + (c * c));
		double backRightMotorSpeed = Math.sqrt ((a * a) + (d * d));

		// Calculate the wheel angle
		double frontRightMotorAngle = Math.atan2 (b, d) * 180 / Math.PI;
		double frontLeftMotorAngle = Math.atan2 (b, c) * 180 / Math.PI;
		double backLeftMotorAngle = Math.atan2 (a, c) * 180 / Math.PI;
		double backRightMotorAngle = Math.atan2 (a, d) * 180 / Math.PI;

		// Normalize the wheel speed so they never exceed 1.0
		Double[] speeds = { frontRightMotorSpeed, frontLeftMotorSpeed, backLeftMotorSpeed, backRightMotorSpeed };
		double maxSpeed = Collections.max(Arrays.asList(speeds));
		if(maxSpeed >  1.0) {
			frontRightMotorSpeed /= maxSpeed;
			frontLeftMotorSpeed /= maxSpeed;
			backLeftMotorSpeed /= maxSpeed;
			backRightMotorSpeed /= maxSpeed;
		}

		// Slow down wheel speed when wheels are changing angle and flip the wheel direction if the angle difference is greater than 90

		// Get the current wheel angle
		double ipnfra = fSharedInputValues.getNumeric("ipn_drivetrain_front_right_angle");
		double ipnfla = fSharedInputValues.getNumeric("ipn_drivetrain_front_left_angle");
		double ipnbla = fSharedInputValues.getNumeric("ipn_drivetrain_back_left_angle");
		double ipnbra = fSharedInputValues.getNumeric("ipn_drivetrain_back_right_angle");

		// Scale the wheel speed  based on the cube of the difference in angle
		// Cos of zero difference is a scale factor of 1
		// A difference in angle greater than 90 degrees causes a negative value due to cos
		double scaleFrontRightSpeed = Math.pow((Math.cos((frontRightMotorAngle - ipnfra)*Math.PI/180)),3);
		double scaleFrontleftSpeed = Math.pow((Math.cos((frontLeftMotorAngle - ipnfla)*Math.PI/180)),3);
		double scaleBackLeftSpeed = Math.pow((Math.cos((backLeftMotorAngle - ipnbla)*Math.PI/180)),3);
		double scaleBackRightSpeed = Math.pow((Math.cos((backRightMotorAngle - ipnbra)*Math.PI/180)),3);

		// If the scale is negative, flip the wheel angle 180 degrees as the wheel speed will be negative
		if(scaleFrontRightSpeed < 0){ frontRightMotorAngle += 180; }
		if(scaleFrontleftSpeed < 0){ frontLeftMotorAngle += 180; }
		if(scaleBackLeftSpeed < 0){ backLeftMotorAngle += 180; }
		if(scaleBackRightSpeed < 0){ backRightMotorAngle += 180; }

		// Scale the wheel speed
		frontRightMotorSpeed = frontRightMotorSpeed * scaleFrontRightSpeed;
		frontLeftMotorSpeed = frontLeftMotorSpeed * scaleFrontleftSpeed;
		backLeftMotorSpeed = backLeftMotorSpeed * scaleBackLeftSpeed;
		backRightMotorSpeed = backRightMotorSpeed * scaleBackRightSpeed;

		// Rotate around one wheel
		if (fSharedInputValues.getBoolean("ipb_driver_dpad_up")){
			// Spin around left front wheel
			frontRightMotorSpeed = rightJs_xAxis * (fRobotWidth / fDiameter);
			frontLeftMotorSpeed = 0;
			backLeftMotorSpeed = rightJs_xAxis * (fRobotLength / fDiameter);
			backRightMotorSpeed = rightJs_xAxis;
			frontRightMotorAngle = 180;
			frontLeftMotorAngle = 0;
			backLeftMotorAngle = 90;
			backRightMotorAngle = 135;
		} else if (fSharedInputValues.getBoolean("ipb_driver_dpad_right")){
			// Spin around right front wheel
			frontRightMotorSpeed = 0;
			frontLeftMotorSpeed = rightJs_xAxis * (fRobotWidth / fDiameter);
			backLeftMotorSpeed = rightJs_xAxis;
			backRightMotorSpeed = rightJs_xAxis * (fRobotLength / fDiameter);
			frontRightMotorAngle = 0;
			frontLeftMotorAngle = 0;
			backLeftMotorAngle = 45;
			backRightMotorAngle = 90;

		} else if (fSharedInputValues.getBoolean("ipb_driver_dpad_down")){
			// Spin around right back wheel
			frontRightMotorSpeed = rightJs_xAxis * (fRobotLength / fDiameter);
			frontLeftMotorSpeed = rightJs_xAxis;
			backLeftMotorSpeed = rightJs_xAxis * (fRobotWidth / fDiameter);
			backRightMotorSpeed = 0;
			frontRightMotorAngle = -90;
			frontLeftMotorAngle = -45;
			backLeftMotorAngle = 0;
			backRightMotorAngle = 0;
		} else if (fSharedInputValues.getBoolean("ipb_driver_dpad_left")){
			// Spin around left back wheel
			frontRightMotorSpeed = rightJs_xAxis;
			frontLeftMotorSpeed = rightJs_xAxis * (fRobotLength / fDiameter);
			backLeftMotorSpeed = 0;
			backRightMotorSpeed = rightJs_xAxis * (fRobotWidth / fDiameter);
			frontRightMotorAngle = -135;
			frontLeftMotorAngle = -90;
			backLeftMotorAngle = 0;
			backRightMotorAngle = 180;
		}

		// Set the motors
		fSharedOutputValues.setNumeric("opn_drivetrain_front_right_speed", "percent", frontRightMotorSpeed);
		fSharedOutputValues.setNumeric("opn_drivetrain_front_left_speed", "percent", frontLeftMotorSpeed);
		fSharedOutputValues.setNumeric("opn_drivetrain_back_left_speed", "percent", backLeftMotorSpeed);
		fSharedOutputValues.setNumeric("opn_drivetrain_back_right_speed", "percent", backRightMotorSpeed);

		// set the angle
		fSharedOutputValues.setNumeric("opn_drivetrain_front_right_angle", "position", frontRightMotorAngle);
		fSharedOutputValues.setNumeric("opn_drivetrain_front_left_angle", "position", frontLeftMotorAngle);
		fSharedOutputValues.setNumeric("opn_drivetrain_back_left_angle", "position", backLeftMotorAngle);
		fSharedOutputValues.setNumeric("opn_drivetrain_back_right_angle", "position", backRightMotorAngle);
	}

	@Override
	public void dispose() {
		fSharedOutputValues.setNumeric("opn_drivetrain_front_right_speed", "percent", 0);
		fSharedOutputValues.setNumeric("opn_drivetrain_front_left_speed", "percent", 0);
		fSharedOutputValues.setNumeric("opn_drivetrain_back_left_speed", "percent", 0);
		fSharedOutputValues.setNumeric("opn_drivetrain_back_right_speed", "percent", 0);

		fSharedOutputValues.setNumeric("opn_drivetrain_front_right_angle", "percent", 0);
		fSharedOutputValues.setNumeric("opn_drivetrain_front_left_angle", "percent", 0);
		fSharedOutputValues.setNumeric("opn_drivetrain_back_left_angle", "percent", 0);
		fSharedOutputValues.setNumeric("opn_drivetrain_back_right_angle", "percent", 0);
	}

	@Override
	public boolean isDone() {
		return true;
	}

	@Override
	public Set<String> getSubsystems() {
		return sSubsystems;
	}
}