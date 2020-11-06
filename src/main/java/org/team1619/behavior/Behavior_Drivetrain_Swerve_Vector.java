package org.team1619.behavior;

import org.uacr.models.behavior.Behavior;
import org.uacr.shared.abstractions.InputValues;
import org.uacr.shared.abstractions.OutputValues;
import org.uacr.shared.abstractions.RobotConfiguration;
import org.uacr.utilities.Config;
import org.uacr.utilities.Lists;
import org.uacr.utilities.logging.LogManager;
import org.uacr.utilities.logging.Logger;
import org.uacr.utilities.purepursuit.Point;
import org.uacr.utilities.purepursuit.Vector;
import org.uacr.utilities.purepursuit.VectorList;

import java.util.ArrayList;
import java.util.List;
import java.util.Set;


/**
 * Drives the robot in percent mode, based on the joystick values.
 */

public class Behavior_Drivetrain_Swerve_Vector implements Behavior {

	private static final Logger sLogger = LogManager.getLogger(Behavior_Drivetrain_Swerve_Vector.class);
	private static final Set<String> sSubsystems = Set.of("ss_drivetrain");

	private final InputValues fSharedInputValues;
	private final OutputValues fSharedOutputValues;

	private final VectorList fModulePositions;
	private final VectorList fModuleRotationDirections;
	private final VectorList fCurrentModuleVectors;

	private final List<String> fModuleInputAngleNames;
	private final List<String> fModuleInputSpeedNames;
	private final List<String> fModuleOutputAngleNames;
	private final List<String> fModuleOutputSpeedNames;

	private final String fXAxis;
	private final String fYAxis;
	private final String fRotateAxis;
	private final String fFieldOrientedButton;

	private String mStateName;


	public Behavior_Drivetrain_Swerve_Vector(InputValues inputValues, OutputValues outputValues, Config config, RobotConfiguration robotConfiguration) {
		fSharedInputValues = inputValues;
		fSharedOutputValues = outputValues;

		// Read in the location of the four swerve modules relative to the center of the robot based on standard x,y grid
		List<List<Double>> modulePositionsList = robotConfiguration.getList("global_drivetrain_swerve_vector", "module_positions");
		fModulePositions = new VectorList();
		for(List<Double> d : modulePositionsList){
			fModulePositions.add(new Vector(d));
		}

		// Create a set of vectors to use in calculating rotation by rotating each module vector by 90 degrees
		fModuleRotationDirections = fModulePositions.copy().normalizeAll().rotateAll(90);

		fCurrentModuleVectors = new VectorList(new Vector(), new Vector(), new Vector(), new Vector());

		// Read in motor input and output names to be used in loops below
		fModuleInputAngleNames  = robotConfiguration.getList("global_drivetrain_swerve_vector", "input_angle_names");
		fModuleInputSpeedNames  = robotConfiguration.getList("global_drivetrain_swerve_vector", "input_speed_names");
		fModuleOutputAngleNames = robotConfiguration.getList("global_drivetrain_swerve_vector", "output_angle_names");
		fModuleOutputSpeedNames = robotConfiguration.getList("global_drivetrain_swerve_vector", "output_speed_names");

		// Read in the names of the xbox controls used
		fXAxis = robotConfiguration.getString("global_drivetrain_swerve_vector", "swerve_x");
		fYAxis = robotConfiguration.getString("global_drivetrain_swerve_vector", "swerve_y");
		fRotateAxis = robotConfiguration.getString("global_drivetrain_swerve_vector", "swerve_rotate");
		fFieldOrientedButton = robotConfiguration.getString("global_drivetrain_swerve_vector", "swerve_field_oriented_button");

		mStateName = "Unknown";

		fSharedInputValues.setBoolean("ipb_swerve_field_centric", true);
	}

	@Override
	public void initialize(String stateName, Config config) {
		sLogger.debug("Entering state {}", stateName);

		fCurrentModuleVectors.replaceAll(v -> new Vector());

		mStateName = stateName;
	}

	@Override
	public void update() {
		if (fSharedInputValues.getBooleanRisingEdge(fFieldOrientedButton)) {
			fSharedInputValues.setBoolean("ipb_swerve_field_centric", !fSharedInputValues.getBoolean("ipb_swerve_field_centric"));
		}

		double xAxis = fSharedInputValues.getNumeric(fXAxis);
		double yAxis = fSharedInputValues.getNumeric(fYAxis);
		double rotateAxis = fSharedInputValues.getNumeric(fRotateAxis);

		// This is the orientation of the front of the robot based on the unit circle. It does not have to be 0.
		double robotOrientation = 0;

		if(fSharedInputValues.getBoolean("ipb_swerve_field_centric")) {
			// When using field orientation, forward is always towards the opposite end of the field even if the robot is facing a different direction.
			// To do this, the angle of the robot read from the navx is subtracted from the direction chosen by the driver.
			// For example, if the robot is rotated 15 degrees and the driver chooses straight forward, the actual angle is -15 degrees.
			robotOrientation += -fSharedInputValues.getVector("ipv_navx").get("angle");

			// Add 90 degrees to point up on the simulation page so it makes more sense.
			// This will be removed on the robot.
			// robotOrientation += 90;
		}

		// Swapping X and Y translates coordinate systems from the controller to the robot.
		// The controller use the Y axis for forward/backwards and the X axis for right/left
		// The robot forward/backwards is along the X axis and left/right is along the Y axis
		Vector translation = new Vector(new Point(yAxis, xAxis)).rotate(robotOrientation);

		// Loop through each wheel module and calculate it's new speed and angle
		for(int i = 0; i < fCurrentModuleVectors.size(); i++){
			// The current vector uses the actual wheel angle read in from the encoders
			Vector current = new Vector(fCurrentModuleVectors.get(i).magnitude(), fSharedInputValues.getNumeric(fModuleInputAngleNames.get(i)));
			// The target vector is the combination of the direction vector and the rotation vector
			Vector target = new Vector(translation.add(fModuleRotationDirections.get(i).scale(rotateAxis)));

			// When the joysticks are idle, move the wheel angles to their rotation angle so the robot can spin instantly and move in any direction as quickly as possible.
			if(target.magnitude() == 0.0) {
				target = new Vector(0, fModuleRotationDirections.get(i).angle());
			}

			// If the difference between the target angle and the actual angle is more than 90 degrees, rotate 180 degrees and reverse the motor direction.
			// Ramp up the wheel velocity as the actual angle get closer to the target angle. This prevents the robot from being pulled off course.
			// The cosine is raised to the power of 3 so that the ramp increases faster as the delta in the angle approaches zero.
			double directionScalar = Math.pow(Math.cos(Math.toRadians(target.angle() - current.angle())), 3);

			if (directionScalar < 0) {
				target = target.rotate(180);
			}

			// Update the vector to be sent to the motors
			fCurrentModuleVectors.set(i,target.scale(directionScalar));
		}

		// Scale all motor speed values so that the largest motor value does not exceed 1
		fCurrentModuleVectors.autoScaleAll(VectorList.AutoScaleMode.SCALE_LARGEST_DOWN, 1.0);

		// Output values to motors
		for(int i = 0; i < fCurrentModuleVectors.size(); i++) {
			//todo- profile?
			fSharedOutputValues.setNumeric(fModuleOutputAngleNames.get(i), "position", fCurrentModuleVectors.get(i).angle());
			fSharedOutputValues.setNumeric(fModuleOutputSpeedNames.get(i), "percent", fCurrentModuleVectors.get(i).magnitude());
			fSharedInputValues.setNumeric(fModuleInputSpeedNames.get(i), fCurrentModuleVectors.get(1).magnitude());
		}
	}

	@Override
	public void dispose() {
		sLogger.trace("Leaving state {}", mStateName);

		// Turn drive motors off. Leave wheels in their current positions
		for(int i = 0; i < fCurrentModuleVectors.size(); i++) {
			//todo - position and profile?
			fSharedOutputValues.setNumeric(fModuleOutputAngleNames.get(i), "percent", 0);
			fSharedOutputValues.setNumeric(fModuleOutputSpeedNames.get(i), "percent", 0);
			fSharedInputValues.setNumeric(fModuleInputSpeedNames.get(i), 0);
		}
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