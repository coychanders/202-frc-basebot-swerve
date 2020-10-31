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
		fModulePositions = new VectorList(new Vector(robotConfiguration.getList("global_drivetrain_swerve_vector", "swerve_front_right_module_position")),
				new Vector(robotConfiguration.getList("global_drivetrain_swerve_vector", "swerve_front_left_module_position")),
				new Vector(robotConfiguration.getList("global_drivetrain_swerve_vector", "swerve_back_left_module_position")),
				new Vector(robotConfiguration.getList("global_drivetrain_swerve_vector", "swerve_back_right_module_position")));

		// Rotate module positions clockwise 90 degrees to represent polar coordinance (0 degrees is to the right) instead of field coordinance (0 degrees is straight forward)
//		fModulePositions.rotateAll(-90);

		// ??? Rotate 90 degrees as the zero on the field is 90 degrees counterclockwise from zero on a unit circle
//		fModuleRotationDirections = fModulePositions.copy().normalizeAll().rotateAll(90);
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

		// If using field orientation, calculate the robot's rotation based on the Navx
		double robotOrientation = (fSharedInputValues.getBoolean("ipb_swerve_field_centric")) ? (fSharedInputValues.getVector("ipv_navx").get("angle") - 90) : 0;

		// Create a vector that represents the joysticks position. Then rotate that vector based on the robot's orientation
		Vector translation = new Vector(new Point(xAxis, yAxis)).rotate(robotOrientation);

		// Loop through each wheel module and calculate it's new speed and angle
		for(int i = 0; i < fCurrentModuleVectors.size(); i++){
			Vector current = new Vector(fCurrentModuleVectors.get(i).magnitude(), fSharedInputValues.getNumeric(fModuleInputAngleNames.get(i)));
			Vector target = new Vector(translation.add(fModuleRotationDirections.get(i).scale(rotateAxis)));

			// If the driver let's off the joysticks, rotate the wheels to the straight forward position
			if(target.magnitude() == 0.0) {
				target = new Vector(0, fModuleRotationDirections.get(i).angle());
				//target = new Vector(0, 0);
			}

			//todo why 3
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
//			fSharedOutputValues.setNumeric(fModuleOutputAngleNames.get(i), "percent", 0);
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