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

	private boolean mFieldOriented;

	public Behavior_Drivetrain_Swerve_Vector(InputValues inputValues, OutputValues outputValues, Config config, RobotConfiguration robotConfiguration) {
		fSharedInputValues = inputValues;
		fSharedOutputValues = outputValues;

		// Read in the location of the four swerve modules relative to the center of the robot based on standard x,y grid
		fModulePositions = new VectorList(new Vector(robotConfiguration.getList("global_drivetrain_swerve_vector", "swerve_front_right_module_position")),
				new Vector(robotConfiguration.getList("global_drivetrain_swerve_vector", "swerve_front_left_module_position")),
				new Vector(robotConfiguration.getList("global_drivetrain_swerve_vector", "swerve_back_left_module_position")),
				new Vector(robotConfiguration.getList("global_drivetrain_swerve_vector", "swerve_back_right_module_position")));

		// ??? Rotate 90 degrees as the zero on the field is 90 degrees counterclockwise from zero on a unit circle
		fModuleRotationDirections = fModulePositions.copy().normalizeAll().rotateAll(90);

		fCurrentModuleVectors = new VectorList(new Vector(), new Vector(), new Vector(), new Vector());

		fModuleInputAngleNames = Lists.of(
				"ipn_drivetrain_front_right_angle",
						"ipn_drivetrain_front_left_angle",
						"ipn_drivetrain_back_left_angle",
						"ipn_drivetrain_back_right_angle");

		fModuleInputSpeedNames = Lists.of(
				"ipn_drivetrain_front_right_speed",
						"ipn_drivetrain_front_left_speed",
						"ipn_drivetrain_back_left_speed",
						"ipn_drivetrain_back_right_speed");

		fModuleOutputAngleNames = Lists.of(
				"opn_drivetrain_front_right_angle",
						"opn_drivetrain_front_left_angle",
						"opn_drivetrain_back_left_angle",
						"opn_drivetrain_back_right_angle");

		fModuleOutputSpeedNames = Lists.of(
				"opn_drivetrain_front_right_speed",
						"opn_drivetrain_front_left_speed",
						"opn_drivetrain_back_left_speed",
						"opn_drivetrain_back_right_speed");


		fXAxis = robotConfiguration.getString("global_drivetrain_swerve_vector", "swerve_x");
		fYAxis = robotConfiguration.getString("global_drivetrain_swerve_vector", "swerve_y");
		fRotateAxis = robotConfiguration.getString("global_drivetrain_swerve_vector", "swerve_rotate");
		fFieldOrientedButton = robotConfiguration.getString("global_drivetrain_swerve_vector", "swerve_field_oriented_button");

		mStateName = "Unknown";

		mFieldOriented = true;
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
			mFieldOriented = !mFieldOriented;
		}

		double xAxis = fSharedInputValues.getNumeric(fXAxis);
		double yAxis = fSharedInputValues.getNumeric(fYAxis);
		double rotateAxis = fSharedInputValues.getNumeric(fRotateAxis);

		// If using field orientation, calculate the robot's rotation based on the Navx
		double robotOrientation = (mFieldOriented) ? (-fSharedInputValues.getVector("ipv_navx").get("angle") + 90) : 0;

		//todo - why is point passed in as y, x
		// Create a vector that represents the joysticks position. Then rotate that vector based on the robot's orientation
		Vector translation = new Vector(new Point(yAxis, xAxis)).rotate(robotOrientation);

		// Loop through each wheel module and calculate it's new speed and angle
		for(int i = 0; i < fCurrentModuleVectors.size(); i++){
			Vector current = new Vector(fCurrentModuleVectors.get(i).magnitude(), fSharedInputValues.getNumeric(fModuleInputAngleNames.get(i)));
			Vector target = new Vector(translation.add(fModuleRotationDirections.get(i).scale(rotateAxis)));

			// If the driver let's off the joysticks, rotate the wheels to the straight forward position
			if(target.magnitude() == 0.0) {
				//target = new Vector(0, fModuleRotationDirections.get(i).angle());
				target = new Vector(0, 0);
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
			fSharedOutputValues.setNumeric(fModuleOutputAngleNames.get(i), "percent", fCurrentModuleVectors.get(i).angle());
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