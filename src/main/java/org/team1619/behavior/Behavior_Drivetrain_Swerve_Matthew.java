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
 * Drives the robot in swerve mode, based on the joystick values.
 */

public class Behavior_Drivetrain_Swerve_Matthew implements Behavior {

	private static final Logger sLogger = LogManager.getLogger(Behavior_Drivetrain_Swerve_Vector.class);
	private static final Set<String> sSubsystems = Set.of("ss_drivetrain");

	private final InputValues fSharedInputValues;
	private final OutputValues fSharedOutputValues;

	private final VectorList fModulePositions;
	private final VectorList fModuleRotationDirections;
	private final VectorList fCurrentModuleVectors;

	private final List<String> fModuleAngleInputNames;

	private final String fXAxis;
	private final String fYAxis;
	private final String fRotateAxis;
	private final String fFieldOrientedButton;

	private String mStateName;

	public Behavior_Drivetrain_Swerve_Matthew(InputValues inputValues, OutputValues outputValues, Config config, RobotConfiguration robotConfiguration) {
		fSharedInputValues = inputValues;
		fSharedOutputValues = outputValues;

		// Read in the location of the four swerve modules relative to the center of the robot based on standard x,y grid with the front facing 0 on the unit circle
		fModulePositions = new VectorList(new Vector(robotConfiguration.getList("global_drivetrain_Matthew", "swerve_front_right_module_position")),
				new Vector(robotConfiguration.getList("global_drivetrain_Matthew", "swerve_front_left_module_position")),
				new Vector(robotConfiguration.getList("global_drivetrain_Matthew", "swerve_back_left_module_position")),
				new Vector(robotConfiguration.getList("global_drivetrain_Matthew", "swerve_back_right_module_position")));

		// Create a set of vectors at right angles to the corners of the robot to use to calculate rotation vectors
		fModuleRotationDirections = fModulePositions.copy().normalizeAll().rotateAll(90);

		fCurrentModuleVectors = new VectorList(new Vector(), new Vector(), new Vector(), new Vector());

		fModuleAngleInputNames = Lists.of("ipn_drivetrain_front_right_angle", "ipn_drivetrain_front_left_angle",
				"ipn_drivetrain_back_left_angle", "ipn_drivetrain_back_right_angle");

		fXAxis = robotConfiguration.getString("global_drivetrain_Matthew", "swerve_x");
		fYAxis = robotConfiguration.getString("global_drivetrain_Matthew", "swerve_y");
		fRotateAxis = robotConfiguration.getString("global_drivetrain_Matthew", "swerve_rotate");
		fFieldOrientedButton = robotConfiguration.getString("global_drivetrain_Matthew", "swerve_field_oriented_button");

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
		if(fSharedInputValues.getBoolean("ipb_driver_a")) {
			Vector centerOfRotation = new Vector(new Point(60.0, 0.0));
			double rotationSpeed = 1.0;
			Vector translation = new Vector(new Point(0.0, 0.0));

			VectorList moduleRotationVectors = fModulePositions.copy().subtractAll(centerOfRotation).rotateAll(90).autoScaleAll(VectorList.AutoScaleMode.SCALE_LARGEST_UP_OR_DOWN, 1.0);;

			calculateModuleVectors(translation, moduleRotationVectors, rotationSpeed);
		} else {
			if (fSharedInputValues.getBooleanRisingEdge(fFieldOrientedButton)) {
				fSharedInputValues.setBoolean("ipb_swerve_field_centric", !fSharedInputValues.getBoolean("ipb_swerve_field_centric"));
			}

			double xAxis = fSharedInputValues.getNumeric(fXAxis);
			double yAxis = fSharedInputValues.getNumeric(fYAxis);
			double rotateAxis = fSharedInputValues.getNumeric(fRotateAxis);

			// This is the orientation of the front of the robot based on the unit circle. It does not have to be 0.
			double robotOrientation = 0;

			// When using field orientation, forward is always towards the opposite end of the field even if the robot is facing a different direction.
			// To do this, the angle of the robot read from the navx is subtracted from the direction chosen by the driver.
			// For example, if the robot is rotated 15 degrees and the driver chooses straight forward, the actual angle is -15 degrees.
			if (fSharedInputValues.getBoolean("ipb_swerve_field_centric")) {
				robotOrientation += -fSharedInputValues.getVector("ipv_navx").get("angle");
			}

			// Add 90 degrees to point up on the simulation page so it makes more sense.
			// This will be removed on the robot.
			// robotOrientation += 90;

			// Swapping X and Y translates coordinate systems from the controller to the robot.
			// The controller use the Y axis for forward/backwards and the X axis for right/left
			// The robot forward/backwards is along the X axis and left/right is along the Y axis
			Vector translation = new Vector(new Point(yAxis, xAxis)).rotate(robotOrientation);

			calculateModuleVectors(translation, fModuleRotationDirections, rotateAxis);
		}

		setMotorPowers(fCurrentModuleVectors);
	}

	@Override
	public void dispose() {
		sLogger.trace("Leaving state {}", mStateName);

		fCurrentModuleVectors.replaceAll(v -> new Vector());

		setMotorPowers(fCurrentModuleVectors);
	}

	@Override
	public boolean isDone() {
		return true;
	}

	@Override
	public Set<String> getSubsystems() {
		return sSubsystems;
	}

	private void calculateModuleVectors(Vector translation, VectorList moduleRotationDirections, double rotationSpeed) {
		fCurrentModuleVectors.replaceAll(v -> {
			int i = fCurrentModuleVectors.indexOf(v);
			return calculateModuleVector(fCurrentModuleVectors.get(i), fModuleAngleInputNames.get(i), translation, moduleRotationDirections.get(i), rotationSpeed);
		});
	}

	private Vector calculateModuleVector(Vector last, String currentAngleInput, Vector translation, Vector rotationDirection, Double rotationScalar) {
		Vector current = new Vector(last.magnitude(), fSharedInputValues.getNumeric(currentAngleInput));
		Vector target = new Vector(translation.add(rotationDirection.scale(rotationScalar)));

		if(target.magnitude() == 0.0) {
			// When the joysticks are idle, move the wheel angles to their rotation angle so the robot can spin instantly and move in any direction as quickly as possible.
			target = new Vector(0, rotationDirection.angle());
		}

		// If the difference between the target angle and the actual angle is more than 90 degrees, rotate 180 degrees and reverse the motor direction.
		// Ramp up the wheel velocity as the actual angle get closer to the target angle. This prevents the robot from being pulled off course.
		// The cosine is raised to the power of 3 so that the ramp increases faster as the delta in the angle approaches zero.
		double directionScalar = Math.pow(Math.cos(Math.toRadians(target.angle() - current.angle())), 3);

		if (directionScalar < 0) {
			target = target.rotate(180);
		}

		return target.scale(directionScalar);
	}

	private void setMotorPowers(VectorList moduleVectors) {
		moduleVectors.autoScaleAll(VectorList.AutoScaleMode.SCALE_LARGEST_DOWN, 1.0);

		fSharedOutputValues.setNumeric("opn_drivetrain_front_right_speed", "percent", moduleVectors.get(0).magnitude());
		//todo - should this be position and have a profile?
		fSharedOutputValues.setNumeric("opn_drivetrain_front_right_angle", "percent", moduleVectors.get(0).angle());
		fSharedOutputValues.setNumeric("opn_drivetrain_front_left_speed", "percent", moduleVectors.get(1).magnitude());
		fSharedOutputValues.setNumeric("opn_drivetrain_front_left_angle", "percent", moduleVectors.get(1).angle());
		fSharedOutputValues.setNumeric("opn_drivetrain_back_left_speed", "percent", moduleVectors.get(2).magnitude());
		fSharedOutputValues.setNumeric("opn_drivetrain_back_left_angle", "percent", moduleVectors.get(2).angle());
		fSharedOutputValues.setNumeric("opn_drivetrain_back_right_speed", "percent", moduleVectors.get(3).magnitude());
		fSharedOutputValues.setNumeric("opn_drivetrain_back_right_angle", "percent", moduleVectors.get(3).angle());
		fSharedInputValues.setNumeric("ipn_drivetrain_front_right_speed", moduleVectors.get(0).magnitude());
		fSharedInputValues.setNumeric("ipn_drivetrain_front_left_speed", moduleVectors.get(1).magnitude());
		fSharedInputValues.setNumeric("ipn_drivetrain_back_left_speed", moduleVectors.get(2).magnitude());
		fSharedInputValues.setNumeric("ipn_drivetrain_back_right_speed", moduleVectors.get(3).magnitude());
	}
}