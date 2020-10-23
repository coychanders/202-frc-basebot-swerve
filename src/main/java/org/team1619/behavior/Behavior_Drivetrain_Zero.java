package org.team1619.behavior;

import org.uacr.models.behavior.Behavior;
import org.uacr.shared.abstractions.InputValues;
import org.uacr.shared.abstractions.OutputValues;
import org.uacr.shared.abstractions.RobotConfiguration;
import org.uacr.utilities.Config;
import org.uacr.utilities.Timer;
import org.uacr.utilities.logging.LogManager;
import org.uacr.utilities.logging.Logger;

import java.util.*;

/**
 * Zeros the swerve modules
 */

public class Behavior_Drivetrain_Zero implements Behavior {

	private static final Logger sLogger = LogManager.getLogger(Behavior_Drivetrain_Zero.class);
	private static final Set<String> sSubsystems = Set.of("ss_drivetrain");

	private final InputValues fSharedInputValues;
	private final OutputValues fSharedOutputValues;

	private final Timer fTimeoutTimer;
	private int mTimeoutTime;
	private double mZeroingThreshold;

	public Behavior_Drivetrain_Zero(InputValues inputValues, OutputValues outputValues, Config config, RobotConfiguration robotConfiguration) {
		fSharedInputValues = inputValues;
		fSharedOutputValues = outputValues;

		fTimeoutTimer = new Timer();
		mTimeoutTime = 500;
		mZeroingThreshold = 0.1;
	}

	@Override
	public void initialize(String stateName, Config config) {
		sLogger.debug("Entering state {}", stateName);
		mTimeoutTime = config.getInt("timeout_time");
		mZeroingThreshold = config.getDouble("zeroing_threshold");

		fTimeoutTimer.reset();
		fTimeoutTimer.start(mTimeoutTime);

		fSharedOutputValues.setNumeric("opn_drivetrain_front_right_speed", "percent", 0);
		fSharedOutputValues.setNumeric("opn_drivetrain_front_left_speed", "percent", 0);
		fSharedOutputValues.setNumeric("opn_drivetrain_back_right_speed", "percent", 0);
		fSharedOutputValues.setNumeric("opn_drivetrain_back_left_speed", "percent", 0);

		fSharedOutputValues.setNumeric("opn_drivetrain_front_right_angle", "percent", 0);
		fSharedOutputValues.setNumeric("opn_drivetrain_front_left_angle", "percent", 0);
		fSharedOutputValues.setNumeric("opn_drivetrain_back_right_angle", "percent", 0);
		fSharedOutputValues.setNumeric("opn_drivetrain_back_left_angle", "percent", 0);
	}

	@Override
	public void update() {
		if (!fSharedInputValues.getBoolean("ipb_drivetrain_has_been_zeroed")) {

			//Todo - Should we do this every frame or move them into Init()?
			fSharedOutputValues.setOutputFlag("opn_drivetrain_front_right_angle", "zero");
			fSharedOutputValues.setOutputFlag("opn_drivetrain_front_left_angle", "zero");
			fSharedOutputValues.setOutputFlag("opn_drivetrain_back_right_angle", "zero");
			fSharedOutputValues.setOutputFlag("opn_drivetrain_back_left_angle", "zero");

			if ((Math.abs(fSharedInputValues.getNumeric("ipn_drivetrain_front_right_angle_position")) < mZeroingThreshold)
				&& (Math.abs(fSharedInputValues.getNumeric("ipn_drivetrain_front_left_angle_position")) < mZeroingThreshold)
				&& (Math.abs(fSharedInputValues.getNumeric("ipn_drivetrain_back_left_angle_position")) < mZeroingThreshold)
				&& (Math.abs(fSharedInputValues.getNumeric("ipn_drivetrain_back_left_angle_position")) < mZeroingThreshold))
			{
				sLogger.debug("Drivetrain Zero -> Zeroed");
				//Todo - Is this correct? What is it for? Why is it false?
				//fSharedInputValues.setBoolean("ipb_odometry_has_been_zeroed", false);
				fSharedInputValues.setBoolean("ipb_drivetrain_has_been_zeroed", true);
				return;
			}

			if (fTimeoutTimer.isDone()) {
				sLogger.error("Drivetrain Zero -> Timed Out");
				fTimeoutTimer.reset();
				fSharedInputValues.setBoolean("ipb_drivetrain_has_been_zeroed", true);
			}
		}
	}

	@Override
	public void dispose() {
		//Todo - Do we need this as they are already set this way?
		fSharedOutputValues.setNumeric("opn_drivetrain_front_right_speed", "percent", 0);
		fSharedOutputValues.setNumeric("opn_drivetrain_front_left_speed", "percent", 0);
		fSharedOutputValues.setNumeric("opn_drivetrain_back_right_speed", "percent", 0);
		fSharedOutputValues.setNumeric("opn_drivetrain_back_left_speed", "percent", 0);

		fSharedOutputValues.setNumeric("opn_drivetrain_front_right_angle", "percent", 0);
		fSharedOutputValues.setNumeric("opn_drivetrain_front_left_angle", "percent", 0);
		fSharedOutputValues.setNumeric("opn_drivetrain_back_right_angle", "percent", 0);
		fSharedOutputValues.setNumeric("opn_drivetrain_back_left_angle", "percent", 0);
	}

	@Override
	public boolean isDone() {
		return fSharedInputValues.getBoolean("ipb_drivetrain_has_been_zeroed") || fTimeoutTimer.isDone();
	}

	@Override
	public Set<String> getSubsystems() {
		return sSubsystems;
	}
}