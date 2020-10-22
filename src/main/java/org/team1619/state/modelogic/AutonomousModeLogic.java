package org.team1619.state.modelogic;

import org.uacr.models.state.State;
import org.uacr.robot.AbstractModeLogic;
import org.uacr.shared.abstractions.InputValues;
import org.uacr.shared.abstractions.RobotConfiguration;
import org.uacr.utilities.logging.LogManager;
import org.uacr.utilities.logging.Logger;

/**
 * Handles the isReady and isDone logic for autonomous mode on competition bot
 */

public class AutonomousModeLogic extends AbstractModeLogic {

	private static final Logger sLogger = LogManager.getLogger(AutonomousModeLogic.class);

	private String mAutoOrigin;
	private String mAutoDestination;
	private String mAutoAction;
	private String mCombinedAuto;

	public AutonomousModeLogic(InputValues inputValues, RobotConfiguration robotConfiguration) {
		super(inputValues, robotConfiguration);

		mAutoOrigin = "none";
		mAutoDestination = "none";
		mAutoAction = "none";
		mCombinedAuto = "none";
	}

	@Override
	public void initialize() {
		sLogger.info("***** AUTONOMOUS *****");

		//Reads the values selected on the webdashboard and compiles them into the name of an auto.
		mAutoOrigin = (fSharedInputValues.getString("ips_auto_origin").toLowerCase().replaceAll("\\s", ""));
		mAutoDestination = (fSharedInputValues.getString("ips_auto_destination").toLowerCase().replaceAll("\\s", ""));
		mAutoAction = (fSharedInputValues.getString("ips_auto_action").toLowerCase().replaceAll("\\s", ""));
		mCombinedAuto = "sq_auto_" + mAutoOrigin + "_" + mAutoDestination + "_" + mAutoAction;
		if (mAutoOrigin.equals("doesnotexist") || mAutoDestination.equals("doesnotexist") || mAutoAction.equals("doesnotexist")) {
			mCombinedAuto = "sq_auto_anywhere_3ball_none";
		}
		sLogger.debug(mCombinedAuto);

		fSharedInputValues.setBoolean("ipb_auto_complete", false);
	}

	@Override
	public void update() {

	}

	@Override
	public void dispose() {

	}

	@Override
	public boolean isReady(String name) {
		switch (name) {

		}

		// Check isReady on auto states
		// This reads the string assembled from the webdashboard and checks it against all possible autos until it finds a match
		// If it doesn't find a match it does nothing
		if (!fSharedInputValues.getBoolean("ipb_auto_complete") && fSharedInputValues.getBoolean("ipb_robot_has_been_zeroed")) {
			return name.equals(mCombinedAuto);
		}

		return false;
	}

	@Override
	public boolean isDone(String name, State state) {
		switch (name) {

		}

		//Checks the isDone on zero states and determines when autonomous is done
		if (state.isDone()) {
			if (name.contains("auto")) {
				fSharedInputValues.setBoolean("ipb_auto_complete", true);
			}
			return true;
		}
		return false;
	}
}
