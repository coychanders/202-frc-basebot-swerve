package org.team1619.state.modelogic;

import org.uacr.models.state.State;
import org.uacr.robot.AbstractModeLogic;
import org.uacr.shared.abstractions.InputValues;
import org.uacr.shared.abstractions.RobotConfiguration;
import org.uacr.utilities.logging.LogManager;
import org.uacr.utilities.logging.Logger;

import java.util.ArrayList;
import java.util.List;

/**
 * Handles the isReady and isDone logic for teleop mode on competition bot
 */

public class TeleopModeLogic extends AbstractModeLogic {

	private static final Logger sLogger = LogManager.getLogger(TeleopModeLogic.class);

	private int mMode;

	public TeleopModeLogic(InputValues inputValues, RobotConfiguration robotConfiguration) {
		super(inputValues, robotConfiguration);
	}

	@Override
	public void initialize() {
		sLogger.info("***** TELEOP *****");
		fSharedInputValues.setBoolean("ipb_swerve_vector_mode", true);
		mMode = 0;
	}

	@Override
	public void update() {
		if(fSharedInputValues.getBooleanFallingEdge("ipb_driver_back")){
			mMode = (mMode < 2) ? (mMode + 1) : 0;
		}
	}

	@Override
	public void dispose() {

	}

	@Override
	public boolean isReady(String name) {
		switch (name) {
			case "st_drivetrain_zero":
				return !fSharedInputValues.getBoolean("ipb_drivetrain_has_been_zeroed");
			case "st_drivetrain_swerve_vector":
				return mMode == 0;
			case "st_drivetrain_swerve_math":
				return mMode == 1;
			case "st_drivetrain_swerve_matthew":
				return mMode == 2;

			default:
				return false;
		}
	}

	@Override
	public boolean isDone(String name, State state) {
		switch (name) {
			case "st_drivetrain_swerve_vector":
				return !fSharedInputValues.getBoolean("ipb_swerve_vector_mode");
			default:
				return state.isDone();
		}
	}
}
