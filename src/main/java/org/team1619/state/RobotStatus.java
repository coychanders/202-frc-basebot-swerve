package org.team1619.state;

import org.uacr.robot.AbstractRobotStatus;
import org.uacr.shared.abstractions.InputValues;
import org.uacr.shared.abstractions.RobotConfiguration;
import org.uacr.utilities.LimitedSizeQueue;
import org.uacr.utilities.logging.LogManager;
import org.uacr.utilities.logging.Logger;

import java.util.Map;
import java.util.Queue;

/**
 * Sets flags and does global math and logic for competition bot
 */

public class RobotStatus extends AbstractRobotStatus {

	private static final Logger sLogger = LogManager.getLogger(RobotStatus.class);

	public RobotStatus(InputValues inputValues, RobotConfiguration robotConfiguration) {
		super(inputValues, robotConfiguration);
	}

	@Override
	public void initialize() {
		// Zero
		if (!fSharedInputValues.getBoolean("ipb_robot_has_been_zeroed")) {
//			fSharedInputValues.setBoolean("ipb_example_has_been_zeroed", false);
		}
	}

	@Override
	public void update() {
		if (!fSharedInputValues.getBoolean("ipb_robot_has_been_zeroed")
				/*&& fSharedInputValues.getBoolean("ipb_example_has_been_zeroed")*/) {
			fSharedInputValues.setBoolean("ipb_robot_has_been_zeroed", true);
		}
	}

	@Override
	public void dispose() {

	}
}
