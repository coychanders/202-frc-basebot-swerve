package org.team1619.behavior;

import org.uacr.models.behavior.Behavior;
import org.uacr.shared.abstractions.InputValues;
import org.uacr.shared.abstractions.OutputValues;
import org.uacr.shared.abstractions.RobotConfiguration;
import org.uacr.utilities.Config;
import org.uacr.utilities.logging.LogManager;
import org.uacr.utilities.logging.Logger;
import org.uacr.utilities.Timer;

import java.util.Set;

/**
 * Example behavior to copy for other behaviors
 */

public class Behavior_Example implements Behavior {

	private static final Logger sLogger = LogManager.getLogger(Behavior_Example.class);
	private static final Set<String> sSubsystems = Set.of("ss_example");

	private final InputValues fSharedInputValues;
	private final OutputValues fSharedOutputValues;
	private final String fWhatThisButtonDoes;
	private Timer mTimer;

	private int mConfigurationValue;

	public Behavior_Example(InputValues inputValues, OutputValues outputValues, Config config, RobotConfiguration robotConfiguration) {
		fSharedInputValues = inputValues;
		fSharedOutputValues = outputValues;
		fWhatThisButtonDoes = robotConfiguration.getString("global_example", "what_this_button_does");

		mConfigurationValue = 0;
		mTimer = new Timer();
	}

	@Override
	public void initialize(String stateName, Config config) {
		sLogger.debug("Entering state {}", stateName);

		mConfigurationValue = config.getInt("config_key", 0);
		mTimer.start(mConfigurationValue);
	}

	@Override
	public void update() {
		boolean whatThisButtonDoes = fSharedInputValues.getBoolean(fWhatThisButtonDoes);
		fSharedInputValues.setBoolean("opb_example", whatThisButtonDoes);
	}

	@Override
	public void dispose() {

	}

	@Override
	public boolean isDone() {
		return mTimer.isDone();
	}

	@Override
	public Set<String> getSubsystems() {
		return sSubsystems;
	}
}