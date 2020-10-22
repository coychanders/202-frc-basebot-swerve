package org.team1619;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import org.team1619.services.logging.LoggingService;
import org.team1619.state.RobotModule;
import org.uacr.services.input.InputService;
import org.uacr.services.output.OutputService;
import org.uacr.services.states.StatesService;
import org.uacr.services.webdashboard.WebDashboardService;
import org.uacr.shared.abstractions.FMS;
import org.uacr.shared.concretions.SharedRobotConfiguration;
import org.uacr.utilities.Config;
import org.uacr.utilities.YamlConfigParser;
import org.uacr.utilities.injection.Injector;
import org.uacr.utilities.logging.LogManager;
import org.uacr.utilities.logging.Logger;
import org.uacr.utilities.services.ScheduledMultiService;
import org.uacr.utilities.services.Scheduler;
import org.uacr.utilities.services.managers.AsyncServiceManager;
import org.uacr.utilities.services.managers.ServiceManager;

public class Robot extends TimedRobot {

	private static final Logger sLogger = LogManager.getLogger(Robot.class);

	private final Injector fInjector;
	private final ServiceManager fServiceManager;
	private final InputService fInputService;
	private final FMS fFMS;

	public Robot() {

		YamlConfigParser parser = new YamlConfigParser();
		parser.load("general.yaml");

		Config loggerConfig = parser.getConfig("logger");
		if (loggerConfig.contains("log_level")) {
			LogManager.setLogLevel(loggerConfig.getEnum("log_level", LogManager.Level.class));
		}

		fInjector = new Injector(new RobotModule());
		fInjector.getInstance(SharedRobotConfiguration.class).initialize();

		StatesService statesService = fInjector.getInstance(StatesService.class);
		fInputService = fInjector.getInstance(InputService.class);
		OutputService outputService = fInjector.getInstance(OutputService.class);
		LoggingService loggingService = fInjector.getInstance(LoggingService.class);
		// TODO comment out to turn off webdashboard service
		WebDashboardService webDashboardService = fInjector.getInstance(WebDashboardService.class);

		ScheduledMultiService coreService = new ScheduledMultiService(new Scheduler(10), fInputService, statesService, outputService);
		ScheduledMultiService infoService = new ScheduledMultiService(new Scheduler(30), loggingService, webDashboardService);

		fServiceManager = new AsyncServiceManager(coreService, infoService);

		// TODO comment in to when turning off webdashboard service
		//fServiceManager = new ScheduledLinearServiceManager(new Scheduler(30), Set.of(statesService, fInputService, outputService, loggingService));

		fFMS = fInjector.getInstance(FMS.class);
	}

	public static void main(String... args) {
		System.setProperty("logPath", "/home/lvuser/logs");
		RobotBase.startRobot(Robot::new);
	}

	@Override
	public void robotInit() {
		sLogger.info("Starting services");
		fServiceManager.start();
		fServiceManager.awaitHealthy();

		sLogger.info("********************* ALL SERVICES STARTED *******************************");
	}

	@Override
	public void teleopInit() {
		fFMS.setMode(FMS.Mode.TELEOP);
	}

	@Override
	public void autonomousInit() {
		fFMS.setMode(FMS.Mode.AUTONOMOUS);
	}

	@Override
	public void disabledInit() {
		fFMS.setMode(FMS.Mode.DISABLED);
	}

	@Override
	public void testInit() {
		fFMS.setMode(FMS.Mode.TEST);
	}
}