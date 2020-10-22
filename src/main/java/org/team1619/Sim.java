package org.team1619;

import org.team1619.services.logging.LoggingService;
import org.team1619.state.SimModule;
import org.uacr.services.input.InputService;
import org.uacr.services.output.OutputService;
import org.uacr.services.states.StatesService;
import org.uacr.services.webdashboard.WebDashboardService;
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

public class Sim {

	private static final Logger sLogger = LogManager.getLogger(Sim.class);

	public static void main(String[] args) {

		System.setProperty("logPath", "logs");

		YamlConfigParser parser = new YamlConfigParser();
		parser.load("general.yaml");

		Config loggerConfig = parser.getConfig("logger");
		if (loggerConfig.contains("log_level")) {
			LogManager.setLogLevel(loggerConfig.getEnum("log_level", LogManager.Level.class));
		}

		Injector injector = new Injector(new SimModule());
		injector.getInstance(SharedRobotConfiguration.class).initialize();

		StatesService statesService = injector.getInstance(StatesService.class);
		InputService inputService = injector.getInstance(InputService.class);
		OutputService outputService = injector.getInstance(OutputService.class);
		LoggingService loggingService = injector.getInstance(LoggingService.class);
		WebDashboardService webDashboardService = injector.getInstance(WebDashboardService.class);

		ScheduledMultiService coreService = new ScheduledMultiService(new Scheduler(10), inputService, statesService, outputService);
		ScheduledMultiService infoService = new ScheduledMultiService(new Scheduler(30), loggingService, webDashboardService);

		ServiceManager serviceManager = new AsyncServiceManager(coreService, infoService);

		sLogger.info("Starting services");
		serviceManager.start();
		serviceManager.awaitHealthy();
		sLogger.info("********************* ALL SERVICES STARTED *******************************");
		serviceManager.awaitStopped();
		sLogger.info("All Services stopped");
	}
}
