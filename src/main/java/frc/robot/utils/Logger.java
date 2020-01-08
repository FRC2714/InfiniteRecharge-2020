package frc.robot.utils;

import edu.wpi.first.wpilibj2.command.Subsystem;

import java.io.File;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class Logger {
    private Map<Subsystem, CSVWriter> subsystemLogs;

    private static Logger logger = null;
    private static final String ROOT = "/home/admin/logging/";

    public static Logger getInstance() {
        if (logger == null) return new Logger();
        return logger;
    }

    public Logger() {
        subsystemLogs = new HashMap<>();
    }

    public void addSubsystem(Subsystem s) {
        subsystemLogs.putIfAbsent(s, new CSVWriter(new File(ROOT+s.getClass().getSimpleName() + ".csv")));
    }

    public void putData(Subsystem subsystem, Map<String, Object> data) {
        CSVWriter log = subsystemLogs.get(subsystem);
        log.putAll(data);
        log.write();
        // log.flush(); NOT NECESSARY
    }

    public void putData(String subsystemStr, Map<String, Object> data) {
        for (Subsystem s : subsystemLogs.keySet()) {
            if (s.getClass().getSimpleName().equals(subsystemStr)) {
                putData(s, data);
                return;
            }
        }
    }

}