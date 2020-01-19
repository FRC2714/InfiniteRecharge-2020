package frc.robot.utils;

import java.io.File;
import java.util.HashMap;
import java.util.Map;

public class Logger<T> {
    private Map<T, CSVWriter> logs;

    private static Logger logger = null;
    private static final String ROOT = "/home/admin/logging/";

    public static Logger getInstance() {
        if (logger == null) return new Logger();
        return logger;
    }

    public Logger() {
        logs = new HashMap<>();
    }

    public void add(T t) {
        logs.putIfAbsent(t, new CSVWriter(new File(ROOT + t.getClass().getSimpleName() + ".csv")));
    }

    public void putData(T t, Map<String, Object> data) {
        CSVWriter log = logs.get(t);
        log.putAll(data);
        log.write();
        // log.flush(); NOT NECESSARY
    }

    public void putData(String str, Map<String, Object> data) {
        for (T t : logs.keySet()) {
            if (t.getClass().getSimpleName().equals(str)) {
                putData(t, data);
                return;
            }
        }
    }

}