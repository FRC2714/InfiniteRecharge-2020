package frc.robot.utils;

import java.io.File;
import java.io.IOException;
import java.io.PrintStream;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;


public class CSVWriter {
    private LinkedHashMap<String, String> map;
    private List<String> header;
    private PrintStream printStream;

    public CSVWriter(File file) {
        map = new LinkedHashMap<>();
        try {
            printStream = new PrintStream(file);
        } catch (IOException e) {
            e.printStackTrace();
        }

    }

    public void put(String key, Object value) {
        map.put(key, value.toString());
    }

    public void putAll(Map<String, Object> map) {
        for (Map.Entry<String, Object> entry : map.entrySet()) {
            put(entry.getKey(), entry.getValue().toString());
        }
    }

    public void putLine(String line) {
        printStream.println(line);
    }

    public void clear() {
        map.clear();
    }

    public void write() {
        if (map.size() == 0) return;
        if (header == null) {
            header = new ArrayList<>();
            StringBuilder headerBuilder = new StringBuilder();
            for (Map.Entry<String, String> entry : map.entrySet()) {
                header.add(entry.getKey());
                headerBuilder.append(entry.getKey());
                headerBuilder.append(",");
            }
            headerBuilder.deleteCharAt(headerBuilder.length() - 1);
            if (printStream != null) {
                printStream.println(headerBuilder);
            }
        }
        StringBuilder valueBuilder = new StringBuilder();
        for (String key : header) {
            valueBuilder.append(map.get(key));
            valueBuilder.append(",");
        }
        valueBuilder.deleteCharAt(valueBuilder.length() - 1);
        if (printStream != null) {
            printStream.println(valueBuilder.toString());
        }
    }

    public void flush() {
        printStream.flush();
    }

    public void close() {
        if (printStream != null) {
            try {
                printStream.close();
            } catch (Exception e) {
                //Log.w("CSVWriter", e);
            }
            printStream = null;
        }
    }

}