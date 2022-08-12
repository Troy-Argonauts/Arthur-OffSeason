package com.akash;

import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.logging.*;
import java.io.IOException;

public class log {
    private static final Logger logger = Logger.getLogger(log.class.getName());

    public static String fileName(String logFileName) {
        StringBuilder fileName = new StringBuilder();

        long millis = System.currentTimeMillis();
        Date date = new Date(millis);
        SimpleDateFormat dateFormat = new SimpleDateFormat("YYYY_MM_DD");
        StringBuilder dateString = new StringBuilder(dateFormat.format(date));

        StringBuilder time24Hours = new StringBuilder(String.valueOf(stopwatch.time24Hours()));
        time24Hours.replace(2,3,"_");
        time24Hours.replace(5,6,"_");

        fileName.append("logs/");
        fileName.append(logFileName);
        fileName.append("-");
        fileName.append(dateString);
        fileName.append("-");
        fileName.append(time24Hours);
        fileName.append(".txt");
        return String.valueOf(fileName);
    }
    public static void logCreation(String logFile) throws SecurityException, IOException {
        // Creates a log file with the name entered + the date + the current time
        // Needs a folder called "logs" next to the src folder
        FileHandler handler = new FileHandler(fileName(logFile));
        handler.setFormatter(new SimpleFormatter());
        logger.addHandler(handler);
    }
    public static void logAndOutput(String message) {
        // Logs the message to the log file and prints the message to the console
        // Must create a log file first using log.logCreation
        System.out.println(message);
        log(message);
    }
    public static void log(String logMessage) {
        // Must create a log file first using log.logCreation
        // Logs the message to the log file
        logger.info(logMessage);
    }
}