//package org.firstinspires.ftc.teamcode.common.teleop.misc;
//
//import android.os.Build;
//
//import androidx.annotation.RequiresApi;
//
//import java.io.File;
//import java.io.FileWriter;
//import java.io.IOException;
//import java.nio.file.Paths;
//import java.text.ParseException;
//import java.text.SimpleDateFormat;
//import java.time.LocalDateTime;
//import java.time.format.DateTimeFormatter;
//import java.util.Calendar;
//import java.util.Date;
//import java.util.Locale;
//
//public class Logger {
//
//    public enum LoggerTags {
//        DEFAULT,
//        DEBUG,
//        WARNING,
//        ERROR,
//        CRITICAL
//    }
//
//    private Boolean append;
//    private File logFile = null;
//    private String path = null;
//    private String root = null;
//    private String filename  = null;
//
//    public Logger() {
//        this.append = true;
//        initialize();
//    }
//
//    /**
//     * @param filename
//     */
//    public Logger(String filename) {
//        this.append = true;
//        initialize();
//    }
//
//    /**
//     * @param append
//     */
//    public Logger(Boolean append) {
//        this.append = append;
//        initialize();
//    }
//
//    /**
//     * @param filename
//     * @param append
//     */
//    public Logger(String filename, Boolean append) {
//        this.append = true;
//        initialize();
//    }
//
//    private void initialize() {
//        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O) {
//            this.root = Paths.get(".").toAbsolutePath().normalize().toString();
//        }
//        if(filename != null) {
//            this.path = (this.root + "\\" + filename + ".txt");
//        } else {
//            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O) {
//                this.path = (this.root + "\\Log " + DateTimeFormatter.ofPattern("yyyy-MM-dd").format(LocalDateTime.now()) + ".txt");
//            }
//        }
//        this.logFile = new File(this.path);
//    }
//
//    public String getPath() {
//        return this.path;
//    }
//
//    public void setPath(String path) {
//        this.path = path;
//    }
//
//    /**
//     * Print just a message with default ag
//     * @param msg the message you want to print
//     */
//    public void write(String msg) {
//        LocalDateTime currentTime = null;
//        if (android.os.Build.VERSION.SDK_INT >= android.os.Build.VERSION_CODES.O) {
//            currentTime = LocalDateTime.now();
//        }
//        try {
//                FileWriter writer = new FileWriter(this.logFile, this.append);
//                try {
//                    this.logFile.createNewFile();
//                    String month = null;
//                    if (android.os.Build.VERSION.SDK_INT >= android.os.Build.VERSION_CODES.O) {
//                        month = getMonth(DateTimeFormatter.ofPattern("MMMM d, yyyyy").format(currentTime));
//                        String day = DateTimeFormatter.ofPattern("d").format(currentTime);
//                        String stamp = DateTimeFormatter.ofPattern("yyyy-MM-dd | hh-mm-ss a").format(currentTime);
//                        writer.write(month + " " + day + " " + stamp + " [" + LoggerTags.DEFAULT + "] " + msg + System.lineSeparator());
//                        writer.close();
//                    }
//                } catch (IOException e) {
//                    System.out.println("Could not write to file");
//                    e.printStackTrace();
//                } finally {
//                    writer.close();
//                }
//            } catch(IOException ex) {
//                System.out.println("Error writing to file");
//                ex.printStackTrace();
//            }
//    }
//
//    /**
//     * Write to the file with a specific tag
//     * @param tag the tag that you want
//     * @param msg the message that you want to print
//     */
//    public void write(LoggerTags tag, String msg) {
//        LocalDateTime currentTime = null;
//        if (android.os.Build.VERSION.SDK_INT >= android.os.Build.VERSION_CODES.O) {
//            currentTime = LocalDateTime.now();
//        }
//        try {
//                FileWriter writer = new FileWriter(this.logFile, this.append);
//                try {
//                    this.logFile.createNewFile();
//                    String month = null;
//                    if (android.os.Build.VERSION.SDK_INT >= android.os.Build.VERSION_CODES.O) {
//                        month = getMonth(DateTimeFormatter.ofPattern("MMMM d, yyyy").format(currentTime));
//                        String day = DateTimeFormatter.ofPattern("d").format(currentTime);
//                        String stamp = DateTimeFormatter.ofPattern("yyyy-MM-dd | HH-mm-ss a").format(currentTime);
//                        writer.write(month + " " + day + " " + stamp + " [" + tag + "] " + msg + System.lineSeparator());
//                    }
//                } catch (IOException e) {
//                    System.out.println("Could not write to file");
//                    e.printStackTrace();
//                } finally {
//                    writer.close();
//                }
//            } catch(IOException ex) {
//                System.out.println("Error writing to file");
//                ex.printStackTrace();
//            }
//    }
//
//    private String getMonth(String date) {
//        try {
//            Date d = new SimpleDateFormat("MMMM d, yyyy", Locale.ENGLISH).parse(date);
//            Calendar cal = Calendar.getInstance();
//            cal.setTime(d);
//            String monthName = new SimpleDateFormat("MMM").format(cal.getTime());
//            return monthName;
//        } catch(ParseException parseException) {
//            parseException.printStackTrace();
//            return null;
//        }
//    }
//
//}
