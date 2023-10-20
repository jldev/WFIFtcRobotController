package org.firstinspires.ftc.teamcode.G3GSS;

import android.os.Environment;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;


public class Logger {

    FileWriter log;
    File file;
    Telemetry opModeTelemetry; //the class that writes data to the Driver Station telemetry

    public Logger(boolean append, Telemetry telemetry) {
        try {

            //get or create the log file
            String directory = Environment.getExternalStorageDirectory().getPath();
            file = new File(directory+"/"+ "wfilog.txt");
            file.createNewFile();
            log = new FileWriter(file, append);
            //log.write("Date: " + LocalDateTime.now().toString());

            opModeTelemetry = telemetry;
        }
        catch (IOException e){
            telemetry.addData("Error", "Could not open log file");
        }
        finally {
            closeAndFlush();
        }
    }

    //Prints a simple divider
    public void printDivider() {
        try {
            log = new FileWriter(file, true);
            log.write("\n---------\n");
        }
        catch(IOException e) {
            opModeTelemetry.addData("Error", "Could not write to log");
            opModeTelemetry.update();
        }
        finally {
            closeAndFlush();
        }
    }

    //Writes a string message to the log file
    public void write(String msg) {
        try {
            log = new FileWriter(file, true);
            log.write(msg + "\n");
        }
        catch(IOException e) {
            opModeTelemetry.addData("Error", "Could not write to log");
            opModeTelemetry.update();
        }
        finally {
            closeAndFlush();
        }
    }

    //Writes a double to the log file
    public void write(double msg) {
        try {
            log = new FileWriter(file, true);
            log.write(String.valueOf(msg) + "\n");
        }
        catch(IOException e) {
            opModeTelemetry.addData("Error", "Could not write to log");
            opModeTelemetry.update();
        }
        finally {
            closeAndFlush();
        }
    }

    //Writes an int to the log file
    public void write(int msg) {
        try {
            log = new FileWriter(file, true);
            log.write(String.valueOf(msg) + "\n");
        }
        catch(IOException e) {
            opModeTelemetry.addData("Error", "Could not write to log");
            opModeTelemetry.update();
        }
        finally {
            closeAndFlush();
        }
    }

    //Writes a message indicating the end of the log entry
    public void closeLog() {
        try {
            log = new FileWriter(file, true);
            log.write("Closing log...");
            log.write("\n-----------\n");
        }
        catch(IOException e) {
            opModeTelemetry.addData("Error", "Could not close log");
            opModeTelemetry.update();
        }
        finally {
            closeAndFlush();
        }
    }

    //Flushes the buffer and closes the log file
    private void closeAndFlush() {

        try {
            log.flush();
            log.close();
        }
        catch(IOException ee) {
            opModeTelemetry.addData("Error", "Could not flush and close log");
            opModeTelemetry.update();
        }


    }
}
