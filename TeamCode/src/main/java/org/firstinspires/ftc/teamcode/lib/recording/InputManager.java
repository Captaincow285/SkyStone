package org.firstinspires.ftc.teamcode.lib.recording;

import org.firstinspires.ftc.teamcode.lib.movement.Point;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.lib.movement.MyPosition.worldXPosition;
import static org.firstinspires.ftc.teamcode.lib.movement.MyPosition.worldYPosition;


public class InputManager {

    public BufferedWriter bufferedWriter;
    public FileWriter fileWriter;
    public FileReader fileReader;
    public BufferedReader bufferedReader;

    double[] worldXPoints;
    double[] worldYPoints;

    //double ly, rx, lx;

    //int lTicks, rTicks, sTicks;

    public enum DriveType{
        h4, h3, norm4, norm2
    }

    boolean useGyro, useCV;

    int numDriveMotors, numAuxMotors, numServos;

    int m1Ticks = 0, m2Ticks = 0, m3Ticks = 0, m4Ticks = 0, m5Ticks = 0, m6Ticks = 0, m7Ticks = 0, m8Ticks = 0;

    int s1Pos, s2Pos, s3Pos, s4Pos, s5Pos, s6Pos, s7Pos, s8Pos, s9Pos, s10Pos, s11Pos, s12Pos;

    double wx = 0;
    double wy = 0;
    double wr = 0;

    ArrayList<Point> points = new ArrayList<>();

    int gyroPos;

    int countLines, countReplays;

    public InputManager(DriveType type, int numAuxMotors, int numServos){

        switch (type){
            case h4:{

                numDriveMotors = 4;

            }
            case h3:{

                numDriveMotors = 3;

            }
            case norm4:{

                numDriveMotors = 4;

            }
            case norm2:{

                numDriveMotors = 2;

            }
        }

        this.numAuxMotors = numAuxMotors;
        this.numServos = numServos;

    }

    public InputManager(){

    }

    public void setupRecording(File file){

        // Assume default encoding.
        fileWriter = null;

        try {
            fileWriter = new FileWriter(file);
        } catch (IOException e) {
            e.printStackTrace();
        }

        // Always wrap FileWriter in BufferedWriter.
        bufferedWriter = new BufferedWriter(fileWriter);

    }

    public void writeInputs(){

        try {
            //bufferedWriter.write(m1Ticks + ";" + m2Ticks + ";" + m3Ticks + ";" + m4Ticks + ";" + m5Ticks + ";" + m6Ticks + ";" + m7Ticks + ";" + m8Ticks + ";" + s1Pos + ";" + s2Pos + ";" + s3Pos + ";" + s4Pos + ";" + s5Pos + ";" + s6Pos + ";" + s7Pos + ";" + s8Pos + ";" + s9Pos + ";" + s10Pos + ";" + s11Pos + ";" + s12Pos + ";" + gyroPos);
            bufferedWriter.write(worldYPosition + ";" + worldXPosition);

            bufferedWriter.newLine();
        } catch (IOException e) {
            e.printStackTrace();
        }

        //dt.manualDrive(gamepad1);

    }

    public void setupPlayback(File file){
        String[] tempString;
        String line;

        worldXPoints = new double[50000];
        worldYPoints = new double[50000];

        try {

            fileReader = new FileReader(file);
            bufferedReader = new BufferedReader(fileReader);

            while ((line = bufferedReader.readLine()) != null) {

                tempString = line.split(";");

                double tempY = Double.parseDouble(tempString[0]);
                double tempX = Double.parseDouble(tempString[1]);

                points.add(new Point(tempX, tempY));

                countLines++;
            }

            bufferedReader.close();

            countReplays = 0;
        }
        catch(IOException ex) {
            ex.printStackTrace();
        }
    }

    public void replayInputs(){

    }

    public ArrayList<Point> getPoints(){
        return points;
    }

    public void stopAndReset() {

        try {

            bufferedReader.close();
            bufferedWriter.close();

        } catch (IOException e) {
            e.printStackTrace();
        }

    }

}