package com.example.user.opencv2;

import java.util.Vector;
import java.util.ArrayList;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;

public class ControlVariables {

    public static int trackingmode = 2;
    public static int undetectednum = 100;
    public static boolean of;
    public static int thr = 100;
    public static int successes = 0;
    public static int detected = 0;
    public static  int tracking = 0;
    public static int count = 0;
    public static boolean colorextract = false;
    public static boolean parsedsolution = false;
    public static int solution_iterator = 0;
    public static String solution;
    public static Vector<String> steps;
    public static boolean finished = false;
    public static int stat = 0;

    public static int x1, x2, x3, x4, y1, y2, y3, y4;
    int faces_count = 0;
    int succ = 0;

}
