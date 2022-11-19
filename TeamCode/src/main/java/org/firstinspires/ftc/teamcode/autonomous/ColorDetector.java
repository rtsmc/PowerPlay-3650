package org.firstinspires.ftc.teamcode.Autonomous;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class ColorDetector extends OpenCvPipeline {
    Mat CyanMat = new Mat();
    Mat YellowMat = new Mat();
    Mat MagentaMat = new Mat();
    Rect leftRect = new Rect(new Point(0, 200), new Point(630, 720));
    Rect rightRect = new Rect(new Point(650, 200), new Point(1270, 720));
    double percentColorThreshold = 0.10;
    private int level;
//    private double maxArea = 0;
//    private Rect maxRect;

    @Override
    public Mat processFrame(Mat input) {


        //creates 3 mats
        Imgproc.cvtColor(input, CyanMat, Imgproc.COLOR_RGB2HSV);
        Imgproc.cvtColor(input, YellowMat, Imgproc.COLOR_RGB2HSV);
        Imgproc.cvtColor(input, MagentaMat, Imgproc.COLOR_RGB2HSV);

        //Cyan
        Scalar CyanLowHSV = new Scalar(80, 100, 40);
        Scalar CyanHighHSV = new Scalar(100, 255, 255);

        //Yellow
        Scalar YellowLowHSV = new Scalar(20, 100, 40);
        Scalar YellowHighHSV = new Scalar(40, 255, 255);

        //Magenta
        Scalar MagentaLowHSV = new Scalar(140, 100, 40);
        Scalar MagentaHighHSV = new Scalar(160, 255, 255);


        Core.inRange(CyanMat, CyanLowHSV, CyanHighHSV, CyanMat);
        Core.inRange(YellowMat, YellowLowHSV, YellowHighHSV, YellowMat);
        Core.inRange(MagentaMat, MagentaLowHSV, MagentaHighHSV, MagentaMat);


        int CyanValue = Core.sumElems(CyanMat).val[0];
        int YellowValue = Core.sumElems(YellowMat).val[0];
        int MagentaValue = Core.sumElems(MagentaMat).val[0];

        CyanMat.release();
        YellowMat.release();
        MagentaMat.release();

        if (CyanValue > MagentaValue) {
            if (CyanValue > YellowValue) {
                return 0;
            }
            return 1;
        }
        if (MagentaValue > YellowValue) {
            return 2;
        }
        return 1;
    }

    public int getLevel(){
        return level;
    }
}