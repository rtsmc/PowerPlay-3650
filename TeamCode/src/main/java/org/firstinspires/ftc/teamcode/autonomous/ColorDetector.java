package org.firstinspires.ftc.teamcode.autonomous;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class ColorDetector extends OpenCvPipeline {
    Mat mat = new Mat();
    Rect rect = new Rect(new Point(1140, 60), new Point(770, 620));
    double magentaValue;
    double yellowValue;
    double cyanValue;
    private int level;

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

        //Cyan
        Scalar CyanLowHSV = new Scalar(75, 20, 20);
        Scalar CyanHighHSV = new Scalar(100, 255, 255);

        //Yellow
        Scalar YellowLowHSV = new Scalar(15, 20, 20);
        Scalar YellowHighHSV = new Scalar(40, 255, 255);

        //Magenta
        Scalar MagentaLowHSV = new Scalar(130, 20, 20);
        Scalar MagentaHighHSV = new Scalar(170, 255, 255);

        Mat cyanMat = mat.submat(rect);
        Mat yellowMat = mat.submat(rect);
        Mat magentaMat = mat.submat(rect);

        Core.inRange(cyanMat, CyanLowHSV, CyanHighHSV, cyanMat);
        Core.inRange(yellowMat, YellowLowHSV, YellowHighHSV, yellowMat);
        Core.inRange(magentaMat, MagentaLowHSV, MagentaHighHSV, magentaMat);

        Imgproc.rectangle(mat, rect, new Scalar(255, 0, 0));

        double cyanValue = Core.sumElems(cyanMat).val[0] / rect.area() / 255;
        double yellowValue = Core.sumElems(yellowMat).val[0] / rect.area() / 255;
        double magentaValue = Core.sumElems(magentaMat).val[0] / rect.area() / 255;

        cyanMat.release();
        yellowMat.release();
        magentaMat.release();

        if(cyanValue > magentaValue && cyanValue > yellowValue){
            level = 1;  //cyan
        } else if(magentaValue > yellowValue) {
            level = 2;  //magenta
        } else {
            level = 3;  //yellow
        }

        return mat;
    }

    public int getLevel(){
        return level;
    }

    public String getValues(){
        return "magenta " + magentaValue + "cyan " + cyanValue + "yellow " + yellowValue;
    }
}