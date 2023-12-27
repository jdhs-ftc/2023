/*
 * Copyright (c) 2023 Sebastian Erives
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */

package org.firstinspires.ftc.teamcode.vision.pipelines;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point;
import org.opencv.core.Point3;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;


public class WhitePixelProcessor implements VisionProcessor {

    /*
     * These are our variables that will be
     * modifiable from the variable tuner.
     *
     * Scalars in OpenCV are generally used to
     * represent color. So our values in the
     * lower and upper Scalars here represent
     * the Y, Cr and Cb values respectively.
     *
     * YCbCr, like most color spaces, range
     * from 0-255, so we default to those
     * min and max values here for now, meaning
     * that all pixels will be shown.
     */
    public Scalar lower = new Scalar(0, 0, 210);
    public Scalar upper = new Scalar(120, 74, 255);
    public final double canny1 = 100;
    public final double canny2 = 500;
    public double blur = 15;
    public Rect detectedPixelCandidate = null;

    /**
     * This will allow us to choose the color
     * space we want to use on the live field
     * tuner instead of hardcoding it
     */
    final ColorSpace colorSpace = ColorSpace.HSV;

    /*
     * A good practice when typing EOCV pipelines is
     * declaring the Mats you will use here at the top
     * of your pipeline, to reuse the same buffers every
     * time. This removes the need to call mat.release()
     * with every Mat you create on the processFrame method,
     * and therefore, reducing the possibility of getting a
     * memory leak and causing the app to crash due to an
     * "Out of Memory" error.
     */
    private final Mat ycrcbMat       = new Mat();
    private final Mat binaryMat      = new Mat();
    private final Mat maskedInputMat = new Mat();
    private Mat edges = new Mat();

    public Rect detectedPixel;

    /**
     * Enum to choose which color space to choose
     * space we want to use on the live field
     * with the live variable tuner isntead of
     * hardcoding it.
     */
    enum ColorSpace {
        /*
         * Define our "conversion codes" in the enum
         * so that we don't have to do a switch
         * statement in the processFrame method.
         */
        RGB(Imgproc.COLOR_RGBA2RGB),
        HSV(Imgproc.COLOR_RGB2HSV),
        YCrCb(Imgproc.COLOR_RGB2YCrCb),
        Lab(Imgproc.COLOR_RGB2Lab);

        //store cvtCode in a public var
        public int cvtCode;

        //constructor to be used by enum declarations above
        ColorSpace(int cvtCode) {
            this.cvtCode = cvtCode;
        }
    }

    public WhitePixelProcessor(Telemetry telemetry) {
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        /*
         * Converts our input mat from RGB to
         * specified color space by the enum.
         * EOCV ALWAYS returns RGB mats, so you'd
         * always convert from RGB to the color
         * space you want to use.
         *
         * Takes our "input" mat as an input, and outputs
         * to a separate Mat buffer "ycrcbMat"
         */
        Imgproc.cvtColor(frame, ycrcbMat, colorSpace.cvtCode);

        Imgproc.blur(ycrcbMat,ycrcbMat,new Size(blur, blur));

        /*
         * This is where our thresholding actually happens.
         * Takes our "ycrcbMat" as input and outputs a "binary"
         * Mat to "binaryMat" of the same size as our input.
         * "Discards" all the pixels outside the bounds specified
         * by the scalars above (and modifiable with EOCV-Sim's
         * live variable tuner.)
         *
         * Binary meaning that we have either a 0 or 255 value
         * for every pixel.
         *
         * 0 represents our pixels that were outside the bounds
         * 255 represents our pixels that are inside the bounds
         */
        Core.inRange(ycrcbMat, lower, upper, binaryMat);




        /*
         * Release the reusable Mat so that old data doesn't
         * affect the next step in the current processing
         */
        maskedInputMat.release();

        /*
         * Now, with our binary Mat, we perform a "bitwise and"
         * to our input image, meaning that we will perform a mask
         * which will include the pixels from our input Mat which
         * are "255" in our binary Mat (meaning that they're inside
         * the range) and will discard any other pixel outside the
         * range (RGB 0, 0, 0. All discarded pixels will be black)
         */
        Core.bitwise_and(frame, frame, maskedInputMat, binaryMat);

        Imgproc.Canny(maskedInputMat, edges, canny1, canny2);
        //Imgproc.cvtColor(maskedInputMat, edges, Imgproc.COLOR_HSV2BGR);
        //Imgproc.cvtColor(edges, edges, Imgproc.COLOR_BGR2GRAY);
        // https://docs.opencv.org/3.4/da/d0c/tutorial_bounding_rects_circles.html
        // Oftentimes the edges are disconnected. findContours connects these edges.
        // We then find the bounding rectangles of those contours
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(edges, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        MatOfPoint2f[] contoursPoly  = new MatOfPoint2f[contours.size()];
        Rect[] boundRect = new Rect[contours.size()];
        for (int i = 0; i < contours.size(); i++) {
            contoursPoly[i] = new MatOfPoint2f();
            Imgproc.approxPolyDP(new MatOfPoint2f(contours.get(i).toArray()), contoursPoly[i], 3, true);
            boundRect[i] = Imgproc.boundingRect(new MatOfPoint(contoursPoly[i].toArray()));
        }
        detectedPixelCandidate = new Rect();
        for (int i = 0; i != boundRect.length; i++) {
            double aspectRatio = (double) boundRect[i].height / (double) boundRect[i].width;
            if (aspectRatio >= 0.8 && aspectRatio <= 1.2 && boundRect[i].width > 50 && boundRect[i].width < 250) {
                if (boundRect[i].width > detectedPixelCandidate.width) {
                    detectedPixelCandidate = boundRect[i];
                    Imgproc.rectangle(maskedInputMat, boundRect[i], new Scalar(0, 255, 0));
                    Imgproc.putText(maskedInputMat, Integer.toString(boundRect[i].x), new Point(boundRect[i].x,boundRect[i].y), 0, 1, new Scalar(255, 255, 255));
                } else {
                    //Imgproc.rectangle(maskedInputMat, boundRect[i], new Scalar(255, 0, 0));
                    //Imgproc.putText(maskedInputMat, Double.toString(Math.round(100.0 * (double) boundRect[i].height / (double) boundRect[i].width) / 100.0), new Point(boundRect[i].x,boundRect[i].y), 0, 1, new Scalar(0, 76.9, 89.8));
                    Imgproc.putText(maskedInputMat, Integer.toString(boundRect[i].width), new Point(boundRect[i].x,boundRect[i].y), 0, 1, new Scalar(0, 76.9, 89.8));
                }
            }
        }
        if (detectedPixelCandidate.width > 0) {
            detectedPixel = detectedPixelCandidate;
        } else {
            detectedPixel = null;
        }




        /*
        /**
         * Add some nice and informative telemetry messages

        telemetry.addData("[>]", "Change these values in tuner menu");
        telemetry.addData("[Color Space]", colorSpace.name());
        telemetry.addData("[Lower Scalar]", lower);
        telemetry.addData("[Upper Scalar]", upper);
        telemetry.update();
        */

        /*
         * Different from OpenCvPipeline, you cannot return
         * a Mat from processFrame. Therefore, we will take
         * advantage of the fact that anything drawn onto the
         * passed `frame` object will be displayed on the
         * viewport. We will just return null here.
         */
        maskedInputMat.copyTo(frame);
        //edges.copyTo(frame);
        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
    }

    public Rect getDetectedPixel() {
        return detectedPixel;
    }

    Pose poseFromTrapezoid(Point[] points, Mat cameraMatrix, double sizeX , double sizeY)
    {
        // The actual 2d points of the tag detected in the image
        MatOfPoint2f points2d = new MatOfPoint2f(points);

        // The 3d points of the tag in an 'ideal projection'
        Point3[] arrayPoints3d = new Point3[4];
        arrayPoints3d[0] = new Point3(-sizeX/2, sizeY/2, 0);
        arrayPoints3d[1] = new Point3(sizeX/2, sizeY/2, 0);
        arrayPoints3d[2] = new Point3(sizeX/2, -sizeY/2, 0);
        arrayPoints3d[3] = new Point3(-sizeX/2, -sizeY/2, 0);
        MatOfPoint3f points3d = new MatOfPoint3f(arrayPoints3d);

        // Using this information, actually solve for pose
        Pose pose = new Pose();
        Calib3d.solvePnP(points3d, points2d, cameraMatrix, new MatOfDouble(), pose.rvec, pose.tvec, false);

        return pose;
    }

    /*
     * A simple container to hold both rotation and translation
     * vectors, which together form a 6DOF pose.
     */
    class Pose
    {
        Mat rvec;
        Mat tvec;

        public Pose()
        {
            rvec = new Mat(3, 1, CvType.CV_32F);
            tvec = new Mat(3, 1, CvType.CV_32F);
        }

        public Pose(Mat rvec, Mat tvec)
        {
            this.rvec = rvec;
            this.tvec = tvec;
        }
    }
}
