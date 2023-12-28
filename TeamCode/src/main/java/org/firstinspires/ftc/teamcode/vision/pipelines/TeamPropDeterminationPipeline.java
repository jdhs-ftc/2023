package org.firstinspires.ftc.teamcode.vision.pipelines;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class TeamPropDeterminationPipeline extends OpenCvPipeline
{
    boolean isBlue = true; // CHANGE THIS FOR SIM
    boolean useTelemetry = true;
    private final Telemetry telemetry;

    /*
     * An enum to define the skystone position
     */
    public enum PropPosition
    {
        LEFT,
        CENTER,
        RIGHT
    }

    /*
     * Some color constants
     */
    static final Scalar BLUE = new Scalar(0, 0, 255);
    static final Scalar GREEN = new Scalar(0, 255, 0);

    /*
     * The core values which define the location and size of the sample regions
     */
    static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(0,225);
    static final Point REGION2_TOPLEFT_ANCHOR_POINT = new Point(200,250);
    static final Point REGION3_TOPLEFT_ANCHOR_POINT = new Point(475,225);
    static final int SIDE_REGION_WIDTH = 150;
    static final int SIDE_REGION_HEIGHT = 175;
    static final int FRONT_REGION_WIDTH = 250;
    static final int FRONT_REGION_HEIGHT = 100;

    /*
     * Points which actually define the sample region rectangles, derived from above values
     *
     * Example of how points A and B work to define a rectangle
     *
     *   ------------------------------------
     *   | (0,0) Point A                    |
     *   |                                  |
     *   |                                  |
     *   |                                  |
     *   |                                  |
     *   |                                  |
     *   |                                  |
     *   |                  Point B (70,50) |
     *   ------------------------------------
     *
     */
    final Point region1_pointA = new Point(
            REGION1_TOPLEFT_ANCHOR_POINT.x,
            REGION1_TOPLEFT_ANCHOR_POINT.y);
    final Point region1_pointB = new Point(
            REGION1_TOPLEFT_ANCHOR_POINT.x + SIDE_REGION_WIDTH,
            REGION1_TOPLEFT_ANCHOR_POINT.y + SIDE_REGION_HEIGHT);
    final Point region2_pointA = new Point(
            REGION2_TOPLEFT_ANCHOR_POINT.x,
            REGION2_TOPLEFT_ANCHOR_POINT.y);
    final Point region2_pointB = new Point(
            REGION2_TOPLEFT_ANCHOR_POINT.x + FRONT_REGION_WIDTH,
            REGION2_TOPLEFT_ANCHOR_POINT.y + FRONT_REGION_HEIGHT);
    final Point region3_pointA = new Point(
            REGION3_TOPLEFT_ANCHOR_POINT.x,
            REGION3_TOPLEFT_ANCHOR_POINT.y);
    final Point region3_pointB = new Point(
            REGION3_TOPLEFT_ANCHOR_POINT.x + SIDE_REGION_WIDTH,
            REGION3_TOPLEFT_ANCHOR_POINT.y + SIDE_REGION_HEIGHT);

    /*
     * Working variables
     */
    Mat region1_Cr, region2_Cr, region3_Cr;
    Mat region1_Cb, region2_Cb, region3_Cb;
    final Mat YCrCb = new Mat();
    final Mat Cr = new Mat();
    final Mat Cb = new Mat();
    int avg1, avg2, avg3;

    // Volatile since accessed by OpMode thread w/o synchronization
    private volatile PropPosition position = PropPosition.LEFT;
    private volatile int confidence = 0;

    /*
     * This function takes the RGB frame, converts to YCrCb,
     * and extracts the Cb channel to the 'Cb' variable
     */
    void inputExtractChannels(Mat input)
    {
        Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
        Core.extractChannel(YCrCb, Cr, 1);
        Core.extractChannel(YCrCb, Cb, 2);
    }

    public TeamPropDeterminationPipeline(Telemetry telemetry) {
        this.telemetry = telemetry;
        this.useTelemetry = true;
    }
    public TeamPropDeterminationPipeline() {
        this.telemetry = null;
        this.useTelemetry = false;
    }

    public void setBlue(Boolean newIsBlue) {
        isBlue = newIsBlue;
    }
    public void setUseTelemetry(Boolean useTelemetry) {
        this.useTelemetry = useTelemetry;
    }

    @Override
    public void init(Mat firstFrame)
    {
        /*
         * We need to call this in order to make sure the 'Cb'
         * object is initialized, so that the submats we make
         * will still be linked to it on subsequent frames. (If
         * the object were to only be initialized in processFrame,
         * then the submats would become delinked because the backing
         * buffer would be re-allocated the first time a real frame
         * was crunched)
         */
        inputExtractChannels(firstFrame);

        /*
         * Submats are a persistent reference to a region of the parent
         * buffer. Any changes to the child affect the parent, and the
         * reverse also holds true.
         */
        region1_Cr = Cr.submat(new Rect(region1_pointA, region1_pointB));
        region2_Cr = Cr.submat(new Rect(region2_pointA, region2_pointB));
        region3_Cr = Cr.submat(new Rect(region3_pointA, region3_pointB));

        region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
        region2_Cb = Cb.submat(new Rect(region2_pointA, region2_pointB));
        region3_Cb = Cb.submat(new Rect(region3_pointA, region3_pointB));
    }

    @Override
    public Mat processFrame(Mat input)
    {
        /*
         * Overview of what we're doing:
         *
         * We first convert to YCrCb color space, from RGB color space.
         * Why do we do this? Well, in the RGB color space, chroma and
         * luma are intertwined. In YCrCb, chroma and luma are separated.
         * YCrCb is a 3-channel color space, just like RGB. YCrCb's 3 channels
         * are Y, the luma channel (which essentially just a B&W image), the
         * Cr channel, which records the difference from red, and the Cb channel,
         * which records the difference from blue. Because chroma and luma are
         * not related in YCrCb, vision code written to look for certain values
         * in the Cr/Cb channels will not be severely affected by differing
         * light intensity, since that difference would most likely just be
         * reflected in the Y channel.
         *
         * After we've converted to YCrCb, we extract just the 2nd channel, the
         * Cb channel. We do this because stones are bright yellow and contrast
         * STRONGLY on the Cb channel against everything else, including SkyStones
         * (because SkyStones have a black label).
         *
         * We then take the average pixel value of 3 different regions on that Cb
         * channel, one positioned over each stone. The brightest of the 3 regions
         * is where we assume the SkyStone to be, since the normal stones show up
         * extremely darkly.
         *
         * We also draw rectangles on the screen showing where the sample regions
         * are, as well as drawing a solid rectangle over top the sample region
         * we believe is on top of the SkyStone.
         *
         * In order for this whole process to work correctly, each sample region
         * should be positioned in the center of each of the first 3 stones, and
         * be small enough such that only the stone is sampled, and not any of the
         * surroundings.
         */

        /*
         * Get the Cr channel of the input frame after conversion to YCrCb
         */
        inputExtractChannels(input);

        /*
         * Compute the average pixel value of each submat region. We're
         * taking the average of a single channel buffer, so the value
         * we need is at index 0. We could have also taken the average
         * pixel value of the 3-channel image, and referenced the value
         * at index 2 here.
         */
        if (isBlue) {
            avg1 = -(int) Core.mean(region1_Cr).val[0] + (int) Core.mean(region1_Cb).val[0];
            avg2 = -(int) Core.mean(region2_Cr).val[0] + (int) Core.mean(region2_Cb).val[0];
            avg3 = -(int) Core.mean(region3_Cr).val[0] + (int) Core.mean(region3_Cb).val[0];
        } else {
            avg1 = (int) Core.mean(region1_Cr).val[0];
            avg2 = (int) Core.mean(region2_Cr).val[0];
            avg3 = (int) Core.mean(region3_Cr).val[0];
        }


        /*
         * Draw a rectangle showing sample region 1 on the screen.
         * Simply a visual aid. Serves no functional purpose.
         */
        Imgproc.rectangle(
                input, // Buffer to draw on
                region1_pointA, // First point which defines the rectangle
                region1_pointB, // Second point which defines the rectangle
                BLUE, // The color the rectangle is drawn in
                2); // Thickness of the rectangle lines

        /*
         * Draw a rectangle showing sample region 2 on the screen.
         * Simply a visual aid. Serves no functional purpose.
         */
        Imgproc.rectangle(
                input, // Buffer to draw on
                region2_pointA, // First point which defines the rectangle
                region2_pointB, // Second point which defines the rectangle
                BLUE, // The color the rectangle is drawn in
                2); // Thickness of the rectangle lines

        /*
         * Draw a rectangle showing sample region 3 on the screen.
         * Simply a visual aid. Serves no functional purpose.
         */
        Imgproc.rectangle(
                input, // Buffer to draw on
                region3_pointA, // First point which defines the rectangle
                region3_pointB, // Second point which defines the rectangle
                BLUE, // The color the rectangle is drawn in
                2); // Thickness of the rectangle lines


        /*
         * Find the max of the 3 averages
         */
        int maxOneTwo = Math.max(avg1, avg2);
        int max = Math.max(maxOneTwo, avg3);

        /*
         * Now that we found the max, we actually need to go and
         * figure out which sample region that value was from
         */
        if(max == avg1) // Was it from region 1?
        {
            position = PropPosition.LEFT; // Record our analysis
            confidence = max - ((avg2 + avg3) /2);

            /*
             * Draw a solid rectangle on top of the chosen region.
             * Simply a visual aid. Serves no functional purpose.
             */
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    -1); // Negative thickness means solid fill
        }
        else if(max == avg2) // Was it from region 2?
        {
            position = PropPosition.CENTER; // Record our analysis
            confidence = max - ((avg1 + avg3) /2);

            /*
             * Draw a solid rectangle on top of the chosen region.
             * Simply a visual aid. Serves no functional purpose.
             */
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region2_pointA, // First point which defines the rectangle
                    region2_pointB, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    -1); // Negative thickness means solid fill
        }
        else if(max == avg3) // Was it from region 3?
        {
            position = PropPosition.RIGHT; // Record our analysis
            confidence = max - ((avg1 + avg2) /2);

            /*
             * Draw a solid rectangle on top of the chosen region.
             * Simply a visual aid. Serves no functional purpose.
             */
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region3_pointA, // First point which defines the rectangle
                    region3_pointB, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    -1); // Negative thickness means solid fill
        }
        if (useTelemetry && telemetry != null) {
            telemetry.addData("position", position);
            telemetry.addData("avg1", avg1);
            telemetry.addData("avg2", avg2);
            telemetry.addData("avg3", avg3);
            telemetry.addData("confidence", confidence);
            telemetry.update();
        }
        /*
         * Render the 'input' buffer to the viewport. But note this is not
         * simply rendering the raw camera feed, because we called functions
         * to add some annotations to this buffer earlier up.
         */
        return input;
    }

    /*
     * Call this from the OpMode thread to obtain the latest analysis
     */
    public PropPosition getAnalysis()
    {
        return position;
    }
    public int getConfidence() { return confidence; }
}
