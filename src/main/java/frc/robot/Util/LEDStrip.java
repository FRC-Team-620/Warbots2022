package frc.robot.Util;

import java.util.Arrays;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

public class LEDStrip {

    protected static final Error RGB_FORMATTING_ERROR = 
        new Error("RGB color inputs must be of length 3.");

    protected static final int MAX_RGB = 255;

    protected int PWMPort;
    protected AddressableLED lights;
    protected AddressableLEDBuffer buffer;

    public LEDStrip(int size, int PWMPort) {
        this.PWMPort = PWMPort;
        // Must be a PWM header, not MXP or DIO
        this.lights = new AddressableLED(this.PWMPort);

        // Reuse buffer
        // Default to a length of 60, start empty output
        // Length is expensive to set, so only set it once, then just update data
        this.buffer = new AddressableLEDBuffer(size);
        this.lights.setLength(this.buffer.getLength());

        // Set the data
        this.lights.setData(this.buffer);
        this.lights.start();
    }

    public int[] colorAt(int idx) {
        return LEDStrip.colorToRGBArray(this.buffer.getLED(idx));
    }

    public void setSolidColor(int R, int G, int B) {
        for (int i = 0; i < this.buffer.getLength(); i++) {
            // Sets the specified LED to the RGB values for red
            this.buffer.setRGB(i, R, G, B);
        }
        this.lights.setData(this.buffer);
    }

    public void setSolidColor(int[] rgb) {
        if(rgb.length != 3)
            throw LEDStrip.RGB_FORMATTING_ERROR;
        this.setSolidColor(rgb[0], rgb[1], rgb[2]);
    }

    // public void setGradient(int[] startColor, int[] endColor) {
    //     if(startColor.length != 3 || endColor.length != 3)
    //         throw LEDStrip.RGB_FORMATTING_ERROR;
    //     int[][] gradient = LEDStrip.colorGradient(startColor, endColor, 
    //         this.buffer.getLength());
    //     for(int i = 0; i < this.buffer.getLength(); i++)
    //         this.buffer.setRGB(i, gradient[i][0], gradient[i][1], gradient[i][2]);
    //     this.lights.setData(this.buffer);
    // }

    public void setGradient(int[]... colors) {
        this.setGradient(0, colors);
    }

    public void setGradient(int displacement, int[]... colors) {
        int grad[][];
        int lightsPerGrad = this.buffer.getLength()/colors.length;
        int finalGradPos = lightsPerGrad*colors.length-1;
        for(int i = 0; i < colors.length; i++) {
            if(colors[i].length != 3)
                throw LEDStrip.RGB_FORMATTING_ERROR;
            grad = LEDStrip.colorGradient(colors[i], 
                colors[(i+1)%colors.length], lightsPerGrad);
            for(int k = 0; k < lightsPerGrad; k++) {
                int lightIdx = i*lightsPerGrad+k;
                this.buffer.setRGB((lightIdx+displacement)%(finalGradPos+1), 
                    grad[k][0], grad[k][1], grad[k][2]);
            }
        }
        // correcting error resulting from integer division
        int[] finalColor = this.colorAt(finalGradPos);
        for(int i = finalGradPos; i < this.buffer.getLength(); i++)
            this.buffer.setRGB(i, finalColor[0], finalColor[1], finalColor[2]);
        // setting the data
        this.lights.setData(this.buffer);
    }

    public static int[][] colorGradient(int[] startColor, int[] endColor, int steps) {
        if(startColor.length != 3 || endColor.length != 3)
            throw LEDStrip.RGB_FORMATTING_ERROR;
        int gradient[][] = new int[steps][3], color[];
        double proportion;
        
        for(int i = 0; i < gradient.length; i++) {
            proportion = (double)i/(gradient.length-1);
            color = new int[3];
            for(int k = 0; k < color.length; k++)
                color[k] = (int)(proportion*(endColor[k]-startColor[k])+startColor[k]);
            gradient[i] = color;
        }
        return gradient;
    }

    public static int[] colorToRGBArray(Color c) {
        return new int[]{
            (int)(c.red*LEDStrip.MAX_RGB),
            (int)(c.green*LEDStrip.MAX_RGB),
            (int)(c.blue*LEDStrip.MAX_RGB)
        };
    }
}

// class IncorrectRGBFormattingError extends Error {
//     public IncorrectRGBFormattingError() {
//         super("RGB color inputs must be of length 3.");
//     }
// }