package frc.robot.Util;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public enum LEDManager {

    STRIP0(65, 9);

    private LEDStrip strip;
    private LEDManager(int LEDCount, int PWMPort) {
        this.strip = new LEDStrip(LEDCount, PWMPort);
    }

    public void set(Color c) {
        this.strip.setSolidColor(c);
    }
    public void setGradient(Color... colors) {
        this.strip.setGradient(colors);
    }
    public void setGradient(int offset, Color... colors) {
        this.strip.setGradient(offset, colors);
    }
    
}

class LEDStrip {

    protected int PWMPort;
    protected AddressableLED lights;
    protected AddressableLEDBuffer buffer;

    public LEDStrip(int numLights, int PWMPort) {
        this.PWMPort = PWMPort;
        // Must be a PWM header
        this.lights = new AddressableLED(this.PWMPort);

        // Reuse buffer
        // Length is expensive to set, so only set it once, then just update data
        this.buffer = new AddressableLEDBuffer(numLights);
        this.lights.setLength(this.buffer.getLength());

        // Set the data
        this.lights.setData(this.buffer);
        this.lights.start();
    }

    // returns an RGB representation of the light at a given index of the 'AddressableLEDBuffer'
    public Color colorAt(int idx) {
        return this.buffer.getLED(idx);
    }

    // sets the entire 'LEDStrip' to a given color
    public void setSolidColor(Color c) {
        for (int i = 0; i < this.buffer.getLength(); i++) {
            // Sets the specified LED to the RGB values for red
            this.buffer.setLED(i, c);
        }
        this.lights.setData(this.buffer);
    }

    // sets the 'LEDStrip' to a gradient melding between the given colors
    public void setGradient(Color... colors) {
        this.setGradient(0, colors);
    }

    // sets the 'LEDStrip' to a gradient melding between the given colors
    // 'offset' refers to the offset from the start that the gradient begins at, 
    // though it wraps around, so there will be no dead space.
    // 'offset' is useful for looping colors (continuously update 'offset')
    public void setGradient(int offset, Color... colors) {
        Color grad[];
        int lightsPerGrad = this.buffer.getLength()/colors.length;
        int finalGradPos = lightsPerGrad*colors.length-1;
        for(int i = 0; i < colors.length; i++) {
            grad = LEDStrip.colorGradient(colors[i], 
                colors[(i+1)%colors.length], lightsPerGrad);
            for(int k = 0; k < lightsPerGrad; k++) {
                int lightIdx = i*lightsPerGrad+k;
                this.buffer.setLED((lightIdx+offset)%(finalGradPos+1), grad[k]);
            }
        }
        // correcting error resulting from integer division
        Color finalColor = this.colorAt(finalGradPos);
        for(int i = finalGradPos; i < this.buffer.getLength(); i++)
            this.buffer.setLED(i, finalColor);
        // setting the data
        this.lights.setData(this.buffer);
    }

    // returns an array of RGB color arrays encoding the gradient between two given
    // colors over a given number of 'steps' (the smoothness of the gradient)
    public static Color[] colorGradient(Color startColor, Color endColor, int steps) {
        Color gradient[] = new Color[steps], color;
        double proportion;
        
        for(int i = 0; i < gradient.length; i++) {
            proportion = (double)i/(gradient.length-1);
            color = new Color(
                proportion*(endColor.red-startColor.red)+startColor.red,
                proportion*(endColor.green-startColor.green)+startColor.green,
                proportion*(endColor.blue-startColor.blue)+startColor.blue
            );
            gradient[i] = color;
        }
        return gradient;
    }
}

// package frc.robot.Util;

// import edu.wpi.first.wpilibj.AddressableLED;
// import edu.wpi.first.wpilibj.AddressableLEDBuffer;
// import edu.wpi.first.wpilibj.util.Color;

// public class LEDStrip {

//     // protected static final IncorrectRGBFormattingException RGB_FORMATTING_EXCEPTION = 
//     //     new IncorrectRGBFormattingException();

//     // protected static final int MAX_RGB = 255;

//     // public static final Color
//     //     RED = new Color(255, 0, 0),
//     //     ORANGE = new Color(255, 127, 0),
//     //     YELLOW = new Color(255, 255, 0),
//     //     GREEN = new Color(0, 255, 0),
//     //     CYAN = new Color(0, 255, 255),
//     //     BLUE = new Color(0, 0, 255),
//     //     PURPLE = new Color(75, 0, 130),
//     //     MAGENTA = new Color(255, 0, 255),
//     //     WHITE = new Color(255, 255, 255),
//     //     GRAY = new Color(127, 127, 127),
//     //     OFF = new Color(0, 0, 0);

//     protected int PWMPort;
//     protected AddressableLED lights;
//     protected AddressableLEDBuffer buffer;

//     public LEDStrip(int numLights, int PWMPort) {
//         this.PWMPort = PWMPort;
//         // Must be a PWM header, not MXP or DIO
//         this.lights = new AddressableLED(this.PWMPort);

//         // Reuse buffer
//         // Default to a length of 60, start empty output
//         // Length is expensive to set, so only set it once, then just update data
//         this.buffer = new AddressableLEDBuffer(numLights);
//         this.lights.setLength(this.buffer.getLength());

//         // Set the data
//         this.lights.setData(this.buffer);
//         this.lights.start();
//     }

//     // returns an RGB representation of the light at a given index of the 'AddressableLEDBuffer'
//     public Color colorAt(int idx) {
//         return this.buffer.getLED(idx);
//     }

//     // sets the entire 'LEDStrip' to a given color
//     public void setSolidColor(Color c) {
//         for (int i = 0; i < this.buffer.getLength(); i++) {
//             // Sets the specified LED to the RGB values for red
//             this.buffer.setLED(i, c);
//         }
//         this.lights.setData(this.buffer);
//     }

//     // // sets the entire 'LEDStrip' to a given color
//     // public void setSolidColor(int[] rgb) throws IncorrectRGBFormattingException {
//     //     if(rgb.length != 3)
//     //         throw LEDStrip.RGB_FORMATTING_EXCEPTION;
//     //     this.setSolidColor(rgb[0], rgb[1], rgb[2]);
//     // }

//     // public void setGradient(int[] startColor, int[] endColor) {
//     //     if(startColor.length != 3 || endColor.length != 3)
//     //         throw LEDStrip.RGB_FORMATTING_ERROR;
//     //     int[][] gradient = LEDStrip.colorGradient(startColor, endColor, 
//     //         this.buffer.getLength());
//     //     for(int i = 0; i < this.buffer.getLength(); i++)
//     //         this.buffer.setRGB(i, gradient[i][0], gradient[i][1], gradient[i][2]);
//     //     this.lights.setData(this.buffer);
//     // }

//     // sets the 'LEDStrip' to a gradient melding between the given colors
//     public void setGradient(Color... colors) {
//         this.setGradient(0, colors);
//     }

//     // sets the 'LEDStrip' to a gradient melding between the given colors
//     // 'offset' refers to the offset from the start that the gradient begins at, 
//     // though it wraps around, so there will be no dead space.
//     // 'offset' is useful for looping colors (continuously update 'offset')
//     public void setGradient(int offset, Color... colors) {
//         Color grad[];
//         int lightsPerGrad = this.buffer.getLength()/colors.length;
//         int finalGradPos = lightsPerGrad*colors.length-1;
//         for(int i = 0; i < colors.length; i++) {
//             // if(colors[i].length != 3)
//             //     throw LEDStrip.RGB_FORMATTING_EXCEPTION;
//             grad = LEDStrip.colorGradient(colors[i], 
//                 colors[(i+1)%colors.length], lightsPerGrad);
//             for(int k = 0; k < lightsPerGrad; k++) {
//                 int lightIdx = i*lightsPerGrad+k;
//                 this.buffer.setLED((lightIdx+offset)%(finalGradPos+1), grad[k]);
//             }
//         }
//         // correcting error resulting from integer division
//         Color finalColor = this.colorAt(finalGradPos);
//         for(int i = finalGradPos; i < this.buffer.getLength(); i++)
//             this.buffer.setLED(i, finalColor);
//         // setting the data
//         this.lights.setData(this.buffer);
//     }

//     // returns an array of RGB color arrays encoding the gradient between two given
//     // colors over a given number of 'steps' (the smoothness of the gradient)
//     public static Color[] colorGradient(Color startColor, Color endColor, int steps) {
//         Color gradient[] = new Color[steps], color;
//         double proportion;
        
//         for(int i = 0; i < gradient.length; i++) {
//             proportion = (double)i/(gradient.length-1);
//             color = new Color(
//                 proportion*(endColor.red-startColor.red)+startColor.red,
//                 proportion*(endColor.green-startColor.green)+startColor.green,
//                 proportion*(endColor.blue-startColor.blue)+startColor.blue
//             );
//             // for(int k = 0; k < color.length; k++)
//             //     color[k] = (int)(proportion*(endColor[k]-startColor[k])+startColor[k]);
//             gradient[i] = color;
//         }
//         return gradient;
//     }

//     // // converts a 'color' object into an array representation of an RGB
//     // public static int[] colorToRGBArray(Color c) {
//     //     return new int[]{
//     //         (int)(c.red*LEDStrip.MAX_RGB),
//     //         (int)(c.green*LEDStrip.MAX_RGB),
//     //         (int)(c.blue*LEDStrip.MAX_RGB)
//     //     };
//     // }
// }

// // class IncorrectRGBFormattingException extends Exception {
// //     public IncorrectRGBFormattingException() {
// //         super("RGB color inputs must be of length 3.");
// //     }
// // }