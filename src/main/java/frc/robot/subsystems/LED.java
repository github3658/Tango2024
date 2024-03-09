package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.led.CANdle;

public class LED extends SubsystemBase {
    /* COLOR ENUM */
    public enum Color {
        Green(0),
        Yellow(1),
        Blue(2),
        Orange(3),
        Red(4),
        White(5),
        Magenta(6);
        public final int value;
        Color(int value) {
            this.value = value;
        }
    }
    //private Color e_ColorTarget;

    /* PATTERN ENUM */
    public enum Pattern {
        Solid,
        Strobe,
        Rainbow
    }
    private Pattern e_PatternTarget;

    /* CONSTANTS (prefix: c) */
    private final int c_CANDleID = 11;
    //private final int c_LEDCount = 61;

    /* COLORS (prefix: color) */
    private final int[] color_Yellow = {255, 255, 0};
    private final int[] color_Green = {0, 255, 0};
    private final int[] color_Blue = {0, 0, 255};
    private final int[] color_Orange = {255, 128, 0};
    private final int[] color_Red = {255, 0, 0};
    private final int[] color_White = {255,255,255};
    private final int[] color_Magenta = {217,1,122};

    /* CANDLE AND ANIMATIONS */
    public final CANdle m_CANdle = new CANdle(c_CANDleID, "3658CANivore");

    /* OTHER VARIABLES */
    private float f_Brightness = 1;
    private int i_Timer = 0;
    private int i_ColorOffset = 0;
    private int[] i_CurrentColor = {0,0,0};
    private int i_CurrentColorIndex = 0;
    private boolean b_Raw = false;

    public LED() {
        //e_ColorTarget = Color.White;
        e_PatternTarget = Pattern.Solid;
        m_CANdle.configFactoryDefault();
    }

    @Override
    public void periodic() {
        if (e_PatternTarget == Pattern.Strobe) {
            if (i_Timer % 12 == 0) {
                // Switch from one color to the other
                i_ColorOffset = (1 - i_ColorOffset);
            }
        }
        else if (e_PatternTarget == Pattern.Rainbow) {
            if (i_Timer % 5 == 0) {
                i_ColorOffset++;
            }
        }
        else {
            i_ColorOffset = 0;
        }

        if (!b_Raw) {
            i_CurrentColor = ColorToIntArray(Color.values()[(i_CurrentColorIndex+i_ColorOffset)%Color.values().length]);
        }
        m_CANdle.setLEDs(
            Math.round(i_CurrentColor[0]*f_Brightness),
            Math.round(i_CurrentColor[1]*f_Brightness),
            Math.round(i_CurrentColor[2]*f_Brightness)
        );

        i_Timer ++;
    }

    private int[] ColorToIntArray(Color colorInput) {
        int[] c = {0,0,0};
        if (colorInput == Color.Blue) {
            c = color_Blue;
        }
        else if (colorInput == Color.Green) {
            c = color_Green;
        }
        else if (colorInput == Color.Magenta) {
            c = color_Magenta;
        }
        else if (colorInput == Color.Orange) {
            c = color_Orange;
        }
        else if (colorInput == Color.Red) {
            c = color_Red;
        }
        else if (colorInput == Color.White) {
            c = color_White;
        }
        else if (colorInput == Color.Yellow) {
            c = color_Yellow;
        }
        return c;
    }

    public void SetColor(Color color) {
        i_CurrentColorIndex = color.value;
        b_Raw = false;
    }

    public void SetColorRaw(int r, int g, int b) {
        int[] array = {r,g,b};
        i_CurrentColor = array;
        b_Raw = true;
    }

    public void SetPattern(Pattern pattern) {
        e_PatternTarget = pattern;
    }

    public void setBrightness(float brightness) {
        f_Brightness = brightness;
    }
}