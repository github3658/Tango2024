package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.StrobeAnimation;

public class LED extends SubsystemBase {
    /* COLOR ENUM */
    public enum Color {
        Green(0),
        Yellow(1),
        Blue(2),
        Orange(3),
        Red(4),
        White(5);
        public final int value;
        Color(int value) {
            this.value = value;
        }
    }
    private Color e_ColorTarget;

    /* PATTERN ENUM */
    public enum Pattern {
        Solid,
        Strobe
    }
    private Pattern e_PatternTarget;

    /* CONSTANTS (prefix: c) */
    private final int c_CANDleID = 11;
    private final int c_LEDCount = 2; // TODO: Number

    /* COLORS (prefix: color) */
    private final int[] color_Yellow = {233, 225, 24};
    private final int[] color_Green = {38, 236, 58};
    private final int[] color_Blue = {47, 181, 243};
    private final int[] color_Orange = {243, 135, 13};
    private final int[] color_Red = {255, 0, 0};
    private final int[] color_White = {255,255,255};

    /* CANDLE AND ANIMATIONS */
    public final CANdle m_CANdle = new CANdle(c_CANDleID, "3658CANivore");

    /* OTHER VARIABLES */
    private int i_Timer = 0;
    private int i_ColorOffset = 0;
    private int[] i_CurrentColor = {0,0,0};
    private int i_CurrentColorIndex = 0;

    public LED() {
        e_ColorTarget = Color.White;
        e_PatternTarget = Pattern.Solid;
        m_CANdle.configFactoryDefault();
    }

    @Override
    public void periodic() {
        if (e_PatternTarget == Pattern.Strobe) {
            if (i_Timer % 12 == 0) {
                // Switch from one color to the other
                i_ColorOffset = (1 - i_ColorOffset);
                i_CurrentColor = ColorToIntArray(Color.values()[(i_CurrentColorIndex+i_ColorOffset)%Color.values().length]);
            }
        }
        else {
            i_ColorOffset = 0;
        }

        i_CurrentColor = ColorToIntArray(Color.values()[(i_CurrentColorIndex+i_ColorOffset)%Color.values().length]);
        m_CANdle.setLEDs(i_CurrentColor[0],i_CurrentColor[1],i_CurrentColor[2]);

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
    }

    public void SetPattern(Pattern pattern) {
        e_PatternTarget = pattern;
    }
}