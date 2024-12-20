package frc.robot.utilities;

import edu.wpi.first.wpilibj2.command.Command;

/**
 * Holds a group of commands that can be run to get data to characterise a mechanism using SysID.
 *
 * @param dynamicForward A {@link Command} that runs the mechanism in the forward direction at a
 *     constant voltage
 * @param dynamicReverse A {@link Command} that runs the mechanism in the backwards direction at a
 *     constant voltage
 * @param quasistaticForward A {@link Command} that runs the mechanism in the forward direction at
 *     an increasing voltage
 * @param quasistaticReverse A {@link Command} that runs the mechanism in the backwards direction at
 *     an increasing voltage
 * @see <a href =
 *     "https://docs.wpilib.org/en/stable/docs/software/advanced-controls/system-identification/introduction.html">SysID</a>
 */
public record SysIDCommands(
    Command dynamicForward,
    Command dynamicReverse,
    Command quasistaticForward,
    Command quasistaticReverse) {}
