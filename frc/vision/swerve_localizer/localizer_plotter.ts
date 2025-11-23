// Provides a plot for debugging drivetrain-related issues.
import {AosPlotter} from '../../../aos/network/www/aos_plotter';
import * as proxy from '../../../aos/network/www/proxy';
import {BLUE, BROWN, CYAN, GREEN, PINK, RED, WHITE} from '../../../aos/network/www/colors';

import Connection = proxy.Connection;

const TIME = AosPlotter.TIME;
const DEFAULT_WIDTH = AosPlotter.DEFAULT_WIDTH;
const DEFAULT_HEIGHT = AosPlotter.DEFAULT_HEIGHT;

export function plotLocalizer(conn: Connection, element: Element): void {
  const aosPlotter = new AosPlotter(conn);

  const position = aosPlotter.addMessageSource("/drivetrain",
      "frc.control_loops.drivetrain.Position");
  const status = aosPlotter.addMessageSource(
      '/drivetrain', 'frc.control_loops.drivetrain.Status');
  const output = aosPlotter.addMessageSource(
      '/drivetrain', 'frc.control_loops.drivetrain.Output');
  const localizer = aosPlotter.addMessageSource(
      '/localizer', 'frc.vision.swerve_localizer.Status');
  const rio_inputs = aosPlotter.addMessageSource(
      '/drivetrain', 'frc.control_loops.drivetrain.RioLocalizerInputs');

  // Drivetrain Velocities
  const velocityPlot = aosPlotter.addPlot(element);
  velocityPlot.plot.getAxisLabels().setTitle('Velocity Plots');
  velocityPlot.plot.getAxisLabels().setXLabel(TIME);
  velocityPlot.plot.getAxisLabels().setYLabel('Wheel Velocity (m/s)');

  const leftVelocity =
      velocityPlot.addMessageLine(status, ['estimated_left_velocity']);
  leftVelocity.setColor(RED);
  const rightVelocity =
      velocityPlot.addMessageLine(status, ['estimated_right_velocity']);
  rightVelocity.setColor(GREEN);

  const leftSpeed = velocityPlot.addMessageLine(position, ["left_speed"]);
  leftSpeed.setColor(BLUE);
  const rightSpeed = velocityPlot.addMessageLine(position, ["right_speed"]);
  rightSpeed.setColor(BROWN);

  const ekfLeftVelocity = velocityPlot.addMessageLine(
      localizer, ['state', 'left_velocity']);
  ekfLeftVelocity.setColor(RED);
  ekfLeftVelocity.setPointSize(0.0);
  const ekfRightVelocity = velocityPlot.addMessageLine(
      localizer, ['state', 'right_velocity']);
  ekfRightVelocity.setColor(GREEN);
  ekfRightVelocity.setPointSize(0.0);

  // Lateral velocity
  const lateralPlot = aosPlotter.addPlot(element);
  lateralPlot.plot.getAxisLabels().setTitle('Lateral Velocity');
  lateralPlot.plot.getAxisLabels().setXLabel(TIME);
  lateralPlot.plot.getAxisLabels().setYLabel('Velocity (m/s)');

  lateralPlot.addMessageLine(localizer, ['state', 'lateral_velocity']).setColor(CYAN);

  // Drivetrain Voltage
  const voltagePlot = aosPlotter.addPlot(element);
  voltagePlot.plot.getAxisLabels().setTitle('Voltage Plots');
  voltagePlot.plot.getAxisLabels().setXLabel(TIME);
  voltagePlot.plot.getAxisLabels().setYLabel('Voltage (V)')

  voltagePlot.addMessageLine(localizer, ['state', 'left_voltage_error'])
      .setColor(RED)
      .setDrawLine(false);
  voltagePlot.addMessageLine(localizer, ['state', 'right_voltage_error'])
      .setColor(GREEN)
      .setDrawLine(false);
  voltagePlot.addMessageLine(output, ['left_voltage'])
      .setColor(RED)
      .setPointSize(0);
  voltagePlot.addMessageLine(output, ['right_voltage'])
      .setColor(GREEN)
      .setPointSize(0);
  voltagePlot.addMessageLine(rio_inputs, ['left_voltage'])
      .setColor(RED)
      .setDrawLine(false);
  voltagePlot.addMessageLine(rio_inputs, ['right_voltage'])
      .setColor(GREEN)
      .setDrawLine(false);

  // Heading
  const yawPlot = aosPlotter.addPlot(element);
  yawPlot.plot.getAxisLabels().setTitle('Robot Yaw');
  yawPlot.plot.getAxisLabels().setXLabel(TIME);
  yawPlot.plot.getAxisLabels().setYLabel('Yaw (rad)');

  yawPlot.addMessageLine(status, ['localizer', 'theta']).setColor(GREEN);

  yawPlot.addMessageLine(localizer, ['down_estimator', 'yaw']).setColor(BLUE);

  yawPlot.addMessageLine(localizer, ['state', 'theta']).setColor(RED);

  // Pitch/Roll
  const orientationPlot = aosPlotter.addPlot(element);
  orientationPlot.plot.getAxisLabels().setTitle('Orientation');
  orientationPlot.plot.getAxisLabels().setXLabel(TIME);
  orientationPlot.plot.getAxisLabels().setYLabel('Angle (rad)');

  orientationPlot.addMessageLine(localizer, ['down_estimator', 'lateral_pitch'])
      .setColor(RED)
      .setLabel('roll');
  orientationPlot
      .addMessageLine(localizer, ['down_estimator', 'longitudinal_pitch'])
      .setColor(GREEN)
      .setLabel('pitch');

  const stillPlot = aosPlotter.addPlot(element, [DEFAULT_WIDTH, DEFAULT_HEIGHT / 3]);
  stillPlot.plot.getAxisLabels().setTitle('Still Plot');
  stillPlot.plot.getAxisLabels().setXLabel(TIME);
  stillPlot.plot.getAxisLabels().setYLabel('bool, g\'s');
  stillPlot.plot.setDefaultYRange([-0.1, 1.1]);

  stillPlot.addMessageLine(localizer, ['down_estimator', 'gravity_magnitude'])
      .setColor(WHITE)
      .setDrawLine(false);

  // Absolute X Position
  const xPositionPlot = aosPlotter.addPlot(element);
  xPositionPlot.plot.getAxisLabels().setTitle('X Position');
  xPositionPlot.plot.getAxisLabels().setXLabel(TIME);
  xPositionPlot.plot.getAxisLabels().setYLabel('X Position (m)');

  xPositionPlot.addMessageLine(status, ['x']).setColor(RED);
  xPositionPlot.addMessageLine(localizer, ['down_estimator', 'position_x'])
      .setColor(BLUE);
  xPositionPlot.addMessageLine(localizer, ['state', 'x']).setColor(CYAN);

  // Absolute Y Position
  const yPositionPlot = aosPlotter.addPlot(element);
  yPositionPlot.plot.getAxisLabels().setTitle('Y Position');
  yPositionPlot.plot.getAxisLabels().setXLabel(TIME);
  yPositionPlot.plot.getAxisLabels().setYLabel('Y Position (m)');

  const localizerY = yPositionPlot.addMessageLine(status, ['y']);
  localizerY.setColor(RED);
  yPositionPlot.addMessageLine(localizer, ['down_estimator', 'position_y'])
      .setColor(BLUE);
  yPositionPlot.addMessageLine(localizer, ['state', 'y']).setColor(CYAN);
}
