// Robot settings — edits deploy/pathplanner/settings.json, the same file the
// PathPlanner app's Settings page writes. PathPlannerLib reads it on the robot
// (RobotConfig.fromGUISettings) to plan feasible trajectories.
import { Fragment } from 'react';
import type { PPSettings } from '../../pathplanner/types';
import { FIELD } from '../../field/fieldData';
import { Num } from './PathPanel';

const IN = 39.3701;

// Values in settings.json that are still guesses (see SimRobotConstants.java)
const TO_MEASURE: (keyof PPSettings)[] = ['robotMass', 'robotMOI', 'wheelCOF', 'driveMotorType', 'driveCurrentLimit', 'robotWidth', 'robotLength'];

const FIELDS: { key: keyof PPSettings; label: string; step: number; unit: string }[] = [
  { key: 'robotLength', label: 'Length with bumpers', step: 0.01, unit: 'm' },
  { key: 'robotWidth', label: 'Width with bumpers', step: 0.01, unit: 'm' },
  { key: 'robotMass', label: 'Mass (with battery, bumpers)', step: 0.5, unit: 'kg' },
  { key: 'robotMOI', label: 'Moment of inertia', step: 0.1, unit: 'kg·m²' },
  { key: 'wheelCOF', label: 'Wheel friction (COF)', step: 0.05, unit: '' },
  { key: 'driveWheelRadius', label: 'Wheel radius', step: 0.0005, unit: 'm' },
  { key: 'driveGearing', label: 'Drive gearing', step: 0.01, unit: ':1' },
  { key: 'maxDriveSpeed', label: 'Max drive speed', step: 0.05, unit: 'm/s' },
  { key: 'driveCurrentLimit', label: 'Drive current limit', step: 1, unit: 'A' },
  { key: 'defaultMaxVel', label: 'Default max velocity (new paths)', step: 0.1, unit: 'm/s' },
  { key: 'defaultMaxAccel', label: 'Default max accel (new paths)', step: 0.1, unit: 'm/s²' },
];

const MOTORS = ['krakenX60', 'krakenX60FOC', 'falcon500', 'falcon500FOC', 'vortex', 'NEO', 'CIM', 'miniCIM'];

export function SettingsPanel({ settings, update, simSize }: { settings: PPSettings; update: (s: PPSettings) => void; simSize: number[] }) {
  const set = (k: keyof PPSettings, v: unknown) => update({ ...settings, [k]: v });
  const sizeMismatch = simSize.length >= 2 &&
    (Math.abs(simSize[0] - settings.robotLength) > 0.01 || Math.abs(simSize[1] - settings.robotWidth) > 0.01);
  const rules = FIELD.robotRules;

  return (
    <>
      <section className="panel">
        <h3>Robot (PathPlanner)</h3>
        <div className="form-grid">
          {FIELDS.map((f) => (
            <Fragment key={f.key}>
              <label className={TO_MEASURE.includes(f.key) ? 'to-measure' : ''} title={TO_MEASURE.includes(f.key) ? 'Placeholder — TO MEASURE' : ''}>
                {f.label}{f.unit ? ` (${f.unit})` : ''}{TO_MEASURE.includes(f.key) ? ' *' : ''}
              </label>
              <Num value={settings[f.key] as number} step={f.step} onChange={(v) => set(f.key, v)} />
            </Fragment>
          ))}
          <label className="to-measure">Drive motor *</label>
          <select value={settings.driveMotorType} onChange={(e) => set('driveMotorType', e.target.value)}>
            {MOTORS.map((m) => <option key={m} value={m}>{m}</option>)}
          </select>
        </div>
        <p className="muted tiny">* placeholder — measure the robot. Module positions come from TunerConstants (±0.292 m, ±0.254 m).</p>
      </section>
      <section className="panel">
        <h3>Checks</h3>
        <p className="small">{(settings.robotLength * IN).toFixed(1)} × {(settings.robotWidth * IN).toFixed(1)} in with bumpers.</p>
        <p className="muted tiny">Rules: frame perimeter ≤ {(rules.maxPerimeterM * IN).toFixed(0)} in (R104), bumpers ≤ {(rules.maxBumperExtensionM * IN).toFixed(1)} in past the frame (R403), height ≤ {(rules.maxHeightM * IN).toFixed(0)} in.</p>
        {sizeMismatch && (
          <p className="warn small">⚠ The simulator draws the robot {(simSize[0] * IN).toFixed(1)} × {(simSize[1] * IN).toFixed(1)} in (SimRobotConstants.java). Keep both in sync.</p>
        )}
        <p className="muted tiny">Changes apply to the next Run and preview (the robot sim re-reads this file). Restart the sim to update the auto chooser.</p>
      </section>
    </>
  );
}
