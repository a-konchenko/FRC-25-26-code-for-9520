import { useEffect, useState } from 'react';
import { RunTab } from './tabs/RunTab/RunTab';
import { PlanTab } from './tabs/PlanTab/PlanTab';
import { FIELDS, VARIANT_LABEL, isVariant, type Variant } from './field/fieldData';
import { setSetting, useConnected, useNT } from './nt/store';
import { load, save } from './util/storage';
import type { RunKind } from './pathplanner/plan';

export interface RunRequest { kind: RunKind; name: string; nonce: number; }

type TabId = 'plan' | 'run';

export function App() {
  const [tab, setTab] = useState<TabId>(() => load('tab', 'run' as TabId));
  const [variant, setVariant] = useState<Variant>(() => {
    const v = load<unknown>('variant', 'andymark');
    return isVariant(v) ? v : 'andymark';
  });
  useEffect(() => save('tab', tab), [tab]);
  useEffect(() => save('variant', variant), [variant]);
  const field = FIELDS[variant];
  // The robot sim runs the same field (walls, FUEL, tags the cameras see): tell it which one
  useEffect(() => { setSetting('field', field.simField); }, [field.simField]);

  const [runRequest, setRunRequest] = useState<RunRequest | null>(null);
  const runFromPlan = (kind: RunKind, name: string) => {
    setRunRequest({ kind, name, nonce: Date.now() });
    setTab('run');
  };

  const connected = useConnected();
  const codeVariant = useNT<string>('/Sim/Field/robotCodeVariant', 'andymark');
  const simField = useNT<string>('/Sim/Field/active', '2026');

  return (
    <div className="app">
      <header className="topbar">
        <div className="brand">
          <span className="brand-num">9520</span>
          <span className="brand-name">Autonomy Simulator</span>
          <span className="muted small">{field.game}</span>
        </div>
        <nav className="tabs">
          <button className={tab === 'plan' ? 'active' : ''} onClick={() => setTab('plan')}>Plan</button>
          <button className={tab === 'run' ? 'active' : ''} onClick={() => setTab('run')}>Run</button>
        </nav>
        <div className="topbar-right">
          <label className="small" title="The two official 2026 field builds (slightly different tag positions), or our practice space B8 (15 × 11 ft, no targets yet)">
            Field&nbsp;
            <select value={variant} onChange={(e) => { if (isVariant(e.target.value)) setVariant(e.target.value); }}>
              {(Object.keys(FIELDS) as Variant[]).map((v) => <option key={v} value={v}>{VARIANT_LABEL[v]}</option>)}
            </select>
          </label>
          {field.hasTags && variant !== codeVariant && <span className="warn small" title="VisionSubsystem loads the AndyMark layout">≠ robot code ({codeVariant})</span>}
          {!field.hasTags && <span className="muted small" title="No AprilTags or field elements on B8 yet: cameras see nothing, no FUEL is staged">no targets yet</span>}
          {connected && simField !== field.simField && <span className="warn small" title="The robot sim is still on another field (an older sim build ignores the Field choice)">robot sim on {simField}</span>}
          <span className={`conn ${connected ? 'conn-on' : 'conn-off'}`}>{connected ? '● robot sim connected' : '○ robot sim not running'}</span>
        </div>
      </header>
      {/* Both tabs stay mounted so a recorded run survives switching tabs */}
      <main>
        <div className="tab-pane" hidden={tab !== 'plan'}><PlanTab variant={variant} onRun={runFromPlan} /></div>
        <div className="tab-pane" hidden={tab !== 'run'}><RunTab variant={variant} runRequest={runRequest} /></div>
      </main>
    </div>
  );
}
