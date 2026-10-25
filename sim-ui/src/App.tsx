import { useEffect, useState } from 'react';
import { RunTab } from './tabs/RunTab/RunTab';
import { PlanTab } from './tabs/PlanTab/PlanTab';
import type { Variant } from './field/fieldData';
import { FIELD } from './field/fieldData';
import { useConnected, useNT } from './nt/store';
import { load, save } from './util/storage';
import type { RunKind } from './pathplanner/plan';

export interface RunRequest { kind: RunKind; name: string; nonce: number; }

type TabId = 'plan' | 'run';

export function App() {
  const [tab, setTab] = useState<TabId>(() => load('tab', 'run' as TabId));
  const [variant, setVariant] = useState<Variant>(() => load('variant', 'andymark' as Variant));
  useEffect(() => save('tab', tab), [tab]);
  useEffect(() => save('variant', variant), [variant]);

  const [runRequest, setRunRequest] = useState<RunRequest | null>(null);
  const runFromPlan = (kind: RunKind, name: string) => {
    setRunRequest({ kind, name, nonce: Date.now() });
    setTab('run');
  };

  const connected = useConnected();
  const codeVariant = useNT<string>('/Sim/Field/robotCodeVariant', 'andymark');

  return (
    <div className="app">
      <header className="topbar">
        <div className="brand">
          <span className="brand-num">9520</span>
          <span className="brand-name">Autonomy Simulator</span>
          <span className="muted small">{FIELD.game}</span>
        </div>
        <nav className="tabs">
          <button className={tab === 'plan' ? 'active' : ''} onClick={() => setTab('plan')}>Plan</button>
          <button className={tab === 'run' ? 'active' : ''} onClick={() => setTab('run')}>Run</button>
        </nav>
        <div className="topbar-right">
          <label className="small" title="Two official field builds with slightly different tag positions">
            Field&nbsp;
            <select value={variant} onChange={(e) => setVariant(e.target.value as Variant)}>
              <option value="andymark">AndyMark</option>
              <option value="welded">Welded</option>
            </select>
          </label>
          {variant !== codeVariant && <span className="warn small" title="VisionSubsystem loads the AndyMark layout">≠ robot code ({codeVariant})</span>}
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
