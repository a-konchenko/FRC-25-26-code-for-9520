// Auto builder: an .auto is a tree of command groups (sequential / parallel /
// race / deadline) holding path, named and wait commands.
import type { PPAuto, PPCommand, PPCommandType } from '../../pathplanner/types';
import { commandNames } from '../../pathplanner/types';

interface Props {
  auto: PPAuto;
  update: (a: PPAuto, pushHistory?: boolean) => void;
  pathNames: string[];
  namedOptions: string[];
  connected: boolean;
}

const GROUPS: PPCommandType[] = ['sequential', 'parallel', 'race', 'deadline'];
const GROUP_HELP: Record<string, string> = {
  sequential: 'one after another',
  parallel: 'all at once, until all finish',
  race: 'all at once, until any finishes',
  deadline: 'all at once, until the FIRST one finishes',
};

export function AutoPanel({ auto, update, pathNames, namedOptions, connected }: Props) {
  const used = commandNames(auto.command);
  const unknownNamed = connected ? used.filter((n) => !namedOptions.includes(n)) : [];
  const missingPaths = collectPaths(auto.command).filter((p) => !pathNames.includes(p));
  const names = [...new Set([...namedOptions, ...used])].sort();

  const setRoot = (cmd: PPCommand) => update({ ...auto, command: cmd });

  return (
    <>
      {(unknownNamed.length > 0 || missingPaths.length > 0) && (
        <section className="panel">
          <h3>Warnings</h3>
          {missingPaths.map((p) => <p key={p} className="warn small">⚠ Path “{p}” doesn't exist.</p>)}
          {unknownNamed.map((n) => <p key={n} className="warn small">⚠ Named command “{n}” is not registered in RobotContainer.</p>)}
        </section>
      )}
      <section className="panel">
        <h3>Auto</h3>
        <label className="toggle">
          <input type="checkbox" checked={auto.resetOdom} onChange={(e) => update({ ...auto, resetOdom: e.target.checked })} />
          <span>Reset odometry to the first path's start</span>
        </label>
        <p className="muted tiny">On: the robot code assumes it starts exactly at the first path's start pose. Off: it keeps whatever pose it had.</p>
      </section>
      <section className="panel">
        <h3>Steps</h3>
        <CommandNode cmd={auto.command} onChange={setRoot} pathNames={pathNames} names={names} depth={0} />
      </section>
    </>
  );
}

function CommandNode({ cmd, onChange, onDelete, onMove, pathNames, names, depth }: {
  cmd: PPCommand;
  onChange: (c: PPCommand) => void;
  onDelete?: () => void;
  onMove?: (dir: -1 | 1) => void;
  pathNames: string[];
  names: string[];
  depth: number;
}) {
  const isGroup = GROUPS.includes(cmd.type);
  const children = cmd.data.commands ?? [];
  const setChildren = (cs: PPCommand[]) => onChange({ ...cmd, data: { ...cmd.data, commands: cs } });
  const addChild = (c: PPCommand) => setChildren([...children, c]);

  const changeType = (type: PPCommandType) => {
    if (GROUPS.includes(type)) onChange({ type, data: { commands: isGroup ? children : [cmd] } });
    else if (type === 'path') onChange({ type, data: { pathName: pathNames[0] ?? '' } });
    else if (type === 'named') onChange({ type, data: { name: names[0] ?? '' } });
    else onChange({ type: 'wait', data: { waitTime: 1.0 } });
  };

  return (
    <div className={`cmd cmd-${cmd.type} ${depth === 0 ? 'cmd-root' : ''}`}>
      <div className="cmd-row">
        <select className="cmd-type" value={cmd.type} onChange={(e) => changeType(e.target.value as PPCommandType)}>
          <optgroup label="Group">{GROUPS.map((g) => <option key={g} value={g}>{g}</option>)}</optgroup>
          <optgroup label="Step">
            <option value="path">path</option>
            <option value="named">named</option>
            <option value="wait">wait</option>
          </optgroup>
        </select>
        {cmd.type === 'path' && (
          <select className="grow" value={cmd.data.pathName ?? ''} onChange={(e) => onChange({ ...cmd, data: { ...cmd.data, pathName: e.target.value } })}>
            {!pathNames.includes(cmd.data.pathName ?? '') && <option value={cmd.data.pathName}>{cmd.data.pathName} (missing)</option>}
            {pathNames.map((p) => <option key={p} value={p}>{p}</option>)}
          </select>
        )}
        {cmd.type === 'named' && (
          <select className="grow" value={cmd.data.name ?? ''} onChange={(e) => onChange({ ...cmd, data: { ...cmd.data, name: e.target.value } })}>
            {names.map((n) => <option key={n} value={n}>{n}</option>)}
          </select>
        )}
        {cmd.type === 'wait' && (
          <>
            <input type="number" className="num" step={0.1} min={0} value={cmd.data.waitTime ?? 0}
              onChange={(e) => onChange({ ...cmd, data: { ...cmd.data, waitTime: Number(e.target.value) } })} />
            <span className="small">s</span>
          </>
        )}
        {isGroup && <span className="grow" />}
        {onMove && <button className="small-btn" onClick={() => onMove(-1)} title="Move up">↑</button>}
        {onMove && <button className="small-btn" onClick={() => onMove(1)} title="Move down">↓</button>}
        {onDelete && <button className="small-btn danger" onClick={onDelete} title="Remove">✕</button>}
      </div>
      {isGroup && <div className="muted tiny cmd-help">{GROUP_HELP[cmd.type]}</div>}
      {isGroup && (
        <div className="cmd-children">
          {children.map((c, i) => (
            <CommandNode key={i} cmd={c} depth={depth + 1} pathNames={pathNames} names={names}
              onChange={(nc) => setChildren(children.map((x, j) => (j === i ? nc : x)))}
              onDelete={() => setChildren(children.filter((_, j) => j !== i))}
              onMove={(dir) => {
                const j = i + dir;
                if (j < 0 || j >= children.length) return;
                const cs = [...children];
                [cs[i], cs[j]] = [cs[j], cs[i]];
                setChildren(cs);
              }} />
          ))}
          <div className="cmd-add">
            <button className="small-btn" onClick={() => addChild({ type: 'path', data: { pathName: pathNames[0] ?? '' } })}>＋ path</button>
            <button className="small-btn" onClick={() => addChild({ type: 'named', data: { name: names[0] ?? '' } })}>＋ named</button>
            <button className="small-btn" onClick={() => addChild({ type: 'wait', data: { waitTime: 1.0 } })}>＋ wait</button>
            <button className="small-btn" onClick={() => addChild({ type: 'parallel', data: { commands: [] } })}>＋ group</button>
          </div>
        </div>
      )}
    </div>
  );
}

function collectPaths(cmd: PPCommand): string[] {
  if (cmd.type === 'path') return cmd.data.pathName ? [cmd.data.pathName] : [];
  return (cmd.data.commands ?? []).flatMap(collectPaths);
}
