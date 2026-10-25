// Top-down 2D field, drawn to scale in meters (WPILib blue-origin coordinates).
//
// Children are drawn in FIELD coordinates: x right (toward red), y up,
// angles counter-clockwise in radians — exactly like the robot code. The
// y-flip to screen coordinates happens once, here.
import { createContext, useCallback, useContext, useMemo, useRef, type ReactNode } from 'react';
import { FIELD, getTags, type FieldElement, type Variant } from './fieldData';
import { useNTFrame } from '../nt/store';

const MARGIN = 0.45; // meters of space around the field

interface FieldCtx { length: number; width: number; }
const Ctx = createContext<FieldCtx>({ length: 16.5, width: 8.0 });
export const useField = () => useContext(Ctx);

export interface FieldPointer {
  x: number;
  y: number;
  buttons: number;
  shift: boolean;
}

interface Props {
  variant: Variant;
  children?: ReactNode;
  showTagIds?: boolean;
  onPointerDown?: (p: FieldPointer) => void;
  onPointerMove?: (p: FieldPointer) => void;
  onPointerUp?: (p: FieldPointer) => void;
  cursor?: string;
}

export function FieldView({ variant, children, showTagIds = true, onPointerDown, onPointerMove, onPointerUp, cursor }: Props) {
  useNTFrame(); // tags arrive over NT
  const v = FIELD.variants[variant];
  const L = v.length, W = v.width;
  const flipRef = useRef<SVGGElement>(null);
  const tags = getTags(variant);

  const toField = useCallback((e: React.PointerEvent): FieldPointer | null => {
    const g = flipRef.current;
    const m = g?.getScreenCTM();
    if (!g || !m) return null;
    const pt = new DOMPoint(e.clientX, e.clientY).matrixTransform(m.inverse());
    return { x: pt.x, y: pt.y, buttons: e.buttons, shift: e.shiftKey };
  }, []);

  const handlers = {
    onPointerDown: (e: React.PointerEvent) => {
      const p = toField(e);
      if (p && onPointerDown) {
        try { (e.target as Element).setPointerCapture?.(e.pointerId); } catch { /* synthetic / inactive pointer */ }
        onPointerDown(p);
      }
    },
    onPointerMove: (e: React.PointerEvent) => { const p = toField(e); if (p) onPointerMove?.(p); },
    onPointerUp: (e: React.PointerEvent) => { const p = toField(e); if (p) onPointerUp?.(p); },
  };

  const ctx = useMemo(() => ({ length: L, width: W }), [L, W]);
  const start = FIELD.startingLineFromWallM;

  return (
    <svg
      className="field"
      viewBox={`${-MARGIN} ${-MARGIN} ${L + 2 * MARGIN} ${W + 2 * MARGIN}`}
      style={{ cursor }}
      {...handlers}
    >
      <defs>
        <pattern id="bump-hatch" width="0.12" height="0.12" patternUnits="userSpaceOnUse" patternTransform="rotate(45)">
          <rect width="0.12" height="0.12" fill="var(--bump-fill)" />
          <line x1="0" y1="0" x2="0" y2="0.12" stroke="var(--bump-line)" strokeWidth="0.035" />
        </pattern>
        <marker id="arrow" viewBox="0 0 10 10" refX="8" refY="5" markerWidth="5" markerHeight="5" orient="auto-start-reverse">
          <path d="M0,0 L10,5 L0,10 z" fill="context-stroke" />
        </marker>
      </defs>

      <Ctx.Provider value={ctx}>
        <g ref={flipRef} transform={`matrix(1 0 0 -1 0 ${W})`}>
          {/* carpet + zones */}
          <rect x={0} y={0} width={L} height={W} className="carpet" />
          <rect x={0} y={0} width={start} height={W} className="zone zone-blue" />
          <rect x={L - start} y={0} width={start} height={W} className="zone zone-red" />

          {/* starting lines, center line, alliance walls */}
          <line x1={start} y1={0} x2={start} y2={W} className="line line-blue" />
          <line x1={L - start} y1={0} x2={L - start} y2={W} className="line line-red" />
          <line x1={L / 2} y1={0} x2={L / 2} y2={W} className="line line-center" />
          <rect x={0} y={0} width={L} height={W} className="guardrail" />
          <line x1={0} y1={0} x2={0} y2={W} className="wall wall-blue" />
          <line x1={L} y1={0} x2={L} y2={W} className="wall wall-red" />

          {v.elements.map((e) => <Element key={e.id} el={e} />)}

          {/* AprilTags: short bar on the mounting face + tick showing which way it faces */}
          {tags.map((t) => {
            const c = Math.cos(t.yaw), s = Math.sin(t.yaw);
            const half = 0.0825; // 6.5 in tag
            return (
              <g key={t.id} className="tag">
                <title>{`Tag ${t.id}  x=${t.x.toFixed(3)} y=${t.y.toFixed(3)} z=${t.z.toFixed(3)} m  facing ${Math.round((t.yaw * 180) / Math.PI)}°`}</title>
                <line x1={t.x - s * half} y1={t.y + c * half} x2={t.x + s * half} y2={t.y - c * half} className="tag-face" />
                <line x1={t.x} y1={t.y} x2={t.x + c * 0.12} y2={t.y + s * 0.12} className="tag-dir" />
                {showTagIds && <FieldText x={t.x + c * 0.25} y={t.y + s * 0.25} className="tag-id">{t.id}</FieldText>}
              </g>
            );
          })}

          {children}
        </g>
      </Ctx.Provider>

      {tags.length === 0 && (
        <text x={L / 2} y={W / 2} className="field-hint" textAnchor="middle">
          AprilTags appear when the robot sim is running
        </text>
      )}
    </svg>
  );
}

function Element({ el }: { el: FieldElement }) {
  const pts = el.poly.map(([x, y]) => `${x},${y}`).join(' ');
  const xs = el.poly.map((p) => p[0]), ys = el.poly.map((p) => p[1]);
  const cx = (Math.min(...xs) + Math.max(...xs)) / 2;
  const { width: W } = useField();
  let cy = (Math.min(...ys) + Math.max(...ys)) / 2;
  // Trench tags sit mid-structure; nudge the label toward the field center
  if (el.type === 'low-clearance') cy += Math.sign(W / 2 - cy) * 0.45;
  return (
    <g className={`el el-${el.type} el-${el.alliance}`}>
      <title>{`${el.name} (${el.alliance}) — ${el.type}${el.heightM ? `, ${el.heightM.toFixed(2)} m` : ''}\n${el.cite}${el.approx ? '\n⚠ position approximate — verify on the field drawings' : ''}`}</title>
      <polygon points={pts} fill={el.type === 'bump' ? 'url(#bump-hatch)' : undefined} />
      {el.type !== 'marking' && <FieldText x={cx} y={cy} className="el-label">{el.name}</FieldText>}
      {el.type === 'marking' && <FieldText x={cx + (el.alliance === 'blue' ? 0.35 : -0.35)} y={cy} className="el-label el-label-small">{el.name}{el.approx ? '*' : ''}</FieldText>}
    </g>
  );
}

/** Upright text placed at a field coordinate (undoes the y-flip locally). */
export function FieldText({ x, y, children, className }: { x: number; y: number; children: ReactNode; className?: string }) {
  return (
    <text transform={`translate(${x} ${y}) scale(1 -1)`} className={className} textAnchor="middle" dominantBaseline="central">
      {children}
    </text>
  );
}
