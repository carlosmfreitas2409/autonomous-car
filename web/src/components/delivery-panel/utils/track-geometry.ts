import type { Letter, Mission, TrackLayout } from '@/types';

export const VIEW = { x: 30, y: -40, w: 1940, h: 1110 };

export const ROAD = 110;
const R = 190;
const TOP = 135;
const BOTTOM = 973;
const LEFT = 130;
const RIGHT = 1870;
const MID = 560;

const EXIT_X: Record<number, number> = { 1: 700, 2: 1220 };
const DETOUR_X: Record<number, number> = { 1: EXIT_X[1] + R, 2: EXIT_X[2] + R };

export function roads(layout: TrackLayout): Record<string, string> {
  const x1 = EXIT_X[1];
  const x2 = EXIT_X[2];
  const paths: Record<string, string> = {
    'main-2': `M ${x1} ${TOP} L ${x2} ${TOP}`,
    'main-1':
      `M ${x2} ${TOP} L ${RIGHT - R} ${TOP} A ${R} ${R} 0 0 1 ${RIGHT} ${TOP + R} ` +
      `L ${RIGHT} ${BOTTOM - R} A ${R} ${R} 0 0 1 ${RIGHT - R} ${BOTTOM} L ${x1} ${BOTTOM} ` +
      `A ${R} ${R} 0 0 1 ${x1 - R} ${BOTTOM - R} A ${R} ${R} 0 0 0 ${x1 - 2 * R} ${BOTTOM - 2 * R} ` +
      `A ${R} ${R} 0 0 1 ${LEFT} ${BOTTOM - 3 * R} L ${LEFT} ${TOP + R} A ${R} ${R} 0 0 1 ${LEFT + R} ${TOP} L ${x1} ${TOP}`,
  };
  for (const exit of layout.exits) {
    const x = EXIT_X[exit];
    paths[`detour-${exit}`] =
      `M ${x} ${TOP} A ${R} ${R} 0 0 1 ${x + R} ${TOP + R} ` +
      `L ${x + R} ${BOTTOM - R} A ${R} ${R} 0 0 1 ${x} ${BOTTOM}`;
  }
  return paths;
}

export function exitPoint(exit: number): { x: number; y: number } {
  return { x: EXIT_X[exit], y: TOP };
}

export function letterPositions(layout: TrackLayout): Record<Letter, { x: number; y: number }> {
  const out = {} as Record<Letter, { x: number; y: number }>;
  const onMain = (Object.keys(layout.letters) as Letter[]).filter(
    (l) => layout.letters[l] === null,
  );
  for (const letter of Object.keys(layout.letters) as Letter[]) {
    const exit = layout.letters[letter];
    if (exit !== null) {
      out[letter] = { x: DETOUR_X[exit], y: MID };
    } else {
      const i = onMain.indexOf(letter);
      const spread = onMain.length > 1 ? (i / (onMain.length - 1) - 0.5) * 240 : 0;
      out[letter] = { x: RIGHT, y: MID + spread };
    }
  }
  return out;
}

type Node =
  | 'start'
  | 'right'
  | `exit-${number}`
  | `mid-${number}`
  | `merge-${number}`
  | `letter-${Letter}`;
const START_X = 440;
const point = (x: number, y: number) => ({ x, y });

function nodePoint(node: Node, layout: TrackLayout): { x: number; y: number } {
  const [kind, arg] = node.split('-');
  const n = Number(arg);
  if (kind === 'start') return point(START_X, TOP);
  if (kind === 'right') return point(RIGHT, TOP + R);
  if (kind === 'exit') return point(EXIT_X[n], TOP);
  if (kind === 'mid') return point(DETOUR_X[n], MID);
  if (kind === 'merge') return point(EXIT_X[n], BOTTOM);
  return letterPositions(layout)[arg as Letter];
}

function mainLetters(layout: TrackLayout): Letter[] {
  const at = letterPositions(layout);
  return (Object.keys(layout.letters) as Letter[])
    .filter((l) => layout.letters[l] === null)
    .sort((a, b) => at[a].y - at[b].y);
}

function step(node: Node, target: Letter, layout: TrackLayout): [Node, string] {
  const [kind, arg] = node.split('-');
  const n = Number(arg);
  const main = mainLetters(layout);
  const downRight = (from: number): [Node, string] => {
    const next = main.find((l) => letterPositions(layout)[l].y > from);
    if (next) return [`letter-${next}`, `L ${RIGHT} ${letterPositions(layout)[next].y}`];
    return [
      'merge-2',
      `L ${RIGHT} ${BOTTOM - R} A ${R} ${R} 0 0 1 ${RIGHT - R} ${BOTTOM} L ${EXIT_X[2]} ${BOTTOM}`,
    ];
  };
  switch (kind) {
    case 'start':
      return ['exit-1', `L ${EXIT_X[1]} ${TOP}`];
    case 'exit':
      if (layout.letters[target] === n)
        return [`mid-${n}`, `A ${R} ${R} 0 0 1 ${DETOUR_X[n]} ${TOP + R} L ${DETOUR_X[n]} ${MID}`];
      return n === 1
        ? ['exit-2', `L ${EXIT_X[2]} ${TOP}`]
        : ['right', `L ${RIGHT - R} ${TOP} A ${R} ${R} 0 0 1 ${RIGHT} ${TOP + R}`];
    case 'mid':
      return [
        `merge-${n}`,
        `L ${DETOUR_X[n]} ${BOTTOM - R} A ${R} ${R} 0 0 1 ${EXIT_X[n]} ${BOTTOM}`,
      ];
    case 'right':
      return downRight(TOP + R);
    case 'letter':
      return downRight(letterPositions(layout)[arg as Letter].y);
    default:
      if (n === 2) return ['merge-1', `L ${EXIT_X[1]} ${BOTTOM}`];
      return [
        'exit-1',
        `A ${R} ${R} 0 0 1 ${EXIT_X[1] - R} ${BOTTOM - R} A ${R} ${R} 0 0 0 ${EXIT_X[1] - 2 * R} ${BOTTOM - 2 * R} ` +
          `A ${R} ${R} 0 0 1 ${LEFT} ${BOTTOM - 3 * R} L ${LEFT} ${TOP + R} A ${R} ${R} 0 0 1 ${LEFT + R} ${TOP} L ${EXIT_X[1]} ${TOP}`,
      ];
  }
}

const letterNode = (letter: Letter, layout: TrackLayout): Node =>
  layout.letters[letter] === null ? `letter-${letter}` : `mid-${layout.letters[letter]}`;

function walk(layout: TrackLayout, from: Letter | null, target: Letter): [Node, string][] | null {
  let node: Node = from ? letterNode(from, layout) : 'start';
  const goal = letterNode(target, layout);
  const steps: [Node, string][] = [];
  for (let guard = 0; node !== goal && guard < 24; guard++) {
    const [next, d] = step(node, target, layout);
    steps.push([node, d]);
    node = next;
  }
  return node === goal ? steps : null;
}

export function legNodes(
  layout: TrackLayout,
  from: Letter | null,
  target: Letter,
): string[] | null {
  return (
    walk(layout, from, target)
      ?.map(([n]) => n as string)
      .concat(letterNode(target, layout)) ?? null
  );
}

export function legPath(
  layout: TrackLayout,
  from: Letter | null,
  target: Letter,
  skip = 0,
  outOfDetour = false,
): string | null {
  const steps = walk(layout, from, target);
  if (!steps || steps.length === 0) return null;
  let first = 0;
  if (skip > 0) {
    const exits = steps.flatMap(([n], i) => (n.startsWith('exit-') ? [i] : []));
    if (exits.length === 0) return null;
    first = exits[Math.min(skip, exits.length) - 1];
    if (outOfDetour && steps[first + 1]?.[0].startsWith('mid-')) {
      const merge = steps.findIndex(([n], i) => i > first && n.startsWith('merge-'));
      if (merge >= 0) first = merge;
    }
  }
  const p = nodePoint(steps[first][0], layout);
  return (
    `M ${p.x} ${p.y} ` +
    steps
      .slice(first)
      .map(([, d]) => d)
      .join(' ')
  );
}

export function routeLegs(
  layout: TrackLayout,
  mission: Mission,
): { current: string | null; next: string | null } {
  const { pickup, delivery } = mission;
  const none = { current: null, next: null };
  if (!pickup || mission.stage === 'done') return none;
  const toPickup = mission.stage !== 'delivery';
  const toDelivery = !!delivery && delivery !== pickup;
  const pickupAt = mission.route.findIndex((s) => s.endsWith(': pickup'));
  const from = toPickup ? 0 : pickupAt + 1;
  const passed = mission.route
    .slice(from, Math.max(from, mission.step))
    .filter((s) => s.startsWith('exit ')).length;
  const outOfDetour = mission.segment?.kind === 'main';
  if (toPickup) {
    return {
      current: legPath(layout, null, pickup, passed, outOfDetour),
      next: toDelivery ? legPath(layout, pickup, delivery!) : null,
    };
  }
  return {
    current: delivery ? legPath(layout, pickup, delivery, passed, outOfDetour) : null,
    next: null,
  };
}
