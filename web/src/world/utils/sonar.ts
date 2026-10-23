import type * as THREE from 'three';

import type { CartState } from '@/types';

import { type Camera, project } from './camera';
import type { Palette } from './palette';
import type { Strip, StripPoint } from './strips';
import { LANE_REAL_M, SONAR_BEARINGS_DEG } from './units';

export type SonarZone = 'clear' | 'slow' | 'stop';

interface SonarTag {
  text: string;
  x: number;
  z: number;
  dx: number;
  color: string;
}

const MAX_REACH = 5;
const ARC_STEPS = 8;
const ARC_STEP_DEG = 1.4;
const TAG_OFFSET_PX = 28;
const TOP_PX = 16;

export function sonarZone(distanceCm: number | null, slowCm: number, stopCm: number): SonarZone {
  if (distanceCm === null) return 'clear';
  if (distanceCm <= stopCm) return 'stop';
  if (distanceCm <= slowCm) return 'slow';
  return 'clear';
}

export function sonarLayer(
  state: CartState | null,
  colours: { stop: THREE.Color; warn: THREE.Color },
  palette: Palette,
) {
  const strips: Strip[] = [];
  const tags: SonarTag[] = [];
  const obstacle = state?.obstacle;
  if (!obstacle || obstacle.stale) return { strips, tags };

  obstacle.distances_cm.forEach((distance, i) => {
    const zone = sonarZone(distance, obstacle.slow_cm[i] ?? 100, obstacle.stop_cm);
    if (distance === null || zone === 'clear') return;

    const isStop = zone === 'stop';
    const bearing = SONAR_BEARINGS_DEG[i] ?? 0;
    const reach = Math.min(distance / 100 / LANE_REAL_M, MAX_REACH);
    const points: StripPoint[] = [];
    for (let k = -ARC_STEPS; k <= ARC_STEPS; k++) {
      const angle = ((bearing + k * ARC_STEP_DEG) * Math.PI) / 180;
      points.push({ x: reach * Math.sin(angle), z: reach * Math.cos(angle), a: 0.92 });
    }
    strips.push({
      points,
      half: isStop ? 0.035 : 0.025,
      color: isStop ? colours.stop : colours.warn,
    });

    const angle = (bearing * Math.PI) / 180;
    tags.push({
      text: `${distance} cm`,
      x: reach * Math.sin(angle),
      z: reach * Math.cos(angle),
      dx: Math.sign(bearing) * TAG_OFFSET_PX,
      color: isStop ? palette.stopInk : palette.warnInk,
    });
  });

  return { strips, tags };
}

export function drawSonarTags(
  context: CanvasRenderingContext2D,
  camera: Camera,
  tags: SonarTag[],
  palette: Palette,
) {
  context.font = `500 12px ${palette.sans}`;
  context.textAlign = 'center';
  context.textBaseline = 'middle';

  for (const tag of tags) {
    const point = project(camera, tag.x, tag.z);
    if (!point) continue;

    const x = Math.round(point.x + tag.dx);
    const y = Math.max(TOP_PX, Math.round(point.y - 12));
    const width = Math.round(context.measureText(tag.text).width + 14);
    context.beginPath();
    context.roundRect(x - width / 2 + 0.5, y - 9.5, width, 19, 6);
    context.fillStyle = palette.panel;
    context.fill();
    context.strokeStyle = palette.line;
    context.lineWidth = 1;
    context.stroke();
    context.fillStyle = tag.color;
    context.fillText(tag.text, x, y + 0.5);
  }
}
