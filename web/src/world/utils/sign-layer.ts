import type * as THREE from 'three';

import type { Letter, SignTrack, CartState } from '@/types';
import { isActionableSign } from '@/utils/signs';

import { approach } from './math';
import { carried } from './motion';
import type { Pose } from './road';
import { type SignColours, type SignRig, createSignRig } from './sign-rig';
import { LANE_WIDTH_M, SIGN_SETBACK_M, signPlace } from './units';

interface ShownSign {
  rig: SignRig;
  label: string;
  variant: Letter | null;
  track: SignTrack;
  x: number;
  z: number;
  alpha: number;
  isGone: boolean;
  lastSeen: number;
}

const BEHIND_GONE = -0.6;
const UNSEEN_GONE_S = 3;
const KERB = LANE_WIDTH_M / 2 + SIGN_SETBACK_M;

export class SignLayer {
  private readonly scene: THREE.Scene;
  private readonly colours: SignColours;
  private readonly signs = new Map<number, ShownSign>();

  constructor(scene: THREE.Scene, colours: SignColours) {
    this.scene = scene;
    this.colours = colours;
  }

  update(
    state: CartState | null,
    dt: number,
    now: number,
    ds: number,
    turn: number,
    nearestOnLane: (x: number, z: number) => Pose,
  ) {
    for (const shown of this.signs.values()) {
      if (Number.isNaN(shown.x)) continue;
      const moved = carried(shown.x, shown.z, ds, turn);
      shown.x = moved.x;
      shown.z = moved.z;
    }

    const seen = new Set<number>();
    for (const track of state?.signs.tracks ?? []) {
      const place = signPlace(track);
      if (!place || !isActionableSign(track)) continue;
      seen.add(track.id);

      const shown = this.shown(track, place.z, now);
      shown.track = track;
      shown.isGone = false;
      shown.lastSeen = now;

      const near = nearestOnLane(place.x, place.z);
      const x = near.x + KERB * Math.cos(near.angle);
      const z = near.z - KERB * Math.sin(near.angle);
      shown.x = Number.isNaN(shown.x) ? x : approach(shown.x, x, dt, 0.3);
      shown.z = approach(shown.z, z, dt, 0.25);
    }

    for (const [id, shown] of this.signs) {
      if (!seen.has(id) && (shown.z < BEHIND_GONE || now - shown.lastSeen > UNSEEN_GONE_S)) {
        shown.isGone = true;
      }
      shown.alpha = approach(shown.alpha, shown.isGone ? 0 : 1, dt, shown.isGone ? 0.25 : 0.12);
      if (shown.isGone && shown.alpha < 0.02) {
        this.remove(id, shown);
        continue;
      }
      shown.rig.group.position.set(shown.x, 0, -shown.z);
      shown.rig.setOpacity(shown.alpha);
      if (shown.track.label === 'traffic_light') shown.rig.setLight(shown.track.light);
    }
  }

  dispose() {
    for (const shown of this.signs.values()) shown.rig.dispose();
  }

  private shown(track: SignTrack, z: number, now: number) {
    const existing = this.signs.get(track.id);
    if (existing && existing.label === track.label && existing.variant === track.variant) {
      return existing;
    }
    if (existing) {
      this.scene.remove(existing.rig.group);
      existing.rig.dispose();
    }

    const rig = createSignRig(track.label, track.variant, this.colours);
    const created: ShownSign = {
      rig,
      label: track.label,
      variant: track.variant,
      track,
      x: existing?.x ?? Number.NaN,
      z,
      alpha: existing?.alpha ?? 0,
      isGone: false,
      lastSeen: now,
    };
    this.scene.add(rig.group);
    this.signs.set(track.id, created);
    return created;
  }

  private remove(id: number, shown: ShownSign) {
    this.scene.remove(shown.rig.group);
    shown.rig.dispose();
    this.signs.delete(id);
  }
}
