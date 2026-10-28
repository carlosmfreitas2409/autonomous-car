import type { CartState } from '@/types';

import type { LaneReading } from './lane-reader';
import { approach } from './math';
import { LANE_WIDTH_M, signPlace } from './units';

export type Branch = 'detour' | 'straight';

export const BRANCH_CURVATURE = 0.7;
export const BRANCH_TURN = 1.35;

const BRANCH_RADIUS = 1 / BRANCH_CURVATURE;
const NOSE_ANGLE = Math.acos(
  (BRANCH_RADIUS - LANE_WIDTH_M / 2) / (BRANCH_RADIUS + LANE_WIDTH_M / 2),
);

export const NOSE_ALONG_BRANCH = BRANCH_RADIUS * NOSE_ANGLE;
export const NOSE_ALONG_MAIN = (BRANCH_RADIUS + LANE_WIDTH_M / 2) * Math.sin(NOSE_ANGLE);

export interface ForkView {
  at: number;
  take: Branch;
  alpha: number;
  isTurning: boolean;
}

const ARM_M = 4;
const SEEN_FRAMES = 2;
const PAST_S = 0.6;
const PAST_MAX_S = 4;
const PAST_M = 2.0;
const GIVE_UP_S = 8;

export function branchAt(state: CartState, sign: number | null): Branch {
  if (state.path.path === 'detour') return 'detour';
  const decided = state.mission.decided;
  if (sign !== null && decided && decided.sign === sign) return 'straight';
  return state.mission.next?.take ?? 'straight';
}

export class ForkWatch {
  private sign: number | null = null;
  private isByPath = false;
  private lastSign = -Infinity;
  private readonly done = new Set<number>();
  private isPlaced = false;
  private isSeen = false;
  private seenAt = 0;
  private streak = 0;
  private whole = 0;
  private z = 0;
  private alpha = 0;
  private take: Branch = 'straight';
  private isTurning = false;

  update(
    state: CartState | null,
    reading: LaneReading | null,
    dt: number,
    now: number,
    isFresh = true,
    ds = 0,
  ): ForkView | null {
    if (!state) {
      this.disarm();
      this.alpha = 0;
      return null;
    }

    const nearest = this.nearestSign(state);
    const isWaiting = state.path.path === 'detour' && state.path.detour_stage.startsWith('waiting');
    if (nearest) {
      if (!this.isArmed) this.restart(nearest.z);
      else if (this.sign !== null && this.sign !== nearest.id) this.done.add(this.sign);
      if (!this.isPlaced) this.z = nearest.z;
      this.sign = nearest.id;
      this.isPlaced = true;
      this.lastSign = now;
    }
    if (isWaiting) {
      if (!this.isArmed) this.restart(null);
      this.isByPath = true;
    }

    if (this.isArmed) this.track(state, reading, nearest, isWaiting, dt, now, isFresh, ds);
    else this.z -= ds;

    const isShown = this.isArmed && this.isPlaced;
    this.alpha = approach(this.alpha, isShown && state.lane ? 1 : 0, dt, 0.25);
    if (this.alpha < 0.02 && !isShown) return null;
    return { at: this.z, take: this.take, alpha: this.alpha, isTurning: this.isTurning };
  }

  private get isArmed() {
    return this.sign !== null || this.isByPath;
  }

  private nearestSign(state: CartState) {
    let nearest: { id: number; z: number } | null = null;
    for (const track of state.signs.tracks) {
      if (track.label !== 'detour' || !track.confirmed || track.rejected) continue;
      if (this.done.has(track.id)) continue;
      const place = signPlace(track);
      if (!place || track.distance_m! > ARM_M || (nearest && place.z >= nearest.z)) continue;
      nearest = { id: track.id, z: place.z };
    }
    return nearest;
  }

  private track(
    state: CartState,
    reading: LaneReading | null,
    nearest: { id: number; z: number } | null,
    isWaiting: boolean,
    dt: number,
    now: number,
    isFresh: boolean,
    ds: number,
  ) {
    const opening = reading?.opening ?? null;
    if (opening !== null) {
      if (isFresh) this.streak += 1;
      this.whole = 0;
      if (this.streak >= SEEN_FRAMES && !this.isSeen) {
        this.isSeen = true;
        this.seenAt = now;
        if (!this.isPlaced) this.z = opening * LANE_WIDTH_M;
        this.isPlaced = true;
      }
    } else {
      if (isFresh) this.streak = 0;
      if (this.isSeen) this.whole += dt;
    }

    const isBeside =
      opening === null || (reading?.near !== null && opening <= reading!.near! + 0.02);
    if (this.isSeen && !isBeside) {
      this.z = approach(this.z - ds, opening * LANE_WIDTH_M, dt, 0.25);
    } else if (!this.isSeen && nearest) {
      this.z = approach(this.z - ds, nearest.z, dt, 0.3);
    } else {
      this.z -= ds;
    }
    this.take = branchAt(state, this.sign);
    this.isTurning = this.isSeen && state.lane?.anchor === 'right';

    const isDone = this.isTurning ? -this.z >= PAST_M : this.whole >= PAST_S;
    const isPast = this.isSeen && (isDone || now - this.seenAt >= PAST_MAX_S);
    const isMissed =
      !this.isSeen && (this.isByPath ? !isWaiting && !nearest : now - this.lastSign >= GIVE_UP_S);
    if (isPast || isMissed) this.disarm();
  }

  private restart(z: number | null) {
    this.isSeen = false;
    this.streak = 0;
    this.whole = 0;
    this.isTurning = false;
    this.isPlaced = z !== null;
    this.z = z ?? 0;
  }

  private disarm() {
    if (this.sign !== null) this.done.add(this.sign);
    this.sign = null;
    this.isByPath = false;
  }
}
