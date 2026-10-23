import * as THREE from 'three';
import { RoomEnvironment } from 'three/addons/environments/RoomEnvironment.js';

import type { Lane, CartState } from '@/types';

import { chaseCamera } from './camera';
import { createFloorGlow } from './floor-glow';
import { type ForkView, ForkWatch } from './fork-watch';
import { laneCentre } from './lane-fit';
import { LaneHeading, LaneReader, type LaneReading } from './lane-reader';
import { approach } from './math';
import { Trail, groundSpeed } from './motion';
import { type Palette, readPalette } from './palette';
import { pathLayer } from './path-layer';
import { type Pose, type RoadCurve, roadCurve } from './road';
import { roadLayer } from './road-layer';
import { SignLayer } from './sign-layer';
import { drawSonarTags, sonarLayer } from './sonar';
import { Strips } from './strips';
import { LANE_BEHIND_M, LANE_FADE_M, LANE_WIDTH_M } from './units';

interface LaneState {
  isKnown: boolean;
  centre: number;
  heading: number;
  curvature: number;
  anchor: Lane['anchor'];
}

const NEAREST_STEP = 0.1;

export class WorldScene {
  private readonly renderer: THREE.WebGLRenderer;
  private readonly overlay: HTMLCanvasElement;
  private readonly labels: CanvasRenderingContext2D;
  private readonly scene = new THREE.Scene();
  private readonly camera = new THREE.PerspectiveCamera(50, 1, 0.05, 60);
  private readonly palette: Palette = readPalette();
  private readonly colours;
  private readonly road = new Strips(900, 0.001, 1);
  private readonly marks = new Strips(1200, 0.004, 3);
  private readonly path = new Strips(64, 0.006, 4);
  private readonly sonar = new Strips(96, 0.008, 5);
  private readonly signs: SignLayer;
  private readonly reader = new LaneReader();
  private readonly crossing = new LaneHeading();
  private readonly trail = new Trail();
  private readonly forks = new ForkWatch();
  private readonly disposables: { dispose: () => void }[] = [];
  private size = { w: 1, h: 1, dpr: 1 };
  private lane: LaneState = { isKnown: false, centre: 0, heading: 0, curvature: 0, anchor: 'auto' };
  private speed = 0;
  private travelled = 0;
  private steering = 0;
  private mainAlpha = 0;
  private curve: RoadCurve = roadCurve({ centre: 0, heading: 0, curvature: 0 }, LANE_FADE_M);
  private lastRead: { lane: Lane; reading: LaneReading } | null = null;
  private fork: ForkView | null = null;

  constructor(canvas: HTMLCanvasElement, overlay: HTMLCanvasElement) {
    this.overlay = overlay;
    this.labels = overlay.getContext('2d')!;
    this.renderer = new THREE.WebGLRenderer({
      canvas,
      antialias: true,
      powerPreference: 'high-performance',
    });
    this.renderer.outputColorSpace = THREE.SRGBColorSpace;
    this.renderer.toneMapping = THREE.NeutralToneMapping;
    this.renderer.setClearColor(this.palette.world);

    const pmrem = new THREE.PMREMGenerator(this.renderer);
    this.scene.environment = pmrem.fromScene(new RoomEnvironment(), 0.04).texture;
    pmrem.dispose();
    this.scene.environmentIntensity = 0.9;

    const sun = new THREE.DirectionalLight('#ffffff', 1.4);
    sun.position.set(-2, 5, 4);
    const glow = createFloorGlow();
    this.disposables.push(...glow.disposables);
    this.scene.add(
      sun,
      new THREE.HemisphereLight('#ffffff', '#d9d5cc', 0.5),
      glow.mesh,
      this.road.mesh,
      this.marks.mesh,
      this.path.mesh,
      this.sonar.mesh,
    );

    const p = this.palette;
    this.colours = {
      road: new THREE.Color(p.road),
      kerb: new THREE.Color(p.ink),
      lost: new THREE.Color(p.stopSoft),
      marking: new THREE.Color(p.marking),
      stop: new THREE.Color(p.stop),
      warn: new THREE.Color(p.warn),
      accent: new THREE.Color(p.accent),
      manual: new THREE.Color(p.ink2),
    };
    this.signs = new SignLayer(this.scene, { stop: p.stop, warn: p.warn, ok: p.ok });
  }

  resize(w: number, h: number, dpr: number) {
    this.size = { w, h, dpr };
    this.renderer.setPixelRatio(dpr);
    this.renderer.setSize(w, h, false);
    this.overlay.width = Math.round(w * dpr);
    this.overlay.height = Math.round(h * dpr);

    const camera = chaseCamera(w, h);
    const [cx, cy] = camera.centre;
    const fullW = 2 * Math.max(cx, w - cx);
    const fullH = 2 * Math.max(cy, h - cy);
    this.camera.fov = THREE.MathUtils.radToDeg(2 * Math.atan(fullH / 2 / camera.focal));
    this.camera.aspect = fullW / fullH;
    this.camera.setViewOffset(fullW, fullH, fullW / 2 - cx, fullH / 2 - cy, w, h);
    this.camera.position.set(camera.position[0], camera.position[1], -camera.position[2]);
    this.camera.lookAt(camera.target[0], camera.target[1], -camera.target[2]);
    this.camera.updateProjectionMatrix();
  }

  frame(state: CartState | null, dt: number, now: number) {
    this.speed = approach(this.speed, groundSpeed(state), dt, 0.3);
    const ds = this.speed * dt;
    const turn = this.lane.isKnown ? this.lane.curvature * ds : 0;
    this.travelled += ds;

    this.updateLane(state, dt, now, ds, turn);
    this.updatePath(state, dt);
    this.signs.update(state, dt, now / 1000, ds, turn, (x, z) => this.nearestOnLane(x, z));
    const sonar = sonarLayer(state, this.colours, this.palette);
    this.sonar.set(sonar.strips);

    this.renderer.render(this.scene, this.camera);
    const { w, h, dpr } = this.size;
    this.labels.setTransform(dpr, 0, 0, dpr, 0, 0);
    this.labels.clearRect(0, 0, w, h);
    drawSonarTags(this.labels, chaseCamera(w, h), sonar.tags, this.palette);
  }

  dispose() {
    this.signs.dispose();
    for (const strips of [this.road, this.marks, this.path, this.sonar]) strips.dispose();
    for (const thing of this.disposables) thing.dispose();
    this.scene.environment?.dispose();
    this.renderer.dispose();
  }

  private nearestOnLane(x: number, z: number): Pose {
    if (!this.lane.isKnown) return { x: 0, z, angle: 0 };
    let best = this.curve(0);
    let bestDistance = Infinity;
    for (let s = 0; s <= LANE_FADE_M; s += NEAREST_STEP) {
      const point = this.curve(s);
      const distance = Math.hypot(point.x - x, point.z - z);
      if (distance < bestDistance) {
        bestDistance = distance;
        best = point;
      }
    }
    return best;
  }

  private updateLane(state: CartState | null, dt: number, now: number, ds: number, turn: number) {
    const lane = state?.lane ?? null;
    this.crossing.travel(ds);
    this.trail.advance(ds, turn, LANE_BEHIND_M);

    const isFresh = lane !== null && lane !== this.lastRead?.lane;
    if (isFresh) this.lastRead = { lane, reading: this.reader.read(lane) };
    const reading = lane ? this.lastRead!.reading : null;
    this.fork = this.forks.update(state, reading, dt, now / 1000, isFresh, ds);

    if (lane && reading) this.followLane(lane, reading, isFresh, dt);
    this.curve = roadCurve(this.lane, LANE_FADE_M);

    if (!this.lane.isKnown || !state) {
      this.mainAlpha = 0;
      this.trail.clear();
      this.road.set([]);
      this.marks.set([]);
      return;
    }

    this.mainAlpha = approach(this.mainAlpha, this.fork?.isTurning ? 1 : 0, dt, 0.3);
    const { roads, marks } = roadLayer({
      lane: this.lane,
      curve: this.curve,
      trail: this.trail,
      fork: this.fork,
      mainAlpha: this.mainAlpha,
      travelled: this.travelled,
      isLive: lane !== null,
      isLost: lane !== null && (lane.state.startsWith('LOST') || lane.offset_px === null),
      colours: this.colours,
    });
    this.road.set(roads);
    this.marks.set(marks);
  }

  private followLane(lane: Lane, reading: LaneReading, isFresh: boolean, dt: number) {
    const centre = lane.offset_px === null ? this.lane.centre : laneCentre(lane) * LANE_WIDTH_M;
    const heading =
      isFresh && lane.offset_px !== null ? this.crossing.read(centre) : this.lane.heading;
    const shape = reading.shape;
    if (!this.lane.isKnown) {
      this.lane = {
        ...this.lane,
        isKnown: true,
        centre,
        heading,
        curvature: shape?.curvature ?? 0,
      };
    }
    this.lane.centre = approach(this.lane.centre, centre, dt, 0.15);
    this.lane.heading = approach(this.lane.heading, heading, dt, 0.35);
    this.lane.curvature = approach(
      this.lane.curvature,
      shape ? shape.curvature : 0,
      dt,
      shape ? 0.8 : 1.2,
    );
    if (!lane.provisional) this.lane.anchor = lane.anchor;
  }

  private updatePath(state: CartState | null, dt: number) {
    this.steering = approach(this.steering, state?.drive.steering ?? 0, dt, 0.25);
    this.path.set(
      pathLayer({
        state,
        isLaneKnown: this.lane.isKnown,
        laneCentre: this.lane.centre,
        curve: this.curve,
        fork: this.fork,
        speed: this.speed,
        steering: this.steering,
        colours: this.colours,
      }),
    );
  }
}
