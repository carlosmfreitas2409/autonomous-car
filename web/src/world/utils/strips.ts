import * as THREE from 'three';

export interface StripPoint {
  x: number;
  z: number;
  a: number;
  half?: number;
}

export interface Strip {
  points: StripPoint[];
  half: number;
  color: THREE.Color;
}

export class Strips {
  readonly mesh: THREE.Mesh;
  private readonly capacity: number;
  private readonly geometry = new THREE.BufferGeometry();
  private readonly position: THREE.BufferAttribute;
  private readonly colour: THREE.BufferAttribute;
  private readonly index: THREE.BufferAttribute;

  constructor(capacity: number, lift: number, order: number) {
    this.capacity = capacity;
    this.position = dynamic(new Float32Array(capacity * 6), 3);
    this.colour = dynamic(new Float32Array(capacity * 8), 4);
    this.index = dynamic(new Uint16Array(capacity * 6), 1);
    this.geometry.setAttribute('position', this.position);
    this.geometry.setAttribute('color', this.colour);
    this.geometry.setIndex(this.index);
    this.geometry.setDrawRange(0, 0);

    const material = new THREE.MeshBasicMaterial({
      vertexColors: true,
      transparent: true,
      depthWrite: false,
      side: THREE.DoubleSide,
      toneMapped: false,
    });
    this.mesh = new THREE.Mesh(this.geometry, material);
    this.mesh.position.y = lift;
    this.mesh.renderOrder = order;
    this.mesh.frustumCulled = false;
  }

  set(strips: Strip[]) {
    const positions = this.position.array as Float32Array;
    const colours = this.colour.array as Float32Array;
    const indices = this.index.array as Uint16Array;
    let vertex = 0;
    let count = 0;

    for (const strip of strips) {
      const n = strip.points.length;
      if (n < 2 || vertex + n > this.capacity) continue;
      const { r, g, b } = strip.color;

      for (let k = 0; k < n; k++) {
        const point = strip.points[k];
        const before = strip.points[Math.max(0, k - 1)];
        const after = strip.points[Math.min(n - 1, k + 1)];
        const tx = after.x - before.x;
        const tz = after.z - before.z;
        const length = Math.hypot(tx, tz) || 1;
        const half = point.half ?? strip.half;
        const nx = (tz / length) * half;
        const nz = (-tx / length) * half;
        const v = (vertex + k) * 2;
        positions.set([point.x - nx, 0, -(point.z - nz), point.x + nx, 0, -(point.z + nz)], v * 3);
        colours.set([r, g, b, point.a, r, g, b, point.a], v * 4);
        if (k < n - 1) {
          indices.set([v, v + 2, v + 1, v + 1, v + 2, v + 3], count);
          count += 6;
        }
      }
      vertex += n;
    }

    this.position.needsUpdate = true;
    this.colour.needsUpdate = true;
    this.index.needsUpdate = true;
    this.geometry.setDrawRange(0, count);
  }

  dispose() {
    this.geometry.dispose();
    (this.mesh.material as THREE.Material).dispose();
  }
}

function dynamic(array: Float32Array | Uint16Array, size: number) {
  return new THREE.BufferAttribute(array, size).setUsage(THREE.DynamicDrawUsage);
}
