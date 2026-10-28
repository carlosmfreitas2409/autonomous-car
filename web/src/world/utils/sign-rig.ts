import * as THREE from 'three';

import letterA from '@assets/sign_cutouts/delivery_sign/variant_a.png';
import letterB from '@assets/sign_cutouts/delivery_sign/variant_b.png';
import letterC from '@assets/sign_cutouts/delivery_sign/variant_c.png';
import detourArt from '@assets/sign_cutouts/detour/detour.png';
import stopArt from '@assets/sign_cutouts/stop/stop.png';

import type { Letter, LightState } from '@/types';

export interface SignColours {
  stop: string;
  warn: string;
  ok: string;
}

export interface SignRig {
  group: THREE.Group;
  setOpacity: (alpha: number) => void;
  setLight: (state: LightState) => void;
  dispose: () => void;
}

interface Art {
  texture: THREE.Texture;
  aspect: Promise<number>;
}

const POLE_HEIGHT = 0.52;
const FACE_HEIGHT = 0.3;
const LIGHT_HEIGHT = 0.3;
const LAMP_OFF = '#45464a';

const LETTER_ART: Record<Letter, string> = { A: letterA, B: letterB, C: letterC };

const artCache = new Map<string, Art>();

function artFor(label: string, variant: Letter | null) {
  if (label === 'stop') return stopArt;
  if (label === 'detour') return detourArt;
  if (label === 'delivery_sign' && variant) return LETTER_ART[variant];
  return null;
}

function loadArt(url: string) {
  const cached = artCache.get(url);
  if (cached) return cached;

  let texture!: THREE.Texture;
  const aspect = new Promise<number>((resolve) => {
    texture = new THREE.TextureLoader().load(url, (loaded) => {
      resolve(loaded.image.width / loaded.image.height);
    });
  });
  texture.colorSpace = THREE.SRGBColorSpace;
  texture.anisotropy = 4;
  const art = { texture, aspect };
  artCache.set(url, art);
  return art;
}

export function createSignRig(
  label: string,
  variant: Letter | null,
  colours: SignColours,
): SignRig {
  const group = new THREE.Group();
  const disposables: { dispose: () => void }[] = [];
  const materials: THREE.Material[] = [];
  const lamps: { name: LightState; material: THREE.MeshBasicMaterial; on: THREE.Color }[] = [];

  function keep<T extends { dispose: () => void }>(thing: T) {
    disposables.push(thing);
    return thing;
  }
  function material<T extends THREE.Material>(thing: T) {
    materials.push(keep(thing));
    return thing;
  }

  const pole = new THREE.Mesh(
    keep(new THREE.CylinderGeometry(0.011, 0.011, POLE_HEIGHT, 12)),
    material(
      new THREE.MeshStandardMaterial({
        color: '#9b968c',
        roughness: 0.45,
        metalness: 0.5,
        transparent: true,
      }),
    ),
  );
  pole.position.y = POLE_HEIGHT / 2;
  group.add(pole);

  if (label === 'traffic_light') {
    const housing = new THREE.Mesh(
      keep(new THREE.BoxGeometry(0.11, LIGHT_HEIGHT, 0.07)),
      material(
        new THREE.MeshStandardMaterial({ color: '#2b2c2f', roughness: 0.55, transparent: true }),
      ),
    );
    housing.position.y = POLE_HEIGHT + LIGHT_HEIGHT / 2;
    group.add(housing);

    const lampGeometry = keep(new THREE.CircleGeometry(0.03, 28));
    const layout: [LightState, string, number][] = [
      ['red', colours.stop, 0.09],
      ['yellow', colours.warn, 0],
      ['green', colours.ok, -0.09],
    ];
    for (const [name, colour, dy] of layout) {
      const lampMaterial = material(
        new THREE.MeshBasicMaterial({ color: LAMP_OFF, transparent: true, toneMapped: false }),
      );
      const lamp = new THREE.Mesh(lampGeometry, lampMaterial);
      lamp.position.set(0, POLE_HEIGHT + LIGHT_HEIGHT / 2 + dy, 0.0365);
      group.add(lamp);
      lamps.push({ name, material: lampMaterial, on: new THREE.Color(colour) });
    }
  } else {
    const url = artFor(label, variant);
    if (url) {
      const { texture, aspect } = loadArt(url);
      const face = new THREE.Mesh(
        keep(new THREE.PlaneGeometry(FACE_HEIGHT, FACE_HEIGHT)),
        material(
          new THREE.MeshBasicMaterial({
            map: texture,
            transparent: true,
            alphaTest: 0.02,
            toneMapped: false,
          }),
        ),
      );
      face.position.y = POLE_HEIGHT + FACE_HEIGHT / 2;
      void aspect.then((ratio) => face.scale.set(ratio, 1, 1));
      group.add(face);
    }
  }

  return {
    group,
    setOpacity(alpha) {
      for (const each of materials) each.opacity = alpha;
    },
    setLight(state) {
      for (const lamp of lamps) lamp.material.color.set(lamp.name === state ? lamp.on : LAMP_OFF);
    },
    dispose() {
      for (const thing of disposables) thing.dispose();
    },
  };
}
