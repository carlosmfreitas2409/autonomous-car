import * as THREE from 'three';

const VERTEX = `
  varying vec2 vUv;
  void main() {
    vUv = uv;
    gl_Position = projectionMatrix * modelViewMatrix * vec4(position, 1.0);
  }
`;

const FRAGMENT = `
  varying vec2 vUv;
  void main() {
    float t = clamp(length((vUv - 0.5) * 2.0), 0.0, 1.0);
    float alpha = t < 0.4
      ? mix(0.62, 0.34, t / 0.4)
      : t < 0.75
        ? mix(0.34, 0.1, (t - 0.4) / 0.35)
        : mix(0.1, 0.0, (t - 0.75) / 0.25);
    gl_FragColor = vec4(1.0, 1.0, 1.0, alpha);
  }
`;

export function createFloorGlow() {
  const geometry = new THREE.PlaneGeometry(16, 18);
  const material = new THREE.ShaderMaterial({
    vertexShader: VERTEX,
    fragmentShader: FRAGMENT,
    transparent: true,
    depthWrite: false,
    toneMapped: false,
  });
  const mesh = new THREE.Mesh(geometry, material);
  mesh.rotation.x = -Math.PI / 2;
  mesh.position.set(0, 0, -2.2);
  mesh.renderOrder = 0;

  return { mesh, disposables: [geometry, material] };
}
