import * as THREE from 'three';
import { invoke } from '@tauri-apps/api/core';
import './style.css';

type Body = { id: string; name: string; position: [number, number, number]; quaternion: [number, number, number, number] };
type Snapshot = { model: string; bodies: Body[] };

const app = document.querySelector<HTMLDivElement>('#app')!;
app.innerHTML = `<aside><h1>melosim</h1><p class="eyebrow">neuromusculoskeletal editor</p><h2>Hierarchy</h2><ul id="hierarchy"></ul></aside><main><canvas id="viewport"></canvas><div class="status" id="status">Loading model…</div></main>`;

const scene = new THREE.Scene(); scene.background = new THREE.Color('#11151c');
const camera = new THREE.PerspectiveCamera(45, 1, 0.01, 100); camera.position.set(1.6, 1.2, 2.2); camera.lookAt(0, .35, 0);
const renderer = new THREE.WebGLRenderer({ canvas: document.querySelector('#viewport') as HTMLCanvasElement, antialias: true });
scene.add(new THREE.HemisphereLight('#dce8ff', '#273043', 2));
const grid = new THREE.GridHelper(3, 30, '#334155', '#1e293b'); scene.add(grid);
const root = new THREE.Group(); scene.add(root);
const bodyMeshes = new Map<string, THREE.Mesh>();
const material = new THREE.MeshStandardMaterial({ color: '#61a5fa', roughness: .62, metalness: .08 });

function resize() { const box = renderer.domElement.parentElement!.getBoundingClientRect(); renderer.setSize(box.width, box.height, false); camera.aspect = box.width / box.height; camera.updateProjectionMatrix(); }
window.addEventListener('resize', resize); resize();
function frame() { renderer.render(scene, camera); requestAnimationFrame(frame); } frame();

function showSnapshot(snapshot: Snapshot) {
  document.title = `${snapshot.model} · melosim`;
  const list = document.querySelector<HTMLUListElement>('#hierarchy')!; list.replaceChildren();
  for (const body of snapshot.bodies) {
    let mesh = bodyMeshes.get(body.id);
    if (!mesh) { mesh = new THREE.Mesh(new THREE.BoxGeometry(.12, .32, .12), material.clone()); mesh.userData.bodyId = body.id; root.add(mesh); bodyMeshes.set(body.id, mesh); }
    mesh.name = body.name; mesh.position.set(...body.position); mesh.quaternion.set(body.quaternion[1], body.quaternion[2], body.quaternion[3], body.quaternion[0]);
    const item = document.createElement('li'); item.textContent = body.name; item.dataset.id = body.id; item.onclick = () => { for (const m of bodyMeshes.values()) (m.material as THREE.MeshStandardMaterial).emissive.set('#000000'); (mesh!.material as THREE.MeshStandardMaterial).emissive.set('#234f82'); document.querySelector('#status')!.textContent = `Selected ${body.name}`; }; list.append(item);
  }
  document.querySelector('#status')!.textContent = `${snapshot.bodies.length} bodies · Rust model snapshot`;
}

invoke<Snapshot>('model_snapshot').then(showSnapshot).catch(() => showSnapshot({ model: 'demo', bodies: [{ id: 'upper', name: 'upper_arm', position: [0, .35, 0], quaternion: [1, 0, 0, 0] }, { id: 'forearm', name: 'forearm', position: [0, .72, 0], quaternion: [1, 0, 0, 0] }] }));
