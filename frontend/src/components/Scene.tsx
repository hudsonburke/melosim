import { useMemo, Suspense } from "react";
import { Canvas, useLoader } from "@react-three/fiber";
import { OrbitControls, Grid, Line } from "@react-three/drei";
import * as THREE from "three";
import { STLLoader } from "three/examples/jsm/loaders/STLLoader.js";
import type { Scene as SceneData, BodyInfo, MeshInfo } from "../types/schema";

// ── Scene ────────────────────────────────────────────────────────────────

interface SceneProps {
  scene: SceneData | null;
  onSelect: (id: number | null) => void;
  selected: number | null;
  showSites: boolean;
}

export default function Scene({ scene, onSelect, selected, showSites }: SceneProps) {
  return (
    <div style={{ width: "100%", height: "100%", position: "relative" }}>
      <Canvas camera={{ position: [2.5, 1.5, 3], fov: 45 }} style={{ background: "#1a1a1a" }}>
        <ambientLight intensity={0.4} />
        <directionalLight position={[5, 10, 5]} intensity={0.8} />
        <directionalLight position={[-3, -2, -5]} intensity={0.3} />
        <Grid infiniteGrid />
        <OrbitControls makeDefault />
        <Suspense fallback={null}>
          {scene && (
            <ModelRenderer scene={scene} selected={selected} onSelect={onSelect} showSites={showSites} />
          )}
        </Suspense>
      </Canvas>
    </div>
  );
}

// ── STL Mesh loader ──────────────────────────────────────────────────────

function STLMesh({ mesh, color, onClick }: {
  mesh: MeshInfo;
  color: string;
  onClick?: () => void;
}) {
  const geometry = useLoader(STLLoader, mesh.url);
  
  const processedGeometry = useMemo(() => {
    if (!geometry) return null;
    const geo = geometry.clone();
    geo.computeBoundingBox();
    const center = new THREE.Vector3();
    geo.boundingBox?.getCenter(center);
    geo.translate(-center.x, -center.y, -center.z);
    
    const size = new THREE.Vector3();
    geo.boundingBox?.getSize(size);
    const maxDim = Math.max(size.x, size.y, size.z);
    const targetSize = 0.15;
    const scaleFactor = targetSize / maxDim;
    geo.scale(scaleFactor, scaleFactor, scaleFactor);
    
    return geo;
  }, [geometry]);

  if (!processedGeometry) return null;

  return (
    <mesh geometry={processedGeometry} onClick={onClick}>
      <meshStandardMaterial color={color} transparent opacity={0.85} />
    </mesh>
  );
}

// ── Model renderer ────────────────────────────────────────────────────────

function ModelRenderer({ scene, selected, onSelect, showSites }: {
  scene: SceneData;
  selected: number | null;
  onSelect: (id: number | null) => void;
  showSites: boolean;
}) {
  const bodyMap = useMemo(() => {
    const m = new Map<number, BodyInfo>();
    for (const b of scene.bodies) m.set(b.id, b);
    return m;
  }, [scene.bodies]);

  const childrenMap = useMemo(() => {
    const m = new Map<number, number[]>();
    for (const b of scene.bodies) {
      const key = b.parent_id ?? 0;
      if (!m.has(key)) m.set(key, []);
      m.get(key)!.push(b.id);
    }
    return m;
  }, [scene.bodies]);

  const meshesByBody = useMemo(() => {
    const m = new Map<number, MeshInfo[]>();
    for (const mesh of scene.meshes) {
      if (!m.has(mesh.parent)) m.set(mesh.parent, []);
      m.get(mesh.parent)!.push(mesh);
    }
    return m;
  }, [scene.meshes]);

  // Pre-compute world positions with O(n) BFS
  const worldPoses = useMemo(
    () => computeWorldPositions(scene, bodyMap, childrenMap),
    [scene, bodyMap, childrenMap],
  );

  // Bodies without STL meshes → one instanced draw call
  const instancedBodies = useMemo(() => {
    const bodiesWithMesh = new Set(meshesByBody.keys());
    const ids: number[] = [];
    const matrices: THREE.Matrix4[] = [];
    const scale = 0.06;

    for (const b of scene.bodies) {
      if (bodiesWithMesh.has(b.id)) continue;
      const pos = worldPoses.get(b.id);
      if (!pos) continue;
      ids.push(b.id);
      const mat = new THREE.Matrix4();
      mat.compose(pos, new THREE.Quaternion(), new THREE.Vector3(scale, scale, scale));
      matrices.push(mat);
    }

    const count = matrices.length;
    const buffer = new Float32Array(count * 16);
    for (let i = 0; i < count; i++) {
      const el = matrices[i].elements;
      for (let j = 0; j < 16; j++) buffer[i * 16 + j] = el[j];
    }
    return { buffer, count, ids };
  }, [scene.bodies, worldPoses, meshesByBody]);

  // Joint lines
  const jointLines = useMemo(() => {
    const points: [THREE.Vector3, THREE.Vector3][] = [];
    for (const j of scene.joints) {
      const p = worldPoses.get(j.body_a);
      const c = worldPoses.get(j.body_b);
      if (p && c) points.push([p, c]);
    }
    return points;
  }, [scene.joints, worldPoses]);

  return (
    <group>
      {/* Instanced bodies (no mesh) */}
      {instancedBodies.count > 0 && (
        <instancedMesh args={[undefined, undefined, instancedBodies.count]} frustumCulled={false}>
          <boxGeometry args={[1, 1, 1]} />
          <meshStandardMaterial color="#3399ff" transparent opacity={0.85} />
          <instancedBufferAttribute attach="instanceMatrix" args={[instancedBodies.buffer, 16]} />
        </instancedMesh>
      )}

      {/* STL meshes */}
      {scene.meshes.map(mesh => {
        const pos = worldPoses.get(mesh.parent);
        if (!pos) return null;
        return (
          <group key={mesh.id} position={pos}>
            <STLMesh
              mesh={mesh}
              color={selected === mesh.parent ? "#ff6600" : "#3399ff"}
              onClick={() => onSelect(selected === mesh.parent ? null : mesh.parent)}
            />
          </group>
        );
      })}

      {/* Sites */}
      {showSites && scene.sites.map(site => {
        const pos = worldPoses.get(site.parent);
        if (!pos) return null;
        return (
          <mesh key={site.id} position={pos}>
            <sphereGeometry args={[0.005, 6, 6]} />
            <meshStandardMaterial color="#ff4444" emissive="#ff4444" emissiveIntensity={0.3} />
          </mesh>
        );
      })}

      {/* Joint lines */}
      {jointLines.map(([a, b], i) => (
        <Line key={i} points={[a, b]} color="#666" lineWidth={1} />
      ))}
    </group>
  );
}

// ── World positions (O(n) BFS) ──────────────────────────────────────────

function computeWorldPositions(
  scene: SceneData,
  bodyMap: Map<number, BodyInfo>,
  childrenMap: Map<number, number[]>,
): Map<number, THREE.Vector3> {
  const m = new Map<number, THREE.Vector3>();
  const queue: number[] = [];

  // Find roots
  for (const b of scene.bodies) {
    if (b.parent_id === null) {
      const t = b.transform.translation;
      m.set(b.id, new THREE.Vector3(t[0], t[1], t[2]));
      queue.push(b.id);
    }
  }

  // BFS using childrenMap — O(n) total
  while (queue.length > 0) {
    const id = queue.shift()!;
    const parentPos = m.get(id)!;
    const children = childrenMap.get(id) || [];
    for (const childId of children) {
      const child = bodyMap.get(childId);
      if (!child) continue;
      const t = child.transform.translation;
      const r = child.transform.rotation;
      const pos = new THREE.Vector3(t[0], t[1], t[2]);
      const quat = new THREE.Quaternion(r[1], r[2], r[3], r[0]);
      pos.applyQuaternion(quat);
      pos.add(parentPos);
      m.set(childId, pos);
      queue.push(childId);
    }
  }

  return m;
}
