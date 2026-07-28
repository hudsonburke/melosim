import { useMemo, useState, useEffect, useRef } from "react";
import { Canvas } from "@react-three/fiber";
import { OrbitControls, Grid } from "@react-three/drei";
import * as THREE from "three";
import type { Scene as SceneData, BodyInfo, MeshInfo, MusclePathInfo } from "../types/schema";

// ── Debug helper ──────────────────────────────────────────────────────────

const DEBUG = true;
function log(msg: string) {
  if (DEBUG) console.log(`[melosim] ${msg}`);
}

// ── Scene ────────────────────────────────────────────────────────────────

interface SceneProps {
  scene: SceneData | null;
  onSelect: (id: number | null) => void;
  selected: number | null;
  showSites: boolean;
  showMuscles?: boolean;
}

export default function Scene({ scene, onSelect, selected, showSites, showMuscles = false }: SceneProps) {
  useEffect(() => {
    if (scene) {
      log(`Scene data: ${scene.bodies.length} bodies, ${scene.meshes.length} meshes, ${scene.joints.length} joints, ${scene.muscle_paths.length} muscle_paths, ${scene.sites.length} sites`);
      // Estimate JSON size
      const jsonSize = JSON.stringify(scene).length;
      log(`Scene JSON size: ${(jsonSize / 1024).toFixed(1)} KB`);
    }
  }, [scene]);

  return (
    <div style={{ width: "100%", height: "100%", position: "relative" }}>
      <Canvas camera={{ position: [2.5, 1.5, 3], fov: 45 }} style={{ background: "#1a1a1a" }}>
        <ambientLight intensity={0.4} />
        <directionalLight position={[5, 10, 5]} intensity={0.8} />
        <directionalLight position={[-3, -2, -5]} intensity={0.3} />
        <Grid infiniteGrid />
        <OrbitControls makeDefault />
        {scene && (
          <ModelRenderer
            scene={scene}
            selected={selected}
            onSelect={onSelect}
            showSites={showSites}
            showMuscles={showMuscles}
          />
        )}
      </Canvas>
    </div>
  );
}

// ── Async STL mesh (non-blocking) ────────────────────────────────────────

function AsyncSTLMesh({ mesh, color, onClick }: {
  mesh: MeshInfo;
  color: string;
  onClick?: () => void;
}) {
  const [geometry, setGeometry] = useState<THREE.BufferGeometry | null>(null);
  const startTime = useRef(Date.now());

  useEffect(() => {
    let cancelled = false;
    log(`Loading mesh: ${mesh.url}`);
    import("three/examples/jsm/loaders/STLLoader.js").then(({ STLLoader }) => {
      const loader = new STLLoader();
      loader.load(
        mesh.url,
        (geo) => {
          if (cancelled) { geo.dispose(); return; }
          const elapsed = Date.now() - startTime.current;
          log(`Mesh loaded in ${elapsed}ms: ${mesh.url} (${(geo.attributes.position.count)} verts)`);
          geo.computeBoundingBox();
          const center = new THREE.Vector3();
          geo.boundingBox!.getCenter(center);
          geo.translate(-center.x, -center.y, -center.z);
          const size = new THREE.Vector3();
          geo.boundingBox!.getSize(size);
          const maxDim = Math.max(size.x, size.y, size.z);
          const scale = 0.15 / maxDim;
          geo.scale(scale, scale, scale);
          setGeometry(geo);
        },
        undefined,
        (err) => {
          log(`Mesh FAILED: ${mesh.url} — ${err}`);
        },
      );
    });
    return () => { cancelled = true; };
  }, [mesh.url]);

  if (!geometry) return null;

  return (
    <mesh geometry={geometry} onClick={onClick}>
      <meshStandardMaterial color={color} transparent opacity={0.85} />
    </mesh>
  );
}

// ── Joint lines ──────────────────────────────────────────────────────────

function JointLines({ scene, worldPoses }: {
  scene: SceneData;
  worldPoses: Map<number, THREE.Vector3>;
}) {
  const positions = useMemo(() => {
    const count = scene.joints.length;
    const pos = new Float32Array(count * 6);
    let i = 0;
    for (const j of scene.joints) {
      const a = worldPoses.get(j.body_a);
      const b = worldPoses.get(j.body_b);
      if (a && b) {
        pos[i++] = a.x; pos[i++] = a.y; pos[i++] = a.z;
        pos[i++] = b.x; pos[i++] = b.y; pos[i++] = b.z;
      }
    }
    return pos;
  }, [scene.joints, worldPoses]);

  if (positions.length === 0) return null;

  return (
    <lineSegments>
      <bufferGeometry>
        <bufferAttribute attach="attributes-position" args={[positions, 3]} />
      </bufferGeometry>
      <lineBasicMaterial color="#666" />
    </lineSegments>
  );
}

// ── Muscle visualization ─────────────────────────────────────────────────

function MuscleVisualization({ musclePaths, worldPoses }: {
  musclePaths: MusclePathInfo[];
  worldPoses: Map<number, THREE.Vector3>;
}) {
  const positions = useMemo(() => {
    const segmentCount = musclePaths.reduce((sum, mp) => sum + Math.max(0, mp.points.length - 1), 0);
    const pos = new Float32Array(segmentCount * 6);
    let offset = 0;
    for (const mp of musclePaths) {
      const pts: THREE.Vector3[] = [];
      for (const pt of mp.points) {
        const bodyPos = worldPoses.get(pt.body);
        if (bodyPos) {
          pts.push(new THREE.Vector3(
            bodyPos.x + pt.location[0],
            bodyPos.y + pt.location[1],
            bodyPos.z + pt.location[2],
          ));
        }
      }
      for (let i = 0; i < pts.length - 1; i++) {
        pos[offset++] = pts[i].x; pos[offset++] = pts[i].y; pos[offset++] = pts[i].z;
        pos[offset++] = pts[i+1].x; pos[offset++] = pts[i+1].y; pos[offset++] = pts[i+1].z;
      }
    }
    return pos;
  }, [musclePaths, worldPoses]);

  if (positions.length === 0) return null;

  return (
    <lineSegments>
      <bufferGeometry>
        <bufferAttribute attach="attributes-position" args={[positions, 3]} />
      </bufferGeometry>
      <lineBasicMaterial color="#ff3366" />
    </lineSegments>
  );
}

// ── Model renderer ────────────────────────────────────────────────────────

function ModelRenderer({ scene, selected, onSelect, showSites, showMuscles }: {
  scene: SceneData;
  selected: number | null;
  onSelect: (id: number | null) => void;
  showSites: boolean;
  showMuscles: boolean;
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

  const worldPoses = useMemo(
    () => {
      const start = Date.now();
      const result = computeWorldPositions(scene, bodyMap, childrenMap);
      log(`computeWorldPositions: ${Date.now() - start}ms (${result.size} bodies)`);
      return result;
    },
    [scene, bodyMap, childrenMap],
  );

  log(`ModelRenderer render: ${scene.meshes.length} meshes, joints=${scene.joints.length}`);

  return (
    <group>
      {/* Display geometry only */}
      {scene.meshes.map(mesh => {
        const pos = worldPoses.get(mesh.parent);
        if (!pos) return null;
        return (
          <group key={mesh.id} position={pos}>
            <AsyncSTLMesh
              mesh={mesh}
              color={selected === mesh.parent ? "#ff6600" : "#3399ff"}
              onClick={() => onSelect(selected === mesh.parent ? null : mesh.parent)}
            />
          </group>
        );
      })}

      {showSites && <SitePoints scene={scene} worldPoses={worldPoses} />}

      {showMuscles && (
        <MuscleVisualization musclePaths={scene.muscle_paths} worldPoses={worldPoses} />
      )}

      <JointLines scene={scene} worldPoses={worldPoses} />
    </group>
  );
}

// ── Site points ──────────────────────────────────────────────────────────

function SitePoints({ scene, worldPoses }: {
  scene: SceneData;
  worldPoses: Map<number, THREE.Vector3>;
}) {
  const positions = useMemo(() => {
    const pos = new Float32Array(scene.sites.length * 3);
    let i = 0;
    for (const s of scene.sites) {
      const p = worldPoses.get(s.parent);
      if (p) {
        pos[i++] = p.x; pos[i++] = p.y; pos[i++] = p.z;
      }
    }
    return pos;
  }, [scene.sites, worldPoses]);

  if (positions.length === 0) return null;

  return (
    <points>
      <bufferGeometry>
        <bufferAttribute attach="attributes-position" args={[positions, 3]} />
      </bufferGeometry>
      <pointsMaterial size={0.005} color="#ff4444" sizeAttenuation />
    </points>
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

  for (const b of scene.bodies) {
    if (b.parent_id === null) {
      const t = b.transform.translation;
      m.set(b.id, new THREE.Vector3(t[0], t[1], t[2]));
      queue.push(b.id);
    }
  }

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
