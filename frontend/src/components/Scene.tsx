import { useMemo, useState, useEffect } from "react";
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

// ── Async mesh (STL/OBJ, non-blocking) ───────────────────────────────────

function AsyncMesh({ mesh, highlight, onClick }: {
  mesh: MeshInfo;
  highlight: boolean;
  onClick?: () => void;
}) {
  const [object, setObject] = useState<THREE.Object3D | null>(null);

  useEffect(() => {
    let cancelled = false;
    const isObj = mesh.url.toLowerCase().endsWith(".obj");
    log(`Loading mesh: ${mesh.url}`);
    // Static import paths so Vite can bundle both loaders
    const load = isObj
      ? import("three/examples/jsm/loaders/OBJLoader.js")
      : import("three/examples/jsm/loaders/STLLoader.js");
    load.then((mod: any) => {
      const loader = isObj ? new mod.OBJLoader() : new mod.STLLoader();
      loader.load(
        mesh.url,
        (loaded: any) => {
          if (cancelled) return;
          const c = mesh.color ?? [0.2, 0.6, 1.0];
          const material = new THREE.MeshStandardMaterial({
            color: new THREE.Color(c[0], c[1], c[2]),
            transparent: true,
            opacity: mesh.opacity ?? 0.85,
          });
          let obj: THREE.Object3D;
          if (isObj) {
            obj = loaded as THREE.Object3D; // OBJLoader returns a Group
            obj.traverse((o) => { if ((o as any).isMesh) (o as any).material = material; });
          } else {
            obj = new THREE.Mesh(loaded as THREE.BufferGeometry, material);
          }
          setObject(obj);
        },
        undefined,
        (err: any) => log(`Mesh FAILED: ${mesh.url} — ${err}`),
      );
    });
    return () => { cancelled = true; };
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [mesh.url]);

  // Selection highlight (materials are baked at load, so recolor in place)
  useEffect(() => {
    if (!object) return;
    const c = mesh.color ?? [0.2, 0.6, 1.0];
    const color = highlight ? new THREE.Color("#ff6600") : new THREE.Color(c[0], c[1], c[2]);
    object.traverse((o) => {
      const m = (o as any).material as THREE.MeshStandardMaterial | undefined;
      if ((o as any).isMesh && m) m.color.copy(color);
    });
  }, [object, highlight, mesh.color]);

  if (!object) return null;

  const r = mesh.rotation ?? [1, 0, 0, 0];
  return (
    <primitive
      object={object}
      onClick={onClick}
      position={mesh.offset}
      quaternion={new THREE.Quaternion(r[1], r[2], r[3], r[0])}
      scale={mesh.scale ?? [1, 1, 1]}
    />
  );
}

// ── Joint lines ──────────────────────────────────────────────────────────

function JointLines({ scene, worldPoses }: {
  scene: SceneData;
  worldPoses: Map<number, WorldPose>;
}) {
  const positions = useMemo(() => {
    const count = scene.joints.length;
    const pos = new Float32Array(count * 6);
    let i = 0;
    for (const j of scene.joints) {
      const a = worldPoses.get(j.body_a)?.pos;
      const b = worldPoses.get(j.body_b)?.pos;
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
  worldPoses: Map<number, WorldPose>;
}) {
  const positions = useMemo(() => {
    const segmentCount = musclePaths.reduce((sum, mp) => sum + Math.max(0, mp.points.length - 1), 0);
    const pos = new Float32Array(segmentCount * 6);
    let offset = 0;
    for (const mp of musclePaths) {
      const pts: THREE.Vector3[] = [];
      for (const pt of mp.points) {
        const pose = worldPoses.get(pt.body);
        if (pose) {
          pts.push(
            new THREE.Vector3(pt.location[0], pt.location[1], pt.location[2])
              .applyQuaternion(pose.quat)
              .add(pose.pos),
          );
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
      const result = computeWorldPoses(scene, bodyMap, childrenMap);
      log(`computeWorldPoses: ${Date.now() - start}ms (${result.size} bodies)`);
      return result;
    },
    [scene, bodyMap, childrenMap],
  );

  log(`ModelRenderer render: ${scene.meshes.length} meshes, joints=${scene.joints.length}`);

  return (
    <group>
      {/* Display geometry only */}
      {scene.meshes.map(mesh => {
        const pose = worldPoses.get(mesh.parent);
        if (!pose) return null;
        return (
          <group key={mesh.id} position={pose.pos} quaternion={pose.quat}>
            <AsyncMesh
              mesh={mesh}
              highlight={selected === mesh.parent}
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
  worldPoses: Map<number, WorldPose>;
}) {
  const positions = useMemo(() => {
    const pos = new Float32Array(scene.sites.length * 3);
    let i = 0;
    for (const s of scene.sites) {
      const pose = worldPoses.get(s.parent);
      if (pose) {
        const p = new THREE.Vector3(s.offset[0], s.offset[1], s.offset[2])
          .applyQuaternion(pose.quat)
          .add(pose.pos);
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

// ── World poses (O(n) BFS) ──────────────────────────────────────────────
// worldQ = parentWorldQ * localQ
// worldP = parentWorldP + parentWorldQ * localP

interface WorldPose {
  pos: THREE.Vector3;
  quat: THREE.Quaternion;
}

function computeWorldPoses(
  scene: SceneData,
  bodyMap: Map<number, BodyInfo>,
  childrenMap: Map<number, number[]>,
): Map<number, WorldPose> {
  const m = new Map<number, WorldPose>();
  const queue: number[] = [];

  const toPose = (b: BodyInfo): WorldPose => {
    const t = b.transform.translation;
    const r = b.transform.rotation;
    return {
      pos: new THREE.Vector3(t[0], t[1], t[2]),
      quat: new THREE.Quaternion(r[1], r[2], r[3], r[0]),
    };
  };

  for (const b of scene.bodies) {
    if (b.parent_id === null) {
      m.set(b.id, toPose(b));
      queue.push(b.id);
    }
  }

  while (queue.length > 0) {
    const id = queue.shift()!;
    const parent = m.get(id)!;
    const children = childrenMap.get(id) || [];
    for (const childId of children) {
      if (m.has(childId)) continue; // guard: self/cyclic parent links (e.g. root id 0 keyed under 0)
      const child = bodyMap.get(childId);
      if (!child) continue;
      const local = toPose(child);
      m.set(childId, {
        pos: local.pos.clone().applyQuaternion(parent.quat).add(parent.pos),
        quat: parent.quat.clone().multiply(local.quat),
      });
      queue.push(childId);
    }
  }

  return m;
}
