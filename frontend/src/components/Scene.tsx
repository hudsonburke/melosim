import { useState, useRef, useMemo, Suspense } from "react";
import { Canvas, useLoader } from "@react-three/fiber";
import { OrbitControls, Grid, Box, Line, Sphere } from "@react-three/drei";
import * as THREE from "three";
import { STLLoader } from "three/examples/jsm/loaders/STLLoader.js";
import type { Scene as SceneData, BodyInfo, JointInfo, SiteInfo, MeshInfo } from "../types/schema";

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

function STLMesh({ mesh, color, opacity, onClick, onPointerOver, onPointerOut }: {
  mesh: MeshInfo;
  color: string;
  opacity: number;
  onClick?: () => void;
  onPointerOver?: () => void;
  onPointerOut?: () => void;
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

  if (!processedGeometry) {
    return (
      <Box args={[0.08, 0.08, 0.08]}
        onClick={onClick} onPointerOver={onPointerOver} onPointerOut={onPointerOut}>
        <meshStandardMaterial color={color} transparent opacity={opacity} />
      </Box>
    );
  }

  return (
    <mesh geometry={processedGeometry}
      onClick={onClick} onPointerOver={onPointerOver} onPointerOut={onPointerOut}>
      <meshStandardMaterial color={color} transparent opacity={opacity} />
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

  const jointByChild = useMemo(() => {
    const m = new Map<number, JointInfo>();
    for (const j of scene.joints) m.set(j.body_b, j);
    return m;
  }, [scene.joints]);

  const meshesByBody = useMemo(() => {
    const m = new Map<number, MeshInfo[]>();
    for (const mesh of scene.meshes) {
      if (!m.has(mesh.parent)) m.set(mesh.parent, []);
      m.get(mesh.parent)!.push(mesh);
    }
    return m;
  }, [scene.meshes]);

  const sitesByBody = useMemo(() => {
    const m = new Map<number, SiteInfo[]>();
    for (const s of scene.sites) {
      if (!m.has(s.parent)) m.set(s.parent, []);
      m.get(s.parent)!.push(s);
    }
    return m;
  }, [scene.sites]);

  const roots = useMemo(() => {
    return scene.bodies.filter(b => b.parent_id === null).map(b => b.id);
  }, [scene.bodies]);

  const worldPoses = useMemo(() => computeWorldPositions(scene), [scene]);

  return (
    <group>
      {roots.map(id => (
        <BodyGroup key={id} bodyId={id} bodyMap={bodyMap} childrenMap={childrenMap}
          jointByChild={jointByChild} meshesByBody={meshesByBody} sitesByBody={sitesByBody}
          selected={selected} onSelect={onSelect} showSites={showSites} />
      ))}
      {scene.joints.map(j => {
        const p = worldPoses.get(j.body_a);
        const c = worldPoses.get(j.body_b);
        if (!p || !c) return null;
        return <Line key={`joint:${j.id}`} points={[p, c]} color="#666" lineWidth={1} />;
      })}
    </group>
  );
}

// ── Body group ────────────────────────────────────────────────────────────

function BodyGroup({ bodyId, bodyMap, childrenMap, jointByChild, meshesByBody, sitesByBody, selected, onSelect, showSites }: {
  bodyId: number;
  bodyMap: Map<number, BodyInfo>;
  childrenMap: Map<number, number[]>;
  jointByChild: Map<number, JointInfo>;
  meshesByBody: Map<number, MeshInfo[]>;
  sitesByBody: Map<number, SiteInfo[]>;
  selected: number | null;
  onSelect: (id: number | null) => void;
  showSites: boolean;
}) {
  const [hovered, setHovered] = useState(false);
  const groupRef = useRef<THREE.Group>(null);

  const body = bodyMap.get(bodyId);
  if (!body) return null;

  const isSelected = selected === bodyId;
  const color = isSelected ? "#ff6600" : hovered ? "#44aaff" : "#3399ff";
  const scale = isSelected ? 1.5 : hovered ? 1.2 : 1.0;
  const children = childrenMap.get(bodyId) || [];
  const meshes = meshesByBody.get(bodyId) || [];
  const sites = sitesByBody.get(bodyId) || [];

  const t = body.transform.translation;
  const r = body.transform.rotation;

  return (
    <group ref={groupRef} position={new THREE.Vector3(t[0], t[1], t[2])}
      quaternion={new THREE.Quaternion(r[1], r[2], r[3], r[0])}>

      {/* Body visualization */}
      {meshes.length > 0 ? (
        meshes.map(mesh => (
          <group key={mesh.id} position={new THREE.Vector3(mesh.offset[0], mesh.offset[1], mesh.offset[2])}>
            <STLMesh
              mesh={mesh}
              color={color}
              opacity={0.85}
              onPointerOver={() => setHovered(true)}
              onPointerOut={() => setHovered(false)}
              onClick={() => onSelect(isSelected ? null : bodyId)}
            />
          </group>
        ))
      ) : (
        <Box args={[0.14 * scale, 0.14 * scale, 0.14 * scale]}
          onPointerOver={() => setHovered(true)}
          onPointerOut={() => setHovered(false)}
          onClick={() => onSelect(isSelected ? null : bodyId)}>
          <meshStandardMaterial color={color} transparent opacity={0.85} />
        </Box>
      )}

      {/* Sites */}
      {showSites && sites.map(site => (
        <group key={site.id} position={new THREE.Vector3(site.offset[0], site.offset[1], site.offset[2])}>
          <Sphere args={[0.005, 6, 6]}>
            <meshStandardMaterial color="#ff4444" emissive="#ff4444" emissiveIntensity={0.3} />
          </Sphere>
        </group>
      ))}

      {/* Joint indicator */}
      {jointByChild.has(bodyId) && (
        <Sphere args={[0.02, 8, 8]}>
          <meshStandardMaterial color="#00ff00" transparent opacity={0.5} />
        </Sphere>
      )}

      {/* Children */}
      {children.map(id => (
        <BodyGroup key={id} bodyId={id} bodyMap={bodyMap} childrenMap={childrenMap}
          jointByChild={jointByChild} meshesByBody={meshesByBody} sitesByBody={sitesByBody}
          selected={selected} onSelect={onSelect} showSites={showSites} />
      ))}
    </group>
  );
}

// ── World positions ────────────────────────────────────────────────────────

function computeWorldPositions(scene: SceneData): Map<number, THREE.Vector3> {
  const m = new Map<number, THREE.Vector3>();
  const bodyMap = new Map<number, BodyInfo>();
  for (const b of scene.bodies) bodyMap.set(b.id, b);

  const visited = new Set<number>();
  const queue: number[] = [];

  for (const b of scene.bodies) {
    if (b.parent_id === null) {
      queue.push(b.id);
      visited.add(b.id);
    }
  }

  while (queue.length > 0) {
    const id = queue.shift()!;
    const body = bodyMap.get(id);
    if (!body) continue;

    const t = body.transform.translation;
    const r = body.transform.rotation;
    const pos = new THREE.Vector3(t[0], t[1], t[2]);
    const quat = new THREE.Quaternion(r[1], r[2], r[3], r[0]);

    const parentId = body.parent_id;
    if (parentId !== null && parentId !== 0) {
      const parentPos = m.get(parentId);
      if (parentPos) {
        pos.applyQuaternion(quat);
        pos.add(parentPos);
      }
    }

    m.set(id, pos);

    for (const b of scene.bodies) {
      if (b.parent_id === id && !visited.has(b.id)) {
        visited.add(b.id);
        queue.push(b.id);
      }
    }
  }

  return m;
}
