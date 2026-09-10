import {
  useEffect,
  useMemo,
  useRef,
  useState,
  type RefObject,
  type ReactNode,
} from "react";
import { Canvas } from "@react-three/fiber";
import {
  OrbitControls,
  TransformControls,
  GizmoHelper,
  GizmoViewport,
  Bounds,
  useBounds,
  Line,
} from "@react-three/drei";
import * as THREE from "three";
import type { Command, MeshData, Node, Snapshot } from "./api";
type Props = {
  snapshot: Snapshot;
  meshes: Map<string, MeshData>;
  selected: string | null;
  select: (id: string | null) => void;
  send: (c: Command) => Promise<boolean>;
  mode: "translate" | "rotate";
  sites: boolean;
  muscles: boolean;
  frames: boolean;
  focus: number;
  placing: string | null;
  place: (body: string, position: [number, number, number]) => void;
};
const worldPosition = (n: Node) =>
  new THREE.Vector3().setFromMatrixPosition(
    new THREE.Matrix4().fromArray(n.matrix),
  );
function Model({
  p,
  objects,
  root,
}: {
  p: Props;
  objects: RefObject<Map<string, THREE.Group>>;
  root: RefObject<THREE.Group | null>;
}) {
  const nodeMap = useMemo(
    () => new Map(p.snapshot.nodes.map((n) => [n.id, n])),
    [p.snapshot.nodes],
  );
  const childMap = useMemo(() => {
    const map = new Map<string | null, Node[]>();
    for (const n of p.snapshot.nodes) {
      const a = map.get(n.parent) || [];
      a.push(n);
      map.set(n.parent, a);
    }
    return map;
  }, [p.snapshot.nodes]);
  const geometry = useMemo(() => {
    const result = new Map<string, THREE.BufferGeometry>();
    p.meshes.forEach((m, key) => {
      const g = new THREE.BufferGeometry();
      g.setAttribute(
        "position",
        new THREE.Float32BufferAttribute(m.positions.flat(), 3),
      );
      g.setIndex(m.indices);
      g.computeVertexNormals();
      result.set(key, g);
    });
    return result;
  }, [p.meshes]);
  useEffect(() => () => geometry.forEach((g) => g.dispose()), [geometry]);
  const bodyOf = (n: Node): Node =>
    n.kind === "body" || !n.parent || !nodeMap.has(n.parent)
      ? n
      : bodyOf(nodeMap.get(n.parent)!);
  const highlighted = (n: Node): boolean =>
    n.id === p.selected ||
    !!(
      n.parent &&
      nodeMap.has(n.parent) &&
      highlighted(nodeMap.get(n.parent)!)
    );
  function renderNode(n: Node): ReactNode {
    const rgba = n.rgba || [0.75, 0.8, 0.85, 1];
    return (
      <group
        key={n.id}
        name={n.name}
        ref={(g) => {
          if (g) objects.current.set(n.id, g);
          else objects.current.delete(n.id);
        }}
        position={n.position}
        quaternion={n.quaternion}
      >
        {n.mesh && geometry.has(n.mesh) && (
          <mesh
            geometry={geometry.get(n.mesh)}
            onClick={(e) => {
              if (e.delta > 4) return;
              e.stopPropagation();
              const body = bodyOf(n);
              if (p.placing) {
                const local = e.point
                  .clone()
                  .applyMatrix4(
                    new THREE.Matrix4().fromArray(body.matrix).invert(),
                  );
                p.place(body.id, local.toArray());
              } else p.select(body.id);
            }}
          >
            <meshStandardMaterial
              color={new THREE.Color(rgba[0], rgba[1], rgba[2])}
              roughness={0.65}
              metalness={0.08}
              emissive={highlighted(n) ? "#124f5e" : "#000000"}
              transparent={rgba[3] < 1}
              opacity={rgba[3]}
              side={THREE.DoubleSide}
            />
          </mesh>
        )}
        {n.kind === "site" && (p.sites || n.id === p.selected) && (
          <mesh
            onClick={(e) => {
              e.stopPropagation();
              p.select(n.id);
            }}
          >
            <sphereGeometry
              args={[n.id === p.selected ? 0.004 : 0.002, 10, 8]}
            />
            <meshBasicMaterial
              color={n.id === p.selected ? "#ffffff" : "#d1a862"}
            />
          </mesh>
        )}
        {p.frames && ["frame", "body"].includes(n.kind) && (
          <axesHelper args={[0.035]} />
        )}
        {(childMap.get(n.id) || [])
          .filter((c) => c.kind !== "coordinate")
          .map(renderNode)}
      </group>
    );
  }
  return (
    <group ref={root}>
      {(childMap.get(null) || [])
        .filter((n) => !["muscle", "cable", "coordinate"].includes(n.kind))
        .map(renderNode)}
      {p.snapshot.nodes
        .filter(
          (n) =>
            n.path.length > 1 &&
            (n.kind === "cable" || p.muscles || n.id === p.selected),
        )
        .map((n) => {
          const points = n.path
            .map((id) => nodeMap.get(id))
            .filter((n): n is Node => !!n)
            .map(worldPosition);
          return (
            points.length > 1 && (
              <Line
                key={n.id}
                points={points}
                color={n.kind === "cable" ? "#55d9cd" : "#a35852"}
                lineWidth={
                  n.id === p.selected ? 4 : n.kind === "cable" ? 2.5 : 1
                }
                onClick={(e) => {
                  e.stopPropagation();
                  p.select(n.id);
                }}
              />
            )
          );
        })}
    </group>
  );
}
function Fit({
  root,
  token,
  epoch,
}: {
  root: RefObject<THREE.Group | null>;
  token: number;
  epoch: number;
}) {
  const bounds = useBounds();
  const fitKey = useRef("");
  useEffect(() => {
    const key = `${epoch}:${token}`;
    if (fitKey.current === key) return;
    if (
      root.current &&
      !new THREE.Box3().setFromObject(root.current).isEmpty()
    ) {
      fitKey.current = key;
      bounds.refresh(root.current).clip().fit();
    }
  }, [token, epoch, bounds]);
  return null;
}
function Scene(p: Props) {
  const objects = useRef(new Map<string, THREE.Group>()),
    root = useRef<THREE.Group>(null);
  const [target, setTarget] = useState<THREE.Group | null>(null);
  useEffect(() => {
    setTarget(
      p.snapshot.nodes.find((n) => n.id === p.selected)?.editable
        ? objects.current.get(p.selected!) || null
        : null,
    );
  }, [p.selected, p.snapshot]);
  return (
    <>
      <ambientLight intensity={0.55} />
      <hemisphereLight args={["#e2eeff", "#39404c", 1.8]} />
      <directionalLight position={[3, 5, 4]} intensity={2.5} />
      <directionalLight position={[-3, 1, -2]} intensity={0.8} />
      <gridHelper
        args={[10, 100, "#31414d", "#202d36"]}
        position={[0, -0.005, 0]}
      />
      <OrbitControls makeDefault minDistance={0.03} maxDistance={20} />
      <Bounds margin={1.35}>
        <Model p={p} objects={objects} root={root} />
        <Fit root={root} token={p.focus} epoch={p.snapshot.epoch} />
      </Bounds>
      {target && !p.placing && (
        <TransformControls
          object={target}
          mode={p.mode}
          space="local"
          size={0.85}
          onMouseUp={() => {
            if (!p.selected) return;
            void p
              .send({
                type: "set_transform",
                id: p.selected,
                position: target.position.toArray(),
                quaternion: target.quaternion.toArray(),
              })
              .then((ok) => {
                if (!ok) {
                  const n = p.snapshot.nodes.find((n) => n.id === p.selected);
                  if (n) {
                    target.position.fromArray(n.position);
                    target.quaternion.fromArray(n.quaternion);
                  }
                }
              });
          }}
        />
      )}
      <GizmoHelper alignment="bottom-right" margin={[70, 70]}>
        <GizmoViewport
          axisColors={["#ec8078", "#87c89c", "#77aee1"]}
          labelColor="#10202c"
        />
      </GizmoHelper>
    </>
  );
}
export function Viewport(p: Props) {
  return (
    <Canvas
      camera={{ position: [1.3, 0.8, 1.5], fov: 42, near: 0.001, far: 100 }}
      dpr={[1, 2]}
      onPointerMissed={() => {
        if (!p.placing) p.select(null);
      }}
    >
      <color attach="background" args={["#101a23"]} />
      <Scene {...p} />
    </Canvas>
  );
}
