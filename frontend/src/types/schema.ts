/**
 * Types matching the melosim Rust server API (GET /scene).
 */

export interface BodyInfo {
  id: number;
  name: string;
  mass: number;
  com: [number, number, number];
  parent_id: number | null;
  transform: TransformInfo;
}

export interface TransformInfo {
  translation: [number, number, number];
  rotation: [number, number, number, number]; // w, x, y, z
}

export interface JointInfo {
  id: number;
  name: string;
  joint_type: string;
  body_a: number;
  body_b: number;
  axis: [number, number, number] | null;
  limits: { lower: number; upper: number } | null;
}

export interface MuscleInfo {
  id: number;
  name: string;
  max_isometric_force: number;
  optimal_fiber_length: number;
  tendon_slack_length: number;
}

export interface MusclePathInfo {
  muscle_id: number;
  muscle_name: string;
  points: MusclePathPoint[];
}

export interface MusclePathPoint {
  body: number;
  location: [number, number, number];
}

export interface SiteInfo {
  id: number;
  name: string;
  parent: number;
  offset: [number, number, number];
}

export interface MeshInfo {
  id: number;
  name: string;
  parent: number;
  path: string;
  offset: [number, number, number];
  rotation?: [number, number, number, number]; // w, x, y, z
  url: string;
  scale?: [number, number, number];
  color?: [number, number, number];
  opacity?: number;
}

export interface Scene {
  num_entities: number;
  bodies: BodyInfo[];
  joints: JointInfo[];
  muscles: MuscleInfo[];
  muscle_paths: MusclePathInfo[];
  sites: SiteInfo[];
  meshes: MeshInfo[];
}
