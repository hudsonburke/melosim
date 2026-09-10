import { invoke, isTauri } from "@tauri-apps/api/core";
export type Vec3 = [number, number, number];
export type Quat = [number, number, number, number];
export type Node = {
  id: string;
  parent: string | null;
  name: string;
  kind: string;
  position: Vec3;
  quaternion: Quat;
  matrix: number[];
  mesh: string | null;
  rgba: Quat | null;
  value: number | null;
  range: [number, number] | null;
  driven: boolean;
  mass: number | null;
  path: string[];
  length: number | null;
  cable: [number, number, number, number, number] | null;
  editable: boolean;
};
export type MeshData = { id: string; positions: Vec3[]; indices: number[] };
export type Snapshot = {
  model: string;
  epoch: number;
  nodes: Node[];
  meshes: MeshData[];
  issues: string[];
  selected: string | null;
  message: string;
  can_undo: boolean;
  can_redo: boolean;
};
export type Command = { type: string; [key: string]: unknown };
export const desktop = isTauri();
export async function command(command: Command): Promise<Snapshot> {
  if (desktop) return invoke<Snapshot>("editor_command", { command });
  let response: Response;
  try {
    response = await fetch("/api/command", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(command),
    });
  } catch {
    throw new Error(
      "Cannot connect to the editor. Start the Rust backend with npm run backend.",
    );
  }
  let data;
  try {
    data = JSON.parse(await response.text());
  } catch {
    throw new Error(
      "The Rust editor is offline. Run npm run backend, then reconnect.",
    );
  }
  if (!response.ok) throw new Error(data.error || "Editor request failed");
  return data;
}
export async function pickPath(
  save = false,
  mesh = false,
): Promise<string | null> {
  return invoke("choose_path", { save, mesh });
}
