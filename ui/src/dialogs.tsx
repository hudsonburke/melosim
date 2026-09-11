import { useState, type FormEvent } from "react";
import { desktop, pickPath, type Node, type Command } from "./api";
export type Dialog =
  | "import"
  | "body"
  | "part"
  | "joint"
  | "cable"
  | "site"
  | "export"
  | "save_project"
  | "open_project";
export function CommandDialog({
  dialog,
  parents,
  cables,
  nodes,
  selected,
  busy,
  error,
  close,
  send,
}: {
  dialog: Dialog;
  parents: Node[];
  cables: Node[];
  nodes: Node[];
  selected: Node | null;
  busy: boolean;
  error: string;
  close: () => void;
  send: (c: Command) => Promise<boolean>;
}) {
  const parentDefault =
    parents.find((p) => p.id === selected?.id)?.id || parents[0]?.id || "";
  const [path, setPath] = useState(""),
    [localError, setLocalError] = useState("");
  const title = {
    import: "Open MuJoCo model",
    body: "Add exoskeleton body",
    part: "Import exoskeleton mesh",
    joint: "Connect a joint",
    cable: "Create cable",
    site: "Add cable path site",
    export: "Export MuJoCo model",
    save_project: "Save project as",
    open_project: "Open project",
  }[dialog];
  const option = (n: Node) => (
    <option key={n.id} value={n.id}>
      {n.name}
    </option>
  );
  async function submit(e: FormEvent<HTMLFormElement>) {
    e.preventDefault();
    const data = new FormData(e.currentTarget),
      text = (k: string) => String(data.get(k) || ""),
      num = (k: string) => Number(data.get(k));
    const values: Record<Dialog, Command> = {
      import: { type: "import", path },
      export: { type: data.has("visualOnly") ? "export_visual" : "export", path },
      save_project: { type: "save_project", path },
      open_project: { type: "open_project", path },
      body: {
        type: "add_body",
        parent: text("parent"),
        name: text("name"),
        mass: num("mass"),
        size: [num("x"), num("y"), num("z")],
      },
      part: {
        type: "import_part",
        parent: text("parent"),
        name: text("name"),
        mass: num("mass"),
        path,
      },
      joint: {
        type: "add_joint",
        parent: text("parent"),
        child: text("child"),
        name: text("name"),
        kind: text("kind"),
        axis: [num("x"), num("y"), num("z")],
      },
      cable: { type: "add_cable", name: text("name") },
      site: {
        type: "add_site",
        parent: text("parent"),
        cable: text("cable"),
        name: text("name"),
        position: [num("x"), num("y"), num("z")],
      },
    };
    await send(values[dialog]);
  }
  return (
    <div className="modal-backdrop">
      <form className="modal" onSubmit={submit}>
        <div className="modal-title">
          <h2>{title}</h2>
          <button type="button" onClick={close} aria-label="Close dialog">
            ×
          </button>
        </div>
        {dialog === "export" && (
          <label>
            <input type="checkbox" name="visualOnly" />
            Visual-only (omit tendons, actuators, constraints, contacts, sensors
            and keyframes)
          </label>
        )}
        {["import", "part", "export", "save_project", "open_project"].includes(
          dialog,
        ) && (
          <>
            <label>
              {dialog === "save_project"
                ? "New project folder (.melosim)"
                : dialog === "open_project"
                  ? "Project folder or project.json path"
                  : dialog === "export"
                    ? "New output path"
                    : "Local file path"}
              <div className="inline-field">
                <input
                  autoFocus
                  required
                  value={path}
                  onChange={(e) => setPath(e.target.value)}
                  placeholder={
                    dialog === "part"
                      ? "/path/to/part.stl"
                      : dialog === "export"
                        ? "/path/to/new-model.xml"
                        : "/path/to/model.xml"
                  }
                />
                {desktop &&
                  !["save_project", "open_project"].includes(dialog) && (
                    <button
                      type="button"
                      onClick={() => {
                        void pickPath(dialog === "export", dialog === "part")
                          .then((p) => {
                            if (p) setPath(p);
                          })
                          .catch((e) => setLocalError(String(e)));
                      }}
                    >
                      Browse
                    </button>
                  )}
              </div>
            </label>
            {!desktop && (
              <p className="hint">
                Use an absolute path on this computer. Native file dialogs are
                available in the desktop app.
              </p>
            )}
          </>
        )}
        {dialog === "import" && (
          <p className="hint">
            Opening a model replaces the current workspace. Export changes you
            want to keep first.
          </p>
        )}
        {dialog === "save_project" && (
          <p className="hint">
            Creates a portable folder with the source scene, assets, and edit
            history. Choose a new folder name. This preserves editor data that
            MJCF export alone cannot retain.
          </p>
        )}
        {dialog === "open_project" && (
          <p className="hint">
            Opening a project replaces the workspace only after successful
            validation. Save current edits first.
          </p>
        )}
        {!["import", "export", "save_project", "open_project"].includes(
          dialog,
        ) && (
          <label>
            Name
            <input
              name="name"
              required
              defaultValue={`${dialog}_${nodes.length + 1}`}
            />
          </label>
        )}
        {["body", "part", "joint", "site"].includes(dialog) && (
          <label>
            Parent body or frame
            <select name="parent" defaultValue={parentDefault} required>
              {parents.map(option)}
            </select>
          </label>
        )}
        {["body", "part"].includes(dialog) && (
          <label>
            Mass (kg)
            <input
              name="mass"
              required
              type="number"
              min="0.0001"
              step="any"
              defaultValue="0.1"
            />
          </label>
        )}
        {dialog === "joint" && (
          <>
            <label>
              Child body
              <select
                name="child"
                required
                defaultValue={
                  nodes.find((n) => n.kind === "body" && n.id !== parentDefault)
                    ?.id
                }
              >
                {nodes.filter((n) => n.kind === "body").map(option)}
              </select>
            </label>
            <label>
              Connection
              <select name="kind">
                <option value="hinge">Hinge</option>
                <option value="slide">Slide</option>
                <option value="fixed">Fixed</option>
              </select>
            </label>
            <p className="hint">
              Preserves the current pose. Add joints to new parts; existing
              articulated body connections are protected.
            </p>
          </>
        )}
        {dialog === "site" && (
          <label>
            Cable
            <select
              name="cable"
              required
              defaultValue={
                selected?.kind === "cable" ? selected.id : cables[0]?.id
              }
            >
              {cables.map(option)}
            </select>
          </label>
        )}
        {["body", "joint", "site"].includes(dialog) && (
          <>
            <h3>
              {dialog === "body"
                ? "Box dimensions (m)"
                : dialog === "joint"
                  ? "Joint axis"
                  : "Local position (m)"}
            </h3>
            <div className="xyz">
              {["x", "y", "z"].map((v, i) => (
                <label key={v}>
                  {v.toUpperCase()}
                  <input
                    name={v}
                    type="number"
                    step="any"
                    required
                    min={dialog === "body" ? "0.001" : undefined}
                    defaultValue={
                      dialog === "body"
                        ? [0.08, 0.04, 0.04][i]
                        : dialog === "joint"
                          ? i === 2
                            ? 1
                            : 0
                          : 0
                    }
                  />
                </label>
              ))}
            </div>
          </>
        )}
        {dialog === "part" && (
          <p className="hint">
            STL and OBJ meshes are compiled by MuJoCo. Geometry and inertial
            properties are retained for export.
          </p>
        )}
        {dialog === "export" && (
          <p className="hint">
            Writes MJCF and portable mesh/texture assets. Imported scene
            appearances and source-only elements are retained. Newly authored
            cable dynamics are not yet fully exported; use Save project to
            preserve editor data.
          </p>
        )}
        {(error || localError) && (
          <p className="error" role="alert">
            {error || localError}
          </p>
        )}
        <div className="modal-actions">
          <button type="button" onClick={close}>
            Cancel
          </button>
          <button className="primary" disabled={busy} type="submit">
            {busy
              ? "Working…"
              : dialog === "export"
                ? "Export"
                : dialog === "import"
                  ? "Open model"
                  : dialog === "save_project"
                    ? "Save project"
                    : dialog === "open_project"
                      ? "Open project"
                      : "Create"}
          </button>
        </div>
      </form>
    </div>
  );
}
