// Debug tool: dump MuJoCo's mesh frame corrections (mesh_pos/mesh_quat)
// for an MJCF file, and verify them against geom AABBs.
// Run: cargo run --example dump_mesh -- tests/fixtures/myo_sim/arm/myoarm.xml
use mujoco_rs::wrappers::mj_model::*;
use mujoco_rs::mujoco_c::*;

fn main() {
    let path = std::env::args().nth(1).expect("usage: dump_mesh <file.xml>");
    let model = MjModel::from_xml(&path).expect("load failed");

    let ngeom = model.ngeom() as usize;
    let mesh_pos = model.mesh_pos();
    let mesh_quat = model.mesh_quat();
    let mesh_scale = model.mesh_scale();
    let geom_dataid = model.geom_dataid();
    let geom_type = model.geom_type();
    let geom_aabb = model.geom_aabb();

    let mut any = false;
    for g in 0..ngeom {
        if geom_type[g] != mjtGeom_::mjGEOM_MESH { continue; }
        let mid = geom_dataid[g] as usize;
        let p = mesh_pos[mid];
        let q = mesh_quat[mid];
        let s = mesh_scale[mid];
        let aabb = geom_aabb[g];
        let moved = p.iter().any(|v| v.abs() > 1e-9)
            || (q[0] - 1.0).abs() > 1e-9
            || q[1..].iter().any(|v| v.abs() > 1e-9)
            || s.iter().any(|v| (v - 1.0).abs() > 1e-9);
        if moved {
            any = true;
            let name = model.id_to_name(MjtObj::mjOBJ_GEOM, g).unwrap_or("?");
            println!("geom {name:20} mesh_pos={p:.4?} mesh_quat={q:.4?} mesh_scale={s:.2?} aabb_center={:.4?}", &aabb[0..3]);
        }
    }
    if !any { println!("no mesh frame corrections (all identity)"); }

    // ── Verify correction convention against geom_aabb ──
    // geom_aabb = AABB of the processed (re-centered/re-oriented) mesh in the geom frame.
    // Transform raw STL verts with both candidate conventions and see which matches.
    let stl = std::env::args().nth(2).expect("usage: dump_mesh <file.xml> <mesh.stl>");
    let verts = read_binary_stl(&stl);
    println!("\n{}: {} verts", stl, verts.len());

    for g in 0..ngeom {
        if geom_type[g] != mjtGeom_::mjGEOM_MESH { continue; }
        let mid = geom_dataid[g] as usize;
        let name = model.id_to_name(MjtObj::mjOBJ_GEOM, g).unwrap_or("?").to_string();
        if !stl.contains(&name) && !stl.contains(name.trim_end_matches("_geom_1")) { continue; }

        let p = mesh_pos[mid];
        let q = mesh_quat[mid];
        let mesh_name = model.id_to_name(MjtObj::mjOBJ_MESH, mid);
        println!("{name} geom_pos={:.6?} mesh_id={mid} mesh_name={mesh_name:?}", model.geom_pos()[g]);

        // Expected composed display transform, computed with MuJoCo's own math:
        //   q_pre = conj(mesh_quat); t_pre = q_pre * (-mesh_pos)
        //   R = geom_quat ⊗ q_pre;  T = geom_pos + geom_quat * t_pre
        let gq = model.geom_quat()[g];
        let gp = model.geom_pos()[g];
        unsafe {
            let q_pre = [q[0], -q[1], -q[2], -q[3]];
            let mut t_pre = [0.0; 3];
            let neg_mp = [-p[0], -p[1], -p[2]];
            mju_rotVecQuat(&mut t_pre, &neg_mp, &q_pre);
            let mut r = [0.0; 4];
            mju_mulQuat(&mut r, &gq, &q_pre);
            let mut rt = [0.0; 3];
            mju_rotVecQuat(&mut rt, &t_pre, &gq);
            let t = [gp[0] + rt[0], gp[1] + rt[1], gp[2] + rt[2]];
            println!("{name} expected_offset={t:.6?} expected_rotation={r:.6?}");
        }
        let qc = [q[0], -q[1], -q[2], -q[3]];
        let aabb = geom_aabb[g];
        for (label, qq) in [("mesh_quat       ", q), ("conj(mesh_quat)", qc)] {
            let (mut lo, mut hi) = ([f64::MAX; 3], [f64::MIN; 3]);
            for v in &verts {
                let d = [v[0] - p[0], v[1] - p[1], v[2] - p[2]];
                let r = rot_quat(qq, d);
                for i in 0..3 { lo[i] = lo[i].min(r[i]); hi[i] = hi[i].max(r[i]); }
            }
            let center = [(lo[0]+hi[0])/2.0, (lo[1]+hi[1])/2.0, (lo[2]+hi[2])/2.0];
            let half = [(hi[0]-lo[0])/2.0, (hi[1]-lo[1])/2.0, (hi[2]-lo[2])/2.0];
            let cerr: f64 = (0..3).map(|i| (center[i]-aabb[i]).abs()).fold(0.0, f64::max);
            let herr: f64 = (0..3).map(|i| (half[i]-aabb[3+i]).abs()).fold(0.0, f64::max);
            println!("{label} center_err={cerr:.6} half_err={herr:.6}");
        }
    }
}

fn rot_quat(q: [f64; 4], v: [f64; 3]) -> [f64; 3] {
    let (w, x, y, z) = (q[0], q[1], q[2], q[3]);
    // v + 2*w*(u×v) + 2*(u×(u×v))
    let c = [y*v[2]-z*v[1], z*v[0]-x*v[2], x*v[1]-y*v[0]];
    let cc = [y*c[2]-z*c[1], z*c[0]-x*c[2], x*c[1]-y*c[0]];
    [v[0]+2.0*(w*c[0]+cc[0]), v[1]+2.0*(w*c[1]+cc[1]), v[2]+2.0*(w*c[2]+cc[2])]
}

fn read_binary_stl(path: &str) -> Vec<[f64; 3]> {
    let data = std::fs::read(path).expect("stl read");
    let n = u32::from_le_bytes(data[80..84].try_into().unwrap()) as usize;
    let mut verts = Vec::with_capacity(n * 3);
    for t in 0..n {
        let base = 84 + t * 50;
        for k in 0..3 {
            let o = base + 12 + k * 12;
            let f = |i: usize| f32::from_le_bytes(data[o+i*4..o+i*4+4].try_into().unwrap()) as f64;
            verts.push([f(0), f(1), f(2)]);
        }
    }
    verts
}
