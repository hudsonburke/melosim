//! Preserve the source scene alongside the editable ECS projection.
use bevy::prelude::*;
use mujoco_rs::wrappers::SpecItem;
use mujoco_rs::wrappers::mj_editing::MjSpec;
use std::{collections::HashMap, path::Path, sync::Arc};
use xmltree::{Element, XMLNode};

#[derive(Component, Clone)]
pub struct MjcfDocument {
    pub xml: String,
    pub assets: HashMap<String, Arc<[u8]>>,
    pub names: HashMap<Entity, String>,
    pub colors: HashMap<Entity, [f32; 4]>,
    pub warning: Option<String>,
}

pub fn parse(xml: &str) -> Result<Element, String> {
    Element::parse(xml.as_bytes()).map_err(|e| e.to_string())
}
pub fn encode(root: &Element) -> Result<String, String> {
    let mut bytes = Vec::new();
    root.write(&mut bytes).map_err(|e| e.to_string())?;
    String::from_utf8(bytes).map_err(|e| e.to_string())
}
pub fn spec_xml(spec: &MjSpec) -> Result<String, String> {
    use mujoco_rs::error::MjEditError;
    match spec.save_xml_string(1024 * 1024) {
        Ok(xml) => Ok(xml),
        Err(MjEditError::XmlBufferTooSmall { required_size }) => spec
            .save_xml_string(required_size + 1)
            .map_err(|e| e.to_string()),
        Err(e) => Err(e.to_string()),
    }
}
fn files(
    root: &mut Element,
    f: &mut impl FnMut(&str, &str) -> Result<String, String>,
) -> Result<(), String> {
    for (key, value) in &mut root.attributes {
        if key == "file" || (root.name == "texture" && key.starts_with("file")) {
            if !value.is_empty() {
                *value = f(&root.name, value)?;
            }
        }
    }
    for node in &mut root.children {
        if let XMLNode::Element(e) = node {
            files(e, f)?;
        }
    }
    Ok(())
}
impl MjcfDocument {
    pub fn capture(
        spec: &MjSpec,
        directory: &Path,
        names: HashMap<Entity, String>,
    ) -> Result<Self, String> {
        Self::from_tree(parse(&spec_xml(spec)?)?, directory, names)
    }

    fn from_tree(
        mut root: Element,
        directory: &Path,
        names: HashMap<Entity, String>,
    ) -> Result<Self, String> {
        let mut meshdir = String::new();
        let mut texturedir = String::new();
        for compiler in root
            .children
            .iter()
            .filter_map(XMLNode::as_element)
            .filter(|e| e.name == "compiler")
        {
            if let Some(v) = compiler.attributes.get("assetdir") {
                meshdir = v.clone();
                texturedir = v.clone();
            }
            if let Some(v) = compiler.attributes.get("meshdir") {
                meshdir = v.clone();
            }
            if let Some(v) = compiler.attributes.get("texturedir") {
                texturedir = v.clone();
            }
        }
        let mut assets = HashMap::new();
        files(&mut root, &mut |kind, file| {
            let subdir = match kind {
                "mesh" => meshdir.as_str(),
                "texture" => texturedir.as_str(),
                _ => "",
            };
            let path = directory.join(subdir).join(file);
            let path = path
                .canonicalize()
                .or_else(|_| directory.join(file).canonicalize())
                .map_err(|e| format!("Cannot preserve asset {file}: {e}"))?;
            let key = path.to_string_lossy().into_owned();
            assets.insert(
                key.clone(),
                Arc::from(std::fs::read(&path).map_err(|e| e.to_string())?),
            );
            Ok(key)
        })?;
        for compiler in root
            .children
            .iter_mut()
            .filter_map(XMLNode::as_mut_element)
            .filter(|e| e.name == "compiler")
        {
            for key in ["meshdir", "texturedir", "assetdir"] {
                compiler.attributes.remove(key);
            }
        }
        Ok(Self {
            xml: encode(&root)?,
            assets,
            names,
            colors: HashMap::new(),
            warning: None,
        })
    }

    /// Retain even an invalid source; saving a project must not erase its unsupported sections.
    pub fn capture_uncompiled(
        path: &Path,
        names: HashMap<Entity, String>,
        warning: String,
    ) -> Result<Self, String> {
        fn expand(path: &Path, active: &mut Vec<std::path::PathBuf>) -> Result<Element, String> {
            let path = path.canonicalize().map_err(|e| e.to_string())?;
            if active.contains(&path) {
                return Err("Cyclic MJCF include".into());
            }
            active.push(path.clone());
            let mut root = parse(&std::fs::read_to_string(&path).map_err(|e| e.to_string())?)?;
            fn walk(
                root: &mut Element,
                dir: &Path,
                active: &mut Vec<std::path::PathBuf>,
            ) -> Result<(), String> {
                let mut children = Vec::new();
                for node in std::mem::take(&mut root.children) {
                    if let XMLNode::Element(mut e) = node {
                        if e.name == "include" {
                            let file = e.attributes.get("file").ok_or("Include without file")?;
                            children.extend(expand(&dir.join(file), active)?.children);
                        } else {
                            walk(&mut e, dir, active)?;
                            children.push(XMLNode::Element(e));
                        }
                    } else {
                        children.push(node);
                    }
                }
                root.children = children;
                Ok(())
            }
            walk(&mut root, path.parent().unwrap(), active)?;
            // Included asset paths are relative to the file that declared them.
            // Leave unresolved paths for the top-level compiler mesh/texture directories.
            files(&mut root, &mut |_, file| {
                Ok(path
                    .parent()
                    .unwrap()
                    .join(file)
                    .canonicalize()
                    .map(|p| p.to_string_lossy().into_owned())
                    .unwrap_or_else(|_| file.to_owned()))
            })?;
            active.pop();
            Ok(root)
        }
        let mut document = Self::from_tree(
            expand(path, &mut Vec::new())?,
            path.parent().unwrap(),
            names,
        )?;
        document.warning = Some(warning);
        Ok(document)
    }

    /// Explicit geometry/appearance export, never used implicitly by normal export.
    pub fn visual_only(&self) -> Result<Self, String> {
        let mut root = parse(&self.xml)?;
        root.children.retain(|node| {
            !node.as_element().is_some_and(|e| {
                matches!(
                    e.name.as_str(),
                    "tendon" | "actuator" | "equality" | "sensor" | "contact" | "keyframe"
                )
            })
        });
        let mut result = self.clone();
        let mut spec = MjSpec::from_xml_string(&encode(&root)?).map_err(|e| e.to_string())?;
        let used: std::collections::HashSet<_> =
            spec.geom_iter().map(|g| g.name().to_owned()).collect();
        for (index, geom) in spec.geom_iter_mut().enumerate() {
            if geom.name().is_empty() {
                let mut name = format!("melosim_geom_{index}");
                while used.contains(&name) {
                    name.push('_');
                }
                geom.set_name(&name).map_err(|e| e.to_string())?;
            }
        }
        spec.compile().map_err(|e| e.to_string())?;
        result.xml = spec_xml(&spec)?;
        result.warning = None;
        Ok(result)
    }

    /// Write a source scene and all external assets without depending on their original files.
    pub fn package(&self, directory: &Path, filename: &str) -> Result<(), String> {
        let stamp = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_nanos();
        let asset_directory = format!("assets_{stamp}");
        std::fs::create_dir_all(directory.join(&asset_directory)).map_err(|e| e.to_string())?;
        let mut root = parse(&self.xml)?;
        let mut copied: HashMap<String, String> = HashMap::new();
        files(&mut root, &mut |_, source| {
            if let Some(path) = copied.get(source) {
                return Ok(path.clone());
            }
            let bytes = self
                .assets
                .get(source)
                .ok_or_else(|| format!("Missing captured asset {source}"))?;
            let extension = Path::new(source)
                .extension()
                .and_then(|e| e.to_str())
                .unwrap_or("bin");
            let relative = format!("{asset_directory}/asset_{}.{}", copied.len(), extension);
            std::fs::write(directory.join(&relative), bytes).map_err(|e| e.to_string())?;
            copied.insert(source.to_owned(), relative.clone());
            Ok(relative)
        })?;
        std::fs::write(directory.join(filename), encode(&root)?).map_err(|e| e.to_string())
    }
}

fn named_mut<'a>(root: &'a mut Element, tag: &str, name: &str) -> Option<&'a mut Element> {
    if root.name == tag && root.attributes.get("name").is_some_and(|n| n == name) {
        return Some(root);
    }
    root.children
        .iter_mut()
        .filter_map(XMLNode::as_mut_element)
        .find_map(|e| named_mut(e, tag, name))
}
fn rename(root: &mut Element, names: &HashMap<String, String>) {
    for (key, value) in &mut root.attributes {
        if matches!(
            key.as_str(),
            "name"
                | "body"
                | "body1"
                | "body2"
                | "joint"
                | "joint1"
                | "joint2"
                | "geom"
                | "geom1"
                | "geom2"
                | "site"
                | "site1"
                | "site2"
                | "tendon"
                | "target"
        ) {
            if let Some(new) = names.get(value) {
                *value = new.clone();
            }
        }
    }
    for e in root.children.iter_mut().filter_map(XMLNode::as_mut_element) {
        rename(e, names);
    }
}

/// Overlay editable physical elements; keep materials, primitives, visual settings,
/// lights, cameras and source-only constructs instead of reconstructing them.
pub fn merge(
    document: &MjcfDocument,
    generated: &MjSpec,
    names: &HashMap<String, String>,
    colors: &std::collections::HashSet<String>,
) -> Result<MjSpec, String> {
    let mut source = parse(&document.xml)?;
    rename(&mut source, names);
    let generated = parse(&spec_xml(generated)?)?;
    fn overlay(
        source: &mut Element,
        generated: &Element,
        colors: &std::collections::HashSet<String>,
    ) {
        for child in generated.children.iter().filter_map(XMLNode::as_element) {
            let name = child
                .attributes
                .get("name")
                .map(String::as_str)
                .unwrap_or("");
            if child.name == "body" {
                if let Some(existing) = named_mut(source, "body", name) {
                    for key in ["pos", "quat"] {
                        existing.attributes.remove(key);
                        if let Some(v) = child.attributes.get(key) {
                            existing.attributes.insert(key.into(), v.clone());
                        }
                    }
                    overlay(existing, child, colors);
                } else {
                    source.children.push(XMLNode::Element(child.clone()));
                }
            } else if matches!(child.name.as_str(), "geom" | "site" | "joint") {
                if let Some(existing) = named_mut(source, &child.name, name) {
                    // Keep the original asset/material reference and all source appearance properties.
                    // Mesh positions must remain in the authored asset's frame (generated uses that too).
                    let keys: &[&str] = if child.name == "geom" {
                        if colors.contains(name) {
                            &["pos", "quat", "rgba"]
                        } else {
                            &["pos", "quat"]
                        }
                    } else if child.name == "site" {
                        &["pos"]
                    } else {
                        &[]
                    };
                    for &key in keys {
                        existing.attributes.remove(key);
                        if let Some(v) = child.attributes.get(key) {
                            existing.attributes.insert(key.into(), v.clone());
                        }
                    }
                } else {
                    source.children.push(XMLNode::Element(child.clone()));
                }
            }
        }
    }
    if let (Some(src), Some(gen_world)) = (
        source.get_mut_child("worldbody"),
        generated.get_child("worldbody"),
    ) {
        overlay(src, gen_world, colors);
    }
    // A solid editor color must also work for MuJoCo's special default gray:
    // geom rgba=.5,.5,.5,1 otherwise defers to the old material's color.
    // Give edited geoms an explicit solid material; leave shared originals intact.
    let mut generated_for_colors = generated.clone();
    for (index, name) in colors.iter().enumerate() {
        let Some(geom) = named_mut(&mut generated_for_colors, "geom", name) else {
            continue;
        };
        let rgba = geom
            .attributes
            .get("rgba")
            .cloned()
            .unwrap_or_else(|| "0.5 0.5 0.5 1".into());
        let mut material_name = format!("melosim_color_{index}");
        while named_mut(&mut source, "material", &material_name).is_some() {
            material_name.push('_');
        }
        let mut material = Element::new("material");
        material
            .attributes
            .insert("name".into(), material_name.clone());
        material.attributes.insert("rgba".into(), rgba.clone());
        if let Some(geom) = named_mut(&mut source, "geom", name) {
            geom.attributes.insert("material".into(), material_name);
            geom.attributes.insert("rgba".into(), rgba);
        }
        if source.get_child("asset").is_none() {
            source
                .children
                .push(XMLNode::Element(Element::new("asset")));
        }
        source
            .get_mut_child("asset")
            .unwrap()
            .children
            .push(XMLNode::Element(material));
    }
    // Add assets needed by new editor geoms; originals retain their own mesh references.
    if let Some(assets) = generated.get_child("asset") {
        let mut additions = assets.clone();
        let used = encode(&source)?;
        additions.children.retain(|n| {
            n.as_element()
                .and_then(|e| e.attributes.get("name"))
                .is_some_and(|name| used.contains(&format!("mesh=\"{name}\"")))
        });
        if let Some(existing) = source.get_mut_child("asset") {
            existing.children.extend(additions.children);
        } else if !additions.children.is_empty() {
            source.children.push(XMLNode::Element(additions));
        }
    }
    for section in ["tendon", "actuator"] {
        if let Some(elements) = generated.get_child(section) {
            let original = source
                .get_child(section)
                .cloned()
                .unwrap_or_else(|| Element::new(section));
            let tendon_names: Vec<_> = source
                .get_child("tendon")
                .into_iter()
                .flat_map(|t| t.children.iter())
                .filter_map(XMLNode::as_element)
                .filter_map(|e| e.attributes.get("name"))
                .cloned()
                .collect();
            let mut added = original.clone();
            for e in elements.children.iter().filter_map(XMLNode::as_element) {
                let name = e.attributes.get("name").cloned().unwrap_or_default();
                let imported = if section == "tendon" {
                    tendon_names.contains(&name.strip_suffix("_tendon").unwrap_or(&name).to_owned())
                } else {
                    tendon_names.contains(&name)
                };
                if !imported
                    && !original
                        .children
                        .iter()
                        .filter_map(XMLNode::as_element)
                        .any(|old| old.attributes.get("name") == Some(&name))
                {
                    added.children.push(XMLNode::Element(e.clone()));
                }
            }
            if let Some(existing) = source.get_mut_child(section) {
                *existing = added;
            } else {
                source.children.push(XMLNode::Element(added));
            }
        }
    }
    MjSpec::from_xml_string(&encode(&source)?).map_err(|e| e.to_string())
}
