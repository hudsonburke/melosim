use crate::world::World;

/// An ordered collection of systems that process the World.
///
/// Each system is a standalone function that reads/writes specific concrete
/// component types. Systems are registered at startup and run in order.
///
/// To add a custom joint type (or any new component):
/// 1. Define a concrete component struct
/// 2. Write a system function: `fn my_system(world: &mut World)`
/// 3. Register: `registry.add("my_system", my_system)`
///
/// No changes to World, no changes to existing systems, no trait objects.
pub struct SystemRegistry {
    systems: Vec<SystemEntry>,
}

struct SystemEntry {
    name: &'static str,
    function: Box<dyn Fn(&mut World)>,
}

impl SystemRegistry {
    pub fn new() -> Self {
        Self {
            systems: Vec::new(),
        }
    }

    /// Register a system function by name.
    pub fn add(&mut self, name: &'static str, function: impl Fn(&mut World) + 'static) {
        self.systems.push(SystemEntry {
            name,
            function: Box::new(function),
        });
    }

    /// Run all registered systems in order.
    pub fn run(&self, world: &mut World) {
        for entry in &self.systems {
            (entry.function)(world);
        }
    }

    /// Number of registered systems.
    pub fn len(&self) -> usize {
        self.systems.len()
    }

    /// Returns true if no systems are registered.
    pub fn is_empty(&self) -> bool {
        self.systems.is_empty()
    }

    /// List registered system names.
    pub fn list(&self) -> impl Iterator<Item = &str> {
        self.systems.iter().map(|e| e.name)
    }
}

impl std::fmt::Debug for SystemRegistry {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let names: Vec<&str> = self.systems.iter().map(|e| e.name).collect();
        f.debug_struct("SystemRegistry")
            .field("systems", &names)
            .finish()
    }
}
