use std::{collections::BTreeMap, path::PathBuf};

#[derive(Debug, Clone, Default)]
pub struct RouteIndex {
    /// route id -> display name
    pub map: BTreeMap<u32, String>,
}

#[derive(Debug, Clone)]
pub struct RouteEntry {
    pub id: u32,
    pub display_name: String,
}

impl RouteIndex {
    pub const INDEX_PATH: &'static str = ".routes";
    pub const ROUTE_EXTENSION: &'static str = "route";

    pub fn load() -> Self {
        let Ok(text) = std::fs::read_to_string(Self::INDEX_PATH) else {
            return Self::default();
        };

        let mut out = Self::default();

        for line in text.lines() {
            let (route_id, name) = match line.trim().split_once('\t') {
                Some((a, b)) => (a.trim(), b.trim()),
                None => (line, line),
            };

            if route_id.is_empty() || name.is_empty() {
                continue;
            }

            let Ok(route_id) = route_id.parse::<u32>() else {
                continue;
            };

            out.map.insert(route_id, name.to_string());
        }

        out
    }

    pub fn entries(&self) -> Vec<RouteEntry> {
        let mut entries: Vec<RouteEntry> = self
            .map
            .iter()
            .map(|(id, name)| RouteEntry {
                id: *id,
                display_name: name.clone(),
            })
            .collect();

        entries.sort_by_key(|entry| entry.id);
        entries
    }

    #[allow(dead_code)]
    pub fn save(&self) -> std::io::Result<()> {
        let mut out = String::new();

        for (route_id, name) in &self.map {
            out.push_str(&route_id.to_string());
            out.push('\t');
            out.push_str(name);
            out.push('\n');
        }

        std::fs::write(Self::INDEX_PATH, out)
    }

    #[allow(dead_code)]
    pub fn update(&mut self, route_id: u32, name: &str) {
        self.map.insert(route_id, name.to_string());
    }

    pub fn display_name(&self, id: u32) -> String {
        self.map.get(&id).cloned().unwrap_or_else(|| id.to_string())
    }

    #[allow(dead_code)]
    pub fn next_id(&self) -> u32 {
        self.map.keys().max().map(|id| id + 1).unwrap_or(1)
    }

    pub fn path_for(id: u32) -> PathBuf {
        PathBuf::from(format!("{id}.{}", Self::ROUTE_EXTENSION))
    }
}
