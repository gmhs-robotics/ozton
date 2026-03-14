use std::{borrow::Cow, collections::BTreeMap, path::PathBuf};

#[derive(Debug, Clone, Default)]
pub struct RouteIndex {
    /// route id -> display name
    pub map: BTreeMap<u32, String>,
}

impl RouteIndex {
    pub const INDEX_PATH: &'static str = ".routes";
    pub const ROUTE_EXTENSION: &'static str = "route";

    pub fn load() -> Self {
        crate::log!("routes.load: reading {}", Self::INDEX_PATH);
        let Ok(text) = std::fs::read_to_string(Self::INDEX_PATH) else {
            crate::log!("routes.load: no index file, using default");
            return Self::default();
        };

        let mut out = Self::default();

        for (line_index, line) in text.lines().enumerate() {
            let (route_id, name) = match line.trim().split_once('\t') {
                Some((a, b)) => (a.trim(), b.trim()),
                None => (line, line),
            };

            if route_id.is_empty() || name.is_empty() {
                crate::log!("routes.load: skipping empty line {}", line_index + 1);
                continue;
            }

            let Ok(route_id) = route_id.parse::<u32>() else {
                crate::log!(
                    "routes.load: skipping unparsable route id on line {}: {}",
                    line_index + 1,
                    route_id
                );
                continue;
            };

            crate::log!(
                "routes.load: line {} -> route {} name={}",
                line_index + 1,
                route_id,
                name
            );
            out.map.insert(route_id, name.to_string());
        }

        crate::log!("routes.load: loaded {} route entries", out.map.len());
        out
    }

    pub fn iter(&self) -> impl Iterator<Item = (u32, &str)> + '_ {
        self.map.iter().map(|(id, name)| (*id, name.as_str()))
    }

    pub fn len(&self) -> usize {
        self.map.len()
    }

    #[allow(dead_code)]
    pub fn save(&self) -> std::io::Result<()> {
        crate::log!(
            "routes.save: writing {} entries to {}",
            self.map.len(),
            Self::INDEX_PATH
        );
        let mut out = String::with_capacity(
            self.map
                .iter()
                .map(|(route_id, name)| route_id.to_string().len() + name.len() + 2)
                .sum(),
        );

        for (route_id, name) in &self.map {
            out.push_str(&route_id.to_string());
            out.push('\t');
            out.push_str(name);
            out.push('\n');
        }

        let result = std::fs::write(Self::INDEX_PATH, out);
        if let Err(error) = &result {
            crate::log!("routes.save: failed: {error}");
        } else {
            crate::log!("routes.save: success");
        }
        result
    }

    #[allow(dead_code)]
    pub fn update(&mut self, route_id: u32, name: &str) {
        crate::log!("routes.update: route_id={} name={}", route_id, name);
        self.map.insert(route_id, name.to_string());
    }

    pub fn display_name(&self, id: u32) -> Option<&str> {
        self.map.get(&id).map(String::as_str)
    }

    pub fn display_name_or_id(&self, id: u32) -> Cow<'_, str> {
        self.display_name(id)
            .map(Cow::Borrowed)
            .unwrap_or_else(|| Cow::Owned(id.to_string()))
    }

    #[allow(dead_code)]
    pub fn next_id(&self) -> u32 {
        let next_id = self.map.keys().max().map(|id| id + 1).unwrap_or(1);
        crate::log!("routes.next_id: {}", next_id);
        next_id
    }

    pub fn path_for(id: u32) -> PathBuf {
        let mut path = id.to_string();
        path.push('.');
        path.push_str(Self::ROUTE_EXTENSION);
        PathBuf::from(path)
    }
}
