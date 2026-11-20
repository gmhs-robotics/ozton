use std::{borrow::Cow, collections::BTreeMap, path::PathBuf};

use crate::store::RouteStore;

#[derive(Debug, Clone, Default)]
pub struct RouteIndex {
    /// route id -> display name
    pub map: BTreeMap<u32, String>,
}

impl RouteIndex {
    pub const INDEX_PATH: &'static str = ".routes";
    pub const ROUTE_EXTENSION: &'static str = "route";

    /// Loads the index, skipping malformed rows. A missing file means no saved routes.
    /// Other read failures are reported before falling back to an empty index.
    pub fn load() -> Self {
        match RouteStore::open(".") {
            Ok(store) => Self {
                map: store
                    .iter()
                    .map(|(id, name)| (id, name.to_owned()))
                    .collect(),
            },
            Err(error) => {
                crate::log_error!("failed to read route index: {error:?}");
                Self::default()
            }
        }
    }

    pub fn iter(&self) -> impl Iterator<Item = (u32, &str)> + '_ {
        self.map.iter().map(|(id, name)| (*id, name.as_str()))
    }

    pub fn len(&self) -> usize {
        self.map.len()
    }

    pub fn is_empty(&self) -> bool {
        self.map.is_empty()
    }

    /// Persist names for existing routes through the append-only route store.
    pub fn save(&self) -> std::io::Result<()> {
        let mut store = RouteStore::open(".").map_err(std::io::Error::other)?;
        if self.map.len() != store.iter().count()
            || self.map.keys().any(|&id| store.file_id(id).is_none())
            || self
                .map
                .values()
                .any(|name| name.trim().is_empty() || name.contains(['\t', '\n', '\r']))
        {
            return Err(std::io::Error::new(
                std::io::ErrorKind::InvalidInput,
                "the route index can only rename existing recordings",
            ));
        }
        for (&id, name) in &self.map {
            store
                .rename_route(id, name)
                .map_err(std::io::Error::other)?;
        }
        Ok(())
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
        let next_id = self
            .map
            .keys()
            .next_back()
            .and_then(|id| id.checked_add(1))
            .or_else(|| (1..=u32::MAX).find(|id| !self.map.contains_key(id)))
            .unwrap_or(0);
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

#[cfg(test)]
mod tests {
    use super::RouteIndex;

    #[test]
    fn next_id_wraps_to_unused_route_without_overwriting() {
        let mut index = RouteIndex::default();
        index.update(1, "first");
        index.update(u32::MAX, "last");
        assert_eq!(index.next_id(), 2);
    }
}
