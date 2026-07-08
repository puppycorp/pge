use std::fmt;
use std::hash::Hash;
use std::marker::PhantomData;

use serde::{Deserialize, Deserializer, Serialize, Serializer};

#[derive(Debug)]
pub struct ArenaId<T> {
    index: usize,
    _phantom: PhantomData<T>,
}

impl<T> ArenaId<T> {
    fn new(index: usize) -> Self {
        Self {
            index,
            _phantom: PhantomData,
        }
    }

    pub fn index(&self) -> usize {
        self.index
    }
}

impl<T> fmt::Display for ArenaId<T> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.index)
    }
}

impl<T> Clone for ArenaId<T> {
    fn clone(&self) -> Self {
        *self
    }
}

impl<T> Copy for ArenaId<T> {}

impl<T> PartialEq for ArenaId<T> {
    fn eq(&self, other: &Self) -> bool {
        self.index == other.index
    }
}

impl<T> Eq for ArenaId<T> {}

impl<T> Hash for ArenaId<T> {
    fn hash<H: std::hash::Hasher>(&self, state: &mut H) {
        self.index.hash(state);
    }
}

impl<T> Serialize for ArenaId<T> {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        self.index.serialize(serializer)
    }
}

impl<'de, T> Deserialize<'de> for ArenaId<T> {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        Ok(Self::new(usize::deserialize(deserializer)?))
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Arena<T> {
    items: Vec<Option<T>>,
    free_slots: Vec<usize>,
}

impl<T> Default for Arena<T> {
    fn default() -> Self {
        Self {
            items: Vec::new(),
            free_slots: Vec::new(),
        }
    }
}

impl<T> Arena<T> {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn insert(&mut self, item: T) -> ArenaId<T> {
        if let Some(index) = self.free_slots.pop() {
            self.items[index] = Some(item);
            ArenaId::new(index)
        } else {
            let index = self.items.len();
            self.items.push(Some(item));
            ArenaId::new(index)
        }
    }

    pub fn get(&self, id: &ArenaId<T>) -> Option<&T> {
        self.items.get(id.index).and_then(Option::as_ref)
    }

    pub fn get_mut(&mut self, id: &ArenaId<T>) -> Option<&mut T> {
        self.items.get_mut(id.index).and_then(Option::as_mut)
    }

    pub fn remove(&mut self, id: &ArenaId<T>) -> Option<T> {
        if id.index >= self.items.len() {
            return None;
        }
        let item = self.items[id.index].take();
        if item.is_some() {
            self.free_slots.push(id.index);
        }
        item
    }

    pub fn contains(&self, id: &ArenaId<T>) -> bool {
        id.index < self.items.len() && self.items[id.index].is_some()
    }

    pub fn len(&self) -> usize {
        self.items.iter().filter(|item| item.is_some()).count()
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    pub fn iter(&self) -> impl Iterator<Item = (ArenaId<T>, &T)> {
        self.items
            .iter()
            .enumerate()
            .filter_map(|(index, item)| item.as_ref().map(|item| (ArenaId::new(index), item)))
    }

    pub fn iter_mut(&mut self) -> impl Iterator<Item = (ArenaId<T>, &mut T)> {
        self.items
            .iter_mut()
            .enumerate()
            .filter_map(|(index, item)| item.as_mut().map(|item| (ArenaId::new(index), item)))
    }
}
