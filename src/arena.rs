use std::fmt;
use std::fmt::Display;
use std::hash::Hash;
use std::marker::PhantomData;

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

impl<T> Display for ArenaId<T> {
	fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
		write!(f, "{}", self.index)
	}
}

impl<T> Clone for ArenaId<T> {
    fn clone(&self) -> Self {
        Self {
            index: self.index,
            _phantom: PhantomData,
        }
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

#[derive(Debug, Clone, PartialEq)]
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
        Arena::default()
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

	pub fn reserve(&mut self, size: usize) {
		self.items.reserve(size);
	}

	pub fn mem_size(&self) -> usize {
		std::mem::size_of_val(&self) + self.items.capacity() * std::mem::size_of::<Option<T>>()
	}

    pub fn get(&self, id: &ArenaId<T>) -> Option<&T> {
        self.items.get(id.index).and_then(|opt| opt.as_ref())
    }

    pub fn get_mut(&mut self, id: &ArenaId<T>) -> Option<&mut T> {
        self.items.get_mut(id.index).and_then(|opt| opt.as_mut())
    }

    pub fn remove(&mut self, id: &ArenaId<T>) -> Option<T> {
        if id.index < self.items.len() {
            let removed_item = self.items[id.index].take();
            if removed_item.is_some() {
                self.free_slots.push(id.index);
            }
            removed_item
        } else {
            None
        }
    }

    pub fn len(&self) -> usize {
        self.items.len()
    }

	pub fn contains(&self, id: &ArenaId<T>) -> bool {
        id.index < self.items.len() && self.items[id.index].is_some()
    }

    pub fn iter(&self) -> ArenaIterator<T> {
        ArenaIterator {
            arena: self,
            current: 0,
        }
    }

    pub fn iter_mut(&mut self) -> ArenaIterMut<T> {
        ArenaIterMut {
            arena: self as *mut _,
            current: 0,
			_marker: PhantomData,
        }
    }
}

pub struct ArenaIterator<'a, T> {
    arena: &'a Arena<T>,
    current: usize,
}

impl<'a, T> Iterator for ArenaIterator<'a, T> {
    type Item = (ArenaId<T>, &'a T);

    fn next(&mut self) -> Option<Self::Item> {
        while self.current < self.arena.items.len() {
            let index = self.current;
            self.current += 1;
            if let Some(ref item) = self.arena.items[index] {
                return Some((ArenaId::new(index), item));
            }
        }
        None
    }
}

pub struct ArenaIterMut<'a, T> {
    arena: *mut Arena<T>,
    current: usize,
    _marker: PhantomData<&'a mut Arena<T>>,
}

impl<'a, T> Iterator for ArenaIterMut<'a, T> {
    type Item = (ArenaId<T>, &'a mut T);

    fn next(&mut self) -> Option<Self::Item> {
        unsafe {
            let len = (*self.arena).items.len();
            let items_ptr = (*self.arena).items.as_mut_ptr();
            while self.current < len {
                let index = self.current;
                self.current += 1;
                let slot_ptr = items_ptr.add(index);
                if let Some(item) = (*slot_ptr).as_mut() {
                    return Some((ArenaId::new(index), item));
                }
            }
            None
        }
    }
}

pub struct ArenaIntoIterator<T> {
    arena: Arena<T>,
    current: usize,
}

impl<T> Iterator for ArenaIntoIterator<T> {
    type Item = (ArenaId<T>, T);

    fn next(&mut self) -> Option<Self::Item> {
        while self.current < self.arena.items.len() {
            let index = self.current;
            self.current += 1;
            if let Some(item) = self.arena.items[index].take() {
                return Some((ArenaId::new(index), item));
            }
        }
        None
    }
}

impl<'a, T> IntoIterator for &'a Arena<T> {
    type Item = (ArenaId<T>, &'a T);
    type IntoIter = ArenaIterator<'a, T>;

    fn into_iter(self) -> Self::IntoIter {
        ArenaIterator {
            arena: self,
            current: 0,
        }
    }
}

impl<'a, T> IntoIterator for &'a mut Arena<T> {
    type Item = (ArenaId<T>, &'a mut T);
    type IntoIter = ArenaIterMut<'a, T>;

    fn into_iter(self) -> Self::IntoIter {
        ArenaIterMut {
            arena: self as *mut _,
            current: 0,
            _marker: PhantomData,
        }
    }
}

impl<T> IntoIterator for Arena<T> {
    type Item = (ArenaId<T>, T);
    type IntoIter = ArenaIntoIterator<T>;

    fn into_iter(self) -> Self::IntoIter {
        ArenaIntoIterator {
            arena: self,
            current: 0,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::HashMap;

    #[derive(Debug, PartialEq)]
    struct Person {
        name: String,
    }

    #[derive(Debug, PartialEq)]
    struct Car {
        model: String,
    }

    #[test]
    fn test_insert_and_get_person() {
        let mut arena = Arena::new();
        let person_id = arena.insert(Person { name: "Alice".to_string() });

        let person = arena.get(&person_id).unwrap();
        assert_eq!(person, &Person { name: "Alice".to_string() });
    }

    #[test]
    fn test_insert_and_get_car() {
        let mut arena = Arena::new();
        let car_id = arena.insert(Car { model: "Tesla Model 3".to_string() });

        let car = arena.get(&car_id).unwrap();
        assert_eq!(car, &Car { model: "Tesla Model 3".to_string() });
    }

    #[test]
    fn test_remove_person() {
        let mut arena = Arena::new();
        let person_id = arena.insert(Person { name: "Alice".to_string() });

        let removed_person = arena.remove(&person_id).unwrap();
        assert_eq!(removed_person, Person { name: "Alice".to_string() });
        assert!(arena.get(&person_id).is_none());
    }

    #[test]
    fn test_remove_car() {
        let mut arena = Arena::new();
        let car_id = arena.insert(Car { model: "Tesla Model 3".to_string() });

        let removed_car = arena.remove(&car_id).unwrap();
        assert_eq!(removed_car, Car { model: "Tesla Model 3".to_string() });
        assert!(arena.get(&car_id).is_none());
    }

    #[test]
    fn test_reuse_slots_person() {
        let mut arena = Arena::new();
        let person_id1 = arena.insert(Person { name: "Alice".to_string() });
        arena.remove(&person_id1);

        let person_id2 = arena.insert(Person { name: "Bob".to_string() });
        let person = arena.get(&person_id2).unwrap();

        assert_eq!(person, &Person { name: "Bob".to_string() });
        assert_eq!(person_id1.index, person_id2.index); // Ensure the slot was reused
    }

    #[test]
    fn test_reuse_slots_car() {
        let mut arena = Arena::new();
        let car_id1 = arena.insert(Car { model: "Tesla Model 3".to_string() });
        arena.remove(&car_id1);

        let car_id2 = arena.insert(Car { model: "Ford Mustang".to_string() });
        let car = arena.get(&car_id2).unwrap();

        assert_eq!(car, &Car { model: "Ford Mustang".to_string() });
        assert_eq!(car_id1.index, car_id2.index); // Ensure the slot was reused
    }

    #[test]
    fn test_get_mut_person() {
        let mut arena = Arena::new();
        let person_id = arena.insert(Person { name: "Alice".to_string() });

        if let Some(person) = arena.get_mut(&person_id) {
            person.name = "Bob".to_string();
        }

        let person = arena.get(&person_id).unwrap();
        assert_eq!(person, &Person { name: "Bob".to_string() });
    }

    #[test]
    fn test_get_mut_car() {
        let mut arena = Arena::new();
        let car_id = arena.insert(Car { model: "Tesla Model 3".to_string() });

        if let Some(car) = arena.get_mut(&car_id) {
            car.model = "Ford Mustang".to_string();
        }

        let car = arena.get(&car_id).unwrap();
        assert_eq!(car, &Car { model: "Ford Mustang".to_string() });
    }

    #[test]
    fn test_iter_person() {
        let mut arena = Arena::new();
        arena.insert(Person { name: "Alice".to_string() });
        arena.insert(Person { name: "Bob".to_string() });

        let mut iter = arena.iter();
        assert_eq!(iter.next(), Some((ArenaId::new(0), &Person { name: "Alice".to_string() })));
        assert_eq!(iter.next(), Some((ArenaId::new(1), &Person { name: "Bob".to_string() })));
        assert_eq!(iter.next(), None);
    }

    #[test]
    fn test_iter_car() {
        let mut arena = Arena::new();
        arena.insert(Car { model: "Tesla Model 3".to_string() });
        arena.insert(Car { model: "Ford Mustang".to_string() });

        let mut iter = arena.iter();
        assert_eq!(iter.next(), Some((ArenaId::new(0), &Car { model: "Tesla Model 3".to_string() })));
        assert_eq!(iter.next(), Some((ArenaId::new(1), &Car { model: "Ford Mustang".to_string() })));
        assert_eq!(iter.next(), None);
    }

    #[test]
    fn test_into_iter_person() {
        let mut arena = Arena::new();
        arena.insert(Person { name: "Alice".to_string() });
        arena.insert(Person { name: "Bob".to_string() });

        let iter: ArenaIntoIterator<_> = arena.into_iter();
        let results: Vec<_> = iter.collect();
        assert_eq!(results, vec![
            (ArenaId::new(0), Person { name: "Alice".to_string() }),
            (ArenaId::new(1), Person { name: "Bob".to_string() }),
        ]);
    }

    #[test]
    fn test_iter_mut_person() {
        let mut arena = Arena::new();
        arena.insert(Person { name: "Alice".to_string() });
        arena.insert(Person { name: "Bob".to_string() });

        let mut iter = arena.iter_mut();
        if let Some((_, person)) = iter.next() {
            person.name = "Charlie".to_string();
        }

        let results: Vec<_> = arena.iter().collect();
        assert_eq!(results, vec![
            (ArenaId::new(0), &Person { name: "Charlie".to_string() }),
            (ArenaId::new(1), &Person { name: "Bob".to_string() }),
        ]);
    }

    #[test]
    fn test_arena_id_as_hashmap_key() {
        let mut map = HashMap::new();
        let id1 = ArenaId::<i32>::new(1);
        let id2 = ArenaId::<i32>::new(1);
        let id3 = ArenaId::<i32>::new(2);

        map.insert(id1, "value1");
        assert_eq!(map.get(&id2), Some(&"value1"));
        assert_eq!(map.get(&id3), None);

        assert_eq!(map.entry(id2).or_insert("value2"), &"value1");
        assert_eq!(map.len(), 1);

        assert_eq!(map.entry(id3).or_insert("value3"), &"value3");
        assert_eq!(map.len(), 2);
    }

	#[derive(Debug, PartialEq)]
    struct TestStruct {
        name: String,
        value: i32,
    }

    #[test]
    fn test_arena_with_struct() {
        let mut arena = Arena::new();
        let mut id_map = HashMap::new();

        // Insert items into the arena and store their IDs in the HashMap
        let id1 = arena.insert(TestStruct { name: "Item 1".to_string(), value: 10 });
        let id2 = arena.insert(TestStruct { name: "Item 2".to_string(), value: 20 });
        let id3 = arena.insert(TestStruct { name: "Item 3".to_string(), value: 30 });

        id_map.insert(id1, "First");
        id_map.insert(id2, "Second");
        id_map.insert(id3, "Third");

        // Test retrieving items from the arena
        assert_eq!(arena.get(&id1), Some(&TestStruct { name: "Item 1".to_string(), value: 10 }));
        assert_eq!(arena.get(&id2), Some(&TestStruct { name: "Item 2".to_string(), value: 20 }));
        assert_eq!(arena.get(&id3), Some(&TestStruct { name: "Item 3".to_string(), value: 30 }));

        // Test using ArenaId as HashMap key
        assert_eq!(id_map.get(&id1), Some(&"First"));
        assert_eq!(id_map.get(&id2), Some(&"Second"));
        assert_eq!(id_map.get(&id3), Some(&"Third"));

        // Test modifying an item in the arena
        if let Some(item) = arena.get_mut(&id2) {
            item.value = 25;
        }
        assert_eq!(arena.get(&id2), Some(&TestStruct { name: "Item 2".to_string(), value: 25 }));

        // Test that ArenaId with the same index works correctly as a key
        let same_as_id2 = ArenaId::<TestStruct>::new(id2.index());
        assert_eq!(id_map.get(&same_as_id2), Some(&"Second"));

        // Test entry API of HashMap with ArenaId
        assert_eq!(id_map.entry(id2.clone()).or_insert("New"), &"Second");
        assert_eq!(id_map.entry(ArenaId::<TestStruct>::new(100)).or_insert("New"), &"New");

        assert_eq!(id_map.len(), 4);
    }
}
