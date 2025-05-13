use std::collections::HashMap;
use crate::core::{BodyHandle, ConstraintHandle};
use crate::error::PhysicsError;
use crate::Result;

/// Generic storage trait for physics objects
pub trait Storage<T, H> {
    /// Creates a new empty storage
    fn new() -> Self;

    /// Adds an item to the storage and returns its handle
    fn add(&mut self, item: T) -> H;

    /// Gets a reference to an item by its handle
    fn get(&self, handle: H) -> Option<&T>;

    /// Gets a mutable reference to an item by its handle
    fn get_mut(&mut self, handle: H) -> Option<&mut T>;

    /// Removes an item from the storage
    fn remove(&mut self, handle: H) -> Option<T>;

    /// Returns the number of items in the storage
    fn len(&self) -> usize;

    /// Returns whether the storage is empty
    fn is_empty(&self) -> bool;

    /// Clears all items from the storage
    fn clear(&mut self);

    /// Returns an iterator over all handles
    fn handles(&self) -> Vec<H>;

    /// Returns an iterator over all items
    fn iter<'a>(&'a self) -> impl Iterator<Item = (H, &'a T)> + 'a where T: 'a;

    /// Returns a mutable iterator over all items
    fn iter_mut<'a>(&'a mut self) -> impl Iterator<Item = (H, &'a mut T)> + 'a where T: 'a;
}

/// Storage for physics bodies
pub struct BodyStorage<T> {
    items: HashMap<BodyHandle, T>,
    next_id: u32,
}

impl<T> Storage<T, BodyHandle> for BodyStorage<T> {
    fn new() -> Self {
        Self {
            items: HashMap::new(),
            next_id: 1, // Start at 1, so 0 can represent invalid handle
        }
    }
    
    fn add(&mut self, item: T) -> BodyHandle {
        let handle = BodyHandle(self.next_id);
        self.next_id += 1;
        self.items.insert(handle, item);
        handle
    }
    
    fn get(&self, handle: BodyHandle) -> Option<&T> {
        self.items.get(&handle)
    }
    
    fn get_mut(&mut self, handle: BodyHandle) -> Option<&mut T> {
        self.items.get_mut(&handle)
    }
    
    fn remove(&mut self, handle: BodyHandle) -> Option<T> {
        self.items.remove(&handle)
    }
    
    fn len(&self) -> usize {
        self.items.len()
    }
    
    fn is_empty(&self) -> bool {
        self.items.is_empty()
    }
    
    fn clear(&mut self) {
        self.items.clear();
    }
    
    fn handles(&self) -> Vec<BodyHandle> {
        self.items.keys().copied().collect()
    }
    
    fn iter<'a>(&'a self) -> impl Iterator<Item = (BodyHandle, &'a T)> + 'a where T: 'a {
        self.items.iter().map(|(h, item)| (*h, item))
    }

    fn iter_mut<'a>(&'a mut self) -> impl Iterator<Item = (BodyHandle, &'a mut T)> + 'a where T: 'a {
        self.items.iter_mut().map(|(h, item)| (*h, item))
    }
}

impl<T> BodyStorage<T> {
    /// Gets a body by its handle, returning an error if not found
    pub fn get_body(&self, handle: BodyHandle) -> Result<&T> {
        self.get(handle)
            .ok_or_else(|| PhysicsError::ResourceNotFound(format!("Body with handle {:?} not found", handle)))
    }
    
    /// Gets a mutable reference to a body by its handle, returning an error if not found
    pub fn get_body_mut(&mut self, handle: BodyHandle) -> Result<&mut T> {
        self.get_mut(handle)
            .ok_or_else(|| PhysicsError::ResourceNotFound(format!("Body with handle {:?} not found", handle)))
    }
}

/// Storage for physics constraints
pub struct ConstraintStorage<T> {
    items: HashMap<ConstraintHandle, T>,
    next_id: u32,
}

impl<T> Storage<T, ConstraintHandle> for ConstraintStorage<T> {
    fn new() -> Self {
        Self {
            items: HashMap::new(),
            next_id: 1, // Start at 1, so 0 can represent invalid handle
        }
    }
    
    fn add(&mut self, item: T) -> ConstraintHandle {
        let handle = ConstraintHandle(self.next_id);
        self.next_id += 1;
        self.items.insert(handle, item);
        handle
    }
    
    fn get(&self, handle: ConstraintHandle) -> Option<&T> {
        self.items.get(&handle)
    }
    
    fn get_mut(&mut self, handle: ConstraintHandle) -> Option<&mut T> {
        self.items.get_mut(&handle)
    }
    
    fn remove(&mut self, handle: ConstraintHandle) -> Option<T> {
        self.items.remove(&handle)
    }
    
    fn len(&self) -> usize {
        self.items.len()
    }
    
    fn is_empty(&self) -> bool {
        self.items.is_empty()
    }
    
    fn clear(&mut self) {
        self.items.clear();
    }
    
    fn handles(&self) -> Vec<ConstraintHandle> {
        self.items.keys().copied().collect()
    }
    
    fn iter<'a>(&'a self) -> impl Iterator<Item = (ConstraintHandle, &'a T)> + 'a where T: 'a {
        self.items.iter().map(|(h, item)| (*h, item))
    }

    fn iter_mut<'a>(&'a mut self) -> impl Iterator<Item = (ConstraintHandle, &'a mut T)> + 'a where T: 'a {
        self.items.iter_mut().map(|(h, item)| (*h, item))
    }
}

impl<T> ConstraintStorage<T> {
    /// Gets a constraint by its handle, returning an error if not found
    pub fn get_constraint(&self, handle: ConstraintHandle) -> Result<&T> {
        self.get(handle)
            .ok_or_else(|| PhysicsError::ResourceNotFound(format!("Constraint with handle {:?} not found", handle)))
    }
    
    /// Gets a mutable reference to a constraint by its handle, returning an error if not found
    pub fn get_constraint_mut(&mut self, handle: ConstraintHandle) -> Result<&mut T> {
        self.get_mut(handle)
            .ok_or_else(|| PhysicsError::ResourceNotFound(format!("Constraint with handle {:?} not found", handle)))
    }
}