
use parking_lot::lock_api::RawMutex;
type SmallMutex = parking_lot::RawMutex;

use itertools::Itertools;

pub struct Locked<'a, T: ?Sized> {
    ptr: &'a mut T,
    mutex: &'a SmallMutex,
}

impl<'a, T: ?Sized> std::ops::Deref for Locked<'a, T> {
    type Target = T;
    fn deref(&self) -> &Self::Target {
        self.ptr
    }
}

impl<'a, T: ?Sized> std::ops::DerefMut for Locked<'a, T> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        self.ptr
    }
}

impl<'a, T: ?Sized> Drop for Locked<'a, T> {
    fn drop(&mut self) {
        // safety: reference guarded by mutex is only accesible while self is alive.
        unsafe {
            self.mutex.unlock();
        }
    }
}

pub struct LockedChunk<'a, T> {
    ptr: &'a mut [T],
    mutex: &'a SmallMutex,
    chunk_start: usize,
    chunk_end: usize,
}

impl<'a, T> LockedChunk<'a, T> {
    pub fn valid_range(&self) -> std::ops::Range<usize> {
        self.chunk_start..self.chunk_end
    }
}

impl<'a, T> std::ops::Index<usize> for LockedChunk<'a, T> {
    type Output = T;
    fn index(&self, index: usize) -> &Self::Output {
        assert!(index >= self.chunk_start);
        assert!(index < self.chunk_end);
        &self.ptr[index]
    }
}

impl<'a, T> std::ops::Index<std::ops::Range<usize>> for LockedChunk<'a, T> {
    type Output = [T];
    fn index(&self, index: std::ops::Range<usize>) -> &Self::Output {
        assert!(index.start >= self.chunk_start);
        assert!(index.end <= self.chunk_end);
        &self.ptr[index]
    }
}

impl<'a, T> std::ops::IndexMut<usize> for LockedChunk<'a, T> {
    fn index_mut(&mut self, index: usize) -> &mut Self::Output {
        assert!(index >= self.chunk_start);
        assert!(index < self.chunk_end);
        &mut self.ptr[index]
    }
}

impl<'a, T> std::ops::IndexMut<std::ops::Range<usize>> for LockedChunk<'a, T> {
    fn index_mut(&mut self, index: std::ops::Range<usize>) -> &mut Self::Output {
        assert!(index.start >= self.chunk_start);
        assert!(index.end <= self.chunk_end);
        &mut self.ptr[index]
    }
}

impl<'a, T> Drop for LockedChunk<'a, T> {
    fn drop(&mut self) {
        // safety: reference guarded by mutex is only accesible while self is alive.
        unsafe {
            self.mutex.unlock();
        }
    }
}

/// just a [`Box`] of `[T]`, except multiple threads can mutably access different chunks of length [`CHUNK_SIZE`] in parallel.
pub struct ChunkedBox<T, const CHUNK_SIZE: usize> {
    /// actual content we want to mutate
    data: Box<[T]>,
    /// holds one entry for every [`CHUNK_SIZE`] many entries of [`Self::data`]
    /// and (if a rest exists) for the remaining number of entries at the end of [`Self::data`].
    chunks_access: Box<[SmallMutex]>,
}

impl<T, const CHUNK_SIZE: usize> ChunkedBox<T, CHUNK_SIZE> {
    pub fn new(data: Box<[T]>) -> Self {
        let nr_chunks = data.len().div_ceil(CHUNK_SIZE);
        let chunks_access =
            (0..nr_chunks).map(|_| SmallMutex::INIT).collect_vec().into_boxed_slice();
        Self {
            data,
            chunks_access,
        }
    }

    pub fn into_boxed_slice(self) -> Box<[T]> {
        self.data
    }

    /// returns a fancy mutable reference to the item at index `i`.
    /// while the reference is active, the respective chunk is locked.
    pub fn lock<'a>(&'a self, i: usize) -> Locked<'a, T> {
        debug_assert!(i < self.data.len());
        let i_outher = i / CHUNK_SIZE;
        let mutex = &self.chunks_access[i_outher];
        mutex.lock();
        // safety: mutex guards access.
        let ptr = unsafe { &mut *(self.data.as_ptr() as *mut T).add(i) };
        Locked { ptr, mutex }
    }

    /// to utilize this funtion as intended, one needs to know beforehand,
    /// that chunks required by the business logic will never go over chunk boundaries of [`self`].
    /// It thus makes sense to set [`Self::chunk_size`] appropriately.
    ///
    /// What is then retourned, is the full chunk mananged by a single mutex, except not offset.
    /// indexing into the result is thus only successfull for index values falling into the same chunk as argument `index`.
    pub fn lock_chunk<'a>(&'a self, index: usize) -> LockedChunk<'a, T> {
        assert!(index < self.data.len());
        let i_outher = index / CHUNK_SIZE;
        let mutex = &self.chunks_access[i_outher];
        mutex.lock();
        // safety: mutex guards access.
        let ptr = unsafe {
            let start = self.data.as_ptr() as *mut T;
            std::slice::from_raw_parts_mut(start, self.data.len())
        };
        let chunk_start = i_outher * CHUNK_SIZE;
        let chunk_end = (chunk_start + CHUNK_SIZE).min(self.data.len());
        LockedChunk {
            ptr,
            mutex,
            chunk_start,
            chunk_end,
        }
    }
}

#[cfg(test)]
mod test {
    use super::*;
    use itertools::Itertools;

    #[test]
    fn access_chunked_data_in_threads_single() {
        let data = vec![0; 1_000_000].into_boxed_slice();
        let chunks = ChunkedBox::<_, 100>::new(data);
        let chunks_ref = &chunks;
        std::thread::scope(|s| {
            let threads = (0..10)
                .map(|i| {
                    s.spawn(move || {
                        for j in 0..1_000_000 {
                            let full_index = (i * 100_000 + j) % 1_000_000;
                            let mut val = chunks_ref.lock(full_index);
                            *val += i;
                        }
                    })
                })
                .collect_vec();
            for thread in threads {
                thread.join().ok();
            }
        });

        let data = chunks.into_boxed_slice();
        assert!(data.iter().all(|&x| x == 45));
    }

    #[test]
    fn access_chunked_data_in_threads_slice() {
        let data = vec![0; 1_000_000].into_boxed_slice();
        let chunks = ChunkedBox::<_, 1000>::new(data);
        let chunks_ref = &chunks;
        std::thread::scope(|s| {
            let threads = (0..10)
                .map(|i| {
                    s.spawn(move || {
                        for j in 0..100_000 {
                            let chunk_start = (i * 1000 + j * 100) % 1_000_000;
                            let chunk_end = chunk_start + 100;
                            if chunk_end > 1_000_000 {
                                continue;
                            }
                            let mut chunk = chunks_ref.lock_chunk(chunk_start);
                            for val in &mut chunk[chunk_start..chunk_end] {
                                *val += i;
                            }
                        }
                    })
                })
                .collect_vec();
            for thread in threads {
                thread.join().ok();
            }
        });

        let data = chunks.into_boxed_slice();
        assert!(data.iter().all(|&x| x == 450));
    }
}
