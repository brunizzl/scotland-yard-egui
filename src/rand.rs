///  linear congruential generator
#[derive(Default, Clone, Copy)]
pub struct Lcg {
    state: u64,
}

impl Lcg {
    pub fn new(seed: u64) -> Self {
        Self { state: seed }
    }

    pub fn next(&mut self) -> u32 {
        let old = self.state;
        //values by Donald Knuth
        self.state = old
            .wrapping_mul(6_364_136_223_846_793_005)
            .wrapping_add(1_442_695_040_888_963_407);
        (old >> 16) as u32 //bits in middle have highest quality
    }

    pub fn next_u64(&mut self) -> u64 {
        let val1 = self.next() as u64;
        let val2 = self.next() as u64;
        (val1 << 32) | val2
    }

    pub fn next_usize(&mut self) -> usize {
        if cfg!(target_pointer_width = "64") {
            self.next_u64() as usize
        } else {
            self.next() as usize
        }
    }

    pub fn usize_hash(seed: usize) -> usize {
        let mut hasher = Self::new(seed as u64);
        hasher.waste(3);
        hasher.next_usize()
    }

    /// produces random value from uniform distribution over interval `-1.0..1.0`
    pub fn next_in_unit_range(&mut self) -> f32 {
        const SCALE: f32 = 2.0 / (u32::MAX as f32);
        (self.next() as f32) * SCALE - 1.0
    }

    pub fn waste(&mut self, nr: usize) {
        for _ in 0..nr {
            self.next();
        }
    }
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn unit_range_distribution() {
        let mut rng = Lcg::new(1337);
        let mut max = -1e10;
        let mut min = 1e10;
        let mut avg = 0.0;
        let mut sig2 = 0.0;
        const N: usize = 10_000;
        for _ in 0..N {
            let x = rng.next_in_unit_range();
            max = x.max(max);
            min = x.min(min);
            avg += x;
            sig2 += x * x;
        }
        avg /= N as f32;
        sig2 /= N as f32;
        assert!((0.99..=1.0).contains(&max));
        assert!((-1.0..-0.99).contains(&min));
        assert!(avg.abs() < 0.01);
        assert!((sig2 - 0.33333).abs() < 0.01);
    }
}
