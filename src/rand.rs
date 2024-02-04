use std::usize;

///  linear congruential generator
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
        self.state = old.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        (old >> 16) as u32 //bits in middle have highest quality
    }

    pub fn waste(&mut self, nr: usize) {
        for _ in 0..nr {
            self.next();
        }
    }
}
