#[derive(Clone, Copy)]
pub struct BidirectionalRange {
    start: usize, //first element included
    end: usize,   //first element not included
    step: isize,
}

impl BidirectionalRange {
    pub fn uninitialized() -> Self {
        Self { start: 0, end: 0, step: 0 }
    }

    pub fn is_uninitialized(&self) -> bool {
        matches!(*self, Self { start: 0, end: 0, step: 0 })
    }

    fn add(a: usize, b: isize) -> usize {
        ((a as isize) + b) as usize
    }

    pub fn new_forward(start: usize, end: usize) -> Self {
        let end = if end < start { start } else { end };
        Self { start, end, step: 1 }
    }

    #[allow(dead_code)]
    pub fn new_backward(start: usize, end: usize) -> Self {
        let end = if end > start { start } else { end };
        Self { start, end, step: -1 }
    }

    pub fn reversed(&self) -> Self {
        let step = -self.step;
        let start = Self::add(self.end, step);
        let end = Self::add(self.start, step);
        Self { start, end, step }
    }
}

impl Iterator for BidirectionalRange {
    type Item = usize;
    fn next(&mut self) -> Option<Self::Item> {
        if self.start == self.end {
            None
        } else {
            let res = self.start;
            self.start = Self::add(self.start, self.step);
            Some(res)
        }
    }
}
