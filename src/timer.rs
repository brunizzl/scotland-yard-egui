pub struct Timer {
    name: String,
    begin: std::time::Instant,
    visible: bool,
}

impl Timer {
    pub fn new(name: impl ToString) -> Self {
        Self::new_visible(name, true)
    }
    pub fn new_visible(name: impl ToString, visible: bool) -> Self {
        Self {
            name: name.to_string(),
            begin: std::time::Instant::now(),
            visible,
        }
    }
}

impl Drop for Timer {
    fn drop(&mut self) {
        if self.visible {
            let end = std::time::Instant::now();
            println!("{} took {:?}", self.name, end - self.begin);
        }
    }
}
